"""Yellow-line segmentation and metric path estimation."""

from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np

from .segmentation import SegmentationResult


@dataclass(frozen=True)
class VisionConfig:
    """Parameters for color segmentation and bird's-eye path fitting."""

    # The supplied camera has a strong magenta cast.  Its yellow/orange
    # center line is therefore often closer to red than to the usual HSV
    # yellow range.  Keep the HSV path for normally balanced cameras and use
    # the signed BGR differences below for this camera's warm cast.
    hsv_lower: Tuple[int, int, int] = (0, 25, 35)
    hsv_upper: Tuple[int, int, int] = (42, 255, 255)
    lab_b_min: int = 110
    adaptive_lab_percentile: float = 0.0
    red_blue_min: int = -8
    red_green_min: int = 25
    warm_luminance_min: int = 145
    prefer_warm_camera_mask: bool = True
    clahe_clip_limit: float = 2.0
    clahe_grid_size: int = 8
    roi_polygon: Tuple[float, ...] = (
        0.10,
        1.00,
        0.90,
        1.00,
        0.65,
        0.45,
        0.35,
        0.45,
    )
    perspective_source: Tuple[float, ...] = (
        0.10,
        1.00,
        0.90,
        1.00,
        0.65,
        0.45,
        0.35,
        0.45,
    )
    birdseye_width: int = 400
    birdseye_height: int = 600
    near_distance_m: float = 0.30
    far_distance_m: float = 2.00
    half_width_m: float = 1.00
    open_kernel: int = 3
    close_kernel: int = 7
    line_close_kernel: int = 31
    min_component_area_px: int = 80
    line_min_area_px: int = 250
    line_min_span_px: int = 40
    line_feature_enabled: bool = True
    line_min_elongation: float = 2.0
    # A symmetric search polygon limits the line search to the road while
    # keeping the same left/right camera margin. The values are normalized
    # x/y pairs in bottom-left, bottom-right, top-right, top-left order.
    line_roi_polygon: Tuple[float, ...] = (
        0.10,
        1.00,
        0.90,
        1.00,
        0.65,
        0.45,
        0.35,
        0.45,
    )
    sample_rows: int = 7
    sample_band_height_px: int = 30
    min_pixels_per_band: int = 20
    polynomial_degree: int = 2
    max_fit_residual_m: float = 0.20
    target_mask_ratio: float = 0.025

    def validate(self) -> None:
        if len(self.roi_polygon) != 8 or len(self.perspective_source) != 8:
            raise ValueError("ROI and perspective_source must each contain 8 values")
        if len(self.line_roi_polygon) != 8:
            raise ValueError("line_roi_polygon must contain 8 values")
        if not all(0.0 <= value <= 1.0 for value in self.roi_polygon):
            raise ValueError("roi_polygon values must be normalized to [0, 1]")
        if not all(0.0 <= value <= 1.0 for value in self.perspective_source):
            raise ValueError("perspective_source values must be normalized to [0, 1]")
        if not all(0.0 <= value <= 1.0 for value in self.line_roi_polygon):
            raise ValueError("line_roi_polygon values must be normalized to [0, 1]")
        if self.birdseye_width <= 0 or self.birdseye_height <= 0:
            raise ValueError("bird's-eye dimensions must be positive")
        if self.far_distance_m <= self.near_distance_m:
            raise ValueError("far_distance_m must exceed near_distance_m")
        if self.half_width_m <= 0.0:
            raise ValueError("half_width_m must be positive")
        if self.sample_rows < self.polynomial_degree + 1:
            raise ValueError(
                "sample_rows must support the configured polynomial degree"
            )
        if self.open_kernel <= 0 or self.close_kernel <= 0 or self.line_close_kernel <= 0:
            raise ValueError("morphology kernels must be positive")
        if self.line_min_area_px <= 0 or self.line_min_span_px <= 0:
            raise ValueError("line component limits must be positive")
        if self.line_min_elongation < 1.0:
            raise ValueError("line_min_elongation must be at least 1.0")
        if not -128 <= self.red_blue_min <= 255 or not 0 <= self.red_green_min <= 255:
            raise ValueError("warm-color channel differences must be in valid byte range")
        if not 0 <= self.warm_luminance_min <= 255:
            raise ValueError("warm_luminance_min must be in [0, 255]")
        if not 0.0 <= self.adaptive_lab_percentile <= 100.0:
            raise ValueError("adaptive_lab_percentile must be in [0, 100]")
        if self.target_mask_ratio <= 0.0:
            raise ValueError("target_mask_ratio must be positive")


@dataclass(frozen=True)
class PathEstimate:
    """Polynomial y(x) in robot coordinates (x forward, y left)."""

    coefficients: np.ndarray
    confidence: float
    residual_m: float
    points_xy: np.ndarray

    def lateral(self, forward_x: float) -> float:
        return float(np.polyval(self.coefficients, forward_x))

    def slope(self, forward_x: float) -> float:
        derivative = np.polyder(self.coefficients, 1)
        return float(np.polyval(derivative, forward_x))

    def curvature(self, forward_x: float) -> float:
        first = self.slope(forward_x)
        second = float(np.polyval(np.polyder(self.coefficients, 2), forward_x))
        return second / ((1.0 + first * first) ** 1.5)


@dataclass(frozen=True)
class VisionResult:
    mask: np.ndarray
    birdseye_mask: np.ndarray
    estimate: Optional[PathEstimate]
    roi_polygon_px: np.ndarray
    line_roi_polygon_px: Optional[np.ndarray] = None
    road_mask: Optional[np.ndarray] = None
    raw_line_mask: Optional[np.ndarray] = None
    centerline_points_px: np.ndarray = field(
        default_factory=lambda: np.empty((0, 2), dtype=np.float32)
    )


class YellowLineVision:
    """Segment a yellow guide line and fit its centerline in ground coordinates."""

    def __init__(
        self, config: VisionConfig, segmenter: Optional[object] = None
    ):
        config.validate()
        self.config = config
        self._segmenter = segmenter
        self._previous_near_lateral: Optional[float] = None

    @staticmethod
    def _normalized_points(
        values: Sequence[float], width: int, height: int
    ) -> np.ndarray:
        points = np.asarray(values, dtype=np.float32).reshape(4, 2)
        scale = np.asarray([max(width - 1, 1), max(height - 1, 1)], dtype=np.float32)
        return points * scale

    def _color_mask(
        self,
        frame_bgr: np.ndarray,
        roi_mask: np.ndarray,
        candidate_gate_mask: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
        luminance, channel_a, channel_b = cv2.split(lab)

        clahe = cv2.createCLAHE(
            clipLimit=self.config.clahe_clip_limit,
            tileGridSize=(self.config.clahe_grid_size, self.config.clahe_grid_size),
        )
        corrected_luminance = clahe.apply(luminance)
        corrected_lab = cv2.merge([corrected_luminance, channel_a, channel_b])
        corrected_bgr = cv2.cvtColor(corrected_lab, cv2.COLOR_LAB2BGR)
        hsv = cv2.cvtColor(corrected_bgr, cv2.COLOR_BGR2HSV)

        hsv_mask = cv2.inRange(
            hsv,
            np.asarray(self.config.hsv_lower, dtype=np.uint8),
            np.asarray(self.config.hsv_upper, dtype=np.uint8),
        )
        roi_b_values = channel_b[roi_mask > 0]
        percentile = (
            float(np.percentile(roi_b_values, self.config.adaptive_lab_percentile))
            if roi_b_values.size
            else float(self.config.lab_b_min)
        )
        adaptive_min = int(np.clip(max(self.config.lab_b_min, percentile), 0, 255))
        lab_mask = cv2.inRange(channel_b, adaptive_min, 255)

        # In this camera, the yellow center line is recorded as peach/orange
        # or pink/orange because the whole frame is magenta shifted.  Signed
        # channel differences remain useful in that situation: the line has
        # more red than blue and more red than green, while the purple road
        # and green shoulder do not satisfy both conditions consistently.
        frame_signed = frame_bgr.astype(np.int16)
        blue, green, red = cv2.split(frame_signed)
        warm_mask = (
            (red - blue >= self.config.red_blue_min)
            & (red - green >= self.config.red_green_min)
            & (luminance >= self.config.warm_luminance_min)
        ).astype(np.uint8) * 255

        warm_lab_mask = cv2.bitwise_and(warm_mask, lab_mask)
        if self.config.prefer_warm_camera_mask and np.any(warm_lab_mask):
            # Prefer the cast-aware mask whenever this frame contains a warm
            # candidate.  The broad HSV mask otherwise treats the magenta
            # road and indoor floor as yellow and can win over the real line.
            mask = warm_lab_mask
        else:
            # Keep a conventional HSV fallback for balanced-camera frames
            # and for synthetic/unit-test images with ordinary yellow paint.
            mask = cv2.bitwise_and(hsv_mask, lab_mask)
        mask = cv2.bitwise_and(mask, roi_mask)

        height, width = mask.shape
        line_polygon = self._normalized_points(
            self.config.line_roi_polygon,
            width,
            height,
        ).astype(np.int32)
        line_roi_mask = np.zeros_like(mask)
        cv2.fillPoly(line_roi_mask, [line_polygon], 255)
        mask = cv2.bitwise_and(mask, line_roi_mask)

        candidate_gate = None
        if candidate_gate_mask is not None:
            if candidate_gate_mask.shape != mask.shape:
                raise ValueError(
                    "candidate_gate_mask must match the input frame dimensions"
                )
            candidate_gate = np.where(candidate_gate_mask > 0, 255, 0).astype(
                np.uint8
            )
            # In mixed mode this is YOLOP's road-gated lane mask. Apply it
            # before component selection so a larger warm sidewalk component
            # cannot win and erase the valid model-supported centerline.
            mask = cv2.bitwise_and(mask, candidate_gate)

        open_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.config.open_kernel, self.config.open_kernel),
        )
        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (self.config.close_kernel, self.config.close_kernel),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        line_close_size = max(
            3,
            int(round(self.config.line_close_kernel * width / 1280.0)),
        )
        if line_close_size % 2 == 0:
            line_close_size += 1
        line_close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (line_close_size, line_close_size),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, line_close_kernel)
        if candidate_gate is not None:
            # Closing may grow pixels outside the model lane. Keep the mixed
            # result a strict subset of YOLOP's road-gated line mask.
            mask = cv2.bitwise_and(mask, candidate_gate)
        mask = self._remove_small_components(mask)
        return self._keep_dominant_line(mask, width)

    def _model_mask(
        self, frame_bgr: np.ndarray, roi_mask: np.ndarray
    ) -> Tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
        result = self._segmenter.segment(frame_bgr)
        if not isinstance(result, SegmentationResult):
            raise TypeError("segmenter.segment() must return SegmentationResult")
        mask = cv2.bitwise_and(result.line_mask, roi_mask)
        raw_line_mask = (
            cv2.bitwise_and(result.raw_line_mask, roi_mask)
            if result.raw_line_mask is not None
            else None
        )
        return self._remove_small_components(mask), result.road_mask, raw_line_mask

    def _remove_small_components(self, mask: np.ndarray) -> np.ndarray:
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        filtered = np.zeros_like(mask)
        for label in range(1, count):
            if stats[label, cv2.CC_STAT_AREA] >= self.config.min_component_area_px:
                filtered[labels == label] = 255
        return filtered

    @staticmethod
    def _component_elongation(labels: np.ndarray, label: int) -> float:
        """Return the PCA major/minor axis ratio for one component."""

        points_yx = np.column_stack(np.nonzero(labels == label)).astype(np.float32)
        if len(points_yx) < 3:
            return 0.0
        centered = points_yx - np.mean(points_yx, axis=0, keepdims=True)
        covariance = centered.T @ centered / float(len(points_yx) - 1)
        eigenvalues = np.linalg.eigvalsh(covariance)
        minor = max(float(eigenvalues[0]), 1.0e-6)
        major = max(float(eigenvalues[-1]), minor)
        return float(np.sqrt(major / minor))

    def _keep_dominant_line(self, mask: np.ndarray, width: int) -> np.ndarray:
        """Keep the dominant line-shaped warm-color component in the ROI.

        Color-only segmentation also finds warm pixels on grass, curbs and
        indoor objects. The guide line is expected to be a long, narrow
        component. In addition to area and span, the optional PCA feature
        gate rejects compact blobs using their major/minor axis ratio.
        """

        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask, connectivity=8
        )
        if count <= 1:
            return np.zeros_like(mask)

        scale = max(width / 1280.0, 0.5)
        minimum_area = max(1, int(round(self.config.line_min_area_px * scale * scale)))
        minimum_span = max(1, int(round(self.config.line_min_span_px * scale)))
        best_label = 0
        best_area = 0
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            span = max(
                int(stats[label, cv2.CC_STAT_WIDTH]),
                int(stats[label, cv2.CC_STAT_HEIGHT]),
            )
            if area < minimum_area or span < minimum_span:
                continue
            if self.config.line_feature_enabled:
                elongation = self._component_elongation(labels, label)
                if elongation < self.config.line_min_elongation:
                    continue
            if area > best_area:
                best_label = label
                best_area = area

        if best_label == 0:
            return np.zeros_like(mask)
        return np.where(labels == best_label, 255, 0).astype(np.uint8)

    def _birdseye(self, mask: np.ndarray) -> np.ndarray:
        height, width = mask.shape
        source = self._normalized_points(
            self.config.perspective_source,
            width,
            height,
        )
        destination = np.asarray(
            [
                [0, self.config.birdseye_height - 1],
                [self.config.birdseye_width - 1, self.config.birdseye_height - 1],
                [self.config.birdseye_width - 1, 0],
                [0, 0],
            ],
            dtype=np.float32,
        )
        transform = cv2.getPerspectiveTransform(source, destination)
        return cv2.warpPerspective(
            mask,
            transform,
            (self.config.birdseye_width, self.config.birdseye_height),
            flags=cv2.INTER_NEAREST,
        )

    def _path_points(self, birdseye_mask: np.ndarray) -> np.ndarray:
        height, width = birdseye_mask.shape
        band_half = max(1, self.config.sample_band_height_px // 2)
        rows = np.linspace(height - band_half - 1, band_half, self.config.sample_rows)
        points = []

        for row in rows:
            center_row = int(round(row))
            y0 = max(0, center_row - band_half)
            y1 = min(height, center_row + band_half + 1)
            ys, xs = np.nonzero(birdseye_mask[y0:y1])
            if xs.size < self.config.min_pixels_per_band:
                continue

            pixel_x = float(np.median(xs))
            forward_fraction = 1.0 - (center_row / max(height - 1, 1))
            forward_x = self.config.near_distance_m + forward_fraction * (
                self.config.far_distance_m - self.config.near_distance_m
            )
            lateral_y = (0.5 - pixel_x / max(width - 1, 1)) * (
                2.0 * self.config.half_width_m
            )
            points.append((forward_x, lateral_y))

        return np.asarray(points, dtype=np.float64).reshape(-1, 2)

    def _fit_path(self, birdseye_mask: np.ndarray) -> Optional[PathEstimate]:
        points = self._path_points(birdseye_mask)
        minimum_points = self.config.polynomial_degree + 1
        if len(points) < minimum_points:
            self._previous_near_lateral = None
            return None

        coefficients = np.polyfit(
            points[:, 0],
            points[:, 1],
            deg=self.config.polynomial_degree,
        )
        predicted = np.polyval(coefficients, points[:, 0])
        residual = float(np.sqrt(np.mean((predicted - points[:, 1]) ** 2)))

        row_confidence = min(1.0, len(points) / self.config.sample_rows)
        mask_ratio = float(np.count_nonzero(birdseye_mask)) / birdseye_mask.size
        area_confidence = min(1.0, mask_ratio / self.config.target_mask_ratio)
        residual_confidence = float(
            np.exp(-residual / max(self.config.max_fit_residual_m, 1e-6))
        )

        near_lateral = float(np.polyval(coefficients, self.config.near_distance_m))
        if self._previous_near_lateral is None:
            temporal_confidence = 1.0
        else:
            temporal_confidence = float(
                np.exp(
                    -abs(near_lateral - self._previous_near_lateral)
                    / max(self.config.max_fit_residual_m, 1e-6)
                )
            )
        self._previous_near_lateral = near_lateral

        confidence = float(
            np.clip(
                0.40 * row_confidence
                + 0.20 * area_confidence
                + 0.25 * residual_confidence
                + 0.15 * temporal_confidence,
                0.0,
                1.0,
            )
        )
        return PathEstimate(coefficients, confidence, residual, points)

    def _project_centerline_to_image(
        self,
        estimate: Optional[PathEstimate],
        image_shape: Tuple[int, int],
        sample_count: int = 32,
    ) -> np.ndarray:
        """Project the fitted metric centerline back into camera pixels."""

        if estimate is None:
            return np.empty((0, 2), dtype=np.float32)

        forward = np.linspace(
            self.config.near_distance_m,
            self.config.far_distance_m,
            max(2, sample_count),
            dtype=np.float32,
        )
        lateral = np.asarray(
            np.polyval(estimate.coefficients, forward), dtype=np.float32
        )
        birdseye_points = np.column_stack(
            [
                (0.5 - lateral / (2.0 * self.config.half_width_m))
                * (self.config.birdseye_width - 1),
                (1.0 - (forward - self.config.near_distance_m)
                 / (self.config.far_distance_m - self.config.near_distance_m))
                * (self.config.birdseye_height - 1),
            ]
        ).astype(np.float32)

        source = self._normalized_points(
            self.config.perspective_source,
            image_shape[1],
            image_shape[0],
        )
        destination = np.asarray(
            [
                [0, self.config.birdseye_height - 1],
                [self.config.birdseye_width - 1, self.config.birdseye_height - 1],
                [self.config.birdseye_width - 1, 0],
                [0, 0],
            ],
            dtype=np.float32,
        )
        inverse = cv2.getPerspectiveTransform(destination, source)
        return cv2.perspectiveTransform(birdseye_points[None, ...], inverse)[0]

    def process(
        self,
        frame_bgr: np.ndarray,
        *,
        candidate_gate_mask: Optional[np.ndarray] = None,
    ) -> VisionResult:
        if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("frame_bgr must be an HxWx3 image")
        height, width = frame_bgr.shape[:2]
        roi_polygon = self._normalized_points(
            self.config.roi_polygon,
            width,
            height,
        ).astype(np.int32)
        line_polygon = self._normalized_points(
            self.config.line_roi_polygon,
            width,
            height,
        ).astype(np.int32)
        roi_mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(roi_mask, [roi_polygon], 255)

        if self._segmenter is None:
            # Kept as an explicit development fallback so the node can still
            # start before a field-trained ONNX model is mounted.  Production
            # deployments should set segmentation_model_path to YOLOP ONNX.
            mask = self._color_mask(
                frame_bgr,
                roi_mask,
                candidate_gate_mask=candidate_gate_mask,
            )
            road_mask = None
            raw_line_mask = None
        else:
            mask, road_mask, raw_line_mask = self._model_mask(frame_bgr, roi_mask)
        birdseye_mask = self._birdseye(mask)
        estimate = self._fit_path(birdseye_mask)
        centerline_points_px = self._project_centerline_to_image(
            estimate, (height, width)
        )
        return VisionResult(
            mask=mask,
            birdseye_mask=birdseye_mask,
            estimate=estimate,
            roi_polygon_px=roi_polygon,
            line_roi_polygon_px=line_polygon,
            road_mask=road_mask,
            raw_line_mask=raw_line_mask,
            centerline_points_px=centerline_points_px,
        )
