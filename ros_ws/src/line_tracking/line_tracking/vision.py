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
    line_first_canny_low: int = 40
    line_first_canny_high: int = 120
    line_first_hough_threshold: int = 24
    line_first_min_length_px: int = 35
    line_first_max_gap_px: int = 24
    line_first_corridor_width_px: int = 31
    line_first_recovery_width_px: int = 81
    line_first_band_close_kernel_px: int = 61
    road_line_canny_low: int = 40
    road_line_canny_high: int = 120
    road_line_hough_threshold: int = 35
    road_line_min_length_px: int = 45
    road_line_max_gap_px: int = 18
    road_line_corridor_width_px: int = 7
    # The camera applies a strong color cast, so gray-road segmentation uses
    # the road's adaptive LAB chroma rather than a fixed HSV gray threshold.
    gray_road_lab_tolerance: float = 24.0
    gray_road_min_luminance: int = 35
    gray_road_max_luminance: int = 230
    # The far end of this fisheye view contains shoulder/grass with a similar
    # color to the pavement. Keep the road estimate conservative for tracking
    # by starting it below the uncertain horizon band.
    gray_road_top_y: float = 0.58
    gray_road_open_kernel: int = 3
    gray_road_close_kernel: int = 21
    gray_road_min_component_area_px: int = 1000
    # Dedicated OpenCV lane-line backend. The standard HSV range handles
    # balanced cameras; the wrap-around range and signed channel differences
    # handle this camera's peach/magenta rendering of yellow paint.
    lane_yellow_hsv_lower: Tuple[int, int, int] = (15, 70, 80)
    lane_yellow_hsv_upper: Tuple[int, int, int] = (42, 255, 255)
    lane_cast_yellow_hue_margin: int = 12
    lane_cast_yellow_saturation_min: int = 45
    lane_cast_yellow_value_min: int = 120
    lane_yellow_red_blue_min: int = 20
    lane_yellow_red_green_min: int = 25
    # The supplied camera renders asphalt as bright, low-saturation purple.
    # A stricter white gate prevents that road texture from becoming a white
    # lane candidate; other cameras can widen it through the CLI options.
    lane_white_saturation_max: int = 40
    lane_white_value_min: int = 200
    lane_color_close_kernel: int = 5
    lane_canny_low: int = 40
    lane_canny_high: int = 120
    lane_hough_threshold: int = 30
    lane_min_length_px: int = 60
    lane_max_gap_px: int = 20
    lane_draw_width_px: int = 5
    lane_min_vertical_ratio: float = 0.15
    lane_min_color_support_ratio: float = 0.60
    # Advanced-Lane-Lines style test backend. Color thresholding is shared
    # with ``detect_lane_lines``; these parameters control bird's-eye sliding
    # windows and the quadratic left/right lane fit used for road vehicles.
    advanced_lane_windows: int = 9
    advanced_lane_margin_px: int = 45
    advanced_lane_min_pixels: int = 25
    advanced_lane_min_points: int = 120
    advanced_lane_min_width_ratio: float = 0.35
    advanced_lane_max_width_ratio: float = 1.05
    advanced_lane_max_width_std_ratio: float = 0.20
    advanced_lane_width_m: float = 3.70
    advanced_lane_visible_distance_m: float = 30.0
    advanced_lane_smoothing_alpha: float = 0.35
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
        if not 0 <= self.line_first_canny_low < self.line_first_canny_high <= 255:
            raise ValueError("line-first Canny thresholds must satisfy 0 <= low < high <= 255")
        if (
            self.line_first_hough_threshold <= 0
            or self.line_first_min_length_px <= 0
            or self.line_first_max_gap_px < 0
            or self.line_first_corridor_width_px <= 0
            or self.line_first_recovery_width_px <= 0
            or self.line_first_band_close_kernel_px <= 0
        ):
            raise ValueError("line-first Hough parameters must be positive")
        if not 0 <= self.road_line_canny_low < self.road_line_canny_high <= 255:
            raise ValueError("road-line Canny thresholds must satisfy 0 <= low < high <= 255")
        if (
            self.road_line_hough_threshold <= 0
            or self.road_line_min_length_px <= 0
            or self.road_line_max_gap_px < 0
            or self.road_line_corridor_width_px <= 0
        ):
            raise ValueError("road-line Hough parameters must be positive")
        if self.gray_road_lab_tolerance <= 0.0:
            raise ValueError("gray_road_lab_tolerance must be positive")
        if not 0 <= self.gray_road_min_luminance < self.gray_road_max_luminance <= 255:
            raise ValueError(
                "gray-road luminance limits must satisfy 0 <= min < max <= 255"
            )
        if not 0.0 < self.gray_road_top_y < 1.0:
            raise ValueError("gray_road_top_y must be in (0, 1)")
        if (
            self.gray_road_open_kernel <= 0
            or self.gray_road_open_kernel % 2 == 0
            or self.gray_road_close_kernel <= 0
            or self.gray_road_close_kernel % 2 == 0
            or self.gray_road_min_component_area_px <= 0
        ):
            raise ValueError("gray-road morphology parameters must be positive odd values")
        if not all(
            0 <= lower <= upper <= limit
            for lower, upper, limit in zip(
                self.lane_yellow_hsv_lower,
                self.lane_yellow_hsv_upper,
                (179, 255, 255),
            )
        ):
            raise ValueError("lane yellow HSV limits are invalid")
        if not 0 <= self.lane_cast_yellow_hue_margin <= 89:
            raise ValueError("lane_cast_yellow_hue_margin must be in [0, 89]")
        if not 0 <= self.lane_cast_yellow_saturation_min <= 255:
            raise ValueError("lane_cast_yellow_saturation_min must be in [0, 255]")
        if not 0 <= self.lane_cast_yellow_value_min <= 255:
            raise ValueError("lane_cast_yellow_value_min must be in [0, 255]")
        if not -255 <= self.lane_yellow_red_blue_min <= 255:
            raise ValueError("lane_yellow_red_blue_min must be in [-255, 255]")
        if not -255 <= self.lane_yellow_red_green_min <= 255:
            raise ValueError("lane_yellow_red_green_min must be in [-255, 255]")
        if not 0 <= self.lane_white_saturation_max <= 255:
            raise ValueError("lane_white_saturation_max must be in [0, 255]")
        if not 0 <= self.lane_white_value_min <= 255:
            raise ValueError("lane_white_value_min must be in [0, 255]")
        if self.lane_color_close_kernel <= 0 or self.lane_color_close_kernel % 2 == 0:
            raise ValueError("lane_color_close_kernel must be a positive odd value")
        if not 0 <= self.lane_canny_low < self.lane_canny_high <= 255:
            raise ValueError("lane Canny thresholds must satisfy 0 <= low < high <= 255")
        if (
            self.lane_hough_threshold <= 0
            or self.lane_min_length_px <= 0
            or self.lane_max_gap_px < 0
            or self.lane_draw_width_px <= 0
        ):
            raise ValueError("lane Hough parameters must be positive")
        if not 0.0 <= self.lane_min_vertical_ratio <= 1.0:
            raise ValueError("lane_min_vertical_ratio must be in [0, 1]")
        if not 0.0 <= self.lane_min_color_support_ratio <= 1.0:
            raise ValueError("lane_min_color_support_ratio must be in [0, 1]")
        if (
            self.advanced_lane_windows <= 0
            or self.advanced_lane_margin_px <= 0
            or self.advanced_lane_min_pixels <= 0
            or self.advanced_lane_min_points <= 0
        ):
            raise ValueError("advanced-lane sliding-window parameters must be positive")
        if not (
            0.0
            < self.advanced_lane_min_width_ratio
            < self.advanced_lane_max_width_ratio
        ):
            raise ValueError("advanced-lane width ratios must be ordered and positive")
        if self.advanced_lane_max_width_std_ratio <= 0.0:
            raise ValueError("advanced_lane_max_width_std_ratio must be positive")
        if (
            self.advanced_lane_width_m <= 0.0
            or self.advanced_lane_visible_distance_m <= 0.0
        ):
            raise ValueError("advanced-lane metric dimensions must be positive")
        if not 0.0 < self.advanced_lane_smoothing_alpha <= 1.0:
            raise ValueError("advanced_lane_smoothing_alpha must be in (0, 1]")
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
    line_feature_mask: Optional[np.ndarray] = None
    centerline_points_px: np.ndarray = field(
        default_factory=lambda: np.empty((0, 2), dtype=np.float32)
    )


@dataclass(frozen=True)
class LaneLineResult:
    """Color-gated Hough lane segments in camera-image coordinates."""

    yellow_line_mask: np.ndarray
    white_line_mask: np.ndarray
    yellow_color_mask: np.ndarray
    white_color_mask: np.ndarray
    roi_polygon_px: np.ndarray


@dataclass(frozen=True)
class RoadLaneResult:
    """Left/right road-lane fit and its projection into the camera image."""

    binary_mask: np.ndarray
    birdseye_mask: np.ndarray
    left_curve_px: np.ndarray = field(
        default_factory=lambda: np.empty((0, 2), dtype=np.float32)
    )
    right_curve_px: np.ndarray = field(
        default_factory=lambda: np.empty((0, 2), dtype=np.float32)
    )
    centerline_points_px: np.ndarray = field(
        default_factory=lambda: np.empty((0, 2), dtype=np.float32)
    )
    lane_polygon_px: np.ndarray = field(
        default_factory=lambda: np.empty((0, 2), dtype=np.float32)
    )
    confidence: float = 0.0
    curvature_m: Optional[float] = None
    center_offset_m: Optional[float] = None

    @property
    def detected(self) -> bool:
        return len(self.left_curve_px) >= 2 and len(self.right_curve_px) >= 2


class YellowLineVision:
    """Segment a yellow guide line and fit its centerline in ground coordinates."""

    def __init__(
        self, config: VisionConfig, segmenter: Optional[object] = None
    ):
        config.validate()
        self.config = config
        self._segmenter = segmenter
        self._previous_near_lateral: Optional[float] = None
        self._previous_left_lane_fit: Optional[np.ndarray] = None
        self._previous_right_lane_fit: Optional[np.ndarray] = None

    @staticmethod
    def _normalized_points(
        values: Sequence[float], width: int, height: int
    ) -> np.ndarray:
        points = np.asarray(values, dtype=np.float32).reshape(4, 2)
        scale = np.asarray([max(width - 1, 1), max(height - 1, 1)], dtype=np.float32)
        return points * scale

    def _yellow_color_candidates(
        self,
        frame_bgr: np.ndarray,
        roi_mask: np.ndarray,
        candidate_gate_mask: Optional[np.ndarray] = None,
        restrict_to_candidate_gate: bool = True,
    ) -> np.ndarray:
        """Build a yellow mask, optionally selected by a line-first gate."""

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

        height, width = roi_mask.shape
        line_polygon = self._normalized_points(
            self.config.line_roi_polygon,
            width,
            height,
        ).astype(np.int32)
        line_roi_mask = np.zeros_like(roi_mask)
        cv2.fillPoly(line_roi_mask, [line_polygon], 255)
        color_gate = cv2.bitwise_and(roi_mask, line_roi_mask)
        selection_gate = color_gate
        if candidate_gate_mask is not None:
            if candidate_gate_mask.shape != roi_mask.shape:
                raise ValueError(
                    "candidate_gate_mask must match the input frame dimensions"
                )
            external_gate = np.where(candidate_gate_mask > 0, 255, 0).astype(
                np.uint8
            )
            selection_gate = cv2.bitwise_and(color_gate, external_gate)

        warm_lab_mask = cv2.bitwise_and(warm_mask, lab_mask)
        warm_lab_mask = cv2.bitwise_and(warm_lab_mask, color_gate)
        hsv_lab_mask = cv2.bitwise_and(hsv_mask, lab_mask)
        hsv_lab_mask = cv2.bitwise_and(hsv_lab_mask, color_gate)
        selected_warm_mask = cv2.bitwise_and(warm_lab_mask, selection_gate)
        if self.config.prefer_warm_camera_mask and np.any(selected_warm_mask):
            # Prefer the cast-aware mask whenever this frame contains a warm
            # candidate.  The broad HSV mask otherwise treats the magenta
            # road and indoor floor as yellow and can win over the real line.
            mask = warm_lab_mask
        else:
            # Keep a conventional HSV fallback for balanced-camera frames
            # and for synthetic/unit-test images with ordinary yellow paint.
            mask = hsv_lab_mask
        if candidate_gate_mask is not None and restrict_to_candidate_gate:
            mask = cv2.bitwise_and(mask, selection_gate)
        return mask


    def _finish_color_mask(
        self,
        mask: np.ndarray,
        candidate_gate_mask: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Clean color pixels and keep the dominant line-shaped component."""

        candidate_gate = None
        if candidate_gate_mask is not None:
            if candidate_gate_mask.shape != mask.shape:
                raise ValueError(
                    "candidate_gate_mask must match the input frame dimensions"
                )
            candidate_gate = np.where(candidate_gate_mask > 0, 255, 0).astype(
                np.uint8
            )
            # This can be YOLOP's road-gated lane mask or a Hough corridor.
            # Apply it before component selection so an unrelated warm
            # component cannot win and erase the supported centerline.
            mask = cv2.bitwise_and(mask, candidate_gate)

        _, width = mask.shape
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

    def _color_mask(
        self,
        frame_bgr: np.ndarray,
        roi_mask: np.ndarray,
        candidate_gate_mask: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        candidates = self._yellow_color_candidates(
            frame_bgr,
            roi_mask,
            candidate_gate_mask=candidate_gate_mask,
        )
        return self._finish_color_mask(
            candidates,
            candidate_gate_mask=candidate_gate_mask,
        )

    def _line_feature_mask(
        self,
        frame_bgr: np.ndarray,
        roi_mask: np.ndarray,
    ) -> np.ndarray:
        """Detect geometric line corridors before considering their color."""

        height, width = roi_mask.shape
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(
            clipLimit=self.config.clahe_clip_limit,
            tileGridSize=(self.config.clahe_grid_size, self.config.clahe_grid_size),
        )
        enhanced = clahe.apply(gray)
        blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)
        edges = cv2.Canny(
            blurred,
            self.config.line_first_canny_low,
            self.config.line_first_canny_high,
        )

        line_polygon = self._normalized_points(
            self.config.line_roi_polygon,
            width,
            height,
        ).astype(np.int32)
        line_roi_mask = np.zeros_like(roi_mask)
        cv2.fillPoly(line_roi_mask, [line_polygon], 255)
        edges = cv2.bitwise_and(edges, roi_mask)
        edges = cv2.bitwise_and(edges, line_roi_mask)

        scale = max(width / 1280.0, 0.5)
        segments = cv2.HoughLinesP(
            edges,
            rho=1.0,
            theta=np.pi / 180.0,
            threshold=max(5, int(round(self.config.line_first_hough_threshold * scale))),
            minLineLength=max(8, int(round(self.config.line_first_min_length_px * scale))),
            maxLineGap=max(0, int(round(self.config.line_first_max_gap_px * scale))),
        )

        feature_mask = np.zeros_like(roi_mask)
        if segments is not None:
            corridor_width = max(
                3,
                int(round(self.config.line_first_corridor_width_px * scale)),
            )
            if corridor_width % 2 == 0:
                corridor_width += 1
            for segment in segments.reshape(-1, 4):
                x1, y1, x2, y2 = (int(value) for value in segment)
                cv2.line(
                    feature_mask,
                    (x1, y1),
                    (x2, y2),
                    255,
                    corridor_width,
                    cv2.LINE_AA,
                )

        feature_mask = cv2.bitwise_and(feature_mask, roi_mask)
        return cv2.bitwise_and(feature_mask, line_roi_mask)

    def detect_gray_road_mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Estimate a gray pavement mask inside the symmetric line ROI.

        The supplied camera has a magenta cast, so the road is not close to
        zero HSV saturation. Instead, use the median LAB ``a/b`` chroma in
        the lower, center part of the ROI as an adaptive road-color reference.
        This keeps the method independent of YOLOP and does not classify the
        line by yellow color.
        """

        if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("frame_bgr must be an HxWx3 image")
        height, width = frame_bgr.shape[:2]
        polygon = self._normalized_points(
            self.config.line_roi_polygon,
            width,
            height,
        ).astype(np.int32)
        # Do not classify the uncertain far-horizon band as pavement. Crop the
        # original ROI at the requested y instead of rebuilding its sides:
        # this changes only the height and preserves the original left/right
        # ROI geometry and width.
        road_polygon = polygon.copy()
        top_y_px = int(round(height * self.config.gray_road_top_y))
        original_top_y_px = int(round(float(polygon[2, 1])))
        if top_y_px > original_top_y_px:
            original_bottom_y_px = int(round(float(polygon[0, 1])))
            denominator = max(original_bottom_y_px - original_top_y_px, 1)
            side_fraction = (top_y_px - original_top_y_px) / denominator
            right_x = polygon[2, 0] + side_fraction * (
                polygon[1, 0] - polygon[2, 0]
            )
            left_x = polygon[3, 0] + side_fraction * (
                polygon[0, 0] - polygon[3, 0]
            )
            road_polygon[2] = (int(round(right_x)), top_y_px)
            road_polygon[3] = (int(round(left_x)), top_y_px)
        roi_mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(roi_mask, [road_polygon], 255)

        lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
        luminance = lab[:, :, 0]
        chroma = lab[:, :, 1:3].astype(np.float32)
        yy, xx = np.indices((height, width))
        reference_region = (
            (roi_mask > 0)
            & (yy >= int(round(height * 0.68)))
            & (xx >= int(round(width * 0.25)))
            & (xx <= int(round(width * 0.75)))
        )
        reference_pixels = chroma[reference_region]
        if reference_pixels.size == 0:
            return np.zeros((height, width), dtype=np.uint8)
        reference = np.median(reference_pixels, axis=0)
        chroma_distance = np.linalg.norm(chroma - reference, axis=2)
        candidate = (
            (roi_mask > 0)
            & (luminance >= self.config.gray_road_min_luminance)
            & (luminance <= self.config.gray_road_max_luminance)
            & (chroma_distance <= self.config.gray_road_lab_tolerance)
        ).astype(np.uint8) * 255

        open_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.config.gray_road_open_kernel, self.config.gray_road_open_kernel),
        )
        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.config.gray_road_close_kernel, self.config.gray_road_close_kernel),
        )
        candidate = cv2.morphologyEx(candidate, cv2.MORPH_OPEN, open_kernel)
        candidate = cv2.morphologyEx(candidate, cv2.MORPH_CLOSE, close_kernel)

        # The pavement should touch the bottom of this camera ROI. Retain all
        # sufficiently large bottom-connected pieces so a painted line or a
        # small illumination change cannot split the two road sides away.
        anchor_start = int(round(height * 0.90))
        labels_count, labels, stats, _ = cv2.connectedComponentsWithStats(
            candidate,
            connectivity=8,
        )
        anchor_labels = set(np.unique(labels[anchor_start:])) - {0}
        filtered = np.zeros_like(candidate)
        for label in anchor_labels:
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area >= self.config.gray_road_min_component_area_px:
                filtered[labels == label] = 255
        if np.any(filtered):
            return filtered
        return candidate

    def detect_lines_in_mask(
        self,
        frame_bgr: np.ndarray,
        mask: np.ndarray,
    ) -> np.ndarray:
        """Detect geometric line candidates only inside a supplied mask.

        This deliberately does not inspect HSV/LAB or any other color cue.
        The output means only that an OpenCV Canny/Hough line was found inside
        the supplied road-area mask.
        """

        if mask.shape != frame_bgr.shape[:2]:
            raise ValueError("mask must match the input frame dimensions")
        height, width = mask.shape
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(
            clipLimit=self.config.clahe_clip_limit,
            tileGridSize=(self.config.clahe_grid_size, self.config.clahe_grid_size),
        )
        enhanced = clahe.apply(gray)
        blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)
        edges = cv2.Canny(
            blurred,
            self.config.road_line_canny_low,
            self.config.road_line_canny_high,
        )
        line_gate = np.where(mask > 0, 255, 0).astype(np.uint8)
        edges = cv2.bitwise_and(edges, line_gate)

        scale = max(width / 1280.0, 0.5)
        segments = cv2.HoughLinesP(
            edges,
            rho=1.0,
            theta=np.pi / 180.0,
            threshold=max(
                5,
                int(round(self.config.road_line_hough_threshold * scale)),
            ),
            minLineLength=max(
                8,
                int(round(self.config.road_line_min_length_px * scale)),
            ),
            maxLineGap=max(
                0,
                int(round(self.config.road_line_max_gap_px * scale)),
            ),
        )
        line_mask = np.zeros_like(line_gate)
        if segments is not None:
            line_width = max(
                1,
                int(round(self.config.road_line_corridor_width_px * scale)),
            )
            for segment in segments.reshape(-1, 4):
                x1, y1, x2, y2 = (int(value) for value in segment)
                cv2.line(
                    line_mask,
                    (x1, y1),
                    (x2, y2),
                    255,
                    line_width,
                    cv2.LINE_AA,
                )
        return cv2.bitwise_and(line_mask, line_gate)

    def detect_lines_in_road_mask(
        self,
        frame_bgr: np.ndarray,
        road_mask: np.ndarray,
    ) -> np.ndarray:
        """Backward-compatible YOLOP-road wrapper around mask-gated Hough."""

        return self.detect_lines_in_mask(frame_bgr, road_mask)

    def _lane_color_masks(
        self,
        frame_bgr: np.ndarray,
        roi_mask: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Return strict yellow and white lane-paint candidates in the ROI."""

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        hue, saturation, value = cv2.split(hsv)
        standard_yellow = cv2.inRange(
            hsv,
            np.asarray(self.config.lane_yellow_hsv_lower, dtype=np.uint8),
            np.asarray(self.config.lane_yellow_hsv_upper, dtype=np.uint8),
        )

        # Yellow paint in the supplied video is peach-colored and straddles
        # OpenCV hue 0/179. Signed channel differences keep purple asphalt and
        # low-luminance vegetation out of this camera-specific branch.
        hue_margin = self.config.lane_cast_yellow_hue_margin
        wraparound_hue = (hue <= hue_margin) | (hue >= 180 - hue_margin)
        frame_signed = frame_bgr.astype(np.int16)
        blue, green, red = cv2.split(frame_signed)
        cast_yellow = (
            wraparound_hue
            & (saturation >= self.config.lane_cast_yellow_saturation_min)
            & (value >= self.config.lane_cast_yellow_value_min)
            & (red - blue >= self.config.lane_yellow_red_blue_min)
            & (red - green >= self.config.lane_yellow_red_green_min)
        ).astype(np.uint8) * 255
        yellow_mask = cv2.bitwise_or(standard_yellow, cast_yellow)

        white_mask = (
            (saturation <= self.config.lane_white_saturation_max)
            & (value >= self.config.lane_white_value_min)
        ).astype(np.uint8) * 255
        yellow_mask = cv2.bitwise_and(yellow_mask, roi_mask)
        white_mask = cv2.bitwise_and(white_mask, roi_mask)

        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (
                self.config.lane_color_close_kernel,
                self.config.lane_color_close_kernel,
            ),
        )
        yellow_mask = cv2.morphologyEx(
            yellow_mask,
            cv2.MORPH_CLOSE,
            close_kernel,
        )
        white_mask = cv2.morphologyEx(
            white_mask,
            cv2.MORPH_CLOSE,
            close_kernel,
        )
        return yellow_mask, white_mask

    def _hough_lane_segments(
        self,
        edges: np.ndarray,
        color_mask: np.ndarray,
    ) -> np.ndarray:
        """Find thin, color-supported, forward-oriented Hough line segments."""

        height, width = color_mask.shape
        scale = max(width / 1280.0, 0.5)
        gate_size = max(3, int(round(5 * scale)))
        if gate_size % 2 == 0:
            gate_size += 1
        color_gate = cv2.dilate(
            color_mask,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (gate_size, gate_size)),
        )
        color_edges = cv2.bitwise_and(edges, color_gate)
        segments = cv2.HoughLinesP(
            color_edges,
            rho=1.0,
            theta=np.pi / 180.0,
            threshold=max(5, int(round(self.config.lane_hough_threshold * scale))),
            minLineLength=max(8, int(round(self.config.lane_min_length_px * scale))),
            maxLineGap=max(0, int(round(self.config.lane_max_gap_px * scale))),
        )

        line_mask = np.zeros((height, width), dtype=np.uint8)
        if segments is None:
            return line_mask
        draw_width = max(1, int(round(self.config.lane_draw_width_px * scale)))
        for segment in segments.reshape(-1, 4):
            x1, y1, x2, y2 = (int(value) for value in segment)
            length = float(np.hypot(x2 - x1, y2 - y1))
            if length <= 0.0:
                continue
            vertical_ratio = abs(y2 - y1) / length
            if vertical_ratio < self.config.lane_min_vertical_ratio:
                continue

            sample_count = max(2, int(np.ceil(length / 4.0)))
            sample_x = np.rint(np.linspace(x1, x2, sample_count)).astype(np.int32)
            sample_y = np.rint(np.linspace(y1, y2, sample_count)).astype(np.int32)
            sample_x = np.clip(sample_x, 0, width - 1)
            sample_y = np.clip(sample_y, 0, height - 1)
            support_ratio = float(np.mean(color_gate[sample_y, sample_x] > 0))
            if support_ratio < self.config.lane_min_color_support_ratio:
                continue
            cv2.line(
                line_mask,
                (x1, y1),
                (x2, y2),
                255,
                draw_width,
                cv2.LINE_AA,
            )
        return line_mask

    def detect_lane_lines(
        self,
        frame_bgr: np.ndarray,
        road_mask: Optional[np.ndarray] = None,
    ) -> LaneLineResult:
        """Detect color-supported lane segments, optionally inside a road mask."""

        if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("frame_bgr must be an HxWx3 image")
        height, width = frame_bgr.shape[:2]
        roi_polygon = self._normalized_points(
            self.config.line_roi_polygon,
            width,
            height,
        ).astype(np.int32)
        roi_mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(roi_mask, [roi_polygon], 255)
        if road_mask is not None:
            if road_mask.shape != (height, width):
                raise ValueError("road_mask must match the input frame dimensions")
            road_gate = np.where(road_mask > 0, 255, 0).astype(np.uint8)
            roi_mask = cv2.bitwise_and(roi_mask, road_gate)
        yellow_color_mask, white_color_mask = self._lane_color_masks(
            frame_bgr,
            roi_mask,
        )

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(
            blurred,
            self.config.lane_canny_low,
            self.config.lane_canny_high,
        )
        edges = cv2.bitwise_and(edges, roi_mask)
        yellow_line_mask = self._hough_lane_segments(edges, yellow_color_mask)
        white_line_mask = self._hough_lane_segments(edges, white_color_mask)
        return LaneLineResult(
            yellow_line_mask=yellow_line_mask,
            white_line_mask=white_line_mask,
            yellow_color_mask=yellow_color_mask,
            white_color_mask=white_color_mask,
            roi_polygon_px=roi_polygon,
        )

    def _perspective_transforms(
        self,
        image_shape: Tuple[int, int],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Return camera-to-bird's-eye and inverse perspective matrices."""

        height, width = image_shape
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
        return (
            cv2.getPerspectiveTransform(source, destination),
            cv2.getPerspectiveTransform(destination, source),
        )

    def _sliding_window_lane_indices(
        self,
        birdseye_mask: np.ndarray,
        nonzero_x: np.ndarray,
        nonzero_y: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Collect left/right lane pixels using histogram-seeded windows."""

        height, width = birdseye_mask.shape
        histogram = np.sum(birdseye_mask[height // 2 :, :] > 0, axis=0)
        midpoint = width // 2
        if not np.any(histogram[:midpoint]) or not np.any(histogram[midpoint:]):
            empty = np.empty(0, dtype=np.int64)
            return empty, empty

        left_current = int(np.argmax(histogram[:midpoint]))
        right_current = int(np.argmax(histogram[midpoint:]) + midpoint)
        window_height = max(1, height // self.config.advanced_lane_windows)
        margin = self.config.advanced_lane_margin_px
        left_indices = []
        right_indices = []

        for window in range(self.config.advanced_lane_windows):
            y_high = height - window * window_height
            y_low = max(0, height - (window + 1) * window_height)
            within_y = (nonzero_y >= y_low) & (nonzero_y < y_high)
            left_window = np.flatnonzero(
                within_y
                & (nonzero_x >= left_current - margin)
                & (nonzero_x < left_current + margin)
            )
            right_window = np.flatnonzero(
                within_y
                & (nonzero_x >= right_current - margin)
                & (nonzero_x < right_current + margin)
            )
            left_indices.append(left_window)
            right_indices.append(right_window)
            if left_window.size >= self.config.advanced_lane_min_pixels:
                left_current = int(np.mean(nonzero_x[left_window]))
            if right_window.size >= self.config.advanced_lane_min_pixels:
                right_current = int(np.mean(nonzero_x[right_window]))

        return np.concatenate(left_indices), np.concatenate(right_indices)

    def _advanced_lane_pixel_indices(
        self,
        birdseye_mask: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Find lane pixels near the previous fit or with sliding windows."""

        nonzero_y, nonzero_x = np.nonzero(birdseye_mask)
        margin = self.config.advanced_lane_margin_px
        if (
            self._previous_left_lane_fit is not None
            and self._previous_right_lane_fit is not None
        ):
            left_center = np.polyval(self._previous_left_lane_fit, nonzero_y)
            right_center = np.polyval(self._previous_right_lane_fit, nonzero_y)
            left_indices = np.flatnonzero(np.abs(nonzero_x - left_center) <= margin)
            right_indices = np.flatnonzero(np.abs(nonzero_x - right_center) <= margin)
            if (
                left_indices.size >= self.config.advanced_lane_min_points
                and right_indices.size >= self.config.advanced_lane_min_points
            ):
                return nonzero_x, nonzero_y, left_indices, right_indices

        left_indices, right_indices = self._sliding_window_lane_indices(
            birdseye_mask,
            nonzero_x,
            nonzero_y,
        )
        return nonzero_x, nonzero_y, left_indices, right_indices

    def _lane_fit_is_valid(
        self,
        left_fit: np.ndarray,
        right_fit: np.ndarray,
        birdseye_shape: Tuple[int, int],
    ) -> Tuple[bool, np.ndarray, np.ndarray, float]:
        """Validate ordering, width and parallelism of a candidate lane pair."""

        height, width = birdseye_shape
        plot_y = np.linspace(0, height - 1, height, dtype=np.float64)
        left_x = np.polyval(left_fit, plot_y)
        right_x = np.polyval(right_fit, plot_y)
        lane_width = right_x - left_x
        median_width = float(np.median(lane_width))
        if median_width <= 0.0:
            return False, left_x, right_x, median_width
        minimum_width = self.config.advanced_lane_min_width_ratio * width
        maximum_width = self.config.advanced_lane_max_width_ratio * width
        width_std_ratio = float(np.std(lane_width)) / median_width
        valid = bool(
            np.all(np.isfinite(left_x))
            and np.all(np.isfinite(right_x))
            and np.all(lane_width >= minimum_width)
            and np.all(lane_width <= maximum_width)
            and width_std_ratio <= self.config.advanced_lane_max_width_std_ratio
        )
        return valid, left_x, right_x, median_width

    def _fit_advanced_lane_pair(
        self,
        birdseye_mask: np.ndarray,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]]:
        """Fit and temporally smooth quadratic x(y) curves for both lanes."""

        nonzero_x, nonzero_y, left_indices, right_indices = (
            self._advanced_lane_pixel_indices(birdseye_mask)
        )
        if (
            left_indices.size < self.config.advanced_lane_min_points
            or right_indices.size < self.config.advanced_lane_min_points
        ):
            self._previous_left_lane_fit = None
            self._previous_right_lane_fit = None
            return None

        left_fit = np.polyfit(
            nonzero_y[left_indices],
            nonzero_x[left_indices],
            2,
        )
        right_fit = np.polyfit(
            nonzero_y[right_indices],
            nonzero_x[right_indices],
            2,
        )
        valid, left_x, right_x, median_width = self._lane_fit_is_valid(
            left_fit,
            right_fit,
            birdseye_mask.shape,
        )
        if not valid:
            self._previous_left_lane_fit = None
            self._previous_right_lane_fit = None
            return None

        alpha = self.config.advanced_lane_smoothing_alpha
        if self._previous_left_lane_fit is not None:
            left_fit = alpha * left_fit + (1.0 - alpha) * self._previous_left_lane_fit
        if self._previous_right_lane_fit is not None:
            right_fit = (
                alpha * right_fit + (1.0 - alpha) * self._previous_right_lane_fit
            )
        valid, left_x, right_x, median_width = self._lane_fit_is_valid(
            left_fit,
            right_fit,
            birdseye_mask.shape,
        )
        if not valid:
            self._previous_left_lane_fit = None
            self._previous_right_lane_fit = None
            return None

        self._previous_left_lane_fit = left_fit
        self._previous_right_lane_fit = right_fit

        height = birdseye_mask.shape[0]
        left_coverage = float(np.ptp(nonzero_y[left_indices])) / max(height - 1, 1)
        right_coverage = float(np.ptp(nonzero_y[right_indices])) / max(height - 1, 1)
        coverage_confidence = min(1.0, left_coverage, right_coverage)
        point_confidence = min(
            1.0,
            min(left_indices.size, right_indices.size)
            / float(self.config.advanced_lane_min_points * 4),
        )
        width_std_ratio = float(np.std(right_x - left_x)) / median_width
        width_confidence = float(
            np.exp(
                -width_std_ratio
                / max(self.config.advanced_lane_max_width_std_ratio, 1.0e-6)
            )
        )
        confidence = float(
            np.clip(
                0.45 * coverage_confidence
                + 0.30 * point_confidence
                + 0.25 * width_confidence,
                0.0,
                1.0,
            )
        )
        return left_fit, right_fit, left_x, right_x, confidence

    def _advanced_lane_metrics(
        self,
        left_fit: np.ndarray,
        right_fit: np.ndarray,
        median_lane_width_px: float,
        birdseye_shape: Tuple[int, int],
    ) -> Tuple[Optional[float], float]:
        """Estimate mean curvature and signed vehicle offset in metres."""

        height, width = birdseye_shape
        metres_per_y = self.config.advanced_lane_visible_distance_m / max(height, 1)
        metres_per_x = self.config.advanced_lane_width_m / max(
            median_lane_width_px,
            1.0,
        )
        plot_y = np.linspace(0, height - 1, height, dtype=np.float64)
        y_metres = plot_y * metres_per_y
        curvatures = []
        for fit in (left_fit, right_fit):
            x_metres = np.polyval(fit, plot_y) * metres_per_x
            fit_metres = np.polyfit(y_metres, x_metres, 2)
            denominator = abs(2.0 * fit_metres[0])
            if denominator > 1.0e-9:
                y_eval = (height - 1) * metres_per_y
                curvature = (
                    (1.0 + (2.0 * fit_metres[0] * y_eval + fit_metres[1]) ** 2)
                    ** 1.5
                ) / denominator
                if np.isfinite(curvature):
                    curvatures.append(float(curvature))

        bottom_y = height - 1
        lane_center = 0.5 * (
            float(np.polyval(left_fit, bottom_y))
            + float(np.polyval(right_fit, bottom_y))
        )
        # Positive means that the camera/vehicle is right of the lane centre.
        center_offset = (0.5 * width - lane_center) * metres_per_x
        mean_curvature = float(np.mean(curvatures)) if curvatures else None
        return mean_curvature, float(center_offset)

    def detect_advanced_lanes(self, frame_bgr: np.ndarray) -> RoadLaneResult:
        """Detect a road vehicle's left/right lane pair and drivable corridor.

        This follows the Advanced-Lane-Lines/Colab flow while reusing this
        package's camera-aware yellow and white thresholds:
        threshold -> perspective warp -> sliding windows -> quadratic fit ->
        inverse perspective projection.
        """

        if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("frame_bgr must be an HxWx3 image")
        lane_candidates = self.detect_lane_lines(frame_bgr)
        binary_mask = cv2.bitwise_or(
            lane_candidates.yellow_color_mask,
            lane_candidates.white_color_mask,
        )
        transform, inverse = self._perspective_transforms(frame_bgr.shape[:2])
        birdseye_mask = cv2.warpPerspective(
            binary_mask,
            transform,
            (self.config.birdseye_width, self.config.birdseye_height),
            flags=cv2.INTER_NEAREST,
        )
        fitted = self._fit_advanced_lane_pair(birdseye_mask)
        if fitted is None:
            return RoadLaneResult(
                binary_mask=binary_mask,
                birdseye_mask=birdseye_mask,
            )

        left_fit, right_fit, left_x, right_x, confidence = fitted
        height = self.config.birdseye_height
        sample_y = np.linspace(0, height - 1, min(height, 96), dtype=np.float32)
        left_birdseye = np.column_stack(
            (np.polyval(left_fit, sample_y), sample_y)
        ).astype(np.float32)
        right_birdseye = np.column_stack(
            (np.polyval(right_fit, sample_y), sample_y)
        ).astype(np.float32)
        center_birdseye = np.column_stack(
            (
                0.5
                * (
                    np.polyval(left_fit, sample_y)
                    + np.polyval(right_fit, sample_y)
                ),
                sample_y,
            )
        ).astype(np.float32)

        def project(points: np.ndarray) -> np.ndarray:
            return cv2.perspectiveTransform(points[None, ...], inverse)[0]

        left_curve = project(left_birdseye)
        right_curve = project(right_birdseye)
        center_curve = project(center_birdseye)
        lane_polygon = np.concatenate((left_curve, right_curve[::-1]), axis=0)
        median_width = float(np.median(right_x - left_x))
        curvature_m, center_offset_m = self._advanced_lane_metrics(
            left_fit,
            right_fit,
            median_width,
            birdseye_mask.shape,
        )
        return RoadLaneResult(
            binary_mask=binary_mask,
            birdseye_mask=birdseye_mask,
            left_curve_px=left_curve,
            right_curve_px=right_curve,
            centerline_points_px=center_curve,
            lane_polygon_px=lane_polygon,
            confidence=confidence,
            curvature_m=curvature_m,
            center_offset_m=center_offset_m,
        )

    def _line_first_mask(
        self,
        frame_bgr: np.ndarray,
        roi_mask: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        feature_mask = self._line_feature_mask(frame_bgr, roi_mask)

        # Hough normally finds both edges of a thick painted stripe. Keeping
        # only a narrow corridor around those edges can therefore split one
        # centerline into two branches. Use the line/color intersection as a
        # seed, then recover the full nearby color region before selecting the
        # dominant component.
        seed_mask = self._yellow_color_candidates(
            frame_bgr,
            roi_mask,
            candidate_gate_mask=feature_mask,
        )
        full_color_mask = self._yellow_color_candidates(
            frame_bgr,
            roi_mask,
            candidate_gate_mask=feature_mask,
            restrict_to_candidate_gate=False,
        )

        _, width = roi_mask.shape
        scale = max(width / 1280.0, 0.5)
        recovery_width = max(
            3,
            int(round(self.config.line_first_recovery_width_px * scale)),
        )
        if recovery_width % 2 == 0:
            recovery_width += 1
        recovery_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (recovery_width, recovery_width),
        )
        recovery_region = cv2.dilate(seed_mask, recovery_kernel)
        recovered_mask = cv2.bitwise_and(full_color_mask, recovery_region)

        band_close_size = max(
            3,
            int(round(self.config.line_first_band_close_kernel_px * scale)),
        )
        if band_close_size % 2 == 0:
            band_close_size += 1
        band_close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (band_close_size, band_close_size),
        )
        recovered_mask = cv2.morphologyEx(
            recovered_mask,
            cv2.MORPH_CLOSE,
            band_close_kernel,
        )
        final_mask = self._finish_color_mask(recovered_mask)
        return final_mask, feature_mask

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
        transform, _ = self._perspective_transforms(mask.shape)
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

        _, inverse = self._perspective_transforms(image_shape)
        return cv2.perspectiveTransform(birdseye_points[None, ...], inverse)[0]

    def process(
        self,
        frame_bgr: np.ndarray,
        *,
        candidate_gate_mask: Optional[np.ndarray] = None,
        line_first: bool = False,
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

        line_feature_mask = None
        if self._segmenter is None:
            # Kept as an explicit development fallback so the node can still
            # start before a field-trained ONNX model is mounted.  Production
            # deployments should set segmentation_model_path to YOLOP ONNX.
            if line_first:
                if candidate_gate_mask is not None:
                    raise ValueError(
                        "line_first and candidate_gate_mask cannot be used together"
                    )
                mask, line_feature_mask = self._line_first_mask(frame_bgr, roi_mask)
            else:
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
            line_feature_mask=line_feature_mask,
            centerline_points_px=centerline_points_px,
        )
