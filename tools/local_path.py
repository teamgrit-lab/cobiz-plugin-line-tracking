"""Surface-center local path extraction and smoothing helpers.

The Swin-L runtime produces a Mapillary surface label map.  This module turns
the selected road/sidewalk regions into a short path in the robot convention
``x=forward, y=left``.  It deliberately keeps the geometry explicit and
configurable because the rosbag contains camera intrinsics but no camera-to-
base extrinsic calibration.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from functools import lru_cache
import math
import threading
from typing import TYPE_CHECKING, Iterable

import cv2
import numpy as np
from scipy.optimize import LinearConstraint, minimize

if TYPE_CHECKING:
    from branch_path import BranchObservation


PATH_MASK_CLASSES = {0: "ROAD_OR_SIDEWALK", 1: "ROAD", 2: "SIDEWALK"}


def selected_path_region(selected_mask: np.ndarray, path_mask_class: int) -> np.ndarray:
    """Select a surface or their union; semantic background stays excluded."""

    if path_mask_class not in PATH_MASK_CLASSES:
        raise ValueError(
            "SWIN_L_PATH_MASK_CLASS must be 0 (road or sidewalk), "
            "1 (road), or 2 (sidewalk)"
        )
    if path_mask_class == 0:
        return (selected_mask == 1) | (selected_mask == 2)
    return selected_mask == path_mask_class


# Preserve the 84% bottom and 24% top widths; cover the lower 55% of the image.
DEFAULT_ROI_POLYGON = (
    0.08,
    1.00,
    0.92,
    1.00,
    0.62,
    0.45,
    0.38,
    0.45,
)

# Wide rectangular road selection retains visible road boundaries.
# Keep this separate from the four-point pixel-to-ground calibration above.
DEFAULT_ROAD_ROI_POLYGON = (0.02, 1.00, 0.98, 1.00, 0.98, 0.47, 0.02, 0.47)


@dataclass(frozen=True)
class LocalPathConfig:
    """Camera-to-ground and local-path settings.

    ``roi_polygon`` is ordered bottom-left, bottom-right, top-right, top-left.
    The destination rectangle is interpreted in meters in the robot frame.
    ``ground_half_width_m`` must be calibrated together with the ROI; it is
    not inferred from CameraInfo because the bag does not contain camera pose.
    """

    near_distance_m: float = 3.0
    far_distance_m: float = 8.0
    ground_half_width_m: float = 4.0
    search_half_width_m: float = 3.5
    roi_polygon: tuple[float, ...] = DEFAULT_ROI_POLYGON
    path_points: int = 20
    bev_width_px: int = 280
    bev_height_px: int = 160
    min_valid_ratio: float = 0.35
    min_sidewalk_width_m: float = 0.12
    close_kernel_px: int = 5
    fit_degree: int = 2
    max_path_lateral_m: float = 3.5
    smoothing_time_constant_sec: float = 0.80
    max_lateral_update_m: float = 0.35
    path_hold_sec: float = 0.90
    path_duration_sec: float = 1.50
    unrestricted_path_mode: bool = False
    branch_preference: str = "center"
    branch_min_width_m: float = 0.60
    branch_margin_m: float = 0.10
    branch_confirm_frames: int = 2
    branch_hold_sec: float = 1.50
    sidewalk_vegetation_bias_m: float = 0.20
    sidewalk_vegetation_max_gap_m: float = 0.30
    sidewalk_vegetation_edge_margin_m: float = 0.45
    road_roi_polygon: tuple[float, ...] = DEFAULT_ROAD_ROI_POLYGON
    road_clearance_enabled: bool = True
    road_edge_margin_m: float = 0.60
    road_hazard_margin_m: float = 0.60
    road_hazard_min_confidence: float = 0.50

    def validate(self) -> None:
        if type(self.road_clearance_enabled) is not bool:
            raise ValueError("road_clearance_enabled must be boolean")
        for name in ("road_edge_margin_m", "road_hazard_margin_m"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        if not math.isfinite(self.road_hazard_min_confidence) or not 0 < self.road_hazard_min_confidence <= 1:
            raise ValueError("road_hazard_min_confidence must be in (0, 1]")
        if self.near_distance_m <= 0.0:
            raise ValueError("near_distance_m must be positive")
        if self.far_distance_m <= self.near_distance_m:
            raise ValueError("far_distance_m must be greater than near_distance_m")
        if self.ground_half_width_m <= 0.0 or self.search_half_width_m <= 0.0:
            raise ValueError("ground/search half widths must be positive")
        if self.search_half_width_m > self.ground_half_width_m:
            raise ValueError("search_half_width_m must not exceed ground_half_width_m")
        if len(self.roi_polygon) != 8:
            raise ValueError("roi_polygon must contain four normalized x/y points")
        if any(value < 0.0 or value > 1.0 for value in self.roi_polygon):
            raise ValueError("roi_polygon values must be normalized to [0, 1]")
        if (len(self.road_roi_polygon) < 6 or len(self.road_roi_polygon) % 2 or not all(
            math.isfinite(value) and 0 <= value <= 1 for value in self.road_roi_polygon
        )):
            raise ValueError("road_roi_polygon must contain at least three finite normalized x/y pairs")
        road_polygon = np.asarray(self.road_roi_polygon, np.float32).reshape(-1, 2)
        if not cv2.isContourConvex(road_polygon) or cv2.contourArea(road_polygon) <= 0:
            raise ValueError("road_roi_polygon must be a nonempty convex polygon")
        if self.path_points < 2:
            raise ValueError("path_points must be at least 2")
        if self.bev_width_px < 8 or self.bev_height_px < 8:
            raise ValueError("bird's-eye dimensions are too small")
        if not 0.0 < self.min_valid_ratio <= 1.0:
            raise ValueError("min_valid_ratio must be in (0, 1]")
        if self.min_sidewalk_width_m <= 0.0:
            raise ValueError("min_sidewalk_width_m must be positive")
        if self.close_kernel_px < 0 or (self.close_kernel_px > 0 and self.close_kernel_px % 2 == 0):
            raise ValueError("close_kernel_px must be zero or a positive odd number")
        if self.fit_degree not in (1, 2):
            raise ValueError("fit_degree must be 1 or 2")
        if self.max_path_lateral_m <= 0.0:
            raise ValueError("max_path_lateral_m must be positive")
        if self.smoothing_time_constant_sec <= 0.0:
            raise ValueError("smoothing_time_constant_sec must be positive")
        if self.max_lateral_update_m <= 0.0:
            raise ValueError("max_lateral_update_m must be positive")
        if self.path_hold_sec < 0.0 or self.path_duration_sec <= 0.0:
            raise ValueError("path hold/duration values are invalid")
        if self.branch_preference not in ("center", "left", "right"):
            raise ValueError("branch_preference must be center, left or right")
        if not all(math.isfinite(value) and value > 0 for value in (
            self.branch_min_width_m, self.branch_margin_m, self.branch_hold_sec,
        )) or 2 * self.branch_margin_m >= self.branch_min_width_m:
            raise ValueError("branch dimensions/hold must be positive; width must exceed twice the margin")
        if type(self.branch_confirm_frames) is not int or self.branch_confirm_frames < 2:
            raise ValueError("branch_confirm_frames must be an integer of at least 2")
        for name in ("sidewalk_vegetation_bias_m", "sidewalk_vegetation_max_gap_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        if (not math.isfinite(self.sidewalk_vegetation_edge_margin_m)
                or self.sidewalk_vegetation_edge_margin_m <= 0):
            raise ValueError("sidewalk_vegetation_edge_margin_m must be finite and positive")


@dataclass(frozen=True)
class LocalPathEstimate:
    """One raw path estimate from one segmentation result."""

    points_xy: np.ndarray
    confidence: float
    valid_ratio: float
    mean_sidewalk_width_m: float
    raw_points_xy: np.ndarray
    branch_observation: BranchObservation | None = None
    vegetation_bias_active: bool = False
    road_center_tracking: bool = False
    road_corridor: np.ndarray | None = None  # Rows: forward x, lower y, upper y.
    road_clearance: dict | None = None

    @property
    def is_valid(self) -> bool:
        return bool(self.points_xy.shape[0] >= 2 and self.confidence > 0.0)


@dataclass(frozen=True)
class SmoothedPath:
    """Path currently safe to publish for local planning/debugging."""

    points_xy: np.ndarray
    confidence: float
    age_sec: float
    source: str
    stop_reason: str | None = None
    branch_status: dict | None = None
    vegetation_bias_active: bool = False
    road_center_tracking: bool = False
    road_corridor: np.ndarray | None = None
    road_clearance: dict | None = None


@dataclass(frozen=True)
class RoadPathResult:
    """Separate an unobserved/failed fit from a corridor excluded by clearance."""

    estimate: LocalPathEstimate | None
    status: str
    reason: str
    clearance: dict
    allowed_mask: np.ndarray
    x_values: np.ndarray
    y_values: np.ndarray

    @property
    def blocked(self) -> bool:
        return self.status == "blocked"

    def corridor_for(self, points: np.ndarray) -> np.ndarray | None:
        """Validate a previous path against every row of the *current* mask."""
        if points is None or len(points) < 2 or not np.isfinite(points).all():
            return None
        xs, ys = self.x_values, self.y_values
        if points[0, 0] < xs[0] - 1e-6 or points[-1, 0] > xs[-1] + 1e-6:
            return None
        rows = np.flatnonzero((xs >= points[0, 0] - 1e-6) & (xs <= points[-1, 0] + 1e-6))
        if len(rows) < 2:
            return None
        values = np.interp(xs[rows], points[:, 0], points[:, 1])
        spacing = float(abs(ys[1] - ys[0]))
        # Vectorize the fresh-mask check: per-row scans are costly on Jetson
        # while other ROS/video processes compete for its CPU cores.
        columns = np.rint(np.interp(values, ys[::-1],
                                   np.arange(len(ys))[::-1])).astype(np.intp)
        allowed = self.allowed_mask[rows] > 0
        if (np.any(abs(ys[columns] - values) > spacing / 2 + 1e-5)
                or not np.all(allowed[np.arange(len(rows)), columns])):
            return None
        grid_columns = np.arange(len(ys))[None, :]
        left = np.max(np.where((grid_columns < columns[:, None]) & ~allowed,
                               grid_columns, -1), axis=1) + 1
        right = np.min(np.where((grid_columns > columns[:, None]) & ~allowed,
                                grid_columns, len(ys)), axis=1) - 1
        # Stay inside the pixel cell despite float32 boundary rounding.
        corridor = np.column_stack((xs[rows], ys[right] - spacing / 2 + 1e-4,
                                    ys[left] + spacing / 2 - 1e-4)).astype(np.float32)
        return corridor if road_path_within_corridor(points, corridor) else None


def normalized_polygon_pixels(
    polygon: Iterable[float], frame_shape: tuple[int, int]
) -> np.ndarray:
    """Convert normalized x/y polygon coordinates to OpenCV pixel points."""

    height, width = frame_shape[:2]
    values = np.asarray(tuple(float(value) for value in polygon), dtype=np.float32)
    if values.size < 6 or values.size % 2:
        raise ValueError("polygon must contain at least three x/y points")
    if not np.isfinite(values).all():
        raise ValueError("polygon points must be finite")
    points = values.reshape(-1, 2).copy()
    points[:, 0] *= max(width - 1, 1)
    points[:, 1] *= max(height - 1, 1)
    return np.rint(points).astype(np.int32)


def pixel_to_ground_homography(
    frame_shape: tuple[int, int], config: LocalPathConfig
) -> np.ndarray:
    """Return a homography that maps image pixels to ground ``(x, y)`` meters."""

    config.validate()
    source = normalized_polygon_pixels(config.roi_polygon, frame_shape).astype(
        np.float32
    )
    destination = np.asarray(
        (
            (config.near_distance_m, config.ground_half_width_m),
            (config.near_distance_m, -config.ground_half_width_m),
            (config.far_distance_m, -config.ground_half_width_m),
            (config.far_distance_m, config.ground_half_width_m),
        ),
        dtype=np.float32,
    )
    return cv2.getPerspectiveTransform(source, destination)


def ground_to_pixel(
    points_xy: np.ndarray, homography_pixel_to_ground: np.ndarray
) -> np.ndarray:
    """Project ground-frame points back into camera pixels."""

    points = np.asarray(points_xy, dtype=np.float32).reshape(-1, 1, 2)
    if not len(points):
        return np.empty((0, 2), dtype=np.float32)
    inverse = np.linalg.inv(homography_pixel_to_ground)
    projected = cv2.perspectiveTransform(points, inverse)
    return projected.reshape(-1, 2)


def _runs(values: np.ndarray) -> list[tuple[int, int]]:
    indices = np.flatnonzero(values)
    if indices.size == 0:
        return []
    breaks = np.flatnonzero(np.diff(indices) > 1)
    starts = np.r_[0, breaks + 1]
    ends = np.r_[breaks, indices.size - 1]
    return [
        (int(indices[start]), int(indices[end])) for start, end in zip(starts, ends)
    ]


@dataclass(frozen=True)
class _BirdseyeGeometry:
    map_x: np.ndarray
    map_y: np.ndarray
    x_values: np.ndarray
    y_values: np.ndarray
    close_kernel: np.ndarray | None


@lru_cache(maxsize=8)
def _birdseye_geometry(
    frame_shape: tuple[int, int], config: LocalPathConfig
) -> _BirdseyeGeometry:
    """Share immutable projection maps across frames and surface classes.

    The frozen config and image dimensions are the cache key, so calibration,
    ROI, BEV size or kernel changes cannot reuse an obsolete map. Bound the
    cache for offline tools that process several resolutions/configurations.
    """

    homography = pixel_to_ground_homography(frame_shape, config)
    x_values = np.linspace(
        config.near_distance_m,
        config.far_distance_m,
        config.bev_height_px,
        dtype=np.float32,
    )
    y_values = np.linspace(
        config.search_half_width_m,
        -config.search_half_width_m,
        config.bev_width_px,
        dtype=np.float32,
    )
    grid_x, grid_y = np.meshgrid(x_values, y_values, indexing="ij")
    ground_points = np.stack((grid_x, grid_y), axis=-1).reshape(-1, 2)
    image_points = ground_to_pixel(ground_points, homography).reshape(
        config.bev_height_px, config.bev_width_px, 2
    )
    map_x = np.ascontiguousarray(image_points[..., 0])
    map_y = np.ascontiguousarray(image_points[..., 1])
    kernel = (
        np.ones((config.close_kernel_px, config.close_kernel_px), dtype=np.uint8)
        if config.close_kernel_px
        else None
    )
    for array in (map_x, map_y, x_values, y_values, kernel):
        if array is not None:
            array.setflags(write=False)
    return _BirdseyeGeometry(map_x, map_y, x_values, y_values, kernel)


def _birdseye_sidewalk(
    sidewalk_mask: np.ndarray,
    config: LocalPathConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Warp each fresh mask using the shared metric projection geometry."""

    geometry = _birdseye_geometry(sidewalk_mask.shape[:2], config)
    birdseye = cv2.remap(
        np.where(sidewalk_mask > 0, 255, 0).astype(np.uint8),
        geometry.map_x,
        geometry.map_y,
        interpolation=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    if geometry.close_kernel is not None:
        birdseye = cv2.morphologyEx(
            birdseye, cv2.MORPH_CLOSE, geometry.close_kernel
        )
    return birdseye, geometry.x_values, geometry.y_values


@lru_cache(maxsize=8)
def _road_birdseye_geometry(frame_shape: tuple[int, int], config: LocalPathConfig):
    """Expand the selection grid without changing the camera calibration.

    The old +/-3.5m search crops the road before taking its midpoint. Project
    the wide selection ROI through the original homography to determine the
    complete visible search width, preserving its original spatial resolution.
    """
    homography = pixel_to_ground_homography(frame_shape, config)
    polygon = normalized_polygon_pixels(config.road_roi_polygon, frame_shape)
    projected = cv2.perspectiveTransform(polygon.astype(np.float32).reshape(-1, 1, 2), homography)
    if not np.isfinite(projected).all():
        raise ValueError("road ROI crosses an invalid ground projection")
    half_width = max(config.search_half_width_m, float(np.abs(projected[:, 0, 1]).max()))
    if half_width > 100:
        raise ValueError("road ROI is too close to the ground-projection horizon")
    width = max(config.bev_width_px, int(math.ceil(
        config.bev_width_px * half_width / config.search_half_width_m)))
    width = min(width, 4096)
    xs = np.linspace(config.near_distance_m, config.far_distance_m, config.bev_height_px, dtype=np.float32)
    ys = np.linspace(half_width, -half_width, width, dtype=np.float32)
    xx, yy = np.meshgrid(xs, ys, indexing="ij")
    pixels = ground_to_pixel(np.stack((xx, yy), axis=-1), homography).reshape(len(xs), len(ys), 2)
    roi = np.zeros(frame_shape[:2], np.uint8)
    cv2.fillConvexPoly(roi, polygon, 1)
    effective = replace(config, search_half_width_m=half_width,
                        ground_half_width_m=max(config.ground_half_width_m, half_width),
                        max_path_lateral_m=max(config.max_path_lateral_m, half_width),
                        bev_width_px=width, unrestricted_path_mode=False)
    return (np.ascontiguousarray(pixels[..., 0]), np.ascontiguousarray(pixels[..., 1]),
            roi, xs, ys, effective)


def _metric_margin_kernel(margin: float, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    rx = int(math.ceil(margin / abs(float(xs[1] - xs[0]))))
    ry = int(math.ceil(margin / abs(float(ys[1] - ys[0]))))
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * ry + 1, 2 * rx + 1))


def road_path_within_corridor(points: np.ndarray, corridor: np.ndarray) -> bool:
    """Check the segments between published points, not only their endpoints."""
    if (len(points) < 2 or len(corridor) < 2 or not np.isfinite(points).all()
            or not np.isfinite(corridor).all() or np.any(np.diff(points[:, 0]) <= 0)):
        return False
    xs = corridor[:, 0]
    if points[0, 0] < xs[0] - 1e-5 or points[-1, 0] > xs[-1] + 1e-5:
        return False
    lower = np.interp(points[:, 0], xs, corridor[:, 1])
    upper = np.interp(points[:, 0], xs, corridor[:, 2])
    if np.any(points[:, 1] < lower - 1e-5) or np.any(points[:, 1] > upper + 1e-5):
        return False
    keep = (xs >= points[0, 0] - 1e-6) & (xs <= points[-1, 0] + 1e-6)
    if not np.any(keep):
        return False
    values = np.interp(xs[keep], points[:, 0], points[:, 1])
    return bool(np.all(values >= corridor[keep, 1] - 1e-5)
                and np.all(values <= corridor[keep, 2] + 1e-5))


def regenerate_road_path(points: np.ndarray, corridor: np.ndarray) -> tuple[np.ndarray | None, str]:
    """Fit inside dense bounds, preferring a straight line over new curvature.

    If no single line fits a curved corridor, retain the valid portions of the
    original fit and insert the offending row knots. Each insertion is bounded
    by the current corridor; the final segments must pass the same hard check.
    """
    if len(points) < 2 or not np.isfinite(points).all():
        return None, "nonfinite_fit"
    if road_path_within_corridor(points, corridor):
        return points, "original_fit"
    if (len(corridor) < 2 or not np.isfinite(corridor).all()
            or np.any(corridor[:, 1] > corridor[:, 2])):
        return None, "invalid_corridor"
    xs = corridor[:, 0].astype(np.float64)
    midpoint = corridor[:, 1:].mean(axis=1).astype(np.float64)
    # Scale x so numerical conditioning does not depend on forward distance.
    origin, span = float(xs[0]), float(xs[-1] - xs[0])
    if span <= 0:
        return None, "invalid_corridor"
    design = np.column_stack(((xs - origin) / span, np.ones(len(xs))))
    inset = np.minimum(.01, (corridor[:, 2] - corridor[:, 1]) * .05)
    lower, upper = corridor[:, 1] + inset, corridor[:, 2] - inset
    try:
        initial = np.linalg.lstsq(design, midpoint, rcond=None)[0]
        fit = minimize(lambda q: float(np.mean((design @ q - midpoint) ** 2)), initial,
            jac=lambda q: 2 * design.T @ (design @ q - midpoint) / len(xs),
            method="SLSQP", constraints=[LinearConstraint(design, lower, upper)],
            options={"ftol": 1e-9, "maxiter": 60})
        candidate = np.column_stack((points[:, 0],
            (points[:, 0] - origin) / span * fit.x[0] + fit.x[1])).astype(np.float32)
        if fit.success and road_path_within_corridor(candidate, corridor):
            return candidate, "constrained_line"
    except (ValueError, FloatingPointError, np.linalg.LinAlgError):
        pass
    # An adaptive polyline handles bends that cannot contain a single line.
    # In the worst case it has one knot per observed row (at most BEV height).
    knots = {float(x): float(np.clip(y, np.interp(x, xs, lower),
                                     np.interp(x, xs, upper))) for x, y in points}
    knots[float(xs[0])] = float(np.clip(np.interp(xs[0], points[:, 0], points[:, 1]), lower[0], upper[0]))
    knots[float(xs[-1])] = float(np.clip(np.interp(xs[-1], points[:, 0], points[:, 1]), lower[-1], upper[-1]))
    for _ in range(len(xs) + 1):
        candidate = np.asarray(sorted(knots.items()), np.float32)
        if road_path_within_corridor(candidate, corridor):
            return candidate, "constrained_polyline"
        values = np.interp(xs, candidate[:, 0], candidate[:, 1])
        violation = np.maximum(lower - values, values - upper)
        row = int(np.argmax(violation))
        if violation[row] <= 1e-6:
            break
        knots[float(xs[row])] = float(np.clip(values[row], lower[row], upper[row]))
    return None, "fit_failed"


def _nearest_road_component(region: np.ndarray, ys: np.ndarray) -> np.ndarray:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(region, connectivity=8)
    if count <= 1:
        return region
    # Begin with the nearest observed road. Never bridge across a curb to a
    # larger disconnected patch farther ahead or across a vegetation strip.
    candidates = []
    for label in range(1, count):
        first = int(stats[label, cv2.CC_STAT_TOP])
        columns = np.flatnonzero(labels[first] == label)
        candidates.append(((first, float(np.abs(ys[columns]).min()),
                            -int(stats[label, cv2.CC_STAT_AREA])), label))
    selected = min(candidates)[1]
    return (labels == selected).astype(np.uint8)


def extract_road_centerline(road_mask: np.ndarray, config: LocalPathConfig,
                            road_hazard_mask: np.ndarray | None = None) -> LocalPathEstimate | None:
    return analyze_road_path(road_mask, config, road_hazard_mask).estimate


def analyze_road_path(road_mask: np.ndarray, config: LocalPathConfig,
                      road_hazard_mask: np.ndarray | None = None) -> RoadPathResult:
    """Find the whole visible road midpoint, retaining connected branch guards."""
    config.validate()
    mask = np.asarray(road_mask, np.uint8)
    if mask.ndim != 2 or not mask.size:
        raise ValueError("road_mask must be a nonempty two-dimensional array")
    if road_hazard_mask is not None and np.asarray(road_hazard_mask).shape != mask.shape:
        raise ValueError("road_hazard_mask must match the road image shape")
    mx, my, roi, xs, ys, effective = _road_birdseye_geometry(mask.shape, config)
    observed = cv2.remap(mask * roi, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
    birdseye = observed
    if config.close_kernel_px:
        kernel = np.ones((config.close_kernel_px, config.close_kernel_px), np.uint8)
        birdseye = cv2.morphologyEx(observed, cv2.MORPH_CLOSE, kernel)
    clearance = None
    before_clearance = birdseye
    if config.road_clearance_enabled:
        # The crop boundary does not invent a hazard beyond the metric grid.
        inner = cv2.erode(observed, _metric_margin_kernel(config.road_edge_margin_m, xs, ys),
                          borderType=cv2.BORDER_CONSTANT, borderValue=1)
        hazard = np.zeros_like(observed)
        if road_hazard_mask is not None:
            # Project all current hazard pixels, including just outside ROI.
            hazard = cv2.remap(np.asarray(road_hazard_mask, np.uint8), mx, my,
                               cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
        blocked = cv2.dilate(hazard, _metric_margin_kernel(config.road_hazard_margin_m, xs, ys),
                             borderType=cv2.BORDER_CONSTANT, borderValue=0)
        birdseye = _nearest_road_component((birdseye & inner & (blocked == 0)).astype(np.uint8), ys)
        clearance = {"enabled": True, "edge_margin_m": config.road_edge_margin_m,
                     "hazard_margin_m": config.road_hazard_margin_m,
                     "hazard_pixels": int(np.count_nonzero(hazard)),
                     "observed_road_pixels": int(np.count_nonzero(observed)),
                     "allowed_road_pixels": int(np.count_nonzero(birdseye)),
                     "path_clamped": False}
    estimate = extract_metric_centerline(birdseye, xs, ys, effective, unclosed=observed)
    def result(estimate, status, reason):
        metrics = {**(clearance or {}), "status": status, "reason": reason,
                   "blocked": status == "blocked", "using_previous_path": False}
        if estimate is not None and clearance is not None:
            estimate = replace(estimate, road_clearance=metrics)
        analyzed = RoadPathResult(estimate, status, reason, metrics, birdseye, xs, ys)
        if estimate is not None and clearance is not None:
            fresh_corridor = analyzed.corridor_for(estimate.points_xy)
            if fresh_corridor is None:
                return result(None, "calculation_failed", "fresh_mask_validation_failed")
            analyzed = replace(analyzed, estimate=replace(estimate, road_corridor=fresh_corridor))
        return analyzed
    if estimate is None:
        # Evidence elsewhere in the image does not prove this corridor is
        # blocked. Compare the visible road with and without the exclusions.
        baseline = (extract_metric_centerline(_nearest_road_component(before_clearance, ys),
                    xs, ys, effective, unclosed=observed) if clearance is not None else None)
        if baseline is not None:
            return result(None, "blocked", "clearance_leaves_no_passage")
        return result(None, "insufficient_observation", "insufficient_road_support")
    raw = estimate.raw_points_xy
    support = cv2.remap(roi, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
    widths, complete = [], []
    spacing = float(abs(ys[1] - ys[0]))
    for x, y in raw:
        row = int(np.argmin(np.abs(xs - x)))
        runs = _runs(birdseye[row] > 0)
        match = next(((a, b) for a, b in runs if ys[b] - spacing <= y <= ys[a] + spacing), None)
        a, b = match if match else (0, 0)
        widths.append((b - a + 1) * spacing)
        # A road clipped by the image/selection border has an artificial
        # midpoint. Prefer the longest stretch with both actual edges visible.
        original = next(((i, j) for i, j in _runs(observed[row] > 0)
                         if ys[j] - spacing <= y <= ys[i] + spacing), None)
        complete.append(bool(original and original[0] > 0 and original[1] < len(ys) - 1
                             and support[row, original[0] - 1] and support[row, original[1] + 1]))
    full = np.flatnonzero(complete)
    if len(full):
        groups = np.split(full, np.flatnonzero(np.diff(raw[full, 0]) > 0.10) + 1)
        best = max(groups, key=len)
        if len(best) >= 12 and raw[best[-1], 0] - raw[best[0], 0] >= 0.5:
            raw, widths = raw[best], np.asarray(widths)[best]
    widths = np.asarray(widths)
    # Start with a line: segmentation edge jitter must not invent an S bend.
    # Accept a quadratic only with broad support and a clear improvement.
    try:
        coefficients = np.polyfit(raw[:, 0], raw[:, 1], 1)
    except (ValueError, FloatingPointError, np.linalg.LinAlgError):
        return result(None, "calculation_failed", "linear_fit_failed")
    residual = raw[:, 1] - np.polyval(coefficients, raw[:, 0])
    linear_rms = float(np.sqrt(np.mean(residual ** 2)))
    if config.fit_degree >= 2 and np.ptp(raw[:, 0]) >= 2.5 and linear_rms > 0.20:
        try:
            quadratic = np.polyfit(raw[:, 0], raw[:, 1], 2)
        except (ValueError, FloatingPointError, np.linalg.LinAlgError):
            quadratic = coefficients
        rms = float(np.sqrt(np.mean((raw[:, 1] - np.polyval(quadratic, raw[:, 0])) ** 2)))
        if rms < linear_rms * 0.65:
            coefficients = quadratic
    # Do not extrapolate a quadratic into buildings/vegetation where no road
    # rows were observed. Constrain the fit to its selected corridor per row.
    forward = np.linspace(float(raw[0, 0]), float(raw[-1, 0]), config.path_points)
    centers = np.interp(forward, raw[:, 0], raw[:, 1])
    half = np.interp(forward, raw[:, 0], widths) / 2
    margin = np.minimum(0.10, half * 0.2)
    fitted = np.polyval(coefficients, forward)
    fitted = np.clip(fitted, centers - half + margin, centers + half - margin)
    points = np.column_stack((forward, fitted)).astype(np.float32)
    # Keep dense row constraints so smoothing cannot cut
    # through a narrow excluded strip between the published path points.
    corridor = np.column_stack((raw[:, 0], raw[:, 1] - widths / 2,
                                raw[:, 1] + widths / 2)).astype(np.float32)
    corridor[:, 1] += 1e-4
    corridor[:, 2] -= 1e-4
    if clearance is not None and not road_path_within_corridor(points, corridor):
        points, method = regenerate_road_path(points, corridor)
        if points is None:
            return result(None, "calculation_failed", method)
        clearance["regeneration_method"] = method
    elif clearance is not None:
        clearance["regeneration_method"] = "original_fit"
    estimate = replace(estimate, points_xy=points,
                   raw_points_xy=raw, mean_sidewalk_width_m=float(np.mean(widths)),
                   road_corridor=corridor, road_clearance=clearance,
                   road_center_tracking=True)
    return result(estimate, "regenerated" if clearance is not None and
                  clearance["regeneration_method"] != "original_fit" else "valid", "path_available")


def extract_sidewalk_centerline(
    sidewalk_mask: np.ndarray, config: LocalPathConfig
) -> LocalPathEstimate | None:
    """Extract and fit a sidewalk centerline from a binary semantic mask.

    Each metric distance row selects one contiguous sidewalk interval.  The
    interval is tracked from near to far so a second disconnected sidewalk
    patch does not randomly replace the current path.  A quadratic fit gives
    the planner a compact, smooth path while preserving the raw centers for
    overlay review.
    """

    config.validate()
    mask = np.asarray(sidewalk_mask)
    if mask.ndim != 2:
        raise ValueError("sidewalk_mask must be a two-dimensional array")
    birdseye, x_values, y_values = _birdseye_sidewalk(mask, config)
    unclosed = None
    if config.branch_preference != "center":
        unclosed, _, _ = _birdseye_sidewalk(mask, replace(config, close_kernel_px=0))
    return extract_metric_centerline(birdseye, x_values, y_values, config,
                                    unclosed=unclosed)


def extract_metric_centerline(
    birdseye: np.ndarray, x_values: np.ndarray, y_values: np.ndarray,
    config: LocalPathConfig, *, unclosed: np.ndarray | None = None,
) -> LocalPathEstimate | None:
    """Fit the selected semantic region in its configured metric grid."""
    config.validate()
    birdseye = np.asarray(birdseye, dtype=np.uint8).copy()
    if birdseye.shape != (len(x_values), len(y_values)):
        raise ValueError("metric region/grid dimensions differ")
    meters_per_column = (
        2.0 * config.search_half_width_m / max(config.bev_width_px - 1, 1)
    )
    min_width_px = max(
        1, int(math.ceil(config.min_sidewalk_width_m / meters_per_column))
    )

    raw: list[tuple[float, float, float]] = []
    previous_y: float | None = None
    for row, forward_x in enumerate(x_values):
        candidates = []
        for start, end in _runs(birdseye[row] > 0):
            width_px = end - start + 1
            if width_px < min_width_px:
                continue
            center_column = (start + end) / 2.0
            center_y = float(
                np.interp(center_column, np.arange(y_values.size), y_values)
            )
            width_m = width_px * meters_per_column
            score = float(width_px)
            if previous_y is not None:
                # Prefer continuity, but allow a real sidewalk widening to win.
                score -= 0.20 * abs(center_y - previous_y) / meters_per_column
            candidates.append((score, center_y, width_m))
        if not candidates:
            continue
        _, center_y, width_m = max(candidates, key=lambda item: item[0])
        previous_y = center_y
        raw.append((float(forward_x), center_y, width_m))

    valid_ratio = len(raw) / max(len(x_values), 1)
    minimum_points = (
        2
        if config.unrestricted_path_mode
        else max(3, int(math.ceil(config.min_valid_ratio * len(x_values))))
    )
    if len(raw) < minimum_points:
        return None

    raw_points = np.asarray([(x, y) for x, y, _ in raw], dtype=np.float32)
    widths = np.asarray([width for _, _, width in raw], dtype=np.float32)
    degree = min(config.fit_degree, len(raw_points) - 1)
    try:
        coefficients = np.polyfit(raw_points[:, 0], raw_points[:, 1], degree)
        fitted_y = np.polyval(coefficients, x_values).astype(np.float32)
    except (FloatingPointError, np.linalg.LinAlgError, ValueError):
        fitted_y = np.interp(x_values, raw_points[:, 0], raw_points[:, 1]).astype(
            np.float32
        )
    fitted_y = np.clip(
        fitted_y,
        -config.max_path_lateral_m,
        config.max_path_lateral_m,
    )
    points = np.column_stack(
        (
            np.linspace(
                config.near_distance_m, config.far_distance_m, config.path_points
            ),
            np.interp(
                np.linspace(
                    config.near_distance_m, config.far_distance_m, config.path_points
                ),
                x_values,
                fitted_y,
            ),
        )
    ).astype(np.float32)
    mean_width = float(np.mean(widths)) if widths.size else 0.0
    width_support = min(1.0, mean_width / max(config.ground_half_width_m * 0.5, 1e-6))
    confidence = float(np.clip(0.75 * valid_ratio + 0.25 * width_support, 0.0, 1.0))
    estimate = LocalPathEstimate(
        points_xy=points,
        confidence=confidence,
        valid_ratio=float(valid_ratio),
        mean_sidewalk_width_m=mean_width,
        raw_points_xy=raw_points,
    )
    if config.branch_preference != "center":
        from branch_path import observe_branches

        # Closing may bridge separate patches. Branch connectivity must use the
        # observed mask, while the legacy single-corridor fit stays unchanged.
        if unclosed is None:
            unclosed = birdseye
        estimate = replace(estimate, branch_observation=observe_branches(
            unclosed, x_values, y_values, config, float(raw_points[0, 1])
        ))
    return estimate


class LocalPathSmoother:
    """Time-aware EMA that retains a path briefly between Swin-L updates."""

    def __init__(self, config: LocalPathConfig) -> None:
        config.validate()
        self.config = config
        self._lock = threading.Lock()
        self._points: np.ndarray | None = None
        self._confidence = 0.0
        self._last_update: float | None = None
        from branch_path import BranchSelector

        self._branch_selector = BranchSelector(config)
        self._stop_reason: str | None = None
        self._constrained_branch = False
        self._vegetation_bias_active = False
        self._road_center_tracking = False
        self._road_corridor = None
        self._road_clearance = None
        self._road_validation_hold = False

    def reset(self, *, preserve_branch: bool = False) -> None:
        """Discard path history, retaining a committed fork on sensor loss if requested."""
        with self._lock:
            self._points = None
            self._confidence = 0.0
            self._last_update = None
            if not preserve_branch:
                self._branch_selector.reset()
            self._stop_reason = None
            self._constrained_branch = False
            self._vegetation_bias_active = False
            self._road_center_tracking = False
            self._road_corridor = None
            self._road_clearance = None
            self._road_validation_hold = False

    def update_road(self, result: RoadPathResult, timestamp_sec: float) -> SmoothedPath | None:
        """Hold only a recent path revalidated against fresh allowed pixels."""
        if result.estimate is not None:
            return self.update(result.estimate, timestamp_sec)
        with self._lock:
            recent = (self._last_update is not None and
                      0 <= timestamp_sec - self._last_update <= self.config.path_hold_sec)
            if not result.blocked and recent and self._road_center_tracking and self._stop_reason is None:
                corridor = result.corridor_for(self._points)
                if corridor is not None:
                    self._road_corridor = corridor
                    self._road_clearance = {**result.clearance, "using_previous_path": True}
                    self._road_validation_hold = True
                    # Never refresh the age with a failed extraction.
                    return self._current_unlocked(timestamp_sec)
        self.reset(preserve_branch=True)
        return self.update(None, timestamp_sec)

    def update(
        self, estimate: LocalPathEstimate | None, timestamp_sec: float
    ) -> SmoothedPath | None:
        with self._lock:
            self._road_validation_hold = False
            choice = self._branch_selector.update(estimate, timestamp_sec)
            estimate = choice.estimate
            self._stop_reason = choice.stop_reason
            self._constrained_branch = choice.constrained
            self._road_corridor = (estimate.road_corridor.copy() if estimate is not None
                                   and estimate.road_corridor is not None else None)
            self._road_clearance = (dict(estimate.road_clearance)
                                    if estimate is not None and estimate.road_clearance is not None else None)
            if choice.stop_reason is not None:
                # A distinct guard survives ordinary path-loss bypass and hold.
                self._points = np.empty((0, 2), dtype=np.float32)
                self._confidence = 0.0
                self._last_update = timestamp_sec
                self._vegetation_bias_active = False
                self._road_center_tracking = False
                return self._current_unlocked(timestamp_sec)
            if estimate is None and self._road_center_tracking and self.config.road_clearance_enabled:
                # Fresh evidence found no admissible road. Do not retain an
                # older road through the normal path-hold window.
                self._points = None
                self._confidence = 0.0
                self._last_update = None
                self._road_center_tracking = False
                return None
            if choice.constrained and estimate is not None:
                # An EMA across branches can cross the non-drivable fork gap.
                # These points have already been smoothed inside one corridor.
                self._points = estimate.points_xy.copy()
                self._confidence = estimate.confidence
                self._last_update = timestamp_sec
                self._vegetation_bias_active = estimate.vegetation_bias_active
                self._road_center_tracking = estimate.road_center_tracking
                return self._current_unlocked(timestamp_sec)
            road_tracking = estimate is not None and estimate.road_center_tracking
            if self.config.unrestricted_path_mode and not road_tracking:
                if estimate is None or not estimate.is_valid:
                    self._points = None
                    self._confidence = 0.0
                    self._last_update = None
                    self._vegetation_bias_active = False
                    self._road_center_tracking = False
                    return None
                self._points = np.asarray(estimate.points_xy, dtype=np.float32).copy()
                self._confidence = estimate.confidence
                self._last_update = timestamp_sec
                self._vegetation_bias_active = estimate.vegetation_bias_active
                self._road_center_tracking = estimate.road_center_tracking
                return self._current_unlocked(timestamp_sec)
            if estimate is not None and estimate.is_valid:
                self._vegetation_bias_active = estimate.vegetation_bias_active
                self._road_center_tracking = estimate.road_center_tracking
                target = np.asarray(estimate.points_xy, dtype=np.float32)
                if self._points is None or self._points.shape != target.shape:
                    self._points = target.copy()
                    self._confidence = estimate.confidence
                else:
                    if road_tracking:
                        # Observed forward support can change when a clipped
                        # road edge reappears. Blend y at matching distances.
                        self._points = np.column_stack((target[:, 0], np.interp(
                            target[:, 0], self._points[:, 0], self._points[:, 1]))).astype(np.float32)
                    previous_update = (
                        self._last_update
                        if self._last_update is not None
                        else timestamp_sec
                    )
                    elapsed = max(timestamp_sec - previous_update, 0.0)
                    exponent = -max(elapsed, 1.0 / 30.0) / self.config.smoothing_time_constant_sec
                    alpha = 1.0 - math.exp(exponent)
                    delta = np.clip(
                        target - self._points,
                        -self.config.max_lateral_update_m,
                        self.config.max_lateral_update_m,
                    )
                    self._points = self._points + alpha * delta
                    self._confidence = (
                        1.0 - alpha
                    ) * self._confidence + alpha * estimate.confidence
                if road_tracking and self._road_clearance is not None and self._road_corridor is not None:
                    previous = self._points.copy()
                    self._points[:, 1] = np.clip(self._points[:, 1],
                        np.interp(self._points[:, 0], self._road_corridor[:, 0], self._road_corridor[:, 1]),
                        np.interp(self._points[:, 0], self._road_corridor[:, 0], self._road_corridor[:, 2]))
                    if not road_path_within_corridor(self._points, self._road_corridor):
                        self._points = target.copy()
                    self._road_clearance["path_clamped"] = not np.array_equal(previous, self._points)
                self._last_update = timestamp_sec
            return self._current_unlocked(timestamp_sec)

    def current(self, timestamp_sec: float) -> SmoothedPath | None:
        with self._lock:
            return self._current_unlocked(timestamp_sec)

    def _current_unlocked(self, timestamp_sec: float) -> SmoothedPath | None:
        if self._points is None or self._last_update is None:
            return None
        age = max(timestamp_sec - self._last_update, 0.0)
        unrestricted = self.config.unrestricted_path_mode
        if self._stop_reason is None and (not unrestricted or self._road_validation_hold) and age > self.config.path_hold_sec:
            return None
        return SmoothedPath(
            points_xy=self._points.copy(),
            confidence=float(self._confidence),
            age_sec=float(age),
            source=(
                "branch_guard" if self._stop_reason else
                "road_validated_hold" if self._road_validation_hold else
                "selected_branch" if self._constrained_branch else
                "raw_latest"
                if unrestricted and not self._road_center_tracking
                else "smoothed_hold"
                if age > 0.02
                else "smoothed_update"
            ),
            stop_reason=self._stop_reason,
            branch_status=self._branch_selector.metrics(),
            vegetation_bias_active=self._vegetation_bias_active,
            road_center_tracking=self._road_center_tracking,
            road_corridor=self._road_corridor.copy() if self._road_corridor is not None else None,
            road_clearance=dict(self._road_clearance) if self._road_clearance is not None else None,
        )
