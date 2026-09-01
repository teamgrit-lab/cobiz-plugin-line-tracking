"""Sidewalk-center local path extraction and LiDAR safety helpers.

The Swin-L runtime produces a Mapillary surface label map.  This module turns
the Sidewalk part of that map into a short path in the robot convention
``x=forward, y=left``.  It deliberately keeps the geometry explicit and
configurable because the rosbag contains camera intrinsics but no camera-to-
base extrinsic calibration.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
from typing import Any, Iterable

import cv2
import numpy as np


DEFAULT_ROI_POLYGON = (
    0.08,
    1.00,
    0.92,
    1.00,
    0.62,
    0.22,
    0.38,
    0.22,
)


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

    def validate(self) -> None:
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
        if self.path_points < 2:
            raise ValueError("path_points must be at least 2")
        if self.bev_width_px < 8 or self.bev_height_px < 8:
            raise ValueError("bird's-eye dimensions are too small")
        if not 0.0 < self.min_valid_ratio <= 1.0:
            raise ValueError("min_valid_ratio must be in (0, 1]")
        if self.min_sidewalk_width_m <= 0.0:
            raise ValueError("min_sidewalk_width_m must be positive")
        if self.close_kernel_px < 0 or self.close_kernel_px % 2 == 0:
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


@dataclass(frozen=True)
class LocalPathEstimate:
    """One raw path estimate from one segmentation result."""

    points_xy: np.ndarray
    confidence: float
    valid_ratio: float
    mean_sidewalk_width_m: float
    raw_points_xy: np.ndarray

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


@dataclass(frozen=True)
class LidarSafetyConfig:
    """Conservative point-cloud obstacle gate for the local path."""

    topic: str = "/unitree/slam_lidar/points2"
    timeout_sec: float = 0.35
    obstacle_distance_m: float = 8.0
    stop_distance_m: float = 3.0
    corridor_half_width_m: float = 0.55
    z_min_m: float = -0.40
    z_max_m: float = 0.80
    min_obstacle_points: int = 3

    def validate(self) -> None:
        if self.timeout_sec <= 0.0:
            raise ValueError("LiDAR timeout must be positive")
        if self.obstacle_distance_m <= 0.0:
            raise ValueError("obstacle_distance_m must be positive")
        if not 0.0 < self.stop_distance_m <= self.obstacle_distance_m:
            raise ValueError("stop_distance_m must be within obstacle_distance_m")
        if self.corridor_half_width_m <= 0.0:
            raise ValueError("corridor_half_width_m must be positive")
        if self.z_min_m >= self.z_max_m:
            raise ValueError("LiDAR z bounds are invalid")
        if self.min_obstacle_points < 1:
            raise ValueError("min_obstacle_points must be positive")


@dataclass(frozen=True)
class LidarSafetyResult:
    """LiDAR gate result for a current path."""

    stop: bool
    lidar_available: bool
    obstacle_in_path: bool
    obstacle_count: int
    clearance_m: float | None
    age_sec: float | None
    reason: str


def normalized_polygon_pixels(
    polygon: Iterable[float], frame_shape: tuple[int, int]
) -> np.ndarray:
    """Convert normalized x/y polygon coordinates to OpenCV pixel points."""

    height, width = frame_shape[:2]
    values = np.asarray(tuple(float(value) for value in polygon), dtype=np.float32)
    if values.size != 8:
        raise ValueError("polygon must contain four x/y points")
    points = values.reshape(4, 2).copy()
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


def _birdseye_sidewalk(
    sidewalk_mask: np.ndarray,
    config: LocalPathConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Warp a camera mask onto a metric x-forward/y-left grid."""

    height, width = sidewalk_mask.shape[:2]
    homography = pixel_to_ground_homography((height, width), config)
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
    birdseye = cv2.remap(
        np.where(sidewalk_mask > 0, 255, 0).astype(np.uint8),
        image_points[..., 0],
        image_points[..., 1],
        interpolation=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    if config.close_kernel_px:
        kernel = np.ones(
            (config.close_kernel_px, config.close_kernel_px), dtype=np.uint8
        )
        birdseye = cv2.morphologyEx(birdseye, cv2.MORPH_CLOSE, kernel)
    return birdseye, x_values, y_values


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
    if len(raw) < max(3, int(math.ceil(config.min_valid_ratio * len(x_values)))):
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
    return LocalPathEstimate(
        points_xy=points,
        confidence=confidence,
        valid_ratio=float(valid_ratio),
        mean_sidewalk_width_m=mean_width,
        raw_points_xy=raw_points,
    )


class LocalPathSmoother:
    """Time-aware EMA that retains a path briefly between Swin-L updates."""

    def __init__(self, config: LocalPathConfig) -> None:
        config.validate()
        self.config = config
        self._lock = threading.Lock()
        self._points: np.ndarray | None = None
        self._confidence = 0.0
        self._last_update: float | None = None

    def reset(self) -> None:
        with self._lock:
            self._points = None
            self._confidence = 0.0
            self._last_update = None

    def update(
        self, estimate: LocalPathEstimate | None, timestamp_sec: float
    ) -> SmoothedPath | None:
        with self._lock:
            if estimate is not None and estimate.is_valid:
                target = np.asarray(estimate.points_xy, dtype=np.float32)
                if self._points is None:
                    self._points = target.copy()
                    self._confidence = estimate.confidence
                else:
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
                self._last_update = timestamp_sec
            return self._current_unlocked(timestamp_sec)

    def current(self, timestamp_sec: float) -> SmoothedPath | None:
        with self._lock:
            return self._current_unlocked(timestamp_sec)

    def _current_unlocked(self, timestamp_sec: float) -> SmoothedPath | None:
        if self._points is None or self._last_update is None:
            return None
        age = max(timestamp_sec - self._last_update, 0.0)
        if age > self.config.path_hold_sec:
            return None
        return SmoothedPath(
            points_xy=self._points.copy(),
            confidence=float(self._confidence),
            age_sec=float(age),
            source="smoothed_hold" if age > 0.02 else "smoothed_update",
        )


def pointcloud2_xyz(message: Any) -> np.ndarray:
    """Decode x/y/z from a ROS ``PointCloud2`` with arbitrary point padding."""

    width = int(message.width)
    height = int(message.height)
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    if width <= 0 or height <= 0 or point_step < 12 or row_step < width * point_step:
        raise ValueError("invalid PointCloud2 dimensions or strides")
    fields = {str(field.name): int(field.offset) for field in message.fields}
    if any(name not in fields for name in ("x", "y", "z")):
        raise ValueError("PointCloud2 does not contain x/y/z fields")
    offsets = [fields["x"], fields["y"], fields["z"]]
    if any(offset < 0 or offset + 4 > point_step for offset in offsets):
        raise ValueError("PointCloud2 x/y/z fields exceed point_step")
    endian = ">" if bool(message.is_bigendian) else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [f"{endian}f4"] * 3,
            "offsets": offsets,
            "itemsize": point_step,
        }
    )
    raw = memoryview(bytes(message.data))
    required = row_step * height
    if len(raw) < required:
        raise ValueError("PointCloud2 data is shorter than row_step * height")
    rows = []
    for row in range(height):
        row_bytes = raw[row * row_step:row * row_step + width * point_step]
        rows.append(np.frombuffer(row_bytes, dtype=dtype, count=width))
    values = np.concatenate(rows) if len(rows) > 1 else rows[0]
    return np.column_stack((values["x"], values["y"], values["z"])).astype(
        np.float32, copy=False
    )


class LidarSafetyMonitor:
    """Evaluate a point cloud against the current path corridor."""

    def __init__(self, config: LidarSafetyConfig) -> None:
        config.validate()
        self.config = config
        self._lock = threading.Lock()
        self._points: np.ndarray | None = None
        self._timestamp: float | None = None

    def update(self, points_xyz: np.ndarray, timestamp_sec: float) -> None:
        points = np.asarray(points_xyz, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points_xyz must have shape [N, 3]")
        finite = np.all(np.isfinite(points), axis=1)
        with self._lock:
            self._points = np.ascontiguousarray(points[finite])
            self._timestamp = float(timestamp_sec)

    def evaluate(
        self, path: SmoothedPath | None, timestamp_sec: float
    ) -> LidarSafetyResult:
        with self._lock:
            scan_timestamp = self._timestamp
            scan_points = None if self._points is None else self._points.copy()
        if scan_timestamp is None or scan_points is None:
            return LidarSafetyResult(
                stop=True,
                lidar_available=False,
                obstacle_in_path=False,
                obstacle_count=0,
                clearance_m=None,
                age_sec=None,
                reason="lidar_unavailable",
            )
        age = max(float(timestamp_sec - scan_timestamp), 0.0)
        if age > self.config.timeout_sec:
            return LidarSafetyResult(
                stop=True,
                lidar_available=False,
                obstacle_in_path=False,
                obstacle_count=0,
                clearance_m=None,
                age_sec=age,
                reason="lidar_timeout",
            )
        if path is None:
            return LidarSafetyResult(
                stop=False,
                lidar_available=True,
                obstacle_in_path=False,
                obstacle_count=0,
                clearance_m=None,
                age_sec=age,
                reason="path_unavailable",
            )

        points = scan_points
        in_bounds = np.logical_and.reduce(
            (
                points[:, 0] > 0.05,
                points[:, 0] <= self.config.obstacle_distance_m,
                points[:, 2] >= self.config.z_min_m,
                points[:, 2] <= self.config.z_max_m,
            )
        )
        candidates = points[in_bounds]
        if candidates.size == 0:
            return LidarSafetyResult(
                stop=False,
                lidar_available=True,
                obstacle_in_path=False,
                obstacle_count=0,
                clearance_m=None,
                age_sec=age,
                reason="clear",
            )
        path_y = np.interp(candidates[:, 0], path.points_xy[:, 0], path.points_xy[:, 1])
        on_path = np.abs(candidates[:, 1] - path_y) <= self.config.corridor_half_width_m
        obstacles = candidates[on_path]
        clearance = float(np.min(obstacles[:, 0])) if obstacles.size else None
        enough_points = obstacles.shape[0] >= self.config.min_obstacle_points
        has_close_obstacle = (
            clearance is not None and clearance <= self.config.stop_distance_m
        )
        stop = bool(enough_points and has_close_obstacle)
        return LidarSafetyResult(
            stop=stop,
            lidar_available=True,
            obstacle_in_path=bool(obstacles.size),
            obstacle_count=int(obstacles.shape[0]),
            clearance_m=clearance,
            age_sec=age,
            reason="obstacle_in_path"
            if stop
            else ("obstacle_far" if obstacles.size else "clear"),
        )
