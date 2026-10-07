"""Livox obstacle slices and current observed-space grids in base_link.

No ground fitting or semantic height filtering. The default detection volume is
a base_link height band and a forward horizontal sector around base_link.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np

from lidar_common import cloud_xyz, stamp_ns, validate_transform


@dataclass(frozen=True)
class ObstacleConfig:
    height_reference: str = "base_link"
    min_height_m: float = -0.20
    max_height_m: float = 0.20
    horizontal_range_m: float = 1.50
    fov_deg: float = 190.0
    max_age_sec: float = 0.30
    imu_accel_unit: str = "auto"
    forward_m: float = 6.0
    rear_m: float = 1.0
    half_width_m: float = 2.0
    resolution_m: float = 0.10
    front_m: float = 0.50
    back_m: float = 0.45
    robot_half_width_m: float = 0.30
    margin_m: float = 0.15
    self_front_m: float = 0.30
    self_back_m: float = 0.30
    self_half_width_m: float = 0.16
    lidar_to_ground_m: float = 0.40
    inflation_m: float = 2.0

    def validate(self):
        if self.height_reference not in ("base_link", "lidar_gravity"):
            raise ValueError("obstacle height reference must be base_link or lidar_gravity")
        if not (
            math.isfinite(self.min_height_m)
            and math.isfinite(self.max_height_m)
            and self.min_height_m < 0 < self.max_height_m
        ):
            raise ValueError("obstacle height bounds must straddle the reference origin")
        if self.imu_accel_unit not in ("auto", "g", "mps2"):
            raise ValueError("invalid obstacle IMU acceleration unit")
        for name, value in vars(self).items():
            if isinstance(value, float) and name not in (
                "min_height_m",
                "max_height_m",
            ):
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f"obstacle {name} must be finite and positive")
        if self.fov_deg > 360:
            raise ValueError("obstacle fov_deg must not exceed 360 degrees")
        if self.self_front_m >= self.front_m or self.self_back_m >= self.back_m:
            raise ValueError("self filter must lie inside robot footprint")
        if self.self_half_width_m >= self.robot_half_width_m:
            raise ValueError("self filter must lie inside robot footprint")
        if (
            self.forward_m <= self.front_m + self.margin_m
            or self.rear_m <= self.back_m + self.margin_m
            or self.half_width_m <= self.robot_half_width_m + self.margin_m
        ):
            raise ValueError("observation bounds must contain robot footprint")
        if np.prod(self.shape) > 100_000:
            raise ValueError("obstacle grid exceeds 100000 cells")

    @property
    def shape(self):
        return (
            int(math.ceil((self.forward_m + self.rear_m) / self.resolution_m)) + 1,
            int(math.ceil(2 * self.half_width_m / self.resolution_m)) + 1,
        )

    def indices(self, xy):
        a = np.asarray(xy)
        return np.rint(
            (a + [self.rear_m, self.half_width_m]) / self.resolution_m
        ).astype(int)

    def centers(self):
        x, y = np.meshgrid(
            np.arange(self.shape[0]) * self.resolution_m - self.rear_m,
            np.arange(self.shape[1]) * self.resolution_m - self.half_width_m,
            indexing="ij",
        )
        return np.stack((x, y), axis=-1)

    def in_detection_sector(self, xy):
        """Horizontal range/FOV about base_link, facing the robot's +x axis."""
        xy = np.asarray(xy)
        radius = np.linalg.norm(xy, axis=-1)
        return (radius <= self.horizontal_range_m + 1e-6) & (
            xy[..., 0] >= radius * math.cos(math.radians(self.fov_deg / 2)) - 1e-6
        )


@dataclass(frozen=True)
class ObstacleScan:
    points: np.ndarray
    observed: np.ndarray
    costs: np.ndarray
    stamp_ns: int
    arrival: float
    input_count: int
    up_base: np.ndarray
    origin_base: np.ndarray


def gravity_direction(imu, cloud, config):
    if imu.header.frame_id != cloud.header.frame_id:
        raise ValueError("lidar_imu_frame_mismatch")
    if abs(stamp_ns(imu.header) - stamp_ns(cloud.header)) > 150_000_000:
        raise ValueError("lidar_imu_unsynchronized")
    acc = np.array(
        [
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        ],
        dtype=float,
    )
    norm = np.linalg.norm(acc)
    scale = (
        9.80665
        if config.imu_accel_unit == "g"
        or (config.imu_accel_unit == "auto" and 0.5 <= norm <= 1.5)
        else 1.0
    )
    if not np.isfinite(acc).all() or not 5 <= norm * scale <= 15:
        raise ValueError("lidar_imu_gravity_invalid")
    return acc / norm


def fresh(scan, now, clock_ns, config):
    if scan is None:
        return "lidar_waiting_for_cloud"
    if not 0 <= now - scan.arrival <= config.max_age_sec:
        return "lidar_stale"
    if not -0.05 <= (clock_ns - scan.stamp_ns) / 1e9 <= config.max_age_sec:
        return "lidar_stale"
    return None


def _observed_rays(points, heights, origin, config):
    """Only rays inside the selected height slice establish observed space.

    Rays end at their first measured return; no free-space extrapolation beyond
    an obstacle or through a missing return. Subsampling only removes evidence.
    """
    result = np.zeros(config.shape, bool)
    if not len(points):
        return result
    ids = np.linspace(0, len(points) - 1, min(3000, len(points))).astype(int)
    ends, h = points[ids], heights[ids]
    origin_height = origin[2] if config.height_reference == "base_link" else 0.0
    # Clip long rays before sampling so distant returns still establish dense
    # evidence inside the short-range sector. Never extend beyond a return.
    max_length = config.horizontal_range_m + float(np.linalg.norm(origin[:2]))
    length = np.linalg.norm(ends[:, :2] - origin[:2], axis=1)
    scale = np.minimum(1.0, max_length / np.maximum(length, 1e-9))
    ends = origin + scale[:, None] * (ends - origin)
    h = origin_height + scale * (h - origin_height)
    fractions = np.linspace(
        0,
        1,
        int(
            math.ceil(max_length / (config.resolution_m / 2))
        )
        + 1,
    )
    xy = origin[:2] + fractions[None, :, None] * (ends[:, None, :2] - origin[:2])
    z = origin_height + (h[:, None] - origin_height) * fractions
    cells = config.indices(xy)
    inside = (
        (cells[..., 0] >= 0)
        & (cells[..., 0] < config.shape[0])
        & (cells[..., 1] >= 0)
        & (cells[..., 1] < config.shape[1])
        & (z >= config.min_height_m)
        & (z <= config.max_height_m)
        & config.in_detection_sector(xy)
    )
    cells = cells[inside]
    result[cells[:, 0], cells[:, 1]] = True
    # Rounding a ray sample to a cell must not observe a center outside the FOV.
    result &= config.in_detection_sector(config.centers())
    return result


class ObstacleDetector:
    def __init__(self, config):
        config.validate()
        self.config = config
        self.previous = None

    def update(self, cloud, imu, base_from_lidar, arrival):
        c = self.config
        validate_transform(base_from_lidar)
        source = stamp_ns(cloud.header)
        if source <= 0 or (
            self.previous is not None and source <= self.previous.stamp_ns
        ):
            raise ValueError("lidar_timestamp_invalid")
        raw = cloud_xyz(cloud)
        if not len(raw):
            raise ValueError("lidar_points_unavailable")
        up = base_from_lidar[:3, :3] @ gravity_direction(imu, cloud, c)
        if up[2] < 0.5:
            raise ValueError("lidar_gravity_axis_invalid")
        origin = base_from_lidar[:3, 3].copy()
        points = raw @ base_from_lidar[:3, :3].T + origin
        heights = (
            points[:, 2]
            if c.height_reference == "base_link"
            else (points - origin) @ up
        )
        # Exclude known body returns BEFORE ray tracing: a self-return cannot
        # certify unseen space beyond the robot.
        self_hit = (
            (points[:, 0] >= -c.self_back_m)
            & (points[:, 0] <= c.self_front_m)
            & (np.abs(points[:, 1]) <= c.self_half_width_m)
        )
        observed = _observed_rays(points[~self_hit], heights[~self_hit], origin, c)
        keep = (
            (heights >= c.min_height_m - 1e-6)
            & (heights <= c.max_height_m + 1e-6)
            & (points[:, 0] >= -c.rear_m)
            & (points[:, 0] <= c.forward_m)
            & (np.abs(points[:, 1]) <= c.half_width_m)
            & c.in_detection_sector(points[:, :2])
            & ~self_hit
        )
        cells = np.unique(c.indices(points[keep, :2]), axis=0)
        hits = np.zeros(c.shape, np.uint8)
        if len(cells):
            hits[cells[:, 0], cells[:, 1]] = 1
        xy = cells * c.resolution_m - [c.rear_m, c.half_width_m]
        distance = (
            (
                cv2.distanceTransform(1 - hits, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
                * c.resolution_m
            )
            if len(cells)
            else np.full(c.shape, np.inf)
        )
        costs = np.where(
            distance < c.inflation_m, np.exp(-3 * distance / c.inflation_m), 0.0
        )
        scan = ObstacleScan(xy, observed, costs, source, arrival, len(raw), up, origin)
        self.previous = scan
        return scan


def camera_region(
    mask, info, camera_from_base, scan, config, mask_class, *, return_visibility=False
):
    """Project a configured support plane; this does not estimate floor height."""
    validate_transform(camera_from_base)
    k = np.asarray(info.k, float).reshape(3, 3)
    d = np.asarray(info.d, float)
    if (
        not np.isfinite(k).all()
        or not np.isfinite(d).all()
        or k[0, 0] <= 0
        or k[1, 1] <= 0
        or info.width <= 0
        or info.height <= 0
        or info.distortion_model not in ("plumb_bob", "rational_polynomial")
        or len(d) not in (0, 4, 5, 8, 12, 14)
    ):
        raise ValueError("lidar_camera_calibration_invalid")
    xy = config.centers().reshape(-1, 2)
    up = scan.up_base
    offset = float(up @ scan.origin_base) - config.lidar_to_ground_m
    z = (offset - xy @ up[:2]) / up[2]
    points = np.column_stack((xy, z))
    camera = points @ camera_from_base[:3, :3].T + camera_from_base[:3, 3]
    uv, _ = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), k, d)
    uv = uv.reshape(-1, 2)
    valid = np.isfinite(uv).all(axis=1) & (camera[:, 2] > 0.01)
    uv[~valid] = -1
    px = np.floor(uv[:, 0] * mask.shape[1] / info.width).astype(int)
    py = np.floor(uv[:, 1] * mask.shape[0] / info.height).astype(int)
    valid &= (px >= 0) & (px < mask.shape[1]) & (py >= 0) & (py < mask.shape[0])
    region = np.zeros(len(xy), bool)
    labels = mask[py[valid], px[valid]]
    region[valid] = np.isin(labels, (1, 2)) if mask_class == 0 else labels == mask_class
    region, visible = region.reshape(config.shape), valid.reshape(config.shape)
    return (region, visible) if return_visibility else region
