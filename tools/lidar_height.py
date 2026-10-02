"""Unitree PointCloud2 ground-height fusion, independent of ROS imports.

Transforms are measured rigid transforms, never inferred from frame names.
Unknown cells remain unavailable. Each result uses one scan, without scan
accumulation or per-point motion compensation.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
import threading
from typing import Any

import cv2
import numpy as np


DEFAULT_LIDAR_TOPIC = "/unitree/slam_lidar/points1"
DEFAULT_IMU_TOPIC = "/unitree/slam_lidar/imu1"


@dataclass(frozen=True)
class LidarHeightConfig:
    max_age_sec: float = 0.5
    max_sync_sec: float = 0.15
    grid_resolution_m: float = 0.10
    max_up_m: float = 0.06
    max_down_m: float = 0.08
    max_reference_change_m: float = 0.06
    footprint_radius_m: float = 0.25
    seed_near_m: float = 0.5
    seed_far_m: float = 2.0
    seed_half_width_m: float = 0.5
    plane_threshold_m: float = 0.025
    min_plane_points: int = 30
    min_plane_ratio: float = 0.5
    max_ground_tilt_deg: float = 20.0
    min_cell_points: int = 3

    def validate(self) -> None:
        for name in ("max_age_sec", "max_sync_sec", "grid_resolution_m", "max_up_m",
                     "max_down_m", "max_reference_change_m", "footprint_radius_m", "seed_near_m", "seed_far_m",
                     "seed_half_width_m", "plane_threshold_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"lidar {name} must be finite and positive")
        if self.seed_far_m <= self.seed_near_m:
            raise ValueError("lidar seed_far_m must exceed seed_near_m")
        if not 0 < self.min_plane_ratio <= 1 or not 0 < self.max_ground_tilt_deg < 45:
            raise ValueError("invalid lidar plane ratio/tilt")
        if self.min_plane_points < 3 or self.min_cell_points < 1:
            raise ValueError("invalid lidar minimum support")


def stamp_ns(header: Any) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def parse_transform(value: str) -> np.ndarray | None:
    """Parse row-major target-from-source 4x4 matrix; blank requests ROS TF."""
    if not value.strip():
        return None
    values = [float(v) for v in value.replace(";", ",").split(",")]
    if len(values) != 16:
        raise ValueError("lidar transforms require 16 row-major matrix values")
    matrix = np.asarray(values, dtype=np.float64).reshape(4, 4)
    validate_transform(matrix)
    return matrix


def validate_transform(matrix: np.ndarray) -> None:
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("invalid lidar transform matrix")
    r = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(r), 1, atol=1e-5)):
        raise ValueError("lidar transform must be a rigid, right-handed transform")


def transform_matrix(transform: Any) -> np.ndarray:
    t, q = transform.translation, transform.rotation
    x, y, z, w = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError("invalid TF quaternion")
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    result = np.eye(4)
    result[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    result[:3, 3] = [t.x, t.y, t.z]
    validate_transform(result)
    return result


def cloud_xyz(message: Any) -> np.ndarray:
    """Respect field offsets, row padding, endian and the recorded 26-byte stride."""
    fields = {v.name: v for v in message.fields}
    kinds = {7: "f4", 8: "f8"}
    formats = []
    for name in ("x", "y", "z"):
        f = fields.get(name)
        if f is None or f.datatype not in kinds or f.count != 1:
            raise ValueError("PointCloud2 needs scalar floating-point x/y/z fields")
        formats.append((">" if message.is_bigendian else "<") + kinds[f.datatype])
    if (message.width < 0 or message.height < 1 or message.point_step < 1
            or message.row_step < message.width * message.point_step
            or len(message.data) < message.row_step * message.height):
        raise ValueError("invalid PointCloud2 layout")
    dtype = np.dtype(dict(names=["x", "y", "z"], formats=formats,
                         offsets=[fields[k].offset for k in ("x", "y", "z")],
                         itemsize=message.point_step))
    raw = np.ndarray((message.height, message.width), dtype=dtype,
                     buffer=bytes(message.data), strides=(message.row_step, message.point_step))
    xyz = np.column_stack([raw[k].reshape(-1) for k in ("x", "y", "z")])
    return xyz[np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 0.2)]


@dataclass(frozen=True)
class SensorSample:
    message: Any
    arrival_sec: float


class LidarInputs:
    """Bounded sensor histories, matched by acquisition time rather than arrival."""
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._clouds: deque[SensorSample] = deque(maxlen=12)
        self._imus: deque[SensorSample] = deque(maxlen=512)
        self._infos: deque[SensorSample] = deque(maxlen=8)

    def add(self, kind: str, message: Any, now: float) -> None:
        if kind != "info" and stamp_ns(message.header) <= 0:
            raise ValueError("lidar sensor timestamp must be positive")
        with self._lock:
            getattr(self, "_" + {"cloud": "clouds", "imu": "imus", "info": "infos"}[kind]).append(
                SensorSample(message, now)
            )

    def snapshot(self, camera_header: Any, now: float, config: LidarHeightConfig,
                 clock_ns: int | None = None) -> tuple[SensorSample, Any, np.ndarray]:
        target = stamp_ns(camera_header)
        if target <= 0:
            raise ValueError("lidar_camera_timestamp_invalid")
        with self._lock:
            clouds, imus, infos = tuple(self._clouds), tuple(self._imus), tuple(self._infos)
        if not clouds:
            raise ValueError("lidar_waiting_for_cloud")
        cloud = min(clouds, key=lambda s: abs(stamp_ns(s.message.header) - target))
        source = stamp_ns(cloud.message.header)
        if abs(source - target) / 1e9 > config.max_sync_sec:
            raise ValueError("lidar_camera_unsynchronized")
        if (now - cloud.arrival_sec > config.max_age_sec
                or (clock_ns is not None and not -config.max_sync_sec <=
                    (clock_ns - source) / 1e9 <= config.max_age_sec)):
            raise ValueError("lidar_stale")
        nearby = [s.message for s in imus
                  if abs(stamp_ns(s.message.header) - source) <= 150_000_000
                  and now - s.arrival_sec <= config.max_age_sec]
        if not nearby:
            raise ValueError("lidar_waiting_for_imu")
        if any(m.header.frame_id != cloud.message.header.frame_id for m in nearby):
            raise ValueError("lidar_imu_frame_mismatch")
        acc = np.median([[m.linear_acceleration.x, m.linear_acceleration.y,
                          m.linear_acceleration.z] for m in nearby], axis=0)
        norm = np.linalg.norm(acc)
        if not np.isfinite(acc).all() or not 5 <= norm <= 15:
            raise ValueError("lidar_imu_gravity_invalid")
        matching = [s for s in infos if s.message.header.frame_id == camera_header.frame_id]
        if not matching:
            raise ValueError("lidar_waiting_for_camera_info")
        return cloud, matching[-1].message, acc / norm


def fit_ground(points: np.ndarray, up: np.ndarray, config: LidarHeightConfig
               ) -> tuple[np.ndarray, float, float]:
    """Fit the near, narrow support surface, excluding sensor-height objects."""
    # base_link is the body frame; require a ground patch below the body.
    vertical = points @ up
    selected = ((points[:, 0] >= config.seed_near_m)
                & (points[:, 0] <= config.seed_far_m)
                & (np.abs(points[:, 1]) <= config.seed_half_width_m)
                & (vertical < -0.1) & (vertical > -1.5))
    seeds = points[selected]
    if len(seeds) < config.min_plane_points:
        raise ValueError("lidar_ground_unavailable")
    # Equal spatial weighting prevents the densest scan rings from dominating.
    _, ids = np.unique(np.floor(seeds / 0.04).astype(np.int32), axis=0, return_index=True)
    seeds = seeds[ids]
    if len(seeds) < config.min_plane_points:
        raise ValueError("lidar_ground_unavailable")
    rng = np.random.default_rng(0)
    if len(seeds) > 2000:
        seeds = seeds[rng.choice(len(seeds), 2000, replace=False)]
    best = None
    cosine = math.cos(math.radians(config.max_ground_tilt_deg))
    for _ in range(100):
        a, b, c = seeds[rng.choice(len(seeds), 3, replace=False)]
        n = np.cross(b-a, c-a)
        norm = np.linalg.norm(n)
        if norm < 1e-8:
            continue
        n /= norm
        if n @ up < 0:
            n = -n
        if n @ up < cosine:
            continue
        offset = -float(n @ a)
        inliers = np.abs(seeds @ n + offset) <= config.plane_threshold_m
        count = int(inliers.sum())
        if best is None or count > best[0]:
            best = count, inliers
    if best is None or best[0] < config.min_plane_points or best[0]/len(seeds) < config.min_plane_ratio:
        raise ValueError("lidar_ground_unreliable")
    support = seeds[best[1]]
    center = support.mean(axis=0)
    _, singular, vectors = np.linalg.svd(support-center, full_matrices=False)
    if singular[1] < 0.05:
        raise ValueError("lidar_ground_degenerate")
    normal = vectors[-1]
    if normal @ up < 0:
        normal = -normal
    if normal @ up < cosine:
        raise ValueError("lidar_ground_unreliable")
    return normal, -float(normal @ center), best[0]/len(seeds)


@dataclass
class HeightFusion:
    regions: dict[int, np.ndarray]
    x_values: np.ndarray
    y_values: np.ndarray
    gate: np.ndarray
    metrics: dict[str, Any] = field(default_factory=dict)
    camera_from_base: np.ndarray | None = None
    camera_matrix: np.ndarray | None = None
    distortion: np.ndarray | None = None

    def project_path(self, points_xy: np.ndarray) -> np.ndarray:
        normal = np.asarray(self.metrics["plane_normal_base"])
        if abs(normal[2]) < 1e-6:
            raise ValueError("lidar_ground_unreliable")
        z = -(points_xy @ normal[:2] + self.metrics["plane_offset_m"]) / normal[2]
        points = np.column_stack((points_xy, z))
        camera = points @ self.camera_from_base[:3, :3].T + self.camera_from_base[:3, 3]
        uv, _ = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), self.camera_matrix, self.distortion)
        uv = uv.reshape(-1, 2)
        uv[camera[:, 2] <= 0] = np.nan
        return uv


def fuse_height(selected_mask: np.ndarray, cloud: Any, camera_info: Any,
                image_shape: tuple[int, int], base_from_lidar: np.ndarray,
                camera_from_base: np.ndarray, up_lidar: np.ndarray,
                config: LidarHeightConfig, path_config: Any,
                mask_classes: tuple[int, ...],
                reference_plane: tuple[np.ndarray, float] | None = None) -> HeightFusion:
    validate_transform(base_from_lidar)
    validate_transform(camera_from_base)
    if (camera_info.height, camera_info.width) != tuple(image_shape):
        raise ValueError("lidar_camera_info_size_mismatch")
    k = np.asarray(camera_info.k, dtype=float).reshape(3, 3)
    distortion = np.asarray(camera_info.d, dtype=float)
    if (not np.isfinite(k).all() or not np.isfinite(distortion).all()
            or k[0, 0] <= 0 or k[1, 1] <= 0
            or camera_info.distortion_model not in ("plumb_bob", "rational_polynomial")
            or len(distortion) not in (0, 4, 5, 8, 12, 14)):
        raise ValueError("lidar_camera_calibration_invalid")
    raw = cloud_xyz(cloud)
    points = raw @ base_from_lidar[:3, :3].T + base_from_lidar[:3, 3]
    up = base_from_lidar[:3, :3] @ up_lidar
    norm = np.linalg.norm(up)
    if not np.isfinite(up).all() or norm < 1e-6:
        raise ValueError("lidar_imu_gravity_invalid")
    up /= norm
    if up[2] < 0.5:
        raise ValueError("lidar_gravity_axis_invalid")
    normal, offset, plane_ratio = fit_ground(points, up, config)
    if reference_plane is not None:
        old_normal, old_offset = reference_plane
        previous_alignment = float(old_normal @ up)
        if previous_alignment <= 1e-6:
            raise ValueError("lidar_ground_reference_jump")
        previous_height = old_offset / previous_alignment
        current_height = offset / float(normal @ up)
        if abs(current_height-previous_height) > config.max_reference_change_m:
            raise ValueError("lidar_ground_reference_jump")
    height = (points @ normal + offset) / float(normal @ up)
    # Use only the local collision volume. Overhead returns are not curb height.
    span = path_config.search_half_width_m
    res = config.grid_resolution_m
    nx = int(math.ceil(path_config.far_distance_m / res)) + 1
    ny = int(math.ceil(2*span / res)) + 1
    if nx*ny > 250_000:
        raise ValueError("lidar_grid_too_large")
    keep = ((points[:, 0] >= 0) & (points[:, 0] <= path_config.far_distance_m)
            & (np.abs(points[:, 1]) <= span) & (height > -1.0) & (height < 1.0))
    points, height = points[keep], height[keep]
    gx = np.floor(points[:, 0]/res).astype(int)
    gy = np.floor((span-points[:, 1])/res).astype(int)
    cell_ids = gx*ny+gy
    order = np.argsort(cell_ids)
    sorted_ids = cell_ids[order]
    unique, starts, counts = np.unique(sorted_ids, return_index=True, return_counts=True)
    allowed = np.zeros(nx*ny, dtype=np.uint8)
    for cell, start, count in zip(unique, starts, counts):
        if count < config.min_cell_points:
            continue
        values = height[order[start:start+count]]
        lo, hi = np.percentile(values, (10, 90))
        if lo >= -config.max_down_m and hi <= config.max_up_m:
            allowed[cell] = 1
    allowed = allowed.reshape(nx, ny)
    radius = int(math.ceil(config.footprint_radius_m/res))
    footprint = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*radius+1, 2*radius+1))
    clearance = cv2.erode(allowed, footprint, borderType=cv2.BORDER_CONSTANT, borderValue=0)

    # Project actual 3-D returns onto the raw, distorted camera image.
    camera_points = points @ camera_from_base[:3, :3].T + camera_from_base[:3, 3]
    visible = camera_points[:, 2] > 0.01
    projected = np.full((len(points), 2), -1.0)
    if visible.any():
        uv, _ = cv2.projectPoints(camera_points[visible], np.zeros(3), np.zeros(3), k, distortion)
        projected[visible] = uv.reshape(-1, 2)
    visible &= (np.isfinite(projected).all(axis=1)
                & (projected[:, 0] >= 0) & (projected[:, 0] < camera_info.width)
                & (projected[:, 1] >= 0) & (projected[:, 1] < camera_info.height))
    projected[~visible] = -1
    mh, mw = selected_mask.shape
    px = np.floor(projected[:, 0]*mw/camera_info.width).astype(int)
    py = np.floor(projected[:, 1]*mh/camera_info.height).astype(int)
    visible &= (px >= 0) & (px < mw) & (py >= 0) & (py < mh)
    pixel_ids = py[visible]*mw+px[visible]
    nearest = np.full(mh*mw, np.inf)
    np.minimum.at(nearest, pixel_ids, camera_points[visible, 2])
    # Keep camera-visible returns; background behind a foreground object must
    # not inherit that object's semantic label at the same mask pixel.
    ids = np.flatnonzero(visible)
    visible[ids] &= camera_points[ids, 2] <= nearest[pixel_ids] + 0.10
    labels = np.zeros(len(points), dtype=np.uint8)
    labels[visible] = selected_mask[py[visible], px[visible]]
    xs = np.linspace(path_config.near_distance_m, path_config.far_distance_m,
                     path_config.bev_height_px, dtype=np.float32)
    ys = np.linspace(span, -span, path_config.bev_width_px, dtype=np.float32)
    bx = np.minimum(np.floor(xs/res).astype(int), nx-1)
    by = np.minimum(np.floor((span-ys)/res).astype(int), ny-1)
    gate = clearance[np.ix_(bx, by)].astype(bool)
    regions = {}
    for mask_class in mask_classes:
        semantic = ((labels == 1) | (labels == 2)) if mask_class == 0 else labels == mask_class
        hits = np.bincount(cell_ids[semantic], minlength=nx*ny).reshape(nx, ny)
        region = ((hits >= config.min_cell_points) & (clearance > 0)).astype(np.uint8)
        # Keep only corridors connected to the near current surface.
        _, components = cv2.connectedComponents(region, connectivity=4)
        seed_x = np.arange(nx)*res
        seed_y = span-np.arange(ny)*res
        seed = ((seed_x[:, None] >= config.seed_near_m)
                & (seed_x[:, None] <= config.seed_far_m)
                & (np.abs(seed_y[None, :]) <= config.seed_half_width_m))
        ids = np.unique(components[seed & (region > 0)])
        ids = ids[ids != 0]
        connected = np.isin(components, ids) & (clearance > 0)
        regions[mask_class] = (connected[np.ix_(bx, by)]*255).astype(np.uint8)
    return HeightFusion(regions, xs, ys, gate, dict(
        cloud_stamp_ns=stamp_ns(cloud.header), cloud_frame_id=cloud.header.frame_id,
        point_count=len(raw), plane_normal_base=normal.tolist(), plane_offset_m=offset,
        plane_inlier_ratio=plane_ratio, allowed_bev_ratio=float(gate.mean()),
        max_up_m=config.max_up_m, max_down_m=config.max_down_m,
    ), camera_from_base, k, distortion)
