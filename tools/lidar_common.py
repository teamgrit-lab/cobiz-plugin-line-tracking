"""Measured sensor transforms and PointCloud2 decoding; no terrain estimation."""

from __future__ import annotations
from typing import Any
import numpy as np

def _a2_camera_from_base() -> np.ndarray:
    """A2 URDF front camera optical pose (target camera, source base)."""
    return np.array(
        [
            [0.0, -1.0, 0.0, 0.0336],
            [0.0, 0.0, -1.0, 0.0525],
            [1.0, 0.0, 0.0, -0.3381],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def a2_livox_front_transforms(
    lidar_frame: str, base_frame: str, camera_frame: str
) -> tuple[np.ndarray, np.ndarray]:
    """Measured MID360 + A2 front camera mount, including forward tilt.

    Source: teamgrit-slam/slam/src/grit_slam/config/profiles/
    teamgrit_a2_livox.yaml (grit-lio, file revision 58e074b2).
    The 2026-08-31 checkerboard calibration is independently carried by
    fast_livo/config/a2_livox.yaml. This is not the placeholder X5/360 rig.
    """
    if (
        lidar_frame != "livox_frame"
        or base_frame != "base_link"
        or camera_frame != "camera_optical_frame"
    ):
        raise ValueError("lidar_calibration_frame_mismatch")
    base_from_lidar = np.array(
        [
            [0.012982560, -0.856344989, 0.516240944, 0.373968],
            [0.999608476, -0.001682648, -0.027929602, 0.002419],
            [0.024786027, 0.516401421, 0.855987865, 0.190767],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    validate_transform(base_from_lidar)
    return base_from_lidar, _a2_camera_from_base()


def stamp_ns(header: Any) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def validate_transform(matrix: np.ndarray) -> None:
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("invalid lidar transform matrix")
    r = matrix[:3, :3]
    if (
        not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6)
        or not np.allclose(r.T @ r, np.eye(3), atol=1e-5)
        or not np.isclose(np.linalg.det(r), 1, atol=1e-5)
    ):
        raise ValueError("lidar transform must be a rigid, right-handed transform")


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
    if (
        message.width < 0
        or message.height < 1
        or message.point_step < 1
        or message.row_step < message.width * message.point_step
        or len(message.data) < message.row_step * message.height
    ):
        raise ValueError("invalid PointCloud2 layout")
    dtype = np.dtype(
        dict(
            names=["x", "y", "z"],
            formats=formats,
            offsets=[fields[k].offset for k in ("x", "y", "z")],
            itemsize=message.point_step,
        )
    )
    raw = np.ndarray(
        (message.height, message.width),
        dtype=dtype,
        buffer=bytes(message.data),
        strides=(message.row_step, message.point_step),
    )
    xyz = np.column_stack([raw[k].reshape(-1) for k in ("x", "y", "z")])
    return xyz[np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 1e-6)]
