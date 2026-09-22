"""Fail-closed, low-speed drive decisions from a calibrated Swin-L local path.

This module has no ROS dependency so the control and stop gates can be tested
without a robot. It does not establish camera extrinsic calibration.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from local_path import SmoothedPath


MAX_FORWARD_MPS_HARD_LIMIT = 1.00


@dataclass(frozen=True)
class DriveConfig:
    max_forward_mps: float = 0.50
    max_yaw_rps: float = 0.18
    heading_gain: float = 1.0
    lookahead_m: float = 4.0
    min_confidence: float = 0.49
    max_lateral_target_m: float = 0.75
    max_camera_age_sec: float = 5.00
    max_inference_age_sec: float = 5.00
    max_path_age_sec: float = 0.45

    def validate(self) -> None:
        positive = (
            self.max_forward_mps,
            self.max_yaw_rps,
            self.heading_gain,
            self.lookahead_m,
            self.max_lateral_target_m,
            self.max_camera_age_sec,
            self.max_inference_age_sec,
            self.max_path_age_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("drive limits must be finite and positive")
        if self.max_forward_mps > MAX_FORWARD_MPS_HARD_LIMIT:
            raise ValueError("max_forward_mps must be at most 1.0 m/s")
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0, 1]")


@dataclass(frozen=True)
class DriveDecision:
    vx: float
    vy: float
    yaw_rate: float
    reason: str

    @classmethod
    def stop(cls, reason: str) -> DriveDecision:
        return cls(0.0, 0.0, 0.0, reason)


def decide_drive(
    path: SmoothedPath | None,
    *,
    camera_age_sec: float | None,
    inference_age_sec: float | None,
    config: DriveConfig,
) -> DriveDecision:
    """Only permit low-speed motion with fresh camera/path inputs."""

    config.validate()
    for name, age, maximum in (
        ("camera", camera_age_sec, config.max_camera_age_sec),
        ("inference", inference_age_sec, config.max_inference_age_sec),
    ):
        if age is None or not math.isfinite(age) or age < 0.0 or age > maximum:
            return DriveDecision.stop(f"{name}_stale")
    if path is None:
        return DriveDecision.stop("path_unavailable")
    if (
        not math.isfinite(path.age_sec)
        or path.age_sec < 0.0
        or path.age_sec > config.max_path_age_sec
    ):
        return DriveDecision.stop("path_stale")
    if not math.isfinite(path.confidence) or path.confidence < config.min_confidence:
        return DriveDecision.stop("path_low_confidence")
    points = np.asarray(path.points_xy, dtype=np.float64)
    if (
        points.ndim != 2
        or points.shape[1] != 2
        or points.shape[0] < 2
        or not np.all(np.isfinite(points))
        or not np.all(np.diff(points[:, 0]) > 0.0)
        or points[0, 0] <= 0.0
        or points[0, 0] > config.lookahead_m
        or points[-1, 0] < config.lookahead_m
    ):
        return DriveDecision.stop("path_geometry_invalid")
    lateral = float(np.interp(config.lookahead_m, points[:, 0], points[:, 1]))
    if abs(lateral) > config.max_lateral_target_m:
        return DriveDecision.stop("path_lateral_target_large")
    heading = math.atan2(lateral, config.lookahead_m)
    yaw_rate = float(
        np.clip(config.heading_gain * heading, -config.max_yaw_rps, config.max_yaw_rps)
    )
    return DriveDecision(config.max_forward_mps, 0.0, yaw_rate, "tracking")
