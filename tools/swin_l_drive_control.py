"""Low-speed path tracking with sensor guards and configurable path stops.

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
    stop_on_low_confidence: bool = True
    stop_on_lateral_target: bool = True
    bypass_path_stops: bool = False

    def validate(self) -> None:
        positive = (
            self.max_forward_mps,
            self.max_yaw_rps,
            self.heading_gain,
            self.lookahead_m,
            self.max_lateral_target_m,
            self.max_camera_age_sec,
            self.max_inference_age_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("drive limits must be finite and positive")
        if self.max_forward_mps > MAX_FORWARD_MPS_HARD_LIMIT:
            raise ValueError("max_forward_mps must be at most 1.0 m/s")
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        if any(
            type(enabled) is not bool
            for enabled in (
                self.stop_on_low_confidence,
                self.stop_on_lateral_target,
                self.bypass_path_stops,
            )
        ):
            raise ValueError("stop-check switches must be boolean values")


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
    last_valid_yaw_rate: float | None = None,
) -> DriveDecision:
    """Track a path or optionally hold its last yaw while sensor inputs stay fresh."""

    config.validate()
    for name, age, maximum in (
        ("camera", camera_age_sec, config.max_camera_age_sec),
        ("inference", inference_age_sec, config.max_inference_age_sec),
    ):
        if age is None or not math.isfinite(age) or age < 0.0 or age > maximum:
            return DriveDecision.stop(f"{name}_stale")
    if path is not None and not config.bypass_path_stops and (
        not math.isfinite(path.confidence)
        or (config.stop_on_low_confidence and path.confidence < config.min_confidence)
    ):
        return DriveDecision.stop("path_low_confidence")
    lateral = (
        _target_lateral(path.points_xy, config.lookahead_m) if path is not None else None
    )
    if lateral is None:
        if (
            config.bypass_path_stops
            and last_valid_yaw_rate is not None
            and math.isfinite(last_valid_yaw_rate)
        ):
            return DriveDecision(
                config.max_forward_mps,
                0.0,
                float(np.clip(last_valid_yaw_rate, -config.max_yaw_rps, config.max_yaw_rps)),
                "tracking_path_hold",
            )
        return DriveDecision.stop("path_unavailable")
    if (
        not config.bypass_path_stops
        and config.stop_on_lateral_target
        and abs(lateral) > config.max_lateral_target_m
    ):
        return DriveDecision.stop("path_lateral_target_large")
    heading = math.atan2(lateral, config.lookahead_m)
    yaw_rate = float(
        np.clip(config.heading_gain * heading, -config.max_yaw_rps, config.max_yaw_rps)
    )
    return DriveDecision(config.max_forward_mps, 0.0, yaw_rate, "tracking")


def _target_lateral(points_xy: np.ndarray, lookahead_m: float) -> float | None:
    """Use finite points in forward order, clamping to an available endpoint.

    A single point is sufficient. Duplicate forward distances use their first
    finite point. No numeric coordinates means there is no target to track.
    """

    try:
        points = np.asarray(points_xy, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if points.ndim != 2 or points.shape[1] != 2:
        return None
    points = points[np.all(np.isfinite(points), axis=1)]
    if points.shape[0] == 0:
        return None
    forward, indices = np.unique(points[:, 0], return_index=True)
    lateral = float(np.interp(lookahead_m, forward, points[indices, 1]))
    return lateral if math.isfinite(lateral) else None
