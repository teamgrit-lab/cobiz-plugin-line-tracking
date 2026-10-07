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
MAX_PATH_UNAVAILABLE_INFERENCES = 5
MAX_PATH_RECOVERY_INFERENCES = 3
DRIVE_STOP_CHECKS = (
    "camera_stale",
    "inference_stale",
    "camera_timestamp_invalid",
    "path_unavailable",
    "path_loss_limit",
    "low_confidence",
    "lateral_target",
)


@dataclass(frozen=True)
class DriveConfig:
    max_forward_mps: float = 0.50
    max_yaw_rps: float = 0.18
    heading_gain: float = 1.0
    lookahead_m: float = 4.0
    min_confidence: float = 0.49
    max_target_heading_deg: float = 60.0
    max_camera_age_sec: float = 5.00
    max_inference_age_sec: float = 5.00
    stop_on_camera_stale: bool = False
    stop_on_inference_stale: bool = False
    stop_on_camera_timestamp_invalid: bool = False
    stop_on_path_unavailable: bool = False
    stop_on_path_loss_limit: bool = False
    stop_on_low_confidence: bool = False
    stop_on_lateral_target: bool = False
    path_loss_recovery_enabled: bool = True
    path_recovery_wait_sec: float = 1.0
    path_recovery_yaw_rps: float = 0.18
    path_recovery_confirm_frames: int = 2
    path_recovery_first_direction: str = "left"

    def validate(self) -> None:
        positive = (
            self.max_forward_mps,
            self.max_yaw_rps,
            self.heading_gain,
            self.lookahead_m,
            self.max_target_heading_deg,
            self.max_camera_age_sec,
            self.max_inference_age_sec,
            self.path_recovery_wait_sec,
            self.path_recovery_yaw_rps,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("drive limits must be finite and positive")
        if self.max_forward_mps > MAX_FORWARD_MPS_HARD_LIMIT:
            raise ValueError("max_forward_mps must be at most 1.0 m/s")
        if self.max_target_heading_deg >= 90.0:
            raise ValueError("max_target_heading_deg must be in (0, 90)")
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        if any(
            type(getattr(self, "stop_on_" + name)) is not bool
            for name in DRIVE_STOP_CHECKS
        ):
            raise ValueError("stop-check switches must be boolean values")
        if type(self.path_loss_recovery_enabled) is not bool:
            raise ValueError("path_loss_recovery_enabled must be a boolean value")
        if self.path_recovery_yaw_rps > self.max_yaw_rps:
            raise ValueError("path_recovery_yaw_rps must not exceed max_yaw_rps")
        if (type(self.path_recovery_confirm_frames) is not int
                or self.path_recovery_confirm_frames < 2):
            raise ValueError("path_recovery_confirm_frames must be an integer >= 2")
        if self.path_recovery_first_direction not in ("left", "right"):
            raise ValueError("path_recovery_first_direction must be left or right")

    @property
    def max_lateral_target_m(self) -> float:
        """Equivalent lateral limit at the configured lookahead distance."""

        return self.lookahead_m * math.tan(math.radians(self.max_target_heading_deg))


@dataclass(frozen=True)
class DriveDecision:
    vx: float
    vy: float
    yaw_rate: float
    reason: str

    @classmethod
    def stop(cls, reason: str) -> DriveDecision:
        return cls(0.0, 0.0, 0.0, reason)


class PathLossRecovery:
    """Wait after the path-loss stop, then scan +/-90 degrees from that heading.

    Angles integrate *published* yaw commands, not timer evaluations. This is
    open-loop motion; wheel slip or a controller rejecting Move cannot be measured.
    Each endpoint waits for an image captured there before reversing direction.
    """

    def __init__(self, config: DriveConfig, *, output_hz: float) -> None:
        config.validate()
        self.config = config
        self.period_sec = 1.0 / output_hz
        self.reset()

    def reset(self) -> None:
        self.phase = "idle"
        self.armed = False
        self.angle_rad = 0.0
        self.wait_elapsed_sec = 0.0
        self.confirmed_frames = 0
        self.confirm_since: float | None = None
        self.confirm_source_at: float | None = None
        self.endpoint_since: float | None = None
        self.last_inference: int | None = None
        self.command_at: float | None = None
        self.command = DriveDecision.stop("task_idle")

    @property
    def active(self) -> bool:
        return self.phase != "idle"

    def _advance(self, now: float) -> None:
        if self.command_at is None:
            return
        elapsed = now - self.command_at
        self.command_at = now
        if not self.active or elapsed <= 0:
            return
        if self.command.reason == "path_recovery_waiting":
            self.wait_elapsed_sec += elapsed
        if self.command.reason in (
            "path_recovery_scan_left", "path_recovery_scan_right", "path_recovery_return"
        ):
            # A delayed control loop cannot establish the robot's scan angle.
            # End this attempt instead of extending blind rotation after a stall.
            if elapsed > 2.0 * self.period_sec + 1e-6:
                self.phase = "exhausted"
            else:
                self.angle_rad += self.command.yaw_rate * elapsed

    def record_command(self, decision: DriveDecision, *, now: float) -> None:
        """Call only after publication, including zero/AprilTag/shutdown overrides."""
        self._advance(now)
        self.command = decision
        self.command_at = now
        if decision.reason in ("tracking", "tracking_slow_turn") and decision.vx > 0:
            self.armed = True

    def decide(
        self, normal: DriveDecision, *, now: float, inference_id: int,
        inference_source_at: float | None, sensors_ready: bool,
        sensor_reason: str, path_confidence: float | None,
    ) -> DriveDecision:
        self._advance(now)
        if not self.config.path_loss_recovery_enabled:
            return normal
        if not self.active:
            if normal.reason != "path_recovery_waiting" or not self.armed:
                return normal
            self.phase = "waiting"
            self.wait_elapsed_sec = 0.0
            self.angle_rad = 0.0
            self.last_inference = inference_id

        # Explicit stops, branch guards and obstacle exclusions take priority.
        if normal.reason not in (
            "tracking", "tracking_slow_turn", "tracking_path_hold", "tracking_path_recovery",
            "path_recovery_waiting", "waiting_for_path",
        ):
            self.confirmed_frames = 0
            self.confirm_since = None
            self.confirm_source_at = None
            self.last_inference = inference_id
            return normal
        if not sensors_ready:
            self.confirmed_frames = 0
            self.confirm_since = None
            self.confirm_source_at = None
            self.last_inference = inference_id
            return DriveDecision.stop(sensor_reason)

        usable = (normal.reason in ("tracking", "tracking_slow_turn")
                  and path_confidence is not None and math.isfinite(path_confidence)
                  and path_confidence >= self.config.min_confidence)
        if inference_id != self.last_inference:
            consecutive = self.last_inference is not None and inference_id == self.last_inference + 1
            self.last_inference = inference_id
            if not usable:
                self.confirmed_frames = 0
                self.confirm_since = None
                self.confirm_source_at = None
            elif self.confirm_since is None or not consecutive:
                self.confirm_since = now
                self.confirmed_frames = 1
                self.confirm_source_at = inference_source_at
            elif (inference_source_at is not None and inference_source_at >= self.confirm_since
                  and (self.confirm_source_at is None or inference_source_at > self.confirm_source_at)):
                self.confirmed_frames += 1
                self.confirm_source_at = inference_source_at
        if usable and self.confirmed_frames:
            if self.confirmed_frames >= self.config.path_recovery_confirm_frames:
                self.phase = "idle"
                self.confirmed_frames = 0
                self.confirm_since = None
                self.confirm_source_at = None
                return normal
            return DriveDecision.stop("path_recovery_confirming")

        if self.phase == "waiting":
            if self.wait_elapsed_sec + 1e-9 < self.config.path_recovery_wait_sec:
                return DriveDecision.stop("path_recovery_waiting")
            self.phase = "scan_" + self.config.path_recovery_first_direction
        if self.phase in ("hold_left", "hold_right"):
            if inference_source_at is None or inference_source_at < self.endpoint_since:
                return DriveDecision.stop("path_recovery_" + self.phase)
            first = self.config.path_recovery_first_direction
            second = "right" if first == "left" else "left"
            self.phase = "scan_" + second if self.phase == "hold_" + first else "return"
        targets = {"scan_left": math.pi / 2, "scan_right": -math.pi / 2, "return": 0.0}
        if self.phase in targets:
            remaining = targets[self.phase] - self.angle_rad
            # Returning from the second side reverses with the configured order.
            directions = {"scan_left": 1, "scan_right": -1,
                          "return": 1 if self.config.path_recovery_first_direction == "left" else -1}
            if remaining * directions[self.phase] <= 1e-6:
                self.phase = {"scan_left": "hold_left", "scan_right": "hold_right",
                              "return": "exhausted"}[self.phase]
                self.endpoint_since = now
                return DriveDecision.stop("path_recovery_" + self.phase)
            yaw = math.copysign(min(self.config.path_recovery_yaw_rps,
                                    abs(remaining) / self.period_sec), remaining)
            return DriveDecision(0.0, 0.0, yaw, "path_recovery_" + self.phase)
        return DriveDecision.stop("path_recovery_exhausted")


def decide_drive(
    path: SmoothedPath | None,
    *,
    camera_age_sec: float | None,
    inference_age_sec: float | None,
    config: DriveConfig,
    last_valid_yaw_rate: float | None = None,
    last_valid_forward_mps: float | None = None,
    path_unavailable_inferences: int = 0,
    obstacle_stop_reason: str | None = None,
    path_recovery_inferences: int | None = None,
) -> DriveDecision:
    """Slow forward motion when heading demand exceeds available yaw rate.

    The caller counts consecutive unavailable results at inference completion;
    repeatedly evaluating the same result must not advance that count.
    Semantic path loss retains the last valid forward speed with zero yaw for
    two failed inferences, then stops on the third. The caller's PathLossRecovery
    owns waiting from that stop, scanning and confirmation. Sensor failures retain
    the existing command-hold policy when no semantic recovery count is supplied.
    """

    config.validate()
    if obstacle_stop_reason is not None:
        return DriveDecision.stop(obstacle_stop_reason)
    if path is not None and path.stop_reason is not None:
        return DriveDecision.stop(path.stop_reason)
    for name, age, maximum in (
        ("camera", camera_age_sec, config.max_camera_age_sec),
        ("inference", inference_age_sec, config.max_inference_age_sec),
    ):
        if getattr(config, "stop_on_" + name + "_stale") and (
            age is None or not math.isfinite(age) or age < 0.0 or age > maximum
        ):
            return DriveDecision.stop(f"{name}_stale")
    if path is not None and config.stop_on_low_confidence and (
        not math.isfinite(path.confidence)
        or path.confidence < config.min_confidence
    ):
        return DriveDecision.stop("path_low_confidence")
    lateral = path_target_lateral(path, config.lookahead_m)
    if lateral is None:
        if config.path_loss_recovery_enabled and path_recovery_inferences is not None:
            if config.stop_on_path_unavailable:
                return DriveDecision.stop("path_unavailable")
            if (last_valid_forward_mps is None or not math.isfinite(last_valid_forward_mps)
                    or last_valid_forward_mps <= 0):
                return DriveDecision.stop("waiting_for_path")
            if path_recovery_inferences >= MAX_PATH_RECOVERY_INFERENCES:
                return DriveDecision.stop("path_recovery_waiting")
            return DriveDecision(min(last_valid_forward_mps, config.max_forward_mps),
                                 0.0, 0.0, "tracking_path_recovery")
        if config.stop_on_path_unavailable or (
            config.stop_on_path_loss_limit
            and path_unavailable_inferences >= MAX_PATH_UNAVAILABLE_INFERENCES
        ):
            return DriveDecision.stop("path_unavailable")
        held_speed = (
            config.max_forward_mps
            if last_valid_forward_mps is None
            else last_valid_forward_mps
        )
        if (
            last_valid_yaw_rate is not None
            and math.isfinite(last_valid_yaw_rate)
            and math.isfinite(held_speed)
            and held_speed >= 0.0
        ):
            return DriveDecision(
                min(held_speed, config.max_forward_mps),
                0.0,
                float(np.clip(last_valid_yaw_rate, -config.max_yaw_rps, config.max_yaw_rps)),
                "tracking_path_hold",
            )
        # With no previous usable command there is nothing to hold. Disabling
        # automatic stops must not invent an initial heading or forward speed.
        return DriveDecision.stop("waiting_for_path")
    if (
        config.stop_on_lateral_target
        and abs(lateral) > config.max_lateral_target_m
    ):
        return DriveDecision.stop("path_lateral_target_large")
    heading = math.atan2(lateral, config.lookahead_m)
    requested_yaw = config.heading_gain * heading
    yaw_rate = float(
        np.clip(requested_yaw, -config.max_yaw_rps, config.max_yaw_rps)
    )
    # Once yaw saturates, scale forward speed by the same ratio. This preserves
    # the requested yaw/speed ratio instead of widening the commanded turn.
    speed_scale = config.max_yaw_rps / max(config.max_yaw_rps, abs(requested_yaw))
    return DriveDecision(
        config.max_forward_mps * speed_scale,
        0.0,
        yaw_rate,
        "tracking_slow_turn" if speed_scale < 1.0 else "tracking",
    )


def path_target_lateral(path: SmoothedPath | None, lookahead_m: float) -> float | None:
    """Use finite points in forward order, clamping to an available endpoint.

    A single point is sufficient. Duplicate forward distances use their first
    finite point. No numeric coordinates means there is no target to track.
    """

    if path is None:
        return None
    try:
        points = np.asarray(path.points_xy, dtype=np.float64)
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
