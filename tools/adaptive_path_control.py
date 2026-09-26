"""Curvature/age speed regulation and camera-feedback turn alignment.

No odometry is consumed. Turns are bounded pulses separated by zero commands
and new images captured after settling. The path is not a collision map: this
controller cannot certify footprint clearance or compensate camera motion.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading

import numpy as np

from local_path import SmoothedPath
from swin_l_drive_control import DriveConfig, DriveDecision


@dataclass(frozen=True)
class AdaptiveControlConfig:
    enabled: bool = True
    slow_age_sec: float = 0.50
    stop_age_sec: float = 1.20
    curve_slow_radius_m: float = 4.0
    turn_enter_deg: float = 15.0
    turn_exit_deg: float = 5.0
    turn_stop_sec: float = 0.60
    turn_pulse_sec: float = 0.35
    turn_settle_sec: float = 0.15
    turn_timeout_sec: float = 30.0
    turn_confirm_frames: int = 2
    max_accel_mps2: float = 0.30

    def validate(self) -> None:
        positive = (
            self.slow_age_sec,
            self.stop_age_sec,
            self.curve_slow_radius_m,
            self.turn_enter_deg,
            self.turn_exit_deg,
            self.turn_stop_sec,
            self.turn_pulse_sec,
            self.turn_settle_sec,
            self.turn_timeout_sec,
            self.max_accel_mps2,
        )
        if not all(math.isfinite(value) and value > 0 for value in positive):
            raise ValueError("adaptive control limits must be positive and finite")
        if type(self.enabled) is not bool:
            raise ValueError("adaptive control enabled must be boolean")
        if self.slow_age_sec >= self.stop_age_sec:
            raise ValueError("slow_age_sec must be less than stop_age_sec")
        if not self.turn_exit_deg < self.turn_enter_deg < 90.0:
            raise ValueError("turn angles require 0 < exit < enter < 90 degrees")
        if self.turn_pulse_sec > 0.5:
            raise ValueError("turn pulses without odometry must be at most 0.5 seconds")
        if (
            self.turn_timeout_sec
            <= self.turn_stop_sec + self.turn_pulse_sec + self.turn_settle_sec
        ):
            raise ValueError("turn timeout must exceed stop, pulse and settle time")
        if type(self.turn_confirm_frames) is not int or self.turn_confirm_frames < 2:
            raise ValueError("turn confirmation requires at least two distinct frames")


def path_geometry(
    path: SmoothedPath, lookahead_m: float
) -> tuple[float, float, float, float] | None:
    """Return target lateral, bearing, pursuit curvature and peak path curvature."""
    try:
        points = np.asarray(path.points_xy, dtype=np.float64)
    except (ValueError, TypeError):
        return None
    if points.ndim != 2 or points.shape[1] != 2 or not np.all(np.isfinite(points)):
        return None
    points = points[points[:, 0] > 0]
    if len(points) < 2:
        return None
    _, indices = np.unique(points[:, 0], return_index=True)
    points = points[indices]
    if len(points) < 2:
        return None
    x = float(np.clip(lookahead_m, points[0, 0], points[-1, 0]))
    y = float(np.interp(x, points[:, 0], points[:, 1]))
    heading = math.atan2(y, x)
    pursuit = 2.0 * y / (x * x + y * y)
    peak = abs(pursuit)
    # Three-point circumcircle curvature; inspect the visible path ahead so
    # speed can fall before the lookahead point reaches a bend.
    if len(points) >= 3:
        a = points[1:-1] - points[:-2]
        b = points[2:] - points[1:-1]
        c = points[2:] - points[:-2]
        denominator = (
            np.linalg.norm(a, axis=1)
            * np.linalg.norm(b, axis=1)
            * np.linalg.norm(c, axis=1)
        )
        valid = denominator > 1e-9
        if np.any(valid):
            cross = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
            peak = max(
                peak, float(np.max(2.0 * np.abs(cross[valid]) / denominator[valid]))
            )
    return y, heading, pursuit, peak


class AdaptivePathController:
    def __init__(self, drive: DriveConfig, config: AdaptiveControlConfig):
        drive.validate()
        config.validate()
        self.drive, self.config = drive, config
        # Camera conversion faults can reset the controller from the inference
        # worker while the ROS timer is evaluating a command.
        self._lock = threading.RLock()
        self.reset()

    def reset(self) -> None:
        with self._lock:
            self._reset()

    def _reset(self) -> None:
        self.phase = "tracking"
        self.turn_started: float | None = None
        self.phase_until = 0.0
        self.capture_after = 0.0
        self.seen_frame: int | None = None
        self.confirm_direction: int | None = None
        self.confirm_heading_deg: float | None = None
        self.confirmations = 0
        self.pulse_yaw = 0.0
        self.last_at: float | None = None
        self.last_vx = 0.0
        self.heading_deg: float | None = None
        self.curvature: float | None = None
        self.source_age: float | None = None
        self.last_decision = DriveDecision.stop("control_inactive")

    def metrics(self) -> dict:
        with self._lock:
            return self._metrics()

    def _metrics(self) -> dict:
        return {
            "enabled": self.config.enabled,
            "phase": self.phase,
            "target_heading_deg": self.heading_deg,
            "peak_curvature_per_m": self.curvature,
            "source_age_sec": self.source_age,
            "confirmations": self.confirmations,
            "command_forward_mps": self.last_decision.vx,
            "command_yaw_deg_sec": math.degrees(self.last_decision.yaw_rate),
        }

    def _emit(self, decision: DriveDecision, now: float) -> DriveDecision:
        self.last_vx, self.last_at = decision.vx, now
        self.last_decision = decision
        return decision

    def _stop(self, reason: str, now: float) -> DriveDecision:
        return self._emit(DriveDecision.stop(reason), now)

    def _wait_after_pulse(self, now: float) -> DriveDecision:
        self.phase = "waiting_frame"
        self.capture_after = now + self.config.turn_settle_sec
        self.confirmations = 0
        self.confirm_direction = None
        self.confirm_heading_deg = None
        return self._stop("turn_waiting_frame", now)

    def update(
        self,
        path: SmoothedPath | None,
        *,
        now: float,
        camera_age_sec: float | None,
        inference_age_sec: float | None,
        inference_id: int,
        motion_allowed: bool = True,
    ) -> DriveDecision:
        with self._lock:
            return self._update(
                path,
                now=now,
                camera_age_sec=camera_age_sec,
                inference_age_sec=inference_age_sec,
                inference_id=inference_id,
                motion_allowed=motion_allowed,
            )

    def _update(
        self,
        path: SmoothedPath | None,
        *,
        now: float,
        camera_age_sec: float | None,
        inference_age_sec: float | None,
        inference_id: int,
        motion_allowed: bool,
    ) -> DriveDecision:
        if not math.isfinite(now) or (self.last_at is not None and now < self.last_at):
            self.reset()
            return DriveDecision.stop("control_clock_invalid")
        if not motion_allowed:
            self.reset()
            return self._stop("control_inactive", now)
        # A failed recovery stays stopped until the task/controller is reset.
        if self.phase == "blocked":
            return self._stop("turn_timeout", now)
        for name, age, maximum in (
            ("camera", camera_age_sec, self.drive.max_camera_age_sec),
            ("inference", inference_age_sec, self.drive.max_inference_age_sec),
        ):
            if age is None or not math.isfinite(age) or age < 0 or age > maximum:
                self.reset()
                return self._stop(f"{name}_stale", now)
        self.source_age = max(camera_age_sec, inference_age_sec)
        if (
            self.turn_started is not None
            and now - self.turn_started >= self.config.turn_timeout_sec
        ):
            self.phase = "blocked"
            return self._stop("turn_timeout", now)
        if self.source_age >= self.config.stop_age_sec:
            # Never hold a yaw command across stale perception, including when
            # legacy path-stop bypass switches are enabled.
            if self.phase == "aligning":
                self._wait_after_pulse(now)
            self.confirmations = 0
            return self._stop("perception_delay_stop", now)
        if path is None:
            if self.phase == "aligning":
                self._wait_after_pulse(now)
            self.confirmations = 0
            return self._stop("path_unavailable", now)
        if not math.isfinite(path.confidence) or (
            self.drive.stop_on_low_confidence
            and not self.drive.bypass_path_stops
            and path.confidence < self.drive.min_confidence
        ):
            if self.phase == "aligning":
                self._wait_after_pulse(now)
            self.confirmations = 0
            return self._stop("path_low_confidence", now)
        geometry = path_geometry(path, self.drive.lookahead_m)
        if geometry is None:
            if self.phase == "aligning":
                self._wait_after_pulse(now)
            self.confirmations = 0
            return self._stop("path_unavailable", now)
        lateral, heading, pursuit, peak = geometry
        self.heading_deg, self.curvature = math.degrees(heading), peak
        lateral_large = (
            self.drive.stop_on_lateral_target
            and not self.drive.bypass_path_stops
            and abs(lateral) > self.drive.max_lateral_target_m
        )
        needs_turn = (
            lateral_large or abs(self.heading_deg) >= self.config.turn_enter_deg
        )
        aligned = (
            not lateral_large and abs(self.heading_deg) <= self.config.turn_exit_deg
        )

        if self.phase == "tracking" and needs_turn:
            self.phase = "braking"
            self.turn_started = now
            self.phase_until = now + self.config.turn_stop_sec
            self.capture_after = self.phase_until
            self.seen_frame = inference_id
            self.confirmations = 0
            return self._stop("turn_braking", now)
        if self.phase == "braking":
            if now < self.phase_until:
                return self._stop("turn_braking", now)
            self.phase = "waiting_frame"
        if self.phase == "aligning":
            # Stop a pulse early if new feedback reports alignment/opposite
            # direction. That frame cannot confirm another turn until settled.
            changed = inference_id != self.seen_frame
            if (
                now >= self.phase_until
                or self.source_age > self.config.slow_age_sec
                or (changed and (aligned or heading * self.pulse_yaw <= 0))
            ):
                return self._wait_after_pulse(now)
            return self._emit(
                DriveDecision(0.0, 0.0, self.pulse_yaw, "turn_aligning"), now
            )
        if self.phase == "waiting_frame":
            # Estimate capture time conservatively using ORIGINAL sensor age,
            # not the time the model finished or the metrics timer ticked.
            capture_at = now - inference_age_sec
            if (
                capture_at < self.capture_after
                or self.source_age > self.config.slow_age_sec
                or inference_id == self.seen_frame
            ):
                return self._stop("turn_waiting_frame", now)
            self.seen_frame = inference_id
            direction = 0 if aligned else (1 if heading > 0 else -1)
            consistent = (
                direction == self.confirm_direction
                and self.confirm_heading_deg is not None
                and abs(self.heading_deg - self.confirm_heading_deg)
                <= self.config.turn_enter_deg
            )
            self.confirmations = self.confirmations + 1 if consistent else 1
            self.confirm_direction = direction
            self.confirm_heading_deg = self.heading_deg
            if self.confirmations < self.config.turn_confirm_frames:
                return self._stop("turn_waiting_frame", now)
            if aligned:
                self.phase = "tracking"
                self.turn_started = None
                self.confirmations = 0
                return self._stop("turn_reacquired", now)
            self.phase = "aligning"
            self.phase_until = now + self.config.turn_pulse_sec
            self.pulse_yaw = float(
                np.clip(
                    self.drive.heading_gain * heading,
                    -self.drive.max_yaw_rps,
                    self.drive.max_yaw_rps,
                )
            )
            return self._emit(
                DriveDecision(0.0, 0.0, self.pulse_yaw, "turn_aligning"), now
            )

        curve_speed = min(
            self.drive.max_forward_mps
            / max(1.0, peak * self.config.curve_slow_radius_m),
            self.drive.max_yaw_rps / max(peak, 1e-9),
        )
        age_scale = min(
            1.0,
            (self.config.stop_age_sec - self.source_age)
            / (self.config.stop_age_sec - self.config.slow_age_sec),
        )
        age_speed = self.drive.max_forward_mps * age_scale
        target_speed = min(curve_speed, age_speed)
        dt = 0.0 if self.last_at is None else min(now - self.last_at, 0.2)
        speed = min(target_speed, self.last_vx + self.config.max_accel_mps2 * dt)
        yaw = float(
            np.clip(speed * pursuit, -self.drive.max_yaw_rps, self.drive.max_yaw_rps)
        )
        reason = (
            "tracking_slow_age"
            if age_speed < curve_speed
            else "tracking_slow_curve"
            if curve_speed < self.drive.max_forward_mps
            else "tracking"
        )
        return self._emit(DriveDecision(speed, 0.0, yaw, reason), now)
