"""Confidence-aware quadruped velocity generation for a fitted line path."""

from dataclasses import dataclass
import math

import numpy as np

from .vision import PathEstimate


@dataclass(frozen=True)
class ControllerConfig:
    near_distance_m: float = 0.30
    lookahead_min_m: float = 0.80
    lookahead_speed_gain: float = 1.0
    nominal_vx: float = 0.30
    min_vx: float = 0.05
    max_vx: float = 0.30
    lateral_kp: float = 0.8
    lateral_kd: float = 0.05
    yaw_kp: float = 1.2
    curvature_speed_gain: float = 2.0
    lateral_speed_gain: float = 1.5
    heading_speed_gain: float = 1.0
    max_vy: float = 0.10
    max_yaw_rate: float = 0.40
    filter_alpha: float = 0.20
    max_vx_rate: float = 0.30
    max_vy_rate: float = 0.25
    max_yaw_rate_change: float = 0.80
    slow_confidence: float = 0.70
    stop_confidence: float = 0.40

    def validate(self) -> None:
        if not 0.0 < self.filter_alpha <= 1.0:
            raise ValueError("filter_alpha must be in (0, 1]")
        if not 0.0 <= self.stop_confidence < self.slow_confidence <= 1.0:
            raise ValueError("confidence thresholds must satisfy 0 <= stop < slow <= 1")
        if not 0.0 <= self.min_vx <= self.max_vx:
            raise ValueError("vx limits are inconsistent")
        if self.max_vy <= 0.0 or self.max_yaw_rate <= 0.0:
            raise ValueError("command limits must be positive")


@dataclass(frozen=True)
class VelocityCommand:
    vx: float
    vy: float
    yaw_rate: float
    lateral_error: float = 0.0
    heading_error: float = 0.0
    curvature: float = 0.0


class LineTrackingController:
    """Generate vx, vy and yaw rate, then smooth and rate-limit the command."""

    def __init__(self, config: ControllerConfig):
        config.validate()
        self.config = config
        self._previous = VelocityCommand(0.0, 0.0, 0.0)
        self._previous_lateral_error = 0.0
        self._has_previous_lateral_error = False

    @staticmethod
    def _rate_limit(
        target: float, previous: float, max_rate: float, dt: float
    ) -> float:
        maximum_delta = max_rate * max(dt, 1e-3)
        return previous + float(
            np.clip(target - previous, -maximum_delta, maximum_delta)
        )

    def stop(self) -> VelocityCommand:
        self._previous = VelocityCommand(0.0, 0.0, 0.0)
        self._previous_lateral_error = 0.0
        self._has_previous_lateral_error = False
        return self._previous

    def generate(self, path: PathEstimate, dt: float) -> VelocityCommand:
        if path.confidence < self.config.stop_confidence:
            return self.stop()

        dt = max(dt, 1e-3)
        lateral_error = path.lateral(self.config.near_distance_m)
        lateral_error_rate = (
            (lateral_error - self._previous_lateral_error) / dt
            if self._has_previous_lateral_error
            else 0.0
        )

        lookahead = self.config.lookahead_min_m + (
            self.config.lookahead_speed_gain * self._previous.vx
        )
        heading_error = math.atan(path.slope(lookahead))
        curvature = path.curvature(lookahead)

        vx_raw = self.config.nominal_vx / (
            1.0
            + self.config.curvature_speed_gain * abs(curvature)
            + self.config.lateral_speed_gain * abs(lateral_error)
            + self.config.heading_speed_gain * abs(heading_error)
        )
        vx_raw = float(np.clip(vx_raw, self.config.min_vx, self.config.max_vx))
        vy_raw = float(
            np.clip(
                self.config.lateral_kp * lateral_error
                + self.config.lateral_kd * lateral_error_rate,
                -self.config.max_vy,
                self.config.max_vy,
            )
        )
        yaw_raw = float(
            np.clip(
                self.config.yaw_kp * heading_error + vx_raw * curvature,
                -self.config.max_yaw_rate,
                self.config.max_yaw_rate,
            )
        )

        if path.confidence < self.config.slow_confidence:
            confidence_scale = path.confidence / self.config.slow_confidence
            vx_raw *= 0.5 * confidence_scale
            vy_raw *= confidence_scale
            yaw_raw *= confidence_scale

        alpha = self.config.filter_alpha
        vx_filtered = alpha * vx_raw + (1.0 - alpha) * self._previous.vx
        vy_filtered = alpha * vy_raw + (1.0 - alpha) * self._previous.vy
        yaw_filtered = alpha * yaw_raw + (1.0 - alpha) * self._previous.yaw_rate

        command = VelocityCommand(
            vx=self._rate_limit(
                vx_filtered,
                self._previous.vx,
                self.config.max_vx_rate,
                dt,
            ),
            vy=self._rate_limit(
                vy_filtered,
                self._previous.vy,
                self.config.max_vy_rate,
                dt,
            ),
            yaw_rate=self._rate_limit(
                yaw_filtered,
                self._previous.yaw_rate,
                self.config.max_yaw_rate_change,
                dt,
            ),
            lateral_error=lateral_error,
            heading_error=heading_error,
            curvature=curvature,
        )
        self._previous = command
        self._previous_lateral_error = lateral_error
        self._has_previous_lateral_error = True
        return command
