"""Road cruising and lateral-first Street Light approach, without ROS state."""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np

from swin_l_drive_control import DriveDecision


@dataclass(frozen=True)
class StreetLightConfig:
    stop_distance_m: float = 1.0
    max_lateral_mps: float = 0.15
    forward_mps: float = 0.05
    confirm_frames: int = 3
    clear_frames: int = 3
    min_area_px: int = 24
    min_confidence: float = 0.50
    max_age_sec: float = 1.0
    max_approach_sec: float = 30.0

    def validate(self) -> None:
        for name in ("stop_distance_m", "max_lateral_mps", "forward_mps",
                     "max_age_sec", "max_approach_sec"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"street light {name} must be finite and positive")
        if max(self.forward_mps, self.max_lateral_mps) > 1.0:
            raise ValueError("street light speeds must not exceed 1 m/s")
        for name in ("confirm_frames", "clear_frames", "min_area_px"):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(f"street light {name} must be a positive integer")
        if self.confirm_frames < 2 or self.clear_frames < 2:
            raise ValueError("street light confirmation/clear frames must be >= 2")
        if not math.isfinite(self.min_confidence) or not 0 < self.min_confidence <= 1:
            raise ValueError("street light min_confidence must be in (0, 1]")


@dataclass(frozen=True)
class StreetLightObservation:
    left: float
    right: float
    bottom: float
    area_px: int

    @property
    def center(self) -> float:
        return (self.left + self.right) / 2


def street_light_observations(
    mask: np.ndarray | None, min_area_px: int,
) -> tuple[StreetLightObservation, ...]:
    """Keep separate lights; coordinates are normalized to the score-map size."""
    if mask is None:
        return ()
    mask = np.asarray(mask, dtype=np.uint8)
    if mask.ndim != 2 or not mask.size:
        raise ValueError("invalid Street Light mask")
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    height, width = mask.shape
    return tuple(
        StreetLightObservation(x / width, (x + w) / width, (y + h) / height, int(area))
        for x, y, w, h, area in stats[1:count]
        if area >= min_area_px
    )


class StreetLightApproach:
    def __init__(self, config: StreetLightConfig) -> None:
        config.validate()
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.phase = "cruising"
        self.target = None
        self.hits = self.clear_hits = self.visits = 0
        self.last_inference = None
        self.approach_started = None
        self.metrics = {"state": self.phase, "visits": 0}

    def _cooldown(self, reason: str) -> None:
        self.phase = "passing"
        self.hits = self.clear_hits = 0
        self.target = None
        self.approach_started = None
        self.metrics.update(state=self.phase, transition=reason, confirm_hits=0,
                            clear_hits=0, visits=self.visits)

    def decide(self, normal, observations, *, now, inference_id, inference_age_sec,
               locate):
        c = self.config
        # No retained lateral command, stale image, or path-search rotation may
        # bypass the scenario's observation and road-path requirements.
        if (inference_age_sec is None or not math.isfinite(inference_age_sec)
                or not 0 <= inference_age_sec <= c.max_age_sec):
            return DriveDecision.stop("street_light_inference_stale")
        if normal.reason not in ("tracking", "tracking_slow_turn"):
            return normal
        cruise = DriveDecision(normal.vx, 0.0, 0.0, "street_light_cruising")
        if inference_id != self.last_inference:
            consecutive = self.last_inference is not None and inference_id == self.last_inference + 1
            self.last_inference = inference_id
            if self.phase == "passing":
                self.clear_hits = self.clear_hits + 1 if not observations else 0
                if self.clear_hits >= c.clear_frames:
                    self.phase = "cruising"
            else:
                matching = [o for o in observations
                            if self.target is None or abs(o.center - self.target.center) <= 0.15]
                candidate = (min(matching, key=lambda o: abs(o.center - self.target.center))
                             if matching and self.target is not None else
                             max(matching, key=lambda o: o.area_px) if matching else None)
                if candidate is None:
                    if self.phase == "approaching":
                        self._cooldown("target_lost")
                        return DriveDecision.stop("street_light_target_lost")
                    else:
                        self.target, self.hits, self.phase = None, 0, "cruising"
                else:
                    self.hits = self.hits + 1 if consecutive and self.target is not None else 1
                    self.target = candidate
                    if self.hits >= c.confirm_frames and self.phase != "approaching":
                        self.phase, self.approach_started = "approaching", now
                    elif self.phase != "approaching":
                        self.phase = "confirming"
            self.metrics.update(state=self.phase, confirm_hits=self.hits,
                                clear_hits=self.clear_hits, visits=self.visits)
        if self.phase != "approaching":
            return cruise
        if now - self.approach_started >= c.max_approach_sec:
            self._cooldown("approach_timeout")
            return DriveDecision.stop("street_light_approach_timeout")
        try:
            bearing, distance = locate(self.target)
        except ValueError as error:
            return DriveDecision.stop(str(error))
        if not math.isfinite(bearing) or (distance is not None and
                (not math.isfinite(distance) or distance <= 0)):
            return DriveDecision.stop("street_light_geometry_invalid")
        self.metrics.update(bearing_rad=bearing, range_m=distance,
                            range_source="lidar_column_association" if distance is not None else None)
        if distance is not None and distance <= c.stop_distance_m:
            self.visits += 1
            self._cooldown("reached")
            self.metrics["visits"] = self.visits
            # One zero command ends the approach, then cruising resumes. Do not
            # complete the Cobiz task or reacquire this still-visible light.
            return DriveDecision.stop("street_light_reached")
        vy = float(np.clip(0.5 * bearing, -c.max_lateral_mps, c.max_lateral_mps))
        if abs(bearing) <= 0.05:
            vy = 0.0
        vx = min(normal.vx, c.forward_mps,
                 abs(vy) / 3 if vy else c.forward_mps)
        return DriveDecision(vx, vy, 0.0, "street_light_approaching")
