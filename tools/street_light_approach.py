"""Optional Pole lateral bias on top of normal road tracking.

StreetLight names remain for compatibility with existing settings and telemetry.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

from swin_l_drive_control import DriveDecision


@dataclass(frozen=True)
class StreetLightConfig:
    stop_distance_m: float = 1.0
    max_lateral_mps: float = 0.15
    forward_mps: float = 0.05  # Legacy configuration; road tracking sets forward speed.
    confirm_frames: int = 3
    clear_frames: int = 3
    min_area_px: int = 24
    min_confidence: float = 0.50
    max_age_sec: float = 1.0
    max_approach_sec: float = 30.0
    lidar_min_points: int = 3
    lidar_max_range_m: float = 15.0

    def validate(self) -> None:
        for name in ("stop_distance_m", "max_lateral_mps", "forward_mps",
                     "max_age_sec", "max_approach_sec", "lidar_max_range_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"street light {name} must be finite and positive")
        if max(self.forward_mps, self.max_lateral_mps) > 1.0:
            raise ValueError("street light speeds must not exceed 1 m/s")
        for name in ("confirm_frames", "clear_frames", "min_area_px", "lidar_min_points"):
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
    top: float = 0.0

    @property
    def center(self) -> float:
        return (self.left + self.right) / 2


@dataclass
class _LightTrack:
    id: int
    observation: StreetLightObservation
    hits: int = 1
    misses: int = 0
    visited: bool = False
    retry_blocked: bool = False
    velocity: tuple[float, float] = (0.0, 0.0)


def street_light_observations(
    mask: np.ndarray | None, min_area_px: int,
) -> tuple[StreetLightObservation, ...]:
    """Keep separate poles; coordinates are normalized to the score-map size."""
    if mask is None:
        return ()
    mask = np.asarray(mask, dtype=np.uint8)
    if mask.ndim != 2 or not mask.size:
        raise ValueError("invalid Pole mask")
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    height, width = mask.shape
    return tuple(
        StreetLightObservation(x / width, (x + w) / width, (y + h) / height, int(area), y / height)
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
        self.target_id = None
        self.tracks: dict[int, _LightTrack] = {}
        self.next_id = 1
        self.hits = self.clear_hits = self.visits = 0
        self.last_inference = None
        self.approach_started = None
        self.metrics = {"bias_active": False, "bias_reason": "no_confirmed_light",
                        "lidar_confirmed": False}
        self._update_metrics()

    def _update_metrics(self) -> None:
        self.clear_hits = max((t.misses for t in self.tracks.values()
                               if t.retry_blocked), default=0)
        self.metrics.update(
            state=self.phase, target_id=self.target_id, confirm_hits=self.hits,
            clear_hits=self.clear_hits, visits=self.visits,
            visible_ids=[t.id for t in self.tracks.values() if t.misses == 0],
            visited_ids=[t.id for t in self.tracks.values() if t.visited],
            pending_ids=[t.id for t in self.tracks.values()
                         if t.misses == 0 and not t.visited and not t.retry_blocked
                         and t.id != self.target_id],
        )

    def _track(self, observations, consecutive: bool) -> None:
        """Associate all poles one-to-one, including visited and missing IDs.

        IDs are image-space tracks, not world coordinates. Keep dormant tracks
        for this task so a brief occlusion does not erase a completed visit.
        """
        tracks = list(self.tracks.values())
        # A merged semantic component must not collapse two IDs into one or
        # become a new target. Retain both tracks until they separate again.
        separate = []
        for observation in observations:
            covered = [t for t in tracks
                       if observation.left <= t.observation.center <= observation.right
                       and abs(observation.bottom - t.observation.bottom) <= 0.15]
            if (len(covered) > 1 and observation.right - observation.left
                    > 1.5 * max(t.observation.right - t.observation.left for t in covered)):
                continue
            separate.append(observation)
        observations = separate
        costs = np.full((len(tracks), len(observations)), 1e6)
        for row, track in enumerate(tracks):
            old = track.observation
            steps = min(track.misses + 1, 3) if consecutive else 0
            center = old.center + track.velocity[0] * steps
            bottom = old.bottom + track.velocity[1] * steps
            for column, observation in enumerate(observations):
                dx, dy = abs(observation.center - center), abs(observation.bottom - bottom)
                width_ratio = (observation.right - observation.left) / max(old.right - old.left, 1e-6)
                if dx > 0.15 or dy > 0.15 or not 0.25 <= width_ratio <= 4:
                    continue
                costs[row, column] = (dx + 0.5 * dy + 0.03 * abs(math.log(width_ratio))
                                      + 0.01 * abs(math.log(max(observation.area_px, 1) / max(old.area_px, 1)))
                                      + 0.02 * min(track.misses, 3))
        matched_tracks, matched_observations = set(), set()
        rows, columns = linear_sum_assignment(costs)
        for row, column in zip(rows, columns):
            if costs[row, column] >= 1e6:
                continue
            track, observation = tracks[row], observations[column]
            old = track.observation
            track.velocity = (
                float(np.clip(observation.center - old.center, -0.05, 0.05)),
                float(np.clip(observation.bottom - old.bottom, -0.05, 0.05)),
            ) if consecutive and track.misses == 0 else (0.0, 0.0)
            track.hits = track.hits + 1 if consecutive and track.misses == 0 else 1
            track.observation, track.misses = observation, 0
            matched_tracks.add(track.id)
            matched_observations.add(column)
        for track in tracks:
            if track.id not in matched_tracks:
                track.hits = 0
                track.misses = track.misses + 1 if consecutive else 1
                if track.misses >= self.config.clear_frames:
                    track.retry_blocked = False
        # First seen first served; simultaneous new lights start with the
        # largest component, with a deterministic left-to-right tie break.
        new = (o for i, o in enumerate(observations) if i not in matched_observations)
        for observation in sorted(new, key=lambda o: (-o.area_px, o.center, o.bottom)):
            self.tracks[self.next_id] = _LightTrack(self.next_id, observation)
            self.next_id += 1

    def _cooldown(self, reason: str) -> None:
        track = self.tracks[self.target_id]
        if reason == "reached":
            track.visited = True
            self.visits += 1
        else:
            # Loss/timeout is not a visit. Only this ID waits for its own
            # absence before retry; other visible lights remain eligible.
            track.retry_blocked = track.misses < self.config.clear_frames
        self.phase = "passing"
        self.hits = self.clear_hits = 0
        self.target = self.target_id = None
        self.approach_started = None
        self.metrics.update(transition=reason, bearing_rad=None, range_m=None, range_source=None)
        self._update_metrics()

    def decide(self, normal, observations, *, now, inference_id, inference_age_sec,
               locate):
        c = self.config
        self.metrics.update(bias_active=False, bias_reason="road_tracking", lidar_confirmed=False)
        # Road tracking owns motion and its sensor guards. A stale or missing
        # light observation disables only this optional lateral bias.
        if normal.reason not in ("tracking", "tracking_slow_turn") or normal.vx <= 0:
            self.metrics["bias_reason"] = "road_not_tracking"
            return normal
        if (inference_age_sec is None or not math.isfinite(inference_age_sec)
                or not 0 <= inference_age_sec <= c.max_age_sec):
            self.phase = "cruising"
            self.target = self.target_id = None
            self.hits = 0
            self.approach_started = self.last_inference = None
            self.metrics.update(bias_reason="observations_stale", bearing_rad=None,
                                range_m=None, range_source=None)
            self._update_metrics()
            return normal
        if inference_id != self.last_inference:
            consecutive = self.last_inference is not None and inference_id == self.last_inference + 1
            self.last_inference = inference_id
            self._track(observations, consecutive)
            track = self.tracks.get(self.target_id)
            if track is not None and track.misses:
                if self.phase == "approaching":
                    self._cooldown("target_lost")
                    self.metrics["bias_reason"] = "target_lost"
                    return normal
                self.target = self.target_id = None
                track = None
            if track is None:
                track = next((t for t in self.tracks.values()
                              if t.misses == 0 and not t.visited and not t.retry_blocked), None)
            if track is None:
                self.target = self.target_id = None
                self.hits, self.phase = 0, "cruising"
            else:
                self.target, self.target_id, self.hits = track.observation, track.id, track.hits
                if self.phase != "approaching":
                    self.phase = "confirming"
            self._update_metrics()
        if self.target is None or self.hits < c.confirm_frames:
            self.metrics["bias_reason"] = "no_confirmed_light"
            return normal
        if self.approach_started is not None and now - self.approach_started >= c.max_approach_sec:
            self._cooldown("approach_timeout")
            self.metrics["bias_reason"] = "approach_timeout"
            return normal
        try:
            bearing, distance = locate(self.target)
        except (ValueError, cv2.error) as error:
            self.phase, self.approach_started = "confirming", None
            self.metrics["bias_reason"] = str(error)
            self._update_metrics()
            return normal
        if not math.isfinite(bearing) or (distance is not None and
                (not math.isfinite(distance) or distance <= 0)):
            self.metrics["bias_reason"] = "street_light_geometry_invalid"
            return normal
        if distance is None:
            self.phase, self.approach_started = "confirming", None
            self.metrics.update(bias_reason="lidar_unconfirmed", bearing_rad=None,
                                range_m=None, range_source=None)
            self._update_metrics()
            return normal
        if self.phase != "approaching":
            self.phase, self.approach_started = "approaching", now
        self.metrics["lidar_confirmed"] = True
        self._update_metrics()
        self.metrics.update(bearing_rad=bearing, range_m=distance,
                            range_source="lidar_visual_box_association")
        if distance is not None and distance <= c.stop_distance_m:
            self._cooldown("reached")
            self.metrics["bias_reason"] = "reached"
            return normal
        vy = float(np.clip(0.5 * bearing, -c.max_lateral_mps, c.max_lateral_mps))
        if abs(bearing) <= 0.05:
            self.metrics["bias_reason"] = "bearing_aligned"
            return normal
        self.metrics.update(bias_active=True, bias_reason="approaching")
        return DriveDecision(normal.vx, vy, 0.0, "street_light_approaching")
