from __future__ import annotations

import math
from collections.abc import Hashable, Iterable
from dataclasses import dataclass


@dataclass(frozen=True)
class AprilTagPolicy:
    max_age_sec: float = 1.0
    confirm_window_sec: float = 1.0
    min_hits: int = 3

    def validate(self) -> None:
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (self.max_age_sec, self.confirm_window_sec)
        ):
            raise ValueError("AprilTag timing limits must be positive and finite")
        if type(self.min_hits) is not int or self.min_hits <= 0:
            raise ValueError("AprilTag min_hits must be a positive integer")


@dataclass(frozen=True)
class AprilTagDecision:
    state: str
    stop_now: bool
    just_confirmed: bool
    false_positive: bool
    confirmed_id: int | None
    hit_counts: tuple[tuple[int, int], ...]
    window_elapsed_sec: float | None


class AprilTagStopMonitor:
    def __init__(self, policy: AprilTagPolicy | None = None) -> None:
        self.policy = policy or AprilTagPolicy()
        self.policy.validate()
        self._last_message_at: float | None = None
        self._window_started_at: float | None = None
        self._hits: dict[int, set[Hashable]] = {}
        self._confirmed_id: int | None = None

    def _decision(
        self,
        now: float,
        *,
        stop_now: bool = False,
        just_confirmed: bool = False,
        false_positive: bool = False,
    ) -> AprilTagDecision:
        state = (
            "confirmed"
            if self._confirmed_id is not None
            else "verifying"
            if self._window_started_at is not None
            else "no_tag"
        )
        elapsed = (
            None
            if self._window_started_at is None
            else max(now - self._window_started_at, 0.0)
        )
        return AprilTagDecision(
            state=state,
            stop_now=stop_now,
            just_confirmed=just_confirmed,
            false_positive=false_positive,
            confirmed_id=self._confirmed_id,
            hit_counts=tuple(
                (tag_id, len(frames)) for tag_id, frames in sorted(self._hits.items())
            ),
            window_elapsed_sec=elapsed,
        )

    def _winner(self) -> int | None:
        winners = sorted(
            tag_id
            for tag_id, frames in self._hits.items()
            if len(frames) >= self.policy.min_hits
        )
        return winners[0] if winners else None

    def _start_window(
        self, ids: tuple[int, ...], frame_key: Hashable, now: float
    ) -> None:
        self._window_started_at = now
        self._hits = {tag_id: {frame_key} for tag_id in ids}

    def _clear_window(self) -> None:
        self._window_started_at = None
        self._hits = {}

    def begin_task(
        self, *, ids: Iterable[int], frame_key: Hashable, now: float
    ) -> AprilTagDecision:
        """Seed fresh cached detections without refreshing their heartbeat."""

        self.reset_task()
        normalized = tuple(sorted({int(tag_id) for tag_id in ids}))
        if self.stream_ready(now) and normalized:
            self._start_window(normalized, frame_key, now)
            return self._decision(now, stop_now=True)
        return self.snapshot(now=now)

    def observe(
        self,
        *,
        ids: Iterable[int],
        frame_key: Hashable,
        now: float,
        task_active: bool,
    ) -> AprilTagDecision:
        normalized = tuple(sorted({int(tag_id) for tag_id in ids}))
        self._last_message_at = now
        if not task_active:
            self.reset_task()
            return self.snapshot(now=now)
        if self._confirmed_id is not None:
            return self.snapshot(now=now)
        false_positive = False
        if (
            self._window_started_at is not None
            and now - self._window_started_at >= self.policy.confirm_window_sec
        ):
            winner = self._winner()
            if winner is not None:
                self._confirmed_id = winner
                self._clear_window()
                return self._decision(now, just_confirmed=True)
            self._clear_window()
            false_positive = True
        if self._window_started_at is None and normalized:
            self._start_window(normalized, frame_key, now)
            return self._decision(
                now,
                stop_now=True,
                false_positive=false_positive,
            )
        if self._window_started_at is not None:
            for tag_id in normalized:
                self._hits.setdefault(tag_id, set()).add(frame_key)
        return self._decision(now, false_positive=false_positive)

    def tick(self, *, now: float, task_active: bool) -> AprilTagDecision:
        if not task_active:
            self.reset_task()
            return self.snapshot(now=now)
        if (
            self._confirmed_id is None
            and self._window_started_at is not None
            and now - self._window_started_at >= self.policy.confirm_window_sec
        ):
            winner = self._winner()
            if winner is not None:
                self._confirmed_id = winner
                self._clear_window()
                return self._decision(now, just_confirmed=True)
            self._clear_window()
            return self._decision(now, false_positive=True)
        return self.snapshot(now=now)

    def snapshot(self, *, now: float) -> AprilTagDecision:
        return self._decision(now)

    def message_age_sec(self, now: float) -> float | None:
        if self._last_message_at is None:
            return None
        return max(now - self._last_message_at, 0.0)

    def stream_ready(self, now: float) -> bool:
        age = self.message_age_sec(now)
        return age is not None and age <= self.policy.max_age_sec

    def reset_task(self) -> None:
        self._window_started_at = None
        self._hits = {}
        self._confirmed_id = None


@dataclass(frozen=True)
class AprilTagGate:
    """Which detections may stop line tracking.

    A detection counts only where TAG_APPROACH could take over from it: close
    enough, decoded cleanly enough, and an id the approach accepts. A tag that
    fails is ignored outright, so a far tag never even pauses tracking.
    """

    # Camera-to-tag-centre range. 0 disables the range check.
    max_range_m: float = 3.0
    # TAG_APPROACH min_decision_margin. 0 disables the check.
    min_decision_margin: float = 15.0
    # Empty accepts any id.
    allowed_ids: frozenset[int] = frozenset()
    # Black-square edge, matching TAG_APPROACH default_tag_size_m.
    tag_size_m: float = 0.167

    def validate(self) -> None:
        if not all(
            math.isfinite(value) and value >= 0.0
            for value in (self.max_range_m, self.min_decision_margin)
        ):
            raise ValueError("AprilTag gate limits must be finite and non-negative")
        if not math.isfinite(self.tag_size_m) or self.tag_size_m <= 0.0:
            raise ValueError("AprilTag tag_size_m must be positive and finite")
        if any(type(tag_id) is not int or tag_id < 0 for tag_id in self.allowed_ids):
            raise ValueError("AprilTag allowed ids must be non-negative integers")


def parse_tag_ids(text: str) -> frozenset[int]:
    """Parse "0-30", "1,4,7" or a mix of both; blank means any id."""

    ids: set[int] = set()
    for part in text.replace(" ", "").split(","):
        if not part:
            continue
        low, dash, high = part.partition("-")
        first, last = int(low), int(high) if dash else int(low)
        if first < 0 or last < first:
            raise ValueError(f"invalid AprilTag id range: {part}")
        ids.update(range(first, last + 1))
    return frozenset(ids)


@dataclass(frozen=True)
class CameraModel:
    k: tuple[float, ...]
    d: tuple[float, ...]
    fisheye: bool


def camera_model_from_info(info: object) -> CameraModel | None:
    """Raw-image intrinsics; None when the model cannot undistort corners."""

    k = tuple(float(value) for value in getattr(info, "k", ()))
    d = tuple(float(value) for value in getattr(info, "d", ()))
    model = str(getattr(info, "distortion_model", ""))
    if len(k) != 9 or not all(math.isfinite(v) for v in (*k, *d)) or k[0] <= 0.0:
        return None
    if model == "equidistant" and len(d) == 4:
        return CameraModel(k, d, True)
    if model in ("plumb_bob", "rational_polynomial") and len(d) in (4, 5, 8, 12, 14):
        return CameraModel(k, d, False)
    return None


def tag_range_m(
    corners: Iterable[tuple[float, float]], tag_size_m: float, camera: CameraModel
) -> float | None:
    """Camera-to-tag-centre range from the four raw-image corners.

    The detector runs on the unrectified wide-angle image, so the corners are
    undistorted with the camera's own model before the planar solve. Range is
    the norm of the tag centre, which a reversed corner winding does not change.
    """

    import cv2
    import numpy as np

    points = np.asarray(list(corners), np.float64).reshape(-1, 1, 2)
    if points.shape[0] != 4 or not np.isfinite(points).all():
        return None
    k = np.asarray(camera.k, np.float64).reshape(3, 3)
    d = np.asarray(camera.d, np.float64)
    if camera.fisheye:
        normalized = cv2.fisheye.undistortPoints(points, k, d)
    else:
        normalized = cv2.undistortPoints(points, k, d)
    half = tag_size_m / 2.0
    square = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        np.float64,
    )
    try:
        solved, _rvec, tvec = cv2.solvePnP(
            square, normalized, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
    except cv2.error:
        return None
    if not solved or float(tvec[2][0]) <= 0.0:
        return None
    distance = float(np.linalg.norm(tvec))
    return distance if math.isfinite(distance) else None


def gate_detections(
    detections: Iterable[object], gate: AprilTagGate, camera: CameraModel | None
) -> dict[int, float | None]:
    """Ids that may stop tracking, each with its nearest range (None if unchecked)."""

    accepted: dict[int, float | None] = {}
    for detection in detections:
        tag_id = int(getattr(detection, "id"))
        if gate.allowed_ids and tag_id not in gate.allowed_ids:
            continue
        if gate.min_decision_margin > 0.0 and not (
            float(getattr(detection, "decision_margin", 0.0)) >= gate.min_decision_margin
        ):
            continue
        distance = None
        if gate.max_range_m > 0.0:
            # Without calibration the range is unknown: keep driving rather
            # than stop at a distance nobody measured.
            if camera is None:
                continue
            distance = tag_range_m(
                ((corner.x, corner.y) for corner in getattr(detection, "corners", ())),
                gate.tag_size_m,
                camera,
            )
            if distance is None or distance > gate.max_range_m:
                continue
        previous = accepted.get(tag_id)
        if tag_id not in accepted or (
            distance is not None and (previous is None or distance < previous)
        ):
            accepted[tag_id] = distance
    return accepted
