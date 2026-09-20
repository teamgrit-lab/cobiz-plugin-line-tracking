from __future__ import annotations

from collections.abc import Hashable, Iterable
from dataclasses import dataclass
import math


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
