"""ROS-independent Cobiz LINE_TRACKING task lifecycle and safety timeout."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
import re
from typing import Any, Mapping


ACTION_NAME = "LINE_TRACKING"
_TASK_ID = re.compile(r"[A-Za-z0-9_-]{1,128}\Z")
_ROUTES = {
    "TASK_STARTED": "start",
    "TASK_COMPLETED": "complete",
    "TASK_REJECTED": "reject",
    "TASK_ABORTED": "abort",
}


@dataclass(frozen=True)
class TaskPolicy:
    default_duration_sec: float = 60.0
    max_duration_sec: float = 300.0
    unsafe_timeout_sec: float = 2.0
    startup_hold_sec: float = 2.0
    default_selected_mask: int = 2

    def validate(self) -> None:
        values = (
            self.default_duration_sec,
            self.max_duration_sec,
            self.unsafe_timeout_sec,
            self.startup_hold_sec,
        )
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError("task timing limits must be positive and finite")
        if (
            self.default_duration_sec < self.startup_hold_sec + 1.0
            or self.default_duration_sec > self.max_duration_sec
        ):
            raise ValueError("default duration must exceed startup hold by 1 second")
        if type(
            self.default_selected_mask
        ) is not int or self.default_selected_mask not in (1, 2):
            raise ValueError("default selected_mask must be 1 (road) or 2 (sidewalk)")


@dataclass(frozen=True)
class ActiveTask:
    task_id: str | int
    key: str
    device_id: str | int | None
    device_name: str | None
    started_at: float
    duration_sec: float
    selected_mask: int


def _task_id(event: Mapping[str, Any]) -> tuple[str | int, str] | None:
    value = event.get("task_id")
    if isinstance(value, bool) or not isinstance(value, (str, int)):
        return None
    key = str(value).strip()
    if not _TASK_ID.fullmatch(key):
        return None
    return (key if isinstance(value, str) else value), key


def _payload(event: Mapping[str, Any]) -> Mapping[str, Any]:
    payload = event.get("payload")
    if isinstance(payload, str):
        try:
            payload = json.loads(payload)
        except json.JSONDecodeError as error:
            raise ValueError("invalid_payload_json") from error
    if payload is None:
        payload = {}
    if not isinstance(payload, Mapping):
        raise ValueError("invalid_payload")
    return payload


def requested_selected_mask(event: Mapping[str, Any], default: int) -> int:
    """Return the requested drivable class, falling back to the configured default."""

    return _selected_mask(_payload(event), default)


def _selected_mask(payload: Mapping[str, Any], default: int) -> int:
    value = payload.get("selected_mask", default)
    if type(value) is not int or value not in (1, 2):
        raise ValueError("invalid_selected_mask")
    return value


def _duration(payload: Mapping[str, Any], policy: TaskPolicy) -> float:
    value = payload.get("duration_sec", policy.default_duration_sec)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("invalid_duration_sec")
    duration = float(value)
    if (
        not math.isfinite(duration)
        or not policy.startup_hold_sec + 1.0 <= duration <= policy.max_duration_sec
    ):
        raise ValueError("duration_sec_out_of_range")
    return duration


def task_state(
    task_id: str | int,
    state_type: str,
    *,
    device_id: str | int | None = None,
    device_name: str | None = None,
    reason: str | None = None,
) -> dict[str, Any]:
    """Build the exact /task_state shape consumed by core request_manager."""

    body: dict[str, Any] = {
        "type": state_type,
        "task_id": task_id,
        "task_type": _ROUTES[state_type],
        "action_name": ACTION_NAME,
    }
    if device_id is not None:
        body["device_id"] = device_id
    if device_name is not None:
        body["device_name"] = device_name
    if reason:
        body["reason"] = reason
    return body


class LineTrackingTasks:
    """Accept one finite server task; require fresh safe tracking throughout."""

    def __init__(self, policy: TaskPolicy | None = None) -> None:
        self.policy = policy or TaskPolicy()
        self.policy.validate()
        self.active: ActiveTask | None = None
        self.tracking_seen = False
        self.unsafe_since: float | None = None
        self._recent_ids: deque[str] = deque(maxlen=256)

    def _remember(self, key: str) -> None:
        if len(self._recent_ids) == self._recent_ids.maxlen:
            self._recent_ids.popleft()
        self._recent_ids.append(key)

    def _state(
        self, active: ActiveTask, state_type: str, reason: str | None = None
    ) -> dict[str, Any]:
        return task_state(
            active.task_id,
            state_type,
            device_id=active.device_id,
            device_name=active.device_name,
            reason=reason,
        )

    def handle_event(
        self,
        event: Any,
        *,
        now: float,
        ready_reason: str,
    ) -> dict[str, Any] | None:
        if not isinstance(event, Mapping):
            return None
        event_type = event.get("type")
        action = event.get("action_name")
        identity = _task_id(event)
        if event_type == "TASK_ABORTED":
            if (
                self.active is not None
                and identity is not None
                and identity[1] == self.active.key
                and action in (None, "", ACTION_NAME)
            ):
                return self.finish("TASK_ABORTED", "task_aborted_by_server")
            return None
        if event_type != "TASK_REGISTERED" or action != ACTION_NAME or identity is None:
            return None
        raw_id, key = identity
        if key in self._recent_ids:
            return None
        self._remember(key)
        device_id = event.get("device_id")
        if isinstance(device_id, bool) or not isinstance(device_id, (str, int)):
            device_id = None
        device_name = event.get("device_name")
        if not isinstance(device_name, str):
            device_name = None
        candidate = ActiveTask(
            raw_id,
            key,
            device_id,
            device_name,
            now,
            0.0,
            self.policy.default_selected_mask,
        )
        if self.active is not None:
            return self._state(
                candidate, "TASK_REJECTED", "another_line_tracking_task_active"
            )
        try:
            payload = _payload(event)
            selected_mask = _selected_mask(payload, self.policy.default_selected_mask)
            duration = _duration(payload, self.policy)
        except ValueError as error:
            return self._state(candidate, "TASK_REJECTED", str(error))
        if ready_reason != "tracking":
            return self._state(candidate, "TASK_REJECTED", ready_reason)
        self.active = ActiveTask(
            raw_id, key, device_id, device_name, now, duration, selected_mask
        )
        self.tracking_seen = False
        self.unsafe_since = None
        return self._state(self.active, "TASK_STARTED")

    def tick(self, *, now: float, drive_reason: str) -> dict[str, Any] | None:
        active = self.active
        if active is None:
            return None
        elapsed = now - active.started_at
        if elapsed < self.policy.startup_hold_sec:
            return None
        tracked_before_this_tick = self.tracking_seen
        if drive_reason == "tracking":
            self.tracking_seen = True
            self.unsafe_since = None
        elif self.unsafe_since is None:
            self.unsafe_since = now
        elif now - self.unsafe_since >= self.policy.unsafe_timeout_sec:
            return self.finish("TASK_ABORTED", f"unsafe:{drive_reason}")
        if elapsed >= active.duration_sec:
            if tracked_before_this_tick and drive_reason == "tracking":
                return self.finish("TASK_COMPLETED")
            return self.finish("TASK_ABORTED", f"tracking_unavailable:{drive_reason}")
        return None

    def finish(
        self, state_type: str, reason: str | None = None
    ) -> dict[str, Any] | None:
        active = self.active
        if active is None:
            return None
        result = self._state(active, state_type, reason)
        self.active = None
        self.tracking_seen = False
        self.unsafe_since = None
        return result
