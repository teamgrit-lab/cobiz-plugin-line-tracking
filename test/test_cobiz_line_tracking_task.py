from pathlib import Path
import json
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from cobiz_line_tracking_task import LineTrackingTasks, TaskPolicy, TASK_STOP_CHECKS  # noqa: E402


def _enabled_policy(**overrides):
    flags = {"stop_on_" + name: True for name in TASK_STOP_CHECKS}
    flags.update(overrides)
    return TaskPolicy(**flags)


def event(task_id=123, **extra):
    return {
        "type": "TASK_REGISTERED",
        "task_id": task_id,
        "action_name": "LINE_TRACKING",
        "device_id": 45,
        "device_name": "Dangjin-A2",
        **extra,
    }


def test_unrelated_events_and_missing_ids_do_not_start():
    tasks = LineTrackingTasks()
    assert (
        tasks.handle_event(
            {**event(), "action_name": "Line_tracking"},
            now=0,
            rejection_reason=None,
        )
        is None
    )
    assert (
        tasks.handle_event(
            {**event(), "action_name": "ARM_RESET"}, now=0, rejection_reason=None
        )
        is None
    )
    assert (
        tasks.handle_event({**event(), "task_id": ""}, now=0, rejection_reason=None)
        is None
    )
    assert (
        tasks.handle_event(
            {**event(), "task_id": "../../bad"}, now=0, rejection_reason=None
        )
        is None
    )
    assert tasks.active is None


def test_dynamic_inputs_do_not_reject_a_valid_task():
    tasks = LineTrackingTasks()
    state = tasks.handle_event(event(), now=0, rejection_reason=None)
    assert state["type"] == "TASK_STARTED"
    assert tasks.active is not None


def test_missing_duration_uses_five_hundred_seconds():
    tasks = LineTrackingTasks()

    state = tasks.handle_event(event(), now=0, rejection_reason=None)

    assert state["type"] == "TASK_STARTED"
    assert tasks.active.duration_sec == 500.0


@pytest.mark.parametrize(
    ("requested", "expected"),
    [
        (1000, 1000.0),
        (5000, 5000.0),
        (10000, 10000.0),
        (10000.1, 10000.0),
        (20000, 10000.0),
    ],
)
def test_duration_is_capped_at_ten_thousand_seconds(requested, expected):
    tasks = LineTrackingTasks()

    state = tasks.handle_event(
        event(payload={"duration_sec": requested}), now=0, rejection_reason=None
    )

    assert state["type"] == "TASK_STARTED"
    assert tasks.active.duration_sec == expected


def test_static_control_conflict_rejects_without_activation():
    tasks = LineTrackingTasks()
    state = tasks.handle_event(
        event(), now=0, rejection_reason="multiple_control_publishers"
    )
    assert state["type"] == "TASK_REJECTED"
    assert state["reason"] == "multiple_control_publishers"
    assert tasks.active is None


def test_start_requires_tracking_then_completes_finite_task():
    tasks = LineTrackingTasks(_enabled_policy(default_duration_sec=5, max_duration_sec=10))
    started = tasks.handle_event(
        event(payload={"duration_sec": 4}), now=10, rejection_reason=None
    )
    assert started["type"] == "TASK_STARTED"
    assert started["task_type"] == "start"
    assert tasks.tick(now=11, drive_reason="drive_not_armed") is None
    assert tasks.tick(now=12.1, drive_reason="tracking") is None
    completed = tasks.tick(now=14.1, drive_reason="tracking")
    assert completed["type"] == "TASK_COMPLETED"
    assert completed["task_type"] == "complete"
    assert tasks.active is None


@pytest.mark.parametrize("selected_mask", [0, 1, 2])
@pytest.mark.parametrize("as_json", [False, True])
def test_task_selects_surface_from_object_or_json_payload(selected_mask, as_json):
    tasks = LineTrackingTasks(_enabled_policy(default_selected_mask=2))
    payload = {"selected_mask": selected_mask}
    if as_json:
        payload = json.dumps(payload)

    started = tasks.handle_event(event(payload=payload), now=0, rejection_reason=None)

    assert started["type"] == "TASK_STARTED"
    assert tasks.active.selected_mask == selected_mask
    tasks.finish("TASK_COMPLETED")
    assert tasks.active is None


@pytest.mark.parametrize("selected_mask", [0, 1, 2])
def test_task_uses_configured_mask_when_payload_omits_it(selected_mask):
    tasks = LineTrackingTasks(_enabled_policy(default_selected_mask=selected_mask))

    started = tasks.handle_event(
        event(payload={"duration_sec": 30}), now=0, rejection_reason=None
    )

    assert started["type"] == "TASK_STARTED"
    assert tasks.active.selected_mask == selected_mask


@pytest.mark.parametrize("selected_mask", [3, -1, True, False, 0.0, 1.0, "0", "1", None, {}, []])
def test_invalid_selected_mask_rejected_before_static_control_conflict(selected_mask):
    tasks = LineTrackingTasks()

    state = tasks.handle_event(
        event(payload={"selected_mask": selected_mask}),
        now=0,
        rejection_reason="multiple_control_publishers",
    )

    assert state["type"] == "TASK_REJECTED"
    assert state["reason"] == "invalid_selected_mask"
    assert tasks.active is None


def test_server_abort_requires_active_matching_id_even_without_action_name():
    tasks = LineTrackingTasks()
    tasks.handle_event(event(task_id="a-1"), now=0, rejection_reason=None)
    assert (
        tasks.handle_event(
            {"type": "TASK_ABORTED", "task_id": "other"}, now=1, rejection_reason=None
        )
        is None
    )
    assert (
        tasks.handle_event(
            {"type": "TASK_ABORTED", "task_id": "a-1", "action_name": "ARM_RESET"},
            now=1,
            rejection_reason=None,
        )
        is None
    )
    stopped = tasks.handle_event(
        {"type": "TASK_ABORTED", "task_id": "a-1"}, now=1, rejection_reason=None
    )
    assert stopped["type"] == "TASK_ABORTED"
    assert stopped["task_type"] == "abort"
    assert tasks.active is None


def test_busy_task_rejected_and_duplicate_id_ignored():
    tasks = LineTrackingTasks()
    tasks.handle_event(event(), now=0, rejection_reason=None)
    second = tasks.handle_event(event(task_id=124), now=1, rejection_reason=None)
    assert second["type"] == "TASK_REJECTED"
    assert (
        tasks.handle_event(event(task_id=124), now=2, rejection_reason=None) is None
    )
    assert tasks.active.task_id == 123


@pytest.mark.parametrize(
    "duration", [0, -1, 2, float("nan"), float("inf"), True, "20"]
)
def test_invalid_duration_rejected(duration):
    tasks = LineTrackingTasks()
    state = tasks.handle_event(
        event(payload={"duration_sec": duration}), now=0, rejection_reason=None
    )
    assert state["type"] == "TASK_REJECTED"
    assert tasks.active is None


def test_sustained_unsafe_state_aborts_but_short_blockage_pauses():
    tasks = LineTrackingTasks(_enabled_policy(default_duration_sec=10, max_duration_sec=10))
    tasks.handle_event(event(), now=0, rejection_reason=None)
    assert tasks.tick(now=2.1, drive_reason="tracking") is None
    assert tasks.tick(now=3, drive_reason="path_unavailable") is None
    assert tasks.tick(now=4, drive_reason="tracking") is None
    assert tasks.tick(now=5, drive_reason="camera_stale") is None
    stopped = tasks.tick(now=7.1, drive_reason="camera_stale")
    assert stopped["type"] == "TASK_ABORTED"
    assert stopped["reason"] == "unsafe:camera_stale"


@pytest.mark.parametrize("reason", ["tracking_path_hold", "tracking_slow_turn", "tracking_path_recovery",
                                    "lidar_tracking", "lidar_avoiding", "lidar_returning"])
def test_held_or_slow_turn_remains_permitted_motion_until_task_duration_ends(reason):
    tasks = LineTrackingTasks(_enabled_policy(default_duration_sec=10, max_duration_sec=10))
    tasks.handle_event(event(), now=0)
    assert tasks.tick(now=2.1, drive_reason="tracking") is None
    for now in (3., 5., 8.):
        assert tasks.tick(now=now, drive_reason=reason) is None
    assert tasks.tick(now=10.1, drive_reason=reason)["type"] == "TASK_COMPLETED"


@pytest.mark.parametrize("reason", [
    "path_recovery_waiting", "path_recovery_scan_left", "path_recovery_hold_left",
    "path_recovery_scan_right", "path_recovery_hold_right", "path_recovery_return",
    "path_recovery_confirming",
])
def test_search_can_finish_before_unsafe_timeout_but_honors_task_deadline(reason):
    tasks = LineTrackingTasks(_enabled_policy(default_duration_sec=50, max_duration_sec=50))
    tasks.handle_event(event(), now=0)
    assert tasks.tick(now=2.1, drive_reason="tracking") is None
    for now in (3., 10., 20., 40.):
        assert tasks.tick(now=now, drive_reason=reason) is None
    terminal = tasks.tick(now=50.1, drive_reason=reason)
    assert terminal["type"] == "TASK_ABORTED"
    assert terminal["reason"] == "tracking_unavailable:" + reason


def test_exhausted_search_still_obeys_unsafe_timeout():
    tasks = LineTrackingTasks(_enabled_policy(default_duration_sec=50, max_duration_sec=50))
    tasks.handle_event(event(), now=0)
    tasks.tick(now=2.1, drive_reason="tracking")
    assert tasks.tick(now=3., drive_reason="path_recovery_exhausted") is None
    terminal = tasks.tick(now=5.1, drive_reason="path_recovery_exhausted")
    assert terminal["reason"] == "unsafe:path_recovery_exhausted"


def test_slow_turn_can_start_tracking_after_startup_hold():
    tasks = LineTrackingTasks(_enabled_policy(default_duration_sec=10, max_duration_sec=10))
    tasks.handle_event(event(), now=0)
    assert tasks.tick(now=2.1, drive_reason="tracking_slow_turn") is None
    assert tasks.tracking_seen is True
    assert tasks.tick(now=3.0, drive_reason="path_lateral_target_large") is None
    result = tasks.tick(now=5.1, drive_reason="path_lateral_target_large")
    assert result["reason"] == "unsafe:path_lateral_target_large"


def test_startup_hold_aborts_at_deadline_when_never_ready():
    tasks = LineTrackingTasks(_enabled_policy(default_duration_sec=10, max_duration_sec=10))
    tasks.handle_event(event(), now=0, rejection_reason=None)
    assert tasks.tick(now=1.99, drive_reason="path_unavailable") is None
    stopped = tasks.tick(now=2.0, drive_reason="path_unavailable")
    assert stopped["type"] == "TASK_ABORTED"
    assert stopped["reason"] == "startup:path_unavailable"


def test_never_tracked_cannot_report_completed():
    tasks = LineTrackingTasks(
        _enabled_policy(default_duration_sec=3, max_duration_sec=10, unsafe_timeout_sec=5)
    )
    tasks.handle_event(event(), now=0, rejection_reason=None)
    result = tasks.tick(now=3.1, drive_reason="path_unavailable")
    assert result["type"] == "TASK_ABORTED"


def test_first_tracking_tick_at_deadline_is_not_false_completion():
    tasks = LineTrackingTasks(_enabled_policy(default_duration_sec=3, max_duration_sec=10))
    tasks.handle_event(event(), now=0, rejection_reason=None)
    result = tasks.tick(now=3, drive_reason="tracking")
    assert result["type"] == "TASK_ABORTED"


def test_string_task_id_is_normalized_before_core_report():
    tasks = LineTrackingTasks()
    started = tasks.handle_event(event(task_id=" 123 "), now=0, rejection_reason=None)
    assert started["task_id"] == "123"
    stopped = tasks.handle_event(
        {"type": "TASK_ABORTED", "task_id": 123}, now=1, rejection_reason=None
    )
    assert stopped["task_type"] == "abort"


def test_policy_rejects_subsecond_default_duration():
    with pytest.raises(ValueError, match="default duration"):
        LineTrackingTasks(_enabled_policy(default_duration_sec=0.5))


def test_default_policy_never_automatically_aborts_or_expires():
    tasks = LineTrackingTasks()
    assert all(getattr(tasks.policy, 'stop_on_' + name) is False for name in TASK_STOP_CHECKS)
    tasks.handle_event(event(payload={'duration_sec': 3}), now=0)
    for now in (0.1, 2.1, 4, 10001):
        assert tasks.tick(now=now, drive_reason='waiting_for_path') is None
        assert tasks.active is not None
        assert tasks.unsafe_since is None
    assert tasks.tick(now=10002, drive_reason='tracking') is None
    assert tasks.tracking_seen is True
    result = tasks.handle_event({**event(), 'type': 'TASK_ABORTED'}, now=10003)
    assert result['reason'] == 'task_aborted_by_server'
    assert tasks.active is None


@pytest.mark.parametrize('hold', [False, True])
def test_startup_hold_is_independent_of_readiness_and_timeouts(hold):
    tasks = LineTrackingTasks(TaskPolicy(stop_on_startup_hold=hold))
    tasks.handle_event(event(), now=0)
    assert tasks.tick(now=0.1, drive_reason='tracking') is None
    assert tasks.tracking_seen is (not hold)
    assert tasks.tick(now=2.1, drive_reason='tracking') is None
    assert tasks.tracking_seen


@pytest.mark.parametrize(('check', 'reason'), [
    ('startup_unready', 'startup:waiting_for_path'),
    ('unsafe_timeout', 'unsafe:waiting_for_path'),
    ('task_timeout', 'tracking_unavailable:waiting_for_path'),
])
def test_lifecycle_guards_can_be_reenabled_independently(check, reason):
    tasks = LineTrackingTasks(TaskPolicy(**{'stop_on_' + check: True}))
    tasks.handle_event(event(payload={'duration_sec': 3}), now=0)
    assert tasks.tick(now=0, drive_reason='waiting_for_path') is None
    result = tasks.tick(now=3, drive_reason='waiting_for_path')
    assert result['type'] == 'TASK_ABORTED'
    assert result['reason'] == reason
    assert tasks.active is None


def test_task_timeout_alone_completes_tracking_task():
    tasks = LineTrackingTasks(TaskPolicy(stop_on_task_timeout=True))
    tasks.handle_event(event(payload={'duration_sec': 3}), now=0)
    assert tasks.tick(now=0, drive_reason='tracking') is None
    assert tasks.tick(now=3, drive_reason='tracking')['type'] == 'TASK_COMPLETED'


@pytest.mark.parametrize('check', TASK_STOP_CHECKS)
def test_task_stop_flags_require_boolean_values(check):
    with pytest.raises(ValueError, match='boolean'):
        TaskPolicy(**{'stop_on_' + check: 'false'}).validate()
