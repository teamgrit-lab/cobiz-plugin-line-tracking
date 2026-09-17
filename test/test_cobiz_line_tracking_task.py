from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from cobiz_line_tracking_task import LineTrackingTasks, TaskPolicy  # noqa: E402


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
            ready_reason="tracking",
        )
        is None
    )
    assert (
        tasks.handle_event(
            {**event(), "action_name": "ARM_RESET"}, now=0, ready_reason="tracking"
        )
        is None
    )
    assert (
        tasks.handle_event({**event(), "task_id": ""}, now=0, ready_reason="tracking")
        is None
    )
    assert (
        tasks.handle_event(
            {**event(), "task_id": "../../bad"}, now=0, ready_reason="tracking"
        )
        is None
    )
    assert tasks.active is None


def test_calibration_or_sensor_failure_rejects_without_activation():
    tasks = LineTrackingTasks()
    state = tasks.handle_event(event(), now=0, ready_reason="drive_not_armed")
    assert state == {
        "type": "TASK_REJECTED",
        "task_id": 123,
        "task_type": "reject",
        "action_name": "LINE_TRACKING",
        "device_id": 45,
        "device_name": "Dangjin-A2",
        "reason": "drive_not_armed",
    }
    assert tasks.active is None
    assert tasks.handle_event(event(), now=1, ready_reason="tracking") is None


def test_start_requires_tracking_then_completes_finite_task():
    tasks = LineTrackingTasks(TaskPolicy(default_duration_sec=5, max_duration_sec=10))
    started = tasks.handle_event(
        event(payload={"duration_sec": 4}), now=10, ready_reason="tracking"
    )
    assert started["type"] == "TASK_STARTED"
    assert started["task_type"] == "start"
    assert tasks.tick(now=11, drive_reason="drive_not_armed") is None
    assert tasks.tick(now=12.1, drive_reason="tracking") is None
    completed = tasks.tick(now=14.1, drive_reason="tracking")
    assert completed["type"] == "TASK_COMPLETED"
    assert completed["task_type"] == "complete"
    assert tasks.active is None


@pytest.mark.parametrize("payload", [{"selected_mask": 1}, '{"selected_mask": 1}'])
def test_task_selects_road_from_object_or_json_payload(payload):
    tasks = LineTrackingTasks(TaskPolicy(default_selected_mask=2))

    started = tasks.handle_event(event(payload=payload), now=0, ready_reason="tracking")

    assert started["type"] == "TASK_STARTED"
    assert tasks.active.selected_mask == 1
    tasks.finish("TASK_COMPLETED")
    assert tasks.active is None


def test_task_uses_configured_mask_when_payload_omits_it():
    tasks = LineTrackingTasks(TaskPolicy(default_selected_mask=1))

    started = tasks.handle_event(
        event(payload={"duration_sec": 30}), now=0, ready_reason="tracking"
    )

    assert started["type"] == "TASK_STARTED"
    assert tasks.active.selected_mask == 1


@pytest.mark.parametrize("selected_mask", [0, 3, -1, True, 1.0, "1", None, {}, []])
def test_invalid_selected_mask_rejected_even_when_path_is_unavailable(selected_mask):
    tasks = LineTrackingTasks()

    state = tasks.handle_event(
        event(payload={"selected_mask": selected_mask}),
        now=0,
        ready_reason="path_unavailable",
    )

    assert state["type"] == "TASK_REJECTED"
    assert state["reason"] == "invalid_selected_mask"
    assert tasks.active is None


def test_server_abort_requires_active_matching_id_even_without_action_name():
    tasks = LineTrackingTasks()
    tasks.handle_event(event(task_id="a-1"), now=0, ready_reason="tracking")
    assert (
        tasks.handle_event(
            {"type": "TASK_ABORTED", "task_id": "other"}, now=1, ready_reason="tracking"
        )
        is None
    )
    assert (
        tasks.handle_event(
            {"type": "TASK_ABORTED", "task_id": "a-1", "action_name": "ARM_RESET"},
            now=1,
            ready_reason="tracking",
        )
        is None
    )
    stopped = tasks.handle_event(
        {"type": "TASK_ABORTED", "task_id": "a-1"}, now=1, ready_reason="tracking"
    )
    assert stopped["type"] == "TASK_ABORTED"
    assert stopped["task_type"] == "abort"
    assert tasks.active is None


def test_busy_task_rejected_and_duplicate_id_ignored():
    tasks = LineTrackingTasks()
    tasks.handle_event(event(), now=0, ready_reason="tracking")
    second = tasks.handle_event(event(task_id=124), now=1, ready_reason="tracking")
    assert second["type"] == "TASK_REJECTED"
    assert (
        tasks.handle_event(event(task_id=124), now=2, ready_reason="tracking") is None
    )
    assert tasks.active.task_id == 123


@pytest.mark.parametrize("duration", [0, -1, 2, 301, float("nan"), True, "20"])
def test_invalid_duration_rejected(duration):
    tasks = LineTrackingTasks()
    state = tasks.handle_event(
        event(payload={"duration_sec": duration}), now=0, ready_reason="tracking"
    )
    assert state["type"] == "TASK_REJECTED"
    assert tasks.active is None


def test_sustained_unsafe_state_aborts_but_short_blockage_pauses():
    tasks = LineTrackingTasks(TaskPolicy(default_duration_sec=10, max_duration_sec=10))
    tasks.handle_event(event(), now=0, ready_reason="tracking")
    assert tasks.tick(now=2.1, drive_reason="tracking") is None
    assert tasks.tick(now=3, drive_reason="lidar_clearance_low") is None
    assert tasks.tick(now=4, drive_reason="tracking") is None
    assert tasks.tick(now=5, drive_reason="lidar_unavailable") is None
    stopped = tasks.tick(now=7.1, drive_reason="lidar_unavailable")
    assert stopped["type"] == "TASK_ABORTED"
    assert stopped["reason"] == "unsafe:lidar_unavailable"


def test_never_tracked_cannot_report_completed():
    tasks = LineTrackingTasks(
        TaskPolicy(default_duration_sec=3, max_duration_sec=10, unsafe_timeout_sec=5)
    )
    tasks.handle_event(event(), now=0, ready_reason="tracking")
    result = tasks.tick(now=3.1, drive_reason="path_unavailable")
    assert result["type"] == "TASK_ABORTED"


def test_first_tracking_tick_at_deadline_is_not_false_completion():
    tasks = LineTrackingTasks(TaskPolicy(default_duration_sec=3, max_duration_sec=10))
    tasks.handle_event(event(), now=0, ready_reason="tracking")
    result = tasks.tick(now=3, drive_reason="tracking")
    assert result["type"] == "TASK_ABORTED"


def test_string_task_id_is_normalized_before_core_report():
    tasks = LineTrackingTasks()
    started = tasks.handle_event(event(task_id=" 123 "), now=0, ready_reason="tracking")
    assert started["task_id"] == "123"
    stopped = tasks.handle_event(
        {"type": "TASK_ABORTED", "task_id": 123}, now=1, ready_reason="tracking"
    )
    assert stopped["task_type"] == "abort"


def test_policy_rejects_subsecond_default_duration():
    with pytest.raises(ValueError, match="default duration"):
        LineTrackingTasks(TaskPolicy(default_duration_sec=0.5))
