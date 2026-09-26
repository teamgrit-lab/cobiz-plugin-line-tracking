import math
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from adaptive_path_control import (
    AdaptiveControlConfig,
    AdaptivePathController,
    path_geometry,
)
from local_path import SmoothedPath
from swin_l_drive_control import DriveConfig


def path(degrees=0.0):
    x = np.linspace(3.0, 8.0, 20)
    return SmoothedPath(
        points_xy=np.column_stack((x, x * math.tan(math.radians(degrees)))),
        confidence=0.9,
        age_sec=0.1,
        source="test",
    )


def controller(**overrides):
    return AdaptivePathController(DriveConfig(), AdaptiveControlConfig(**overrides))


def step(control, now, degrees=0, frame=1, age=0.1, **kwargs):
    return control.update(
        path(degrees),
        now=now,
        camera_age_sec=0.1,
        inference_age_sec=age,
        inference_id=frame,
        **kwargs,
    )


def warm(control, degrees=0):
    for now in np.arange(0.0, 2.1, 0.1):
        decision = step(control, float(now), degrees, int(now * 10))
    return decision


def start_pulse(control, degrees=30):
    assert step(control, 0, degrees).reason == "turn_braking"
    assert step(control, 0.7, degrees, frame=2, age=0.05).reason == "turn_waiting_frame"
    result = step(control, 1.1, degrees, frame=3, age=0.05)
    assert result.reason == "turn_aligning"
    return result


def test_straight_path_ramps_up_and_keeps_speed_and_yaw_limits():
    control = controller()
    assert step(control, 0).vx == 0.0
    assert step(control, 0.1).vx == pytest.approx(0.03)
    # A delayed control tick must not produce an acceleration jump.
    assert step(control, 1.1).vx == pytest.approx(0.09)
    control.reset()
    result = warm(control)
    assert result.vx == pytest.approx(0.5)
    assert result.vy == result.yaw_rate == 0


def test_bend_ahead_slows_before_target_exceeds_lateral_guard():
    control = controller()
    warm(control)
    curved = path()
    curved.points_xy[:, 1] = 0.7 * (curved.points_xy[:, 0] - 3) ** 2
    result = control.update(
        curved, now=2.1, camera_age_sec=0.1, inference_age_sec=0.1, inference_id=30
    )
    assert result.reason == "tracking_slow_curve"
    assert 0 < result.vx < 0.2
    assert 0 < result.yaw_rate <= 0.18


@pytest.mark.parametrize("degrees", [-5, 5])
def test_tracking_curvature_preserves_left_right_sign(degrees):
    result = warm(controller(), degrees)
    assert result.yaw_rate * degrees > 0
    assert abs(result.yaw_rate) <= 0.18


def test_age_regulation_slow_stop_and_gradual_recovery():
    control = controller()
    warm(control)
    assert step(control, 2.1, age=0.85).vx == pytest.approx(0.25)
    stopped = step(control, 2.2, age=1.2)
    assert stopped.reason == "perception_delay_stop"
    assert stopped.vx == stopped.yaw_rate == 0
    assert step(control, 2.3, age=0.1).vx == pytest.approx(0.03)


def test_sensor_age_not_path_update_time_controls_delay():
    control = controller()
    fresh_looking_path = replace(path(), age_sec=0)
    result = control.update(
        fresh_looking_path,
        now=0,
        camera_age_sec=0.1,
        inference_age_sec=1.3,
        inference_id=1,
    )
    assert result.reason == "perception_delay_stop"


@pytest.mark.parametrize("degrees", [-30, 30])
def test_turn_requires_stop_and_distinct_frames_captured_after_stop(degrees):
    control = controller()
    assert step(control, 0, degrees).reason == "turn_braking"
    assert step(control, 0.4, degrees, frame=2).reason == "turn_braking"
    # Newly delivered result was captured while braking: cannot authorize yaw.
    assert step(control, 0.7, degrees, frame=3, age=0.3).yaw_rate == 0
    assert step(control, 0.8, degrees, frame=4).yaw_rate == 0
    for now in (0.9, 1.0, 1.1):
        result = step(control, now, degrees, frame=4, age=now - 0.7)
        assert result.reason == "turn_waiting_frame"
        assert result.yaw_rate == 0
    turn = step(control, 1.2, degrees, frame=5)
    assert turn.vx == turn.vy == 0
    assert turn.yaw_rate == pytest.approx(math.copysign(0.18, degrees))
    assert step(control, 1.4, degrees, frame=5, age=0.3).reason == "turn_aligning"
    assert (
        step(control, 1.56, degrees, frame=5, age=0.46).reason == "turn_waiting_frame"
    )
    # Even a new result captured before post-pulse settling cannot restart yaw.
    assert step(control, 1.8, degrees, frame=6, age=0.2).yaw_rate == 0


def test_alignment_hysteresis_and_reacquisition_before_forward_motion():
    control = controller()
    start_pulse(control)
    assert step(control, 1.46, 8, frame=3).reason == "turn_waiting_frame"
    assert step(control, 1.8, 8, frame=4).yaw_rate == 0
    assert step(control, 2.2, 8, frame=5).reason == "turn_aligning"
    # A new aligned observation ends the pulse immediately.
    assert step(control, 2.3, 3, frame=6).reason == "turn_waiting_frame"
    assert step(control, 2.7, 3, frame=7).vx == 0
    result = step(control, 3.1, 3, frame=8)
    assert result.reason == "turn_reacquired"
    assert result.vx == result.yaw_rate == 0
    assert step(control, 3.2, 3, frame=8, age=0.2).vx == pytest.approx(0.03)


def test_conflicting_turn_directions_cannot_confirm_a_pulse():
    control = controller()
    step(control, 0, 30)
    for frame, now in enumerate((0.8, 1.2, 1.6, 2.0), start=2):
        assert step(control, now, 30 if frame % 2 else -30, frame=frame).yaw_rate == 0


def test_large_heading_jumps_reset_confirmation_even_in_same_direction():
    control = controller()
    step(control, 0, 20)
    assert step(control, 0.8, 20, frame=2).yaw_rate == 0
    assert step(control, 1.2, 45, frame=3).yaw_rate == 0
    assert control.confirmations == 1
    assert step(control, 1.6, 43, frame=4).reason == "turn_aligning"


def test_opposite_feedback_stops_pulse_before_any_reverse_turn():
    control = controller()
    start_pulse(control)
    assert step(control, 1.2, -30, frame=4).reason == "turn_waiting_frame"
    assert step(control, 1.3, -30, frame=5).yaw_rate == 0


@pytest.mark.parametrize(
    "fault", ["missing", "nan_confidence", "invalid_points", "late"]
)
def test_fault_interrupts_yaw_even_with_legacy_bypass(fault):
    control = AdaptivePathController(
        DriveConfig(bypass_path_stops=True), AdaptiveControlConfig()
    )
    start_pulse(control)
    current = path(30)
    age = 0.1
    if fault == "missing":
        current = None
    elif fault == "nan_confidence":
        current = replace(current, confidence=float("nan"))
    elif fault == "invalid_points":
        current = replace(current, points_xy=np.array([[3, float("nan")]]))
    else:
        age = 1.2
    result = control.update(
        current, now=1.2, camera_age_sec=0.1, inference_age_sec=age, inference_id=4
    )
    assert result.vx == result.yaw_rate == 0
    assert control.phase == "waiting_frame"


@pytest.mark.parametrize("source", ["camera", "inference"])
@pytest.mark.parametrize("age", [None, -1, float("nan"), float("inf"), 5.1])
def test_camera_and_inference_guards_remain_immediate(source, age):
    control = controller()
    start_pulse(control)
    ages = {"camera_age_sec": 0.1, "inference_age_sec": 0.1}
    ages[source + "_age_sec"] = age
    result = control.update(path(30), now=1.2, inference_id=4, **ages)
    assert result.reason == source + "_stale"
    assert result.vx == result.yaw_rate == 0


def test_recovery_timeout_latches_until_task_reset():
    control = controller(turn_timeout_sec=2.0)
    start_pulse(control)
    result = step(control, 2.1, 30, frame=4)
    assert result.reason == "turn_timeout"
    assert step(control, 2.2, 0, frame=5).reason == "turn_timeout"
    control.reset()
    assert step(control, 2.3, 0, frame=6).reason == "tracking"


def test_inactive_task_or_startup_hold_cannot_accumulate_rotation_permission():
    control = controller()
    for frame in range(5):
        result = step(control, float(frame), 30, frame=frame, motion_allowed=False)
        assert result.vx == result.yaw_rate == 0
        assert control.phase == "tracking"
    assert step(control, 5.0, 30, frame=5).reason == "turn_braking"


def test_clock_reversal_stops_and_clears_turn():
    control = controller()
    start_pulse(control)
    assert step(control, 0.5, 30).reason == "control_clock_invalid"
    assert control.phase == "tracking"


def test_geometry_handles_reverse_order_and_rejects_missing_support():
    straight = path(4)
    expected = path_geometry(straight, 4)
    assert path_geometry(
        replace(straight, points_xy=straight.points_xy[::-1]), 4
    ) == pytest.approx(expected)
    for points in ([], [[3, 0]], [[-1, 0], [-2, 0]], [[3, 0], [4, float("inf")]]):
        assert path_geometry(replace(straight, points_xy=points), 4) is None


@pytest.mark.parametrize(
    "values",
    [
        {"slow_age_sec": 2},
        {"stop_age_sec": float("nan")},
        {"turn_enter_deg": 5},
        {"turn_exit_deg": 20},
        {"turn_pulse_sec": 0.6},
        {"turn_confirm_frames": 1},
        {"turn_confirm_frames": 2.5},
        {"turn_timeout_sec": 0.5},
        {"max_accel_mps2": 0},
    ],
)
def test_invalid_settings_fail_before_motion(values):
    with pytest.raises(ValueError):
        controller(**values)
