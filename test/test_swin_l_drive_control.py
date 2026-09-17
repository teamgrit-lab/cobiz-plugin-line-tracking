from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from local_path import LidarSafetyResult, SmoothedPath  # noqa: E402
from swin_l_drive_control import (  # noqa: E402
    DriveConfig,
    decide_drive,
    lidar_frame_matches_base,
)
import swin_l_local_path_debug as debug  # noqa: E402


def _path(*, lateral: float = 0.2, confidence: float = 0.9, age: float = 0.1):
    return SmoothedPath(
        points_xy=np.column_stack(
            (np.linspace(3.0, 8.0, 20), np.full(20, lateral))
        ).astype(np.float32),
        confidence=confidence,
        age_sec=age,
        source="test",
    )


def _safety(*, stop: bool = False, available: bool = True, clearance=None, age=0.1):
    return LidarSafetyResult(
        stop=stop,
        lidar_available=available,
        obstacle_in_path=clearance is not None,
        obstacle_count=1 if clearance is not None else 0,
        clearance_m=clearance,
        age_sec=age if available else None,
        reason="obstacle_in_path" if stop else "clear",
    )


def _decide(path=None, safety=None, **overrides):
    arguments = dict(
        camera_age_sec=0.1,
        inference_age_sec=0.1,
        other_control_publishers=False,
        enabled=True,
        calibrated=True,
        config=DriveConfig(),
    )
    arguments.update(overrides)
    return decide_drive(path or _path(), safety or _safety(), **arguments)


def test_calibrated_fresh_path_generates_capped_a2_command():
    command = _decide()
    assert command.reason == "tracking"
    assert command.vx == pytest.approx(0.10)
    assert command.vy == 0.0
    assert 0.0 < command.yaw_rate <= 0.18
    assert command.joy_axes() == pytest.approx([0.0, -0.10, command.yaw_rate])
    # a2_control_node negates axes[2], compensating the observed yaw reversal.
    assert -command.joy_axes()[2] == pytest.approx(-command.yaw_rate)


def test_right_path_turns_right_without_lateral_joy():
    command = _decide(_path(lateral=-0.2))
    assert command.reason == "tracking"
    assert command.vy == 0.0
    assert -0.18 <= command.yaw_rate < 0.0
    assert command.joy_axes()[2] < 0.0


def test_a2_joy_axes_correct_both_observed_left_right_reversals():
    from swin_l_drive_control import DriveDecision

    command = DriveDecision(vx=0.10, vy=0.04, yaw_rate=0.08, reason="tracking")
    axes = command.joy_axes()
    assert axes == pytest.approx([0.04, -0.10, 0.08])
    # This is exactly what cobiz-plugin-a2 passes to SportClient::Move.
    assert (-axes[1], -axes[0], -axes[2]) == pytest.approx((0.10, -0.04, -0.08))


@pytest.mark.parametrize(
    "override,reason",
    [
        ({"enabled": False}, "drive_not_armed"),
        ({"calibrated": False}, "drive_not_armed"),
        ({"other_control_publishers": True}, "multiple_control_publishers"),
        ({"camera_age_sec": None}, "camera_stale"),
        ({"camera_age_sec": 0.6}, "camera_stale"),
        ({"inference_age_sec": 0.6}, "inference_stale"),
        ({"path": _path(age=0.6)}, "path_stale"),
        ({"path": _path(confidence=0.5)}, "path_low_confidence"),
        ({"path": _path(lateral=1.0)}, "path_lateral_target_large"),
        ({"safety": _safety(stop=True)}, "lidar_obstacle_in_path"),
        ({"safety": _safety(available=False)}, "lidar_unavailable"),
        ({"safety": _safety(age=0.5)}, "lidar_stale"),
        ({"safety": _safety(clearance=2.0)}, "lidar_clearance_low"),
    ],
)
def test_unsafe_inputs_publish_zero_joy(override, reason):
    path = override.pop("path", _path())
    safety = override.pop("safety", _safety())
    command = _decide(path, safety, **override)
    assert command.reason == reason
    assert command.joy_axes() == [0.0, 0.0, 0.0]


def test_missing_or_malformed_path_stops():
    assert (
        decide_drive(
            None,
            _safety(),
            camera_age_sec=0.1,
            inference_age_sec=0.1,
            other_control_publishers=False,
            enabled=True,
            calibrated=True,
            config=DriveConfig(),
        ).reason
        == "path_unavailable"
    )
    malformed = SmoothedPath(
        points_xy=np.asarray([[4.0, 0.0], [3.0, 0.0]], np.float32),
        confidence=0.9,
        age_sec=0.0,
        source="test",
    )
    assert _decide(malformed).reason == "path_geometry_invalid"


def test_drive_rejects_untransformed_lidar_frames():
    assert lidar_frame_matches_base("base_link", "base_link")
    assert lidar_frame_matches_base("/base_link", "base_link")
    assert not lidar_frame_matches_base("hesai_lidar", "base_link")
    assert not lidar_frame_matches_base("base_link", "map")


def test_task_drive_preflight_requires_pinned_model():
    args = debug.parse_args(["task-drive"])
    assert args.profile == "swin-l-aspect-224x384"
    debug._validate_task_drive_preflight(args)
    args.profile = "swin-l-best-so-far"
    with pytest.raises(ValueError, match="pinned"):
        debug._validate_task_drive_preflight(args)
    args.profile = "swin-l-aspect-224x384"
    args.output_hz = 5.0
    with pytest.raises(ValueError, match="at least 10 Hz"):
        debug._validate_task_drive_preflight(args)


def test_cobiz_task_listener_starts_unarmed_but_still_pins_swin(monkeypatch):
    monkeypatch.setitem(debug.ENV, "SWIN_L_DRIVE_ENABLED", "false")
    monkeypatch.setitem(debug.ENV, "SWIN_L_CALIBRATION_CONFIRMED", "false")
    args = debug.parse_args(["task-drive"])
    assert args.task_event_topic == "/task_event"
    assert args.task_state_topic == "/task_state"
    debug._validate_task_drive_preflight(args)
    args.profile = "swin-l-best-so-far"
    with pytest.raises(ValueError, match="pinned"):
        debug._validate_task_drive_preflight(args)


def test_ros_source_stamp_is_decoded_for_monotonic_checks():
    header = SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=34))
    assert debug._stamp_ns(header) == 12_000_000_034


def test_live_drive_rejects_recorded_and_future_sensor_stamps():
    now = 100_000_000_000
    assert debug._live_source_stamp(now - 100_000_000, now, 0.5)
    assert not debug._live_source_stamp(now - 600_000_000, now, 0.5)
    assert not debug._live_source_stamp(now + 100_000_000, now, 0.5)
    assert not debug._live_source_stamp(0, now, 0.5)


def test_drive_watchdog_uses_original_sensor_age_after_inference():
    age = debug._effective_source_age_sec(
        arrival_sec=10.0,
        source_stamp_ns=99_600_000_000,
        current_monotonic_sec=10.2,
        current_stamp_ns=100_200_000_000,
    )
    assert age == pytest.approx(0.6)
    assert (
        debug._effective_source_age_sec(None, 99_000_000_000, 10, 100_000_000_000)
        is None
    )
