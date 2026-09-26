import math
import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import swin_l_local_path_debug as debug  # noqa: E402
from local_path import SmoothedPath  # noqa: E402
from swin_l_drive_control import (  # noqa: E402
    DriveConfig,
    decide_drive,
)


def _path(*, lateral: float = 0.2, confidence: float = 0.9, age: float = 0.1):
    return SmoothedPath(
        points_xy=np.column_stack(
            (np.linspace(3.0, 8.0, 20), np.full(20, lateral))
        ).astype(np.float32),
        confidence=confidence,
        age_sec=age,
        source="test",
    )


def _decide(path=None, **overrides):
    arguments = dict(
        camera_age_sec=0.1,
        inference_age_sec=0.1,
        config=DriveConfig(),
    )
    arguments.update(overrides)
    return decide_drive(path or _path(), **arguments)


def test_fresh_path_generates_capped_a2_command():
    command = _decide()
    assert command.reason == "tracking"
    assert command.vx == pytest.approx(0.50)
    assert command.vy == 0.0
    assert 0.0 < command.yaw_rate <= 0.18


def test_task_drive_forward_speed_comes_from_environment(monkeypatch):
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_MAX_FORWARD_MPS", "0.35")

    args = debug.parse_args(["task-drive"])
    command = _decide(config=debug._drive_config_from_args(args))

    assert args.max_forward_mps == pytest.approx(0.35)
    assert command.vx == pytest.approx(0.35)


def test_forward_speed_hard_limit_is_one_meter_per_second():
    command = _decide(config=DriveConfig(max_forward_mps=1.00))
    assert command.vx == pytest.approx(1.00)

    with pytest.raises(ValueError, match="1.0"):
        DriveConfig(max_forward_mps=1.0001).validate()


def test_right_path_turns_right_without_lateral_velocity():
    command = _decide(_path(lateral=-0.2))
    assert command.reason == "tracking"
    assert command.vy == 0.0
    assert -0.18 <= command.yaw_rate < 0.0


def test_unrestricted_path_mode_disables_confidence_gate(monkeypatch):
    monkeypatch.setitem(debug.ENV, "SWIN_L_UNRESTRICTED_PATH_MODE", "true")
    args = debug.parse_args(["task-drive"])
    config = debug._drive_config_from_args(args)

    assert args.unrestricted_path_mode is True
    assert config.min_confidence == 0.0
    assert _decide(_path(confidence=0.01), config=config).reason == "tracking"


def test_unrestricted_path_mode_is_enabled_by_default(monkeypatch):
    monkeypatch.delitem(debug.ENV, "SWIN_L_UNRESTRICTED_PATH_MODE", raising=False)

    args = debug.parse_args(["task-drive"])

    assert args.unrestricted_path_mode is True


@pytest.mark.parametrize("enabled", [True, False])
def test_path_quality_stop_switches_come_from_env(monkeypatch, enabled):
    monkeypatch.setitem(debug.ENV, "SWIN_L_UNRESTRICTED_PATH_MODE", "false")
    for check in ("LOW_CONFIDENCE", "LATERAL_TARGET"):
        monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_" + check, str(enabled))
    config = debug._drive_config_from_args(debug.parse_args(["task-drive"]))

    low_confidence = _decide(_path(confidence=0.1), config=config)
    far_target = _decide(_path(lateral=2.0), config=config)

    assert low_confidence.reason == ("path_low_confidence" if enabled else "tracking")
    assert far_target.reason == ("path_lateral_target_large" if enabled else "tracking")
    if not enabled:
        assert far_target.vx == pytest.approx(config.max_forward_mps)
        assert far_target.yaw_rate == pytest.approx(config.max_yaw_rps)


@pytest.mark.parametrize("source", ["camera", "inference"])
@pytest.mark.parametrize("age", [None, 5.1, float("nan"), -1.0])
def test_disabling_quality_stops_cannot_bypass_input_freshness(source, age):
    config = DriveConfig(stop_on_low_confidence=False, stop_on_lateral_target=False)

    command = _decide(
        _path(confidence=0.1, lateral=2.0),
        config=config,
        **{f"{source}_age_sec": age},
    )

    assert command.reason == f"{source}_stale"
    assert (command.vx, command.vy, command.yaw_rate) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("path", [None, _path(confidence=float("nan"))])
def test_disabling_quality_stops_still_requires_usable_path(path):
    command = decide_drive(
        path,
        camera_age_sec=0.1,
        inference_age_sec=0.1,
        config=DriveConfig(stop_on_low_confidence=False, stop_on_lateral_target=False),
    )

    assert command.reason in ("path_unavailable", "path_low_confidence")
    assert (command.vx, command.vy, command.yaw_rate) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    "override,reason",
    [
        ({"camera_age_sec": None}, "camera_stale"),
        ({"camera_age_sec": 5.1}, "camera_stale"),
        ({"inference_age_sec": 5.1}, "inference_stale"),
        ({"path": _path(confidence=0.48)}, "path_low_confidence"),
        ({"path": _path(lateral=1.0)}, "path_lateral_target_large"),
    ],
)
def test_unsafe_inputs_return_zero_velocity(override, reason):
    command = _decide(**override)
    assert command.reason == reason
    assert (command.vx, command.vy, command.yaw_rate) == (0.0, 0.0, 0.0)


def test_missing_path_stops():
    assert (
        decide_drive(
            None,
            camera_age_sec=0.1,
            inference_age_sec=0.1,
            config=DriveConfig(),
        ).reason
        == "path_unavailable"
    )


@pytest.mark.parametrize("confidence", [0.1, float("nan"), float("inf")])
@pytest.mark.parametrize("lateral", [-2.0, 2.0])
def test_master_bypass_overrides_all_confidence_and_lateral_checks(confidence, lateral):
    command = _decide(
        _path(confidence=confidence, lateral=lateral),
        config=DriveConfig(bypass_path_stops=True),
    )
    assert command.reason == "tracking"
    assert command.vx == 0.5
    assert command.yaw_rate == pytest.approx(math.copysign(0.18, lateral))


@pytest.mark.parametrize("points", [None, [], [["bad", 0.2]], [[3., float("nan")]]])
@pytest.mark.parametrize("bypass", [False, True])
@pytest.mark.parametrize("yaw", [-0.9, -0.12, 0., 0.12, 0.9])
def test_missing_or_unusable_target_holds_yaw_only_with_master_bypass(points, bypass, yaw):
    path = None if points is None else replace(_path(), points_xy=np.asarray(points))
    command = decide_drive(
        path, camera_age_sec=0.1, inference_age_sec=0.1,
        config=DriveConfig(bypass_path_stops=bypass), last_valid_yaw_rate=yaw,
    )
    assert command.reason == ("tracking_path_hold" if bypass else "path_unavailable")
    assert command.vx == (0.5 if bypass else 0.0)
    assert command.yaw_rate == (max(-0.18, min(0.18, yaw)) if bypass else 0.0)


@pytest.mark.parametrize("yaw", [None, float("nan"), float("inf")])
def test_master_bypass_requires_a_finite_previous_yaw_for_missing_paths(yaw):
    command = decide_drive(
        None, camera_age_sec=0.1, inference_age_sec=0.1,
        config=DriveConfig(bypass_path_stops=True), last_valid_yaw_rate=yaw,
    )
    assert command.reason == "path_unavailable"
    assert command.vx == command.yaw_rate == 0.0


@pytest.mark.parametrize("points", [None, [], [["bad", 0.2]], [[3., float("nan")]]])
@pytest.mark.parametrize("count", [1, 4, 5, 6, 100])
def test_path_loss_bypass_expires_on_the_fifth_inference(points, count):
    path = None if points is None else replace(_path(), points_xy=np.asarray(points))
    command = decide_drive(
        path, camera_age_sec=0.1, inference_age_sec=0.1,
        config=DriveConfig(bypass_path_stops=True), last_valid_yaw_rate=0.12,
        path_unavailable_inferences=count,
    )
    assert command.reason == ("tracking_path_hold" if count < 5 else "path_unavailable")
    assert command.vx == (0.5 if count < 5 else 0.0)
    assert command.yaw_rate == (0.12 if count < 5 else 0.0)


@pytest.mark.parametrize("source", ["camera", "inference"])
@pytest.mark.parametrize("age", [None, -1., 5.01, float("nan"), float("inf")])
def test_master_bypass_never_overrides_sensor_freshness(source, age):
    ages = {"camera_age_sec": 0.1, "inference_age_sec": 0.1}
    ages[f"{source}_age_sec"] = age
    command = decide_drive(
        None, **ages, config=DriveConfig(bypass_path_stops=True),
        last_valid_yaw_rate=0.12,
    )
    assert command.reason == f"{source}_stale"
    assert (command.vx, command.vy, command.yaw_rate) == (0., 0., 0.)


@pytest.mark.parametrize("age", [0.46, 2.0 / 3.0, 1.0, 5.0, float("nan")])
def test_path_age_alone_does_not_interrupt_tracking(age):
    command = _decide(_path(age=age))

    assert command == _decide(_path(age=0.0))
    assert command.reason == "tracking"


@pytest.mark.parametrize("source", ["camera", "inference"])
def test_old_path_still_obeys_sensor_freshness(source):
    command = _decide(_path(age=6.0), **{f"{source}_age_sec": 5.1})

    assert command.reason == f"{source}_stale"
    assert (command.vx, command.vy, command.yaw_rate) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    "points,lateral",
    [
        ([[5.0, 0.6], [3.0, 0.2]], 0.4),
        ([[3.0, 0.2], [3.0, -0.2], [5.0, 0.6]], 0.4),
        ([[3.0, 0.2], [3.5, 0.4]], 0.4),
        ([[5.0, -0.2], [8.0, -0.4]], -0.2),
        ([[3.0, 0.2]], 0.2),
        ([[-1.0, 0.1], [3.0, 0.3]], 0.3),
        (
            [[3.0, 0.2], [float("nan"), 0.7], [4.0, float("inf")], [5.0, 0.6]],
            0.4,
        ),
    ],
)
def test_partial_or_unordered_paths_use_available_numeric_points(points, lateral):
    path = replace(_path(), points_xy=np.asarray(points, dtype=np.float64))

    command = _decide(path)

    assert command.reason == "tracking"
    assert command.vx == pytest.approx(0.5)
    assert command.yaw_rate == pytest.approx(math.atan2(lateral, 4.0))


@pytest.mark.parametrize(
    "points",
    [
        [],
        [3.0, 0.2, 4.0],
        [[3.0, float("nan")], [float("inf"), 0.0]],
        [["bad", 0.2]],
    ],
)
def test_path_without_numeric_xy_points_is_unavailable(points):
    command = _decide(replace(_path(), points_xy=np.asarray(points)))

    assert command.reason == "path_unavailable"
    assert (command.vx, command.vy, command.yaw_rate) == (0.0, 0.0, 0.0)


def test_endpoint_fallback_still_obeys_lateral_target_limit():
    path = replace(_path(), points_xy=np.asarray([[3.0, 0.8]], dtype=np.float64))

    command = _decide(path)

    assert command.reason == "path_lateral_target_large"
    assert (command.vx, command.vy, command.yaw_rate) == (0.0, 0.0, 0.0)


def test_task_drive_preflight_requires_pinned_model():
    args = debug.parse_args(["task-drive"])
    assert args.profile == "swin-l-aspect-224x384-fp16"
    debug._validate_task_drive_preflight(args)
    args.profile = "swin-l-best-so-far"
    with pytest.raises(ValueError, match="pinned"):
        debug._validate_task_drive_preflight(args)
    args.profile = "swin-l-aspect-224x384-fp16"
    args.output_hz = 5.0
    with pytest.raises(ValueError, match="at least 10 Hz"):
        debug._validate_task_drive_preflight(args)


def test_cobiz_task_listener_pins_swin_profile():
    args = debug.parse_args(["task-drive"])
    assert args.task_event_topic == "/task_event"
    assert args.task_state_topic == "/task_state"
    debug._validate_task_drive_preflight(args)
    args.profile = "swin-l-best-so-far"
    with pytest.raises(ValueError, match="pinned"):
        debug._validate_task_drive_preflight(args)


@pytest.mark.parametrize("profile", [debug.SWIN_L_ASPECT_FP16_PROFILE, debug.R50_PROFILE])
def test_task_drive_prohibits_automatic_backend_fallback(profile):
    args = debug.parse_args(
        ["task-drive", "--profile", profile, "--backend", "pytorch", "--allow-backend-fallback"]
    )
    with pytest.raises(ValueError, match="prohibits"):
        debug._validate_task_drive_preflight(args)


def test_task_drive_selects_pinned_r50_runtime_from_environment(monkeypatch):
    monkeypatch.setitem(debug.ENV, "SWIN_L_PROFILE", debug.R50_PROFILE)
    monkeypatch.setitem(debug.ENV, "SWIN_L_BACKEND", "pytorch")
    monkeypatch.setitem(debug.ENV, "SWIN_L_DEVICE", "cuda")
    args = debug.parse_args(["task-drive"])

    debug._validate_task_drive_preflight(args)
    config = debug._runtime_config(args)
    config.validate()
    profile = debug.resolve_profile(config.profile)
    assert profile.model_id == "facebook/maskformer-resnet50-vistas"
    assert profile.model_revision == "ae4b8c2590c0a090fc32d5c217d78738a2dd4b19"
    assert (profile.input_height, profile.input_width) == (360, 640)
    assert profile.precision == "fp16"
    assert config.backend == "pytorch"
    assert config.device == "cuda"


def test_task_drive_rejects_r50_with_swin_tensorrt_engine():
    args = debug.parse_args(
        ["task-drive", "--profile", debug.R50_PROFILE, "--backend", "tensorrt"]
    )
    with pytest.raises(ValueError, match="R50.*pytorch"):
        debug._validate_task_drive_preflight(args)


@pytest.mark.parametrize("profile", [debug.SWIN_L_ASPECT_FP16_PROFILE, debug.R50_PROFILE])
@pytest.mark.parametrize("field", ["model_id", "model_revision"])
def test_task_drive_keeps_each_profiles_checkpoint_pinned(profile, field):
    pinned = debug.resolve_profile(profile)
    args = debug.parse_args(
        [
            "task-drive", "--profile", profile, "--backend", "pytorch",
            "--model-id", pinned.model_id, "--model-revision", pinned.model_revision,
        ]
    )
    debug._validate_task_drive_preflight(args)
    setattr(args, field, "different-checkpoint")
    with pytest.raises(ValueError, match="cannot override the pinned checkpoint"):
        debug._validate_task_drive_preflight(args)


@pytest.mark.parametrize(
    "arguments,message",
    [
        (["--evaluation-size", "720", "1280"], "360x640"),
        (["--path-frame-id", "camera"], "base_link"),
        (["--output-hz", "5"], "at least 10 Hz"),
    ],
)
def test_r50_keeps_live_path_and_control_requirements(arguments, message):
    args = debug.parse_args(
        ["task-drive", "--profile", debug.R50_PROFILE, "--backend", "pytorch", *arguments]
    )
    with pytest.raises(ValueError, match=message):
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
