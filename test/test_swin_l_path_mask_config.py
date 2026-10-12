from pathlib import Path
import sys

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import swin_l_local_path_debug as debug  # noqa: E402
from local_path import (  # noqa: E402
    LocalPathConfig,
    LocalPathSmoother,
    extract_sidewalk_centerline,
)
from cobiz_line_tracking_task import ActiveTask  # noqa: E402


@pytest.mark.parametrize("mask_class", [0, 1, 2])
@pytest.mark.parametrize("mode", ["ros2", "task-drive"])
def test_live_modes_use_the_env_selected_path_class(monkeypatch, mode, mask_class):
    monkeypatch.setitem(debug.ENV, "SWIN_L_PATH_MASK_CLASS", str(mask_class))

    assert debug.parse_args([mode]).path_mask_class == mask_class


def test_sidewalk_is_the_default_path_class(monkeypatch):
    monkeypatch.delitem(debug.ENV, "SWIN_L_PATH_MASK_CLASS", raising=False)

    assert debug.parse_args(["ros2"]).path_mask_class == 2


def test_street_light_scenario_uses_road_and_is_task_drive_only():
    assert debug.parse_args(["task-drive", "--path-mask-class", "3"]).path_mask_class == 3
    active = ActiveTask("light", "light", None, None, 10.0, 30.0, 3)
    assert debug.active_path_mask_class(active, 2) == 1
    assert debug.active_path_mask_class(None, 3) == 1
    with pytest.raises(SystemExit):
        debug.parse_args(["ros2", "--path-mask-class", "3"])


@pytest.mark.parametrize("mask_class", [0, 1, 2])
def test_command_line_can_override_env_for_offline_path_preview(
    monkeypatch, tmp_path, mask_class
):
    monkeypatch.setitem(debug.ENV, "SWIN_L_PATH_MASK_CLASS", "2")

    args = debug.parse_args(
        [
            "mcap",
            "--input",
            str(tmp_path / "input.mcap"),
            "--output",
            str(tmp_path / "output.mp4"),
            "--path-mask-class",
            str(mask_class),
        ]
    )
    assert args.path_mask_class == mask_class


@pytest.mark.parametrize("value", ["-1", "4", "road", ""])
def test_invalid_path_class_fails_before_ros_starts(monkeypatch, value):
    monkeypatch.setitem(debug.ENV, "SWIN_L_PATH_MASK_CLASS", value)

    with pytest.raises(SystemExit) as error:
        debug.parse_args(["task-drive"])
    assert error.value.code == 2


def test_selected_path_region_excludes_other_classes():
    selected_mask = np.array([[0, 1, 2], [2, 1, 3]], dtype=np.uint8)

    np.testing.assert_array_equal(
        debug.selected_path_region(selected_mask, 1),
        [[False, True, False], [False, True, False]],
    )
    np.testing.assert_array_equal(
        debug.selected_path_region(selected_mask, 2),
        [[False, False, True], [True, False, False]],
    )
    np.testing.assert_array_equal(
        debug.selected_path_region(selected_mask, 0),
        [[False, True, True], [True, True, False]],
    )
    with pytest.raises(ValueError, match="must be 0 .* or 2"):
        debug.selected_path_region(selected_mask, 3)


@pytest.mark.parametrize("surface", [0, 1, 2])
@pytest.mark.parametrize("mask_class", [0, 1, 2])
def test_path_exists_only_when_the_scene_contains_a_requested_surface(surface, mask_class):
    mask = np.full((360, 640), surface, dtype=np.uint8)
    config = LocalPathConfig()

    path = extract_sidewalk_centerline(
        debug.selected_path_region(mask, mask_class), config
    )
    assert (path is not None) == (surface != 0 and mask_class in (0, surface))


def test_task_paths_are_independent_and_active_task_selects_its_own_path():
    mask = np.zeros((360, 640), dtype=np.uint8)
    mask[:, :320] = 1
    mask[:, 320:] = 2
    config = LocalPathConfig()
    smoothers = {key: LocalPathSmoother(config) for key in (0, 1, 2)}

    estimates = debug.update_path_smoothers(mask, smoothers, config, 10.0)
    road = smoothers[1].current(10.1)
    sidewalk = smoothers[2].current(10.1)
    combined = smoothers[0].current(10.1)

    assert all(estimate is not None for estimate in estimates.values())
    assert road is not None and sidewalk is not None
    assert road.points_xy[:, 1].mean() > 0
    assert sidewalk.points_xy[:, 1].mean() < 0
    assert combined is not None
    np.testing.assert_allclose(combined.points_xy[:, 1], 0.0, atol=0.02)
    for mask_class in (0, 1, 2):
        active = ActiveTask("t1", "t1", None, None, 10.0, 30.0, mask_class)
        assert debug.active_path_mask_class(active, 2) == mask_class
    assert debug.active_path_mask_class(None, 2) == 2


@pytest.mark.parametrize("mode", ["ros2", "task-drive"])
@pytest.mark.parametrize("preference", ["left", "right", "center"])
def test_branch_configuration_environment_and_cli(monkeypatch, mode, preference):
    monkeypatch.setitem(debug.ENV, "SWIN_L_BRANCH_PREFERENCE", "center")
    monkeypatch.setitem(debug.ENV, "SWIN_L_BRANCH_MIN_WIDTH_M", "0.8")
    monkeypatch.setitem(debug.ENV, "SWIN_L_BRANCH_MARGIN_M", "0.15")
    monkeypatch.setitem(debug.ENV, "SWIN_L_BRANCH_CONFIRM_FRAMES", "3")
    monkeypatch.setitem(debug.ENV, "SWIN_L_BRANCH_HOLD_SEC", "2.5")
    args = debug.parse_args([mode, "--branch-preference", preference])
    cfg = debug._local_path_config_from_args(args)
    assert cfg.branch_preference == preference
    assert cfg.branch_min_width_m == 0.8
    assert cfg.branch_margin_m == 0.15
    assert cfg.branch_confirm_frames == 3
    assert cfg.branch_hold_sec == 2.5


@pytest.mark.parametrize("mode", ["ros2", "task-drive", "mcap"])
@pytest.mark.parametrize("preference", ["left", "center"])
def test_preference_from_environment_reaches_path_config(monkeypatch, tmp_path, mode, preference):
    monkeypatch.setitem(debug.ENV, "SWIN_L_BRANCH_PREFERENCE", preference)
    arguments = [mode]
    if mode == "mcap":
        arguments += ["--input", str(tmp_path / "input.mcap"), "--output", str(tmp_path / "output.mp4")]
    args = debug.parse_args(arguments)
    assert debug._local_path_config_from_args(args).branch_preference == preference


@pytest.mark.parametrize("name,value", [
    ("PREFERENCE", "none"), ("MIN_WIDTH_M", "nan"), ("MARGIN_M", "-0.1"),
    ("CONFIRM_FRAMES", "1"), ("HOLD_SEC", "inf"),
])
def test_invalid_branch_environment_is_rejected_before_startup(monkeypatch, name, value):
    monkeypatch.setitem(debug.ENV, "SWIN_L_BRANCH_" + name, value)
    with pytest.raises(SystemExit):
        debug.parse_args(["task-drive"])


@pytest.mark.parametrize("mode", ["ros2", "task-drive", "mcap"])
def test_vegetation_bias_settings_reach_all_path_modes(monkeypatch, tmp_path, mode):
    monkeypatch.setitem(debug.ENV, "SWIN_L_SIDEWALK_VEGETATION_BIAS_M", "0.15")
    monkeypatch.setitem(debug.ENV, "SWIN_L_SIDEWALK_VEGETATION_MAX_GAP_M", "0.25")
    monkeypatch.setitem(debug.ENV, "SWIN_L_SIDEWALK_VEGETATION_EDGE_MARGIN_M", "0.50")
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_VEGETATION_MAX_LATERAL_MPS", "0.03")
    arguments = [mode, "--sidewalk-vegetation-bias-m", "0.10"]
    if mode == "mcap":
        arguments += ["--input", str(tmp_path / "input.mcap"), "--output", str(tmp_path / "output.mp4")]
    args = debug.parse_args(arguments)
    cfg = debug._local_path_config_from_args(args)
    assert args.vegetation_max_lateral_mps == 0.03
    if mode == "task-drive":
        assert debug._drive_config_from_args(args).vegetation_max_lateral_mps == 0.03
    assert cfg.sidewalk_vegetation_bias_m == 0.10
    assert cfg.sidewalk_vegetation_max_gap_m == 0.25
    assert cfg.sidewalk_vegetation_edge_margin_m == 0.50


@pytest.mark.parametrize("value", ["0", "-0.01", "nan", "0.16"])
def test_invalid_vegetation_speed_environment_is_rejected(monkeypatch, value):
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_VEGETATION_MAX_LATERAL_MPS", value)
    with pytest.raises(SystemExit):
        debug.parse_args(["task-drive"])


@pytest.mark.parametrize("name,value", [
    ("BIAS_M", "nan"), ("BIAS_M", "-0.1"), ("MAX_GAP_M", "inf"),
    ("EDGE_MARGIN_M", "0"),
])
def test_invalid_vegetation_environment_is_rejected(monkeypatch, name, value):
    monkeypatch.setitem(debug.ENV, "SWIN_L_SIDEWALK_VEGETATION_" + name, value)
    with pytest.raises(SystemExit):
        debug.parse_args(["task-drive"])


def test_image_projection_biases_only_sidewalk_and_returns_to_center_without_vegetation():
    mask = np.zeros((360, 640), np.uint8)
    mask[:, :320] = 2
    vegetation = mask == 0  # Image-right is robot-right (negative y).
    cfg = LocalPathConfig(unrestricted_path_mode=True)
    smoothers = {key: LocalPathSmoother(cfg) for key in (0, 1, 2)}
    baseline = debug.extract_path_estimates(mask, tuple(smoothers), cfg)
    biased = debug.update_path_smoothers(mask, smoothers, cfg, 1.0, vegetation)
    np.testing.assert_allclose(biased[2].points_xy[:, 1],
                               baseline[2].points_xy[:, 1] - 0.20, atol=1e-6)
    np.testing.assert_array_equal(biased[0].points_xy, baseline[0].points_xy)
    assert biased[1] is None
    returned = debug.update_path_smoothers(mask, smoothers, cfg, 2.0, np.zeros_like(mask))
    np.testing.assert_array_equal(smoothers[2].current(2.0).points_xy, baseline[2].points_xy)
    np.testing.assert_array_equal(returned[2].points_xy, baseline[2].points_xy)
    road_mask = np.where(mask == 2, 1, 0).astype(np.uint8)
    road_baseline = debug.extract_path_estimates(road_mask, (0, 1, 2), cfg)
    road_biased = debug.extract_path_estimates(road_mask, (0, 1, 2), cfg, vegetation)
    for key in (0, 1):
        np.testing.assert_array_equal(road_biased[key].points_xy, road_baseline[key].points_xy)
    assert road_biased[2] is None
