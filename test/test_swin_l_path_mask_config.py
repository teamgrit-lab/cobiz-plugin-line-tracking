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


@pytest.mark.parametrize("mask_class", [1, 2])
@pytest.mark.parametrize("mode", ["ros2", "drive", "task-drive"])
def test_live_modes_use_the_env_selected_path_class(monkeypatch, mode, mask_class):
    monkeypatch.setitem(debug.ENV, "SWIN_L_PATH_MASK_CLASS", str(mask_class))

    assert debug.parse_args([mode]).path_mask_class == mask_class


def test_sidewalk_is_the_default_path_class(monkeypatch):
    monkeypatch.delitem(debug.ENV, "SWIN_L_PATH_MASK_CLASS", raising=False)

    assert debug.parse_args(["ros2"]).path_mask_class == 2


def test_command_line_can_override_env_for_offline_path_preview(monkeypatch, tmp_path):
    monkeypatch.setitem(debug.ENV, "SWIN_L_PATH_MASK_CLASS", "2")

    args = debug.parse_args(
        [
            "mcap",
            "--input",
            str(tmp_path / "input.mcap"),
            "--output",
            str(tmp_path / "output.mp4"),
            "--path-mask-class",
            "1",
        ]
    )
    assert args.path_mask_class == 1


@pytest.mark.parametrize("value", ["0", "3", "road", ""])
def test_invalid_path_class_fails_before_ros_starts(monkeypatch, value):
    monkeypatch.setitem(debug.ENV, "SWIN_L_PATH_MASK_CLASS", value)

    with pytest.raises(SystemExit) as error:
        debug.parse_args(["task-drive"])
    assert error.value.code == 2


def test_selected_path_region_excludes_other_classes():
    selected_mask = np.array([[0, 1, 2], [2, 1, 0]], dtype=np.uint8)

    np.testing.assert_array_equal(
        debug.selected_path_region(selected_mask, 1),
        [[False, True, False], [False, True, False]],
    )
    np.testing.assert_array_equal(
        debug.selected_path_region(selected_mask, 2),
        [[False, False, True], [True, False, False]],
    )
    with pytest.raises(ValueError, match="must be 1 .* or 2"):
        debug.selected_path_region(selected_mask, 0)


def test_road_only_scene_produces_a_path_only_in_road_mode():
    road_mask = np.ones((360, 640), dtype=np.uint8)
    config = LocalPathConfig()

    assert (
        extract_sidewalk_centerline(debug.selected_path_region(road_mask, 1), config)
        is not None
    )
    assert (
        extract_sidewalk_centerline(debug.selected_path_region(road_mask, 2), config)
        is None
    )


def test_task_paths_are_independent_and_active_task_selects_its_own_path():
    mask = np.zeros((360, 640), dtype=np.uint8)
    mask[:, :320] = 1
    mask[:, 320:] = 2
    config = LocalPathConfig()
    smoothers = {1: LocalPathSmoother(config), 2: LocalPathSmoother(config)}

    estimates = debug.update_path_smoothers(mask, smoothers, config, 10.0)
    road = smoothers[1].current(10.1)
    sidewalk = smoothers[2].current(10.1)

    assert estimates[1] is not None and estimates[2] is not None
    assert road is not None and sidewalk is not None
    assert road.points_xy[:, 1].mean() > 0
    assert sidewalk.points_xy[:, 1].mean() < 0
    active = ActiveTask("t1", "t1", None, None, 10.0, 30.0, 1)
    assert debug.active_path_mask_class(active, 2) == 1
    assert debug.active_path_mask_class(None, 2) == 2
