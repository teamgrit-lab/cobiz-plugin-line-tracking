from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from debug_surface_follow_video import (  # noqa: E402
    choose_surface,
    preview_yaw,
    video_path_config,
)
from local_path import LocalPathConfig, SmoothedPath  # noqa: E402
from swin_l_drive_control import DriveConfig  # noqa: E402


def test_road_is_used_when_no_sidewalk_is_visible():
    mask = np.ones((360, 640), dtype=np.uint8)
    selected = choose_surface(mask, LocalPathConfig(), DriveConfig())

    assert selected is not None
    assert selected[0] == "ROAD"
    assert selected[1].confidence >= DriveConfig().min_confidence


def test_centered_sidewalk_takes_precedence_over_road():
    mask = np.ones((360, 640), dtype=np.uint8)
    cv2.rectangle(mask, (250, 0), (390, 359), 2, thickness=-1)

    selected = choose_surface(mask, LocalPathConfig(), DriveConfig())

    assert selected is not None
    assert selected[0] == "SIDEWALK"


def test_no_surface_or_far_off_center_surface_stops_preview():
    config = LocalPathConfig()
    drive = DriveConfig()
    assert choose_surface(np.zeros((360, 640), np.uint8), config, drive) is None
    mask = np.zeros((360, 640), np.uint8)
    cv2.rectangle(mask, (0, 0), (120, 359), 1, thickness=-1)
    assert choose_surface(mask, config, drive) is None


def test_video_roi_below_horizon_recovers_visible_road():
    mask = np.zeros((360, 640), np.uint8)
    cv2.rectangle(mask, (0, 169), (639, 359), 1, thickness=-1)
    drive = DriveConfig(min_confidence=0.70)
    assert choose_surface(mask, LocalPathConfig(), drive) is None
    selected = choose_surface(mask, video_path_config(0.55), drive)
    assert selected is not None
    assert selected[0] == "ROAD"


def test_preview_yaw_is_bounded_and_rejects_stale_path():
    config = DriveConfig()
    path = SmoothedPath(
        points_xy=np.asarray([[3.0, 0.2], [8.0, 0.2]], np.float32),
        confidence=0.9,
        age_sec=0.1,
        source="test",
    )
    assert 0 < preview_yaw(path, config) <= config.max_yaw_rps
    assert preview_yaw(None, config) is None
    assert (
        preview_yaw(
            SmoothedPath(path.points_xy, path.confidence, 1.0, path.source), config
        )
        is None
    )
    assert (
        preview_yaw(SmoothedPath(path.points_xy, 0.4, 0.1, path.source), config) is None
    )
