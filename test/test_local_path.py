from types import SimpleNamespace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from local_path import (  # noqa: E402
    LidarSafetyConfig,
    LidarSafetyMonitor,
    LocalPathConfig,
    LocalPathEstimate,
    LocalPathSmoother,
    SmoothedPath,
    extract_sidewalk_centerline,
    pointcloud2_xyz,
)


def test_extracts_a_centerline_from_a_metric_straight_sidewalk():
    config = LocalPathConfig(
        near_distance_m=3.0,
        far_distance_m=8.0,
        roi_polygon=(0.08, 1.0, 0.92, 1.0, 0.62, 0.22, 0.38, 0.22),
        min_valid_ratio=0.20,
    )
    mask = np.zeros((360, 640), dtype=np.uint8)
    polygon = np.asarray([[70, 359], [260, 359], [275, 80], [205, 80]], dtype=np.int32)
    import cv2

    cv2.fillPoly(mask, [polygon], 255)

    estimate = extract_sidewalk_centerline(mask, config)

    assert estimate is not None
    assert estimate.points_xy.shape == (config.path_points, 2)
    assert estimate.valid_ratio >= config.min_valid_ratio
    assert np.all(np.diff(estimate.points_xy[:, 0]) > 0.0)
    # The synthetic polygon is on the image-left sidewalk, so positive y means
    # a left-of-robot path in the configured x-forward/y-left convention.
    mean_lateral = float(np.mean(estimate.points_xy[:, 1]))
    assert 1.0 < mean_lateral < config.max_path_lateral_m


def test_smoother_limits_update_and_holds_last_path():
    config = LocalPathConfig(
        smoothing_time_constant_sec=0.8,
        max_lateral_update_m=0.2,
        path_hold_sec=1.0,
    )
    smoother = LocalPathSmoother(config)
    first = LocalPathEstimate(
        points_xy=np.column_stack((np.linspace(3.0, 8.0, 4), np.zeros(4))).astype(
            np.float32
        ),
        confidence=0.8,
        valid_ratio=1.0,
        mean_sidewalk_width_m=1.0,
        raw_points_xy=np.zeros((4, 2), dtype=np.float32),
    )
    second = LocalPathEstimate(
        points_xy=np.column_stack((np.linspace(3.0, 8.0, 4), np.full(4, 2.0))).astype(
            np.float32
        ),
        confidence=0.8,
        valid_ratio=1.0,
        mean_sidewalk_width_m=1.0,
        raw_points_xy=np.zeros((4, 2), dtype=np.float32),
    )

    smoother.update(first, 0.0)
    updated = smoother.update(second, 0.25)
    held = smoother.current(0.50)
    expired = smoother.current(1.51)

    assert updated is not None
    assert float(np.max(updated.points_xy[:, 1])) < 0.2
    assert held is not None
    assert expired is None


def test_lidar_gate_detects_multiple_points_in_path():
    monitor = LidarSafetyMonitor(
        LidarSafetyConfig(stop_distance_m=3.0, min_obstacle_points=2)
    )
    path = SmoothedPath(
        points_xy=np.column_stack((np.linspace(3.0, 8.0, 10), np.zeros(10))).astype(
            np.float32
        ),
        confidence=0.8,
        age_sec=0.0,
        source="test",
    )
    points = np.asarray(
        [[2.5, 0.1, 0.2], [2.6, -0.1, 0.3], [5.0, 2.0, 0.2]], dtype=np.float32
    )
    monitor.update(points, 1.0)

    result = monitor.evaluate(path, 1.1)

    assert result.stop
    assert result.obstacle_in_path
    assert result.obstacle_count == 2
    assert result.clearance_m == pytest.approx(2.5)


def test_pointcloud2_decoder_supports_hesai_style_26_byte_points():
    import struct

    payload = bytearray(26 * 2)
    struct.pack_into("<fff", payload, 0, 1.0, 2.0, 3.0)
    struct.pack_into("<fff", payload, 26, 4.0, 5.0, 6.0)
    message = SimpleNamespace(
        width=2,
        height=1,
        point_step=26,
        row_step=52,
        is_bigendian=False,
        fields=[
            SimpleNamespace(name="x", offset=0),
            SimpleNamespace(name="y", offset=4),
            SimpleNamespace(name="z", offset=8),
        ],
        data=bytes(payload),
    )

    points = pointcloud2_xyz(message)

    np.testing.assert_allclose(points, [[1, 2, 3], [4, 5, 6]])
