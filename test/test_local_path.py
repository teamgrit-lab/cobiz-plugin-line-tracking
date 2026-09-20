from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from local_path import (  # noqa: E402
    LocalPathConfig,
    LocalPathEstimate,
    LocalPathSmoother,
    extract_sidewalk_centerline,
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
