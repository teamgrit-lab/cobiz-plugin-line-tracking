from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import local_path  # noqa: E402
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


def test_new_result_lifetime_starts_when_result_becomes_available():
    config = LocalPathConfig(path_hold_sec=0.5)
    smoother = LocalPathSmoother(config)
    estimate = LocalPathEstimate(
        points_xy=np.column_stack((np.linspace(3.0, 8.0, 4), np.zeros(4))).astype(
            np.float32
        ),
        confidence=0.8,
        valid_ratio=1.0,
        mean_sidewalk_width_m=1.0,
        raw_points_xy=np.zeros((4, 2), dtype=np.float32),
    )

    # A frame may have waited and spent longer than path_hold_sec in inference.
    # The newly available result must still begin with age zero.
    result_available_at = 10.0
    created = smoother.update(estimate, result_available_at)

    assert created is not None
    assert created.age_sec == 0.0
    assert smoother.current(10.49) is not None
    assert smoother.current(10.51) is None


def test_unrestricted_mode_bypasses_valid_ratio_gate(monkeypatch):
    birdseye = np.zeros((10, 20), dtype=np.uint8)
    birdseye[:2, 6:14] = 255
    x_values = np.linspace(3.0, 8.0, 10, dtype=np.float32)
    y_values = np.linspace(1.0, -1.0, 20, dtype=np.float32)
    monkeypatch.setattr(
        local_path,
        "_birdseye_sidewalk",
        lambda _mask, _config: (birdseye, x_values, y_values),
    )
    mask = np.zeros((10, 20), dtype=np.uint8)

    assert extract_sidewalk_centerline(
        mask, LocalPathConfig(min_valid_ratio=0.90)
    ) is None
    estimate = extract_sidewalk_centerline(
        mask,
        LocalPathConfig(
            min_valid_ratio=0.90,
            unrestricted_path_mode=True,
        ),
    )

    assert estimate is not None
    assert estimate.valid_ratio == 0.2


def test_unrestricted_mode_uses_each_raw_update_without_smoothing_or_hold():
    config = LocalPathConfig(
        smoothing_time_constant_sec=100.0,
        max_lateral_update_m=0.01,
        path_hold_sec=0.01,
        unrestricted_path_mode=True,
    )
    smoother = LocalPathSmoother(config)

    def estimate(lateral: float) -> LocalPathEstimate:
        points = np.column_stack(
            (np.linspace(3.0, 8.0, 4), np.full(4, lateral))
        ).astype(np.float32)
        return LocalPathEstimate(points, 0.1, 0.1, 0.1, points)

    smoother.update(estimate(0.0), 0.0)
    updated = smoother.update(estimate(2.0), 0.01)

    assert updated is not None
    assert np.all(updated.points_xy[:, 1] == 2.0)
    assert updated.source == "raw_latest"
    assert smoother.current(10.0) is not None
    assert smoother.update(None, 10.1) is None
