"""Metric scenes verify vegetation bias without a model or ROS installation."""

from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import local_path
import sidewalk_vegetation
from local_path import LocalPathConfig, LocalPathSmoother, extract_sidewalk_centerline
from sidewalk_vegetation import bias_sidewalk_estimate


X = np.linspace(3, 8, 160, dtype=np.float32)
Y = np.linspace(3.5, -3.5, 280, dtype=np.float32)


def corridor(center=0.0, width=2.0):
    centers = np.broadcast_to(center, X.shape)
    return (abs(Y[None, :] - centers[:, None]) <= width / 2).astype(np.uint8)


@pytest.fixture
def metric(monkeypatch):
    project = lambda mask, _config: (mask.copy(), X, Y)
    monkeypatch.setattr(local_path, "_birdseye_sidewalk", project)
    monkeypatch.setattr(sidewalk_vegetation, "_birdseye_sidewalk", project)


@pytest.mark.parametrize("sign", [1, -1])
def test_adjacent_vegetation_shifts_only_toward_its_side_and_respects_limit(metric, sign):
    cfg = LocalPathConfig(unrestricted_path_mode=True)
    mask = corridor()
    vegetation = corridor(sign * 1.4, 0.6)
    estimate = extract_sidewalk_centerline(mask, cfg)
    original = estimate.points_xy.copy()
    before_masks = mask.copy(), vegetation.copy()
    biased = bias_sidewalk_estimate(estimate, mask, vegetation, cfg)
    delta = biased.points_xy[:, 1] - original[:, 1]
    np.testing.assert_allclose(sign * delta, 0.20, atol=1e-6)
    np.testing.assert_array_equal(estimate.points_xy, original)
    np.testing.assert_array_equal(mask, before_masks[0])
    np.testing.assert_array_equal(vegetation, before_masks[1])
    assert biased.confidence == estimate.confidence
    assert biased.vegetation_bias_active is True
    smoother = LocalPathSmoother(cfg)
    assert smoother.update(biased, 1.0).vegetation_bias_active is True
    returned = bias_sidewalk_estimate(estimate, mask, np.zeros_like(mask), cfg)
    np.testing.assert_array_equal(smoother.update(returned, 2.0).points_xy, original)
    assert smoother.current(2.0).vegetation_bias_active is False
    smoother.update(biased, 3.0)
    smoother.reset()
    assert smoother.current(3.0) is None


@pytest.mark.parametrize("case", ["absent", "far", "both", "speckle", "sparse", "disabled"])
def test_insufficient_or_ambiguous_evidence_keeps_original_path(metric, case):
    cfg = LocalPathConfig(sidewalk_vegetation_bias_m=0 if case == "disabled" else 0.20)
    mask = corridor()
    vegetation = corridor(1.4, 0.6)
    if case == "absent":
        vegetation.fill(0)
    elif case == "far":
        vegetation = corridor(1.9, 0.6)
    elif case == "both":
        vegetation |= corridor(-1.4, 0.6)
    elif case == "speckle":
        vegetation = corridor(1.15, 0.03)
    elif case == "sparse":
        vegetation[16:] = 0
    estimate = extract_sidewalk_centerline(mask, cfg)
    biased = bias_sidewalk_estimate(estimate, mask, vegetation, cfg)
    np.testing.assert_array_equal(biased.points_xy, estimate.points_xy)
    assert biased.vegetation_bias_active is False


@pytest.mark.parametrize("width", [0.8, 1.0, 1.2])
def test_narrow_sidewalk_reduces_bias_and_keeps_edge_margin(metric, width):
    cfg = LocalPathConfig()
    mask = corridor(width=width)
    vegetation = corridor(width / 2 + 0.4, 0.6)
    estimate = extract_sidewalk_centerline(mask, cfg)
    biased = bias_sidewalk_estimate(estimate, mask, vegetation, cfg)
    if width < 2 * cfg.sidewalk_vegetation_edge_margin_m:
        np.testing.assert_array_equal(biased.points_xy, estimate.points_xy)
    else:
        delta = biased.points_xy[:, 1] - estimate.points_xy[:, 1]
        assert np.all((delta > 0) & (delta < cfg.sidewalk_vegetation_bias_m))
        edge = float(Y[np.flatnonzero(mask[0])[0]])
        assert np.all(edge - biased.points_xy[:, 1] >= cfg.sidewalk_vegetation_edge_margin_m - 1e-6)


def test_vegetation_does_not_create_a_path_or_extend_missing_sidewalk_rows(metric):
    cfg = LocalPathConfig(unrestricted_path_mode=True)
    mask = corridor()
    mask[80:] = 0
    vegetation = corridor(1.4, 0.6)
    estimate = extract_sidewalk_centerline(mask, cfg)
    biased = bias_sidewalk_estimate(estimate, mask, vegetation, cfg)
    far = biased.points_xy[:, 0] > X[80]
    np.testing.assert_array_equal(biased.points_xy[far], estimate.points_xy[far])
    assert bias_sidewalk_estimate(None, np.zeros_like(mask), vegetation, cfg) is None
    assert bias_sidewalk_estimate(estimate, mask, None, cfg) is estimate
    with pytest.raises(ValueError, match="geometry"):
        bias_sidewalk_estimate(estimate, mask, vegetation[:1], cfg)


@pytest.mark.parametrize("preference,sign", [("left", 1), ("right", -1)])
def test_confirmed_fork_keeps_identity_and_biases_inside_selected_arm(metric, preference, sign):
    cfg = LocalPathConfig(branch_preference=preference, unrestricted_path_mode=True)
    spread = np.maximum(X - 4, 0) * 0.55
    mask = corridor(spread, 1.2) | corridor(-spread, 1.2)
    vegetation = corridor(sign * (spread + 1.0), 0.4)
    original = extract_sidewalk_centerline(mask, cfg)
    biased = bias_sidewalk_estimate(original, mask, vegetation, cfg)
    assert biased.branch_observation.headings == original.branch_observation.headings
    for before, after in zip(original.branch_observation.paths, biased.branch_observation.paths):
        assert after.intervals == before.intervals
        np.testing.assert_array_equal(after.centers, before.centers)
    baseline, shifted = LocalPathSmoother(cfg), LocalPathSmoother(cfg)
    assert shifted.update(biased, 1.0).stop_reason == "branch_confirming"
    baseline.update(original, 1.0)
    expected = baseline.update(original, 1.5)
    actual = shifted.update(biased, 1.5)
    assert actual.stop_reason is None
    assert actual.vegetation_bias_active is True
    assert actual.branch_status == expected.branch_status
    assert np.mean(sign * (actual.points_xy[-40:, 1] - expected.points_xy[-40:, 1])) > 0.05
    xx = np.linspace(actual.points_xy[0, 0], actual.points_xy[-1, 0], 3000)
    yy = np.interp(xx, actual.points_xy[:, 0], actual.points_xy[:, 1])
    rows = np.rint(np.interp(xx, X, np.arange(len(X)))).astype(int)
    cols = np.rint(np.interp(yy, Y[::-1], np.arange(len(Y))[::-1])).astype(int)
    assert np.all(mask[rows, cols])


@pytest.mark.parametrize("field,value", [
    ("sidewalk_vegetation_bias_m", -0.1),
    ("sidewalk_vegetation_bias_m", float("nan")),
    ("sidewalk_vegetation_max_gap_m", float("inf")),
    ("sidewalk_vegetation_edge_margin_m", 0),
])
def test_invalid_bias_settings_are_rejected(field, value):
    with pytest.raises(ValueError, match=field):
        replace(LocalPathConfig(), **{field: value}).validate()
