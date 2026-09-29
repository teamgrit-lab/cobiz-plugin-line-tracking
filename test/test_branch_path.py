"""Synthetic BEV sequences exercise routing, geometry and 3 Hz state changes."""

from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import local_path
from local_path import LocalPathConfig, LocalPathSmoother, extract_sidewalk_centerline
from swin_l_drive_control import DriveConfig, decide_drive


X = np.linspace(3, 8, 160, dtype=np.float32)
Y = np.linspace(3.5, -3.5, 280, dtype=np.float32)


def corridor(center, width=1.2):
    return (
        abs(Y[None, :] - np.broadcast_to(center, (len(X),))[:, None]) <= width / 2
    ).astype(np.uint8)


def fork(*, shift=0.0, left_width=1.2, right_width=1.2):
    spread = np.maximum(X - 4, 0) * 0.55
    return corridor(spread + shift, left_width) | corridor(-spread + shift, right_width)


@pytest.fixture
def extract(monkeypatch):
    monkeypatch.setattr(
        local_path, "_birdseye_sidewalk", lambda mask, _config: (mask, X, Y)
    )
    return lambda mask, config: extract_sidewalk_centerline(mask, config)


@pytest.fixture(params=("left", "right"))
def preference(request):
    return request.param


def direction(preference):
    return 1 if preference == "left" else -1


def config(preference="right", **kwargs):
    return LocalPathConfig(branch_preference=preference, **kwargs)


def drive(path, lookahead=4.0):
    return decide_drive(
        path,
        camera_age_sec=0,
        inference_age_sec=0,
        config=DriveConfig(lookahead_m=lookahead),
        last_valid_yaw_rate=0.18,
        last_valid_forward_mps=0.5,
    )


@pytest.mark.parametrize("center", [np.zeros_like(X), 0.35 * (X - 3), -0.35 * (X - 3)])
def test_single_corridor_exactly_preserves_legacy_path(extract, center, preference):
    mask = corridor(center)
    cfg = config(preference)
    before = extract(mask, replace(cfg, branch_preference="none"))
    smoother = LocalPathSmoother(cfg)
    after = smoother.update(extract(mask, cfg), 1.0)
    np.testing.assert_array_equal(after.points_xy, before.points_xy)
    assert not after.branch_status["branch_detected"]
    assert after.stop_reason is None


@pytest.mark.parametrize("unrestricted", [False, True])
def test_y_fork_chooses_preferred_arm_after_distinct_inferences_and_stays_inside(
    extract, unrestricted, preference
):
    cfg = config(preference, unrestricted_path_mode=unrestricted)
    sign = direction(preference)
    opposite = "right" if preference == "left" else "left"
    mask = fork(**{f"{opposite}_width": 1.7})
    estimate = extract(mask, cfg)
    smoother = LocalPathSmoother(cfg)
    pending = smoother.update(estimate, 1.0)
    assert pending.stop_reason == "branch_confirming"
    assert drive(pending).vx == 0  # Even with all ordinary stops disabled.
    assert smoother.update(estimate, 1.0).stop_reason == "branch_confirming"
    for t in (1.05, 1.1, 1.2):
        assert smoother.current(t).branch_status["confirmation_hits"] == 1
    selected = smoother.update(estimate, 1 + 1 / 3)
    assert selected.stop_reason is None
    assert sign * selected.points_xy[-1, 1] > 1.5
    assert selected.branch_status["reason"] == f"{preference}_branch_selected"
    assert selected.branch_status["preference_applied"] is True
    assert sign * selected.branch_status["selected_heading_deg"] > 0
    # The common stem can remain straight at 4 m; turn when the target lies on
    # the selected arm, rather than forcing a premature lateral offset.
    assert sign * drive(selected, lookahead=6.0).yaw_rate > 0
    # Check dense segment samples, including between returned waypoints.
    xs = np.linspace(selected.points_xy[0, 0], selected.points_xy[-1, 0], 3000)
    ys = np.interp(xs, selected.points_xy[:, 0], selected.points_xy[:, 1])
    rr = np.rint(np.interp(xs, X, np.arange(len(X)))).astype(int)
    cc = np.rint(np.interp(ys, Y[::-1], np.arange(len(Y))[::-1])).astype(int)
    assert np.all(mask[rr, cc])
    assert np.all(
        abs(np.diff(selected.points_xy[:, 1]) / np.diff(selected.points_xy[:, 0]))
        <= 1.501
    )


def test_disconnected_preferred_strip_is_not_a_fork(extract, preference):
    cfg = config(preference)
    mask = corridor(0) | corridor(direction(preference) * 2.6, 0.8)
    estimate = extract(mask, cfg)
    path = LocalPathSmoother(cfg).update(estimate, 1.0)
    assert not path.branch_status["branch_detected"]
    assert abs(path.points_xy[:, 1]).max() < 0.03


def test_none_keeps_legacy_fork_path_without_confirmation_or_lock(extract):
    cfg = config("none")
    estimate = extract(fork(), cfg)
    assert estimate.branch_observation is None
    smoother = LocalPathSmoother(cfg)
    path = smoother.update(estimate, 1.0)
    np.testing.assert_array_equal(path.points_xy, estimate.points_xy)
    assert path.stop_reason is None
    assert path.branch_status["state"] == "idle"


def test_temporary_hole_that_rejoins_is_not_a_fork(extract):
    cfg = config()
    mask = corridor(0, 2.4)
    mask[(X > 4) & (X < 4.4), :] &= (abs(Y) > 0.2).astype(np.uint8)
    path = LocalPathSmoother(cfg).update(extract(mask, cfg), 1.0)
    assert not path.branch_status["branch_detected"]


def test_preferred_arm_too_narrow_uses_other_usable_arm(extract, preference):
    cfg = config(preference)
    smoother = LocalPathSmoother(cfg)
    estimate = extract(fork(**{f"{preference}_width": 0.35}), cfg)
    smoother.update(estimate, 1.0)
    path = smoother.update(estimate, 1.34)
    assert path.stop_reason is None
    assert direction(preference) * path.points_xy[-1, 1] < -1.5
    assert path.branch_status["reason"] == f"{preference}_unusable_fallback"
    assert path.branch_status["preference_applied"] is False


def test_width_noise_does_not_switch_a_committed_branch(extract, preference):
    cfg = config(preference, unrestricted_path_mode=True)
    opposite = "right" if preference == "left" else "left"
    smoother = LocalPathSmoother(cfg)
    for i in range(12):
        mask = fork(
            shift=0.08 * (-1) ** i, **{f"{opposite}_width": 1.2 + (i % 3) * 0.2}
        )
        path = smoother.update(extract(mask, cfg), 1 + i / 3)
        if i:
            assert path.stop_reason is None
            assert direction(preference) * path.points_xy[-1, 1] > 1.5
            assert path.branch_status["state"] == "locked"


@pytest.mark.parametrize("unrestricted", [False, True])
def test_selected_branch_loss_stops_without_switching_or_holding_yaw(
    extract, unrestricted, preference
):
    cfg = config(preference, unrestricted_path_mode=unrestricted)
    smoother = LocalPathSmoother(cfg)
    estimate = extract(fork(), cfg)
    smoother.update(estimate, 1)
    smoother.update(estimate, 1.34)
    other_only = corridor(-direction(preference) * np.maximum(X - 4, 0) * 0.55)
    lost = smoother.update(extract(other_only, cfg), 1.67)
    assert lost.stop_reason == "branch_path_lost"
    assert (drive(lost).vx, drive(lost).yaw_rate) == (0, 0)
    assert smoother.current(100).stop_reason == "branch_path_lost"
    assert smoother.update(None, 2).stop_reason == "branch_path_lost"
    restored = smoother.update(estimate, 2.34)
    assert restored.stop_reason is None
    assert direction(preference) * restored.points_xy[-1, 1] > 1.5


def test_selected_branch_can_move_toward_image_center_and_resolve(extract, preference):
    cfg = config(preference, branch_hold_sec=6.0)
    smoother = LocalPathSmoother(cfg)
    estimate = extract(fork(), cfg)
    smoother.update(estimate, 1)
    smoother.update(estimate, 1.34)
    for i in range(1, 12):
        shift = i * 0.2
        mask = corridor(direction(preference) * (np.maximum(X - 4, 0) * 0.55 - shift))
        path = smoother.update(extract(mask, cfg), 1.34 + i / 3)
        assert path.stop_reason is None
    assert path.branch_status["state"] == "locked"
    assert abs(path.points_xy[-1, 1]) < 0.3
    path = smoother.update(extract(mask, cfg), 8.0)
    assert path.branch_status["reason"] == "branch_completed"
    path = smoother.update(extract(mask, cfg), 8.34)
    assert path.branch_status["state"] == "idle"
    assert path.points_xy.shape == (cfg.path_points, 2)


def test_reset_and_surface_instances_do_not_share_a_branch(extract, preference):
    cfg = config(preference)
    first, second = LocalPathSmoother(cfg), LocalPathSmoother(cfg)
    estimate = extract(fork(), cfg)
    first.update(estimate, 1)
    first.update(estimate, 1.34)
    assert second.update(estimate, 2).stop_reason == "branch_confirming"
    first.reset()
    assert first.current(2) is None
    assert first.update(estimate, 2).stop_reason == "branch_confirming"


def test_pending_fork_can_disappear_without_breaking_restricted_smoother(extract):
    cfg = config()
    smoother = LocalPathSmoother(cfg)
    smoother.update(extract(fork(), cfg), 1)
    straight = smoother.update(extract(corridor(0), cfg), 1.34)
    assert straight.stop_reason is None
    assert straight.points_xy.shape == (cfg.path_points, 2)


def test_short_fork_does_not_mistake_surviving_other_arm_for_lost_preferred(
    extract, preference
):
    cfg = config(preference)
    spread = np.maximum(X - 6.2, 0) * 0.5
    left, right = corridor(spread, 0.66), corridor(-spread, 0.66)
    smoother = LocalPathSmoother(cfg)
    estimate = extract(left | right, cfg)
    assert smoother.update(estimate, 1).stop_reason == "branch_confirming"
    selected = smoother.update(estimate, 1.34)
    assert direction(preference) * selected.points_xy[-1, 1] > 0.7
    other = right if preference == "left" else left
    lost = smoother.update(extract(other, cfg), 1.67)
    assert lost.stop_reason == "branch_path_lost"
    assert drive(lost).vx == 0


def test_no_usable_fork_and_excessive_graph_complexity_stop(extract):
    cfg = config(branch_min_width_m=2.0)
    path = LocalPathSmoother(cfg).update(extract(fork(), cfg), 1)
    assert path.stop_reason == "branch_no_usable_path"
    assert drive(path).vx == 0
    noisy = np.zeros((len(X), len(Y)), np.uint8)
    for start in range(0, len(Y) - 6, 8):
        noisy[:, start : start + 6] = 1
    path = LocalPathSmoother(cfg).update(extract(noisy, cfg), 1)
    assert path.stop_reason == "branch_graph_ambiguous"
    assert drive(path).vx == 0


@pytest.mark.parametrize(
    "changes",
    [
        {"branch_preference": "center"},
        {"branch_min_width_m": float("nan")},
        {"branch_margin_m": 0.4},
        {"branch_confirm_frames": 1},
        {"branch_hold_sec": float("inf")},
    ],
)
def test_invalid_branch_configuration_is_rejected(changes):
    with pytest.raises(ValueError, match="branch"):
        replace(config(), **changes).validate()
