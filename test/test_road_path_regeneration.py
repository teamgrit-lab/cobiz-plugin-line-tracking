"""Constrained regeneration, fresh validation and distinct stop causes."""
from dataclasses import replace
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import local_path as lp
import swin_l_local_path_debug as debug
from swin_l_drive_control import DriveConfig, DriveDecision, PathLossRecovery, decide_drive
from test_road_clearance import metric_grid


def test_small_interior_fit_violation_regenerates_without_reducing_clearance():
    xs = np.linspace(3., 8., 151)
    corridor = np.column_stack((xs, np.full(len(xs), -1.), np.ones(len(xs))))
    corridor[77, 2] = -.0171
    points = np.column_stack((np.linspace(3., 8., 20), np.zeros(20)))
    assert not lp.road_path_within_corridor(points, corridor)
    regenerated, method = lp.regenerate_road_path(points, corridor)
    assert method == 'constrained_cubic_spline'
    assert lp.road_path_within_corridor(regenerated, corridor)
    assert np.interp(xs[77], regenerated[:, 0], regenerated[:, 1]) <= -.0171


def test_bent_narrow_corridor_gets_a_verified_smooth_curve():
    xs = np.linspace(3., 8., 151)
    center = .8 * np.sin((xs - 3) * 2)
    corridor = np.column_stack((xs, center - .08, center + .08))
    points = np.column_stack((np.linspace(3., 8., 20), np.zeros(20)))
    regenerated, method = lp.regenerate_road_path(points, corridor)
    assert method == 'constrained_cubic_spline'
    assert lp.road_path_within_corridor(regenerated, corridor)
    assert len(regenerated) <= len(xs) + 40
    assert np.rad2deg(abs(np.diff(np.unwrap(np.arctan2(
        np.diff(regenerated[:,1]),np.diff(regenerated[:,0])))))).max() < 10.


def test_no_corridor_extrapolation_or_nonfinite_path_is_accepted():
    corridor = np.array([[3., -1., 1.], [8., -1., 1.]])
    assert not lp.road_path_within_corridor(np.array([[2., 0.], [8., 0.]]), corridor)
    assert not lp.road_path_within_corridor(np.array([[3., 0.], [8., np.nan]]), corridor)


def test_missing_road_and_real_clearance_exclusion_are_distinct(metric_grid):
    xs, ys, road, cfg = metric_grid
    missing = lp.analyze_road_path(np.zeros_like(road), cfg, np.ones_like(road))
    blocked = lp.analyze_road_path(road, cfg, np.ones_like(road))
    narrow = lp.analyze_road_path(np.broadcast_to(abs(ys) < .5, road.shape), cfg)
    assert missing.status == 'insufficient_observation' and not missing.blocked
    assert blocked.blocked and narrow.blocked
    assert blocked.reason == 'clearance_leaves_no_passage'


def test_hazards_outside_observed_road_do_not_report_a_blocked_passage():
    road = np.zeros((360, 640), np.uint8); road[:10] = 1
    hazard = np.zeros_like(road); hazard[:10] = 1
    result = lp.analyze_road_path(road, lp.LocalPathConfig(), hazard)
    assert result.estimate is None and not result.blocked


def failed_fit(monkeypatch, road, cfg, hazard=None):
    monkeypatch.setattr(lp, 'fit_curve', lambda *_args, **_kwargs: None)
    result = lp.analyze_road_path(road, cfg, hazard)
    assert result.status == 'calculation_failed' and not result.blocked
    return result


@pytest.mark.parametrize('unrestricted', [False, True])
def test_fresh_safe_path_hold_is_bounded_and_does_not_refresh_its_age(metric_grid, monkeypatch, unrestricted):
    xs, ys, road, cfg = metric_grid
    cfg = replace(cfg, unrestricted_path_mode=unrestricted, path_hold_sec=.9)
    smoother = lp.LocalPathSmoother(cfg)
    smoother.update_road(lp.analyze_road_path(road, cfg), 0.)
    failure = failed_fit(monkeypatch, road, cfg)
    held = smoother.update_road(failure, .4)
    assert held is not None and held.source == 'road_validated_hold'
    assert held.age_sec == pytest.approx(.4)
    assert held.road_clearance['using_previous_path']
    assert smoother.update_road(failure, .8).age_sec == pytest.approx(.8)
    assert smoother.current(1.) is None
    assert smoother.update_road(failure, 1.1) is None


def test_failed_fit_cannot_hold_a_path_inside_fresh_hazard(metric_grid, monkeypatch):
    xs, ys, road, cfg = metric_grid
    smoother = lp.LocalPathSmoother(cfg)
    smoother.update_road(lp.analyze_road_path(road, cfg), 0.)
    hazard = np.broadcast_to(ys >= 0., road.shape).copy()
    failure = failed_fit(monkeypatch, road, cfg, hazard)
    assert smoother.update_road(failure, .1) is None


def test_confirmed_block_immediately_discards_the_old_path(metric_grid):
    xs, ys, road, cfg = metric_grid
    smoother = lp.LocalPathSmoother(replace(cfg, unrestricted_path_mode=True))
    smoother.update_road(lp.analyze_road_path(road, cfg), 0.)
    blocked = lp.analyze_road_path(road, cfg, np.ones_like(road))
    assert smoother.update_road(blocked, .01) is None


@pytest.mark.parametrize('failures', [1, 2, 3])
def test_unverified_calculation_failure_never_uses_blind_forward_grace(failures):
    decision = decide_drive(None, camera_age_sec=0., inference_age_sec=0.,
        config=DriveConfig(path_loss_recovery_enabled=True), last_valid_forward_mps=.8,
        path_recovery_inferences=failures, road_path_unconfirmed=True)
    assert decision == DriveDecision.stop('road_path_unconfirmed' if failures < 3 else 'path_recovery_waiting')


def test_hard_clearance_block_cancels_rotation_search():
    recovery = PathLossRecovery(DriveConfig(path_loss_recovery_enabled=True), output_hz=10)
    recovery.phase = 'scan_left'; recovery.armed = True
    decision = DriveDecision.stop('road_clearance_blocked')
    result = recovery.decide(decision, now=1., inference_id=3, inference_source_at=1.,
        sensors_ready=True, sensor_reason='', path_confidence=None)
    assert result == decision and not recovery.active and not recovery.armed


def test_runtime_keeps_a_freshly_verified_path_through_one_failed_fit(monkeypatch):
    from types import SimpleNamespace as NS
    from test_lidar_obstacles_ros import enabled_ros, complete_inference
    from test_road_center_tracking import projected_road
    ros = enabled_ros(monkeypatch)
    monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_AVOIDANCE_ENABLED', 'false')
    monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_PATH_LOSS_RECOVERY_ENABLED', 'true')
    cfg = lp.LocalPathConfig(); labels = projected_road(cfg, 2., -2.)
    real = debug.analyze_road_path; failing = [False]
    def analyze(*args, **kwargs):
        result = real(*args, **kwargs)
        if failing[0]:
            return replace(result, estimate=None, status='calculation_failed', reason='simulated',
                clearance={**result.clearance, 'status': 'calculation_failed', 'reason': 'simulated'})
        return result
    monkeypatch.setattr(debug, 'analyze_road_path', analyze)
    monkeypatch.setattr(debug, 'BestSoFarSegmenter', lambda _: NS(device=NS(type='cuda'),
        reset=lambda: None, segment=lambda *_args, **_kwargs: NS(
            selected_mask=labels.copy(), road_hazard_mask=np.zeros_like(labels),
            inference_seconds=.01)))
    def scenario(node):
        ros.start(); ros.now=2.1
        assert complete_inference(node, ros)['drive_reason']=='tracking'
        failing[0]=True; ros.now+=.4
        held=complete_inference(node, ros)
        assert held['drive_reason']=='tracking'
        assert held['road_clearance']['using_previous_path']
        assert held['road_clearance']['status']=='calculation_failed'
        assert not held['road_clearance']['blocked'] and not held['path_recovery']['active']
        # Even without another inference, the old path's age limit still applies.
        ros.now+=.6; node.publish_state()
        assert ros.metrics()['drive_reason']=='road_path_unconfirmed'
        assert not ros.metrics()['path_recovery']['active']
        failing[0]=False
        assert complete_inference(node, ros)['drive_reason']=='tracking'
    ros.run(scenario, '--path-mask-class', '1', '--branch-preference', 'center')
