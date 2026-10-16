"""Committed section policy, fresh evidence, handoffs and pose boundaries."""
from dataclasses import replace
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from local_path import LocalPathConfig, LocalPathEstimate, RoadPathResult
from path_execution import PathExecution, PathExecutionConfig, PathEvidence, make_path_tracker
from road_curve import fit_curve, transform_points


def observation(center=0., width=2., *, offset=0.):
    xs = np.linspace(3., 8., 161, dtype=np.float32)
    ys = np.linspace(4., -4., 321, dtype=np.float32)
    y = center(xs) if callable(center) else np.full_like(xs, center)
    corridor = np.column_stack((xs, y-width, y+width))
    points, geometry = fit_curve(corridor)
    points[:, 1] += offset
    estimate = LocalPathEstimate(points, .95, 1., width*2, points,
                                road_center_tracking=True, road_corridor=corridor,
                                road_geometry=geometry)
    allowed = abs(ys[None, :]-y[:, None]) <= width
    return RoadPathResult(estimate, 'regenerated', 'test', {}, allowed, xs, ys)


def make(mode=1, **kwargs):
    return PathExecution(LocalPathConfig(unrestricted_path_mode=True),
                         PathExecutionConfig(**kwargs), mode)


def feed(tracker, result, t, pose=None, frame_key=None):
    return tracker.update_road(result, t, pose, frame_key=frame_key)


def test_candidates_do_not_replace_committed_section_until_boundary():
    tr = make()
    r = observation()
    assert feed(tr, r, 0.).stop_reason == 'path_execution_waiting_initial_candidate'
    assert feed(tr, r, .5).stop_reason is None
    initial = tr.current(.5).points_xy.copy()
    moved = observation(center=.2)
    feed(tr, moved, 1.)
    feed(tr, moved, 1.5)
    np.testing.assert_array_equal(tr.advance(1.6).points_xy, initial)
    assert tr.metrics(1.6)['generation'] == 1
    joined = tr.advance(2.)
    assert joined.stop_reason is None
    assert tr.metrics(2.)['generation'] == 2
    assert abs(joined.points_xy[0, 1]-initial[0, 1]) < .001
    assert joined.points_xy[-1, 1] > .1
    assert np.max(abs(np.diff(np.arctan2(np.diff(joined.points_xy[:, 1]),
                                        np.diff(joined.points_xy[:, 0]))))) < math.radians(10)


def test_road_odom_progress_takes_precedence_over_time_boundary():
    tr = make(segment_sec=.1)
    r = observation(center=.2)
    feed(tr, r, 0.); feed(tr, r, .2)
    initial = tr.current(.2).points_xy.copy()
    tr.advance(.2, (0., 0., 0.))
    tr.advance(.3, (.2, .2, .2))
    tr.advance(.4, (.4, .2, .2))
    np.testing.assert_array_equal(tr.current(.4).points_xy, initial)
    assert .39 < tr.metrics(.4)['segment_progress_m'] < .41
    assert tr.metrics(.4)['progress_source'] == 'odom'
    assert tr.metrics(.4)['motion_compensation_enabled']
    assert tr.metrics(.4)['generation'] == 1  # elapsed time cannot win over odom


def test_road_committed_path_is_rebased_at_capture_pose_without_adopting_candidate():
    tr = make(1)
    r = observation()
    feed(tr, r, 0., (0., 0., 0.)); feed(tr, r, .5, (0., 0., 0.))
    initial = tr.current(.5).points_xy.copy()
    pose = (.1, .05, .02)
    feed(tr, r, 1., pose)
    np.testing.assert_allclose(tr.current(1.).points_xy,
                               transform_points(initial, (0., 0., 0.), pose), atol=1e-6)
    assert tr.metrics(1.)['generation'] == 1
    assert tr.current(1.).road_geometry['observation_pose'] == pose


def test_road_pose_availability_change_reconfirms_instead_of_mixing_frames():
    tr = make(1)
    r = observation()
    feed(tr, r, 0., (0., 0., 0.)); feed(tr, r, .5, (0., 0., 0.))
    assert feed(tr, r, 1., None).stop_reason == 'path_execution_waiting_replan_candidate'
    assert feed(tr, r, 1.5, None).stop_reason is None
    assert tr.metrics(1.5)['generation'] == 2


def test_failed_road_fit_can_hold_only_while_current_corridor_is_verified():
    tr = make()
    r = observation()
    feed(tr, r, 0.); feed(tr, r, .5)
    empty = replace(r, estimate=None, status='calculation_failed')
    feed(tr, empty, 1.)
    assert tr.current(1.).stop_reason is None
    assert tr.current(3.51).stop_reason == 'path_execution_observation_expired'
    assert tr.current(3.51).age_sec == 0.


def test_failed_road_calculation_without_verified_corridor_cannot_hold_old_path():
    tr = make(1)
    r = observation()
    feed(tr, r, 0.); feed(tr, r, .5)
    failed = replace(r, estimate=None, status='calculation_failed',
                     allowed_mask=np.zeros_like(r.allowed_mask))
    assert feed(tr, failed, 1.).stop_reason == 'path_execution_invalidated'
    assert tr.path_loss_at == 1.


def test_only_accepted_missing_observations_request_road_recovery():
    tr = make()
    road = observation()
    feed(tr, road, 0., frame_key=1)
    assert tr.path_loss_at is None  # Waiting for the second visible candidate.
    feed(tr, road, .5, frame_key=2)
    failed = replace(road, estimate=None, status='insufficient_observation',
                     allowed_mask=np.zeros_like(road.allowed_mask))
    feed(tr, failed, 1., frame_key=3)
    assert tr.path_loss_at == 1.
    feed(tr, failed, 1.1, frame_key=3)
    assert tr.path_loss_at == 1.  # Duplicate cannot create a new failure.
    tr.update_road(failed, 5., observed_at=1.2, frame_key=4)
    assert tr.path_loss_at == 1.  # Stale source cannot create a new failure.
    feed(tr, road, 5.1, frame_key=5)
    assert tr.path_loss_at is None
    assert tr.recovery_candidate(5.1) is not None
    assert tr.current(5.1).stop_reason is not None  # Candidate cannot move yet.
    assert tr.recovery_candidate(6.61) is None
    feed(tr, road, 5.6, frame_key=6)
    assert tr.current(5.6).stop_reason is None
    assert tr.recovery_candidate(5.6) is None  # Now use the committed path.


def test_positive_hazard_invalidates_road_immediately():
    tr = make()
    r = observation()
    feed(tr, r, 0.); feed(tr, r, .5)
    e = replace(r.estimate, road_center_tracking=False, road_corridor=None, road_geometry=None)
    p = tr.update(e, 1., evidence=PathEvidence(r, np.ones_like(r.allowed_mask)))
    assert p.stop_reason == 'path_execution_invalidated'
    assert tr.metrics(1.)['candidate_confirmations'] == 0


def test_real_road_block_is_not_a_wait_for_segment_completion():
    tr = make(1)
    r = observation()
    feed(tr, r, 0.); feed(tr, r, .5)
    blocked = replace(r, estimate=None, status='blocked', allowed_mask=np.zeros_like(r.allowed_mask))
    assert feed(tr, blocked, .6).stop_reason == 'path_execution_invalidated'
    assert tr.path_loss_at is None
    assert tr.recovery_candidate(.6) is None


def test_control_ticks_and_duplicate_capture_do_not_confirm_candidate():
    tr = make()
    r = observation()
    feed(tr, r, 0., frame_key=10)
    for t in np.arange(.1, .5, .1):
        tr.advance(float(t))
    assert tr.metrics(.4)['candidate_confirmations'] == 1
    feed(tr, r, .5, frame_key=10)
    assert tr.metrics(.5)['generation'] == 0
    feed(tr, r, 1., frame_key=11)
    assert tr.metrics(1.)['generation'] == 1


def test_boundary_without_fresh_connected_candidate_stops_instead_of_driving_blind():
    tr = make()
    r = observation()
    feed(tr, r, 0.); feed(tr, r, .5)
    assert tr.advance(2.01).stop_reason == 'path_execution_waiting_next_candidate'
    feed(tr, observation(center=1.), 2.1)
    feed(tr, observation(center=1.), 2.2)
    assert tr.advance(2.3).stop_reason == 'path_execution_waiting_connected_candidate'


def test_confirmed_candidate_and_join_must_stay_inside_fresh_allowed_area():
    tr = make()
    wide = observation()
    feed(tr, wide, 0.); feed(tr, wide, .5)
    narrow = observation(center=.4, width=.15)
    # Both old and new corridors are currently observed, separated by a gap.
    # A blend must not cut across the non-drivable pixels between them.
    narrow = replace(narrow, allowed_mask=narrow.allowed_mask | observation(width=.1).allowed_mask)
    feed(tr, narrow, 1.); feed(tr, narrow, 1.5)
    # Old y=0 cannot be blended through the new y=[.25,.55] corridor.
    assert tr.advance(2.).stop_reason == 'path_execution_waiting_connected_candidate'


def test_curved_handoff_has_no_float32_duplicate_or_zero_length_segments():
    tr = make(1)
    r = observation(center=lambda x: .08*(x-3.)**2)
    feed(tr, r, 0.); feed(tr, r, .5)
    shifted = observation(center=lambda x: .08*(x-3.)**2+.1)
    feed(tr, shifted, 1.); feed(tr, shifted, 1.5)
    p = tr.advance(2.)
    assert p.stop_reason is None
    assert tr.metrics(2.)['generation'] == 2
    assert np.min(np.diff(p.points_xy[:, 0])) > 1e-4


def test_disconnected_reference_replans_after_stopping_instead_of_waiting_forever():
    tr = make()
    r = observation()
    feed(tr, r, 0.); feed(tr, r, .5)
    moved = observation(center=1.)
    feed(tr, moved, 1.); feed(tr, moved, 1.5)
    assert tr.advance(2.).stop_reason == 'path_execution_waiting_connected_candidate'
    assert tr.advance(3.).stop_reason == 'path_execution_waiting_replan_candidate'
    assert tr.metrics(3.)['candidate_confirmations'] == 0
    feed(tr, moved, 3.1)
    assert tr.current(3.1).stop_reason is not None
    feed(tr, moved, 3.6)
    assert tr.current(3.6).stop_reason is None


def test_pose_jump_and_task_reset_discard_old_active_and_candidates():
    tr = make()
    r = observation()
    feed(tr, r, 0.); feed(tr, r, .5)
    tr.advance(.5, (0., 0., 0.))
    assert tr.advance(.6, (10., 0., 0.)).stop_reason == 'path_execution_pose_discontinuity'
    feed(tr, r, 1.); feed(tr, r, 1.5)
    tr.reset()
    assert tr.metrics(1.6)['generation'] == 0
    assert tr.metrics(1.6)['candidate_confirmations'] == 0
    assert tr.current(1.6).stop_reason == 'path_execution_waiting_initial_candidate'


def test_stale_capture_cannot_refresh_or_confirm_the_path():
    tr = make()
    r = observation()
    e = replace(r.estimate, road_center_tracking=False, road_corridor=None, road_geometry=None)
    for t in (5., 6.):
        tr.update(e, t, evidence=PathEvidence(r, observed_at=1.))
    assert tr.metrics(6.)['generation'] == 0


def test_one_hz_inference_with_latency_confirms_distinct_captures():
    tr = make()
    r = observation()
    e = replace(r.estimate, road_center_tracking=False, road_corridor=None, road_geometry=None)
    tr.update(e, .8, evidence=PathEvidence(r, observed_at=0., frame_key=10))
    tr.update(e, 1.8, evidence=PathEvidence(r, observed_at=1., frame_key=11))
    assert tr.metrics(1.8)['generation'] == 1
    assert tr.current(1.8).age_sec == pytest.approx(.8)
    tr.update(e, 1.9, evidence=PathEvidence(r, observed_at=1.1, frame_key=9))
    assert tr.current(1.9).age_sec == pytest.approx(.9)


def test_future_capture_does_not_poison_later_healthy_sequence():
    tr = make()
    r = observation()
    e = replace(r.estimate, road_center_tracking=False, road_corridor=None, road_geometry=None)
    tr.update(e, .5, evidence=PathEvidence(r, observed_at=10., frame_key=100))
    tr.update(e, 1., evidence=PathEvidence(r, observed_at=1., frame_key=1))
    tr.update(e, 1.5, evidence=PathEvidence(r, observed_at=1.5, frame_key=2))
    assert tr.current(1.5).stop_reason is None


def test_only_road_uses_execution_policy_even_when_enabled():
    cfg = LocalPathConfig()
    assert not isinstance(make_path_tracker(cfg, PathExecutionConfig(), 0), PathExecution)
    assert not isinstance(make_path_tracker(cfg, PathExecutionConfig(), 2), PathExecution)
    assert isinstance(make_path_tracker(cfg, PathExecutionConfig(), 1), PathExecution)
    assert not isinstance(make_path_tracker(cfg, PathExecutionConfig(enabled=False), 1), PathExecution)
    with pytest.raises(ValueError, match='road mode 1'):
        PathExecution(cfg, PathExecutionConfig(), 2)


def test_sidewalk_factory_keeps_immediate_raw_updates_and_original_path_extent():
    cfg = LocalPathConfig(unrestricted_path_mode=True)
    tracker = make_path_tracker(cfg, PathExecutionConfig(), 2)
    r = observation()
    initial = replace(r.estimate, road_center_tracking=False, road_geometry=None, road_corridor=None)
    assert tracker.update(initial, 0.).stop_reason is None
    moved = replace(initial, points_xy=initial.points_xy+np.array([0., .5], np.float32))
    latest = tracker.update(moved, .1)
    np.testing.assert_array_equal(latest.points_xy, moved.points_xy)
    assert latest.source == 'raw_latest'
    assert latest.points_xy[-1, 0] == 8.
    assert tracker.update(None, .2) is None


def test_sidewalk_missing_path_keeps_original_live_and_replay_policies():
    import swin_l_local_path_debug as debug
    cfg = LocalPathConfig(unrestricted_path_mode=False, branch_preference='center')
    e = replace(observation().estimate, road_center_tracking=False,
                road_geometry=None, road_corridor=None)
    selected = np.full((360, 640), 2, np.uint8)
    for replay in (False, True):
        tracker = make_path_tracker(cfg, PathExecutionConfig(), 2)
        tracker.update(e, 0.)
        debug._update_path_tracker(tracker, 2, None, {}, selected, None, cfg, .4,
                                   reset_missing=not replay)
        assert (tracker.current(.4) is not None) == replay


@pytest.mark.parametrize('settings', [{'segment_m': float('nan')}, {'confirm_frames': 1},
                                      {'segment_sec': 0.}, {'max_join_heading_deg': 90.}])
def test_invalid_execution_settings_rejected(settings):
    with pytest.raises(ValueError):
        PathExecutionConfig(**settings).validate()
