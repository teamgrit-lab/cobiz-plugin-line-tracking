"""Payload, pose timing, tag sequence and closed-loop line-following behavior."""
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from cobiz_line_tracking_task import LineTrackingTasks
from tag_straight import (BodyPose, BodyPoseHistory, TagRouteController,
                          TagRouteSpec, parse_tag_route, wrap)
from tag_route_camera import TagRouteCamera


def pose(t=0., yaw=0., x=0., y=0., sequence=None):
    return BodyPose(100. + t, t, x, y, yaw, sequence if sequence is not None else round(t * 1000) + 1)


def tag(tag_id=7, error=10., x=0., y=1., distance=1., status='ok'):
    return dict(tag_id=tag_id, pose_status=status, parallel_error_deg=error,
                range_m=distance, position_body_m=[x, y, 0.])


def observe(route, t, tags=None, body=None, age=0.):
    route.observe([tag()] if tags is None else tags, stamp_sec=100. + t,
                  now=t + age, source_age_sec=age, captured_pose=body or pose(t))


def ready_route():
    route = TagRouteController(TagRouteSpec(7, 8), now=0., duration_sec=60.)
    observe(route, 0., [])
    return route


def straight_route():
    route = ready_route()
    for t in (.1, .2, .3):
        observe(route, t, [tag(error=0.)])
        route.control(now=t, pose=pose(t))
    for t in (.4, .5, .6):
        observe(route, t, [])
        route.control(now=t, pose=pose(t))
    assert route.phase == 'straight'
    return route


@pytest.mark.parametrize('as_json', [False, True])
@pytest.mark.parametrize('complete', [False, True])
def test_ids_are_retained_from_object_or_json_cobiz_payload(as_json, complete):
    payload = dict(selected_mask=2, start_tag_id=7, end_tag_id=8, tag_size_m=.20,
                   complete_on_end_tag=complete)
    tasks = LineTrackingTasks()
    result = tasks.handle_event(dict(type='TASK_REGISTERED', action_name='LINE_TRACKING',
        task_id='route', payload=json.dumps(payload) if as_json else payload), now=0.)
    assert result['type'] == 'TASK_STARTED'
    assert tasks.active.tag_route == TagRouteSpec(7, 8, .20, complete)


def test_arrival_defaults_to_continuing_sidewalk():
    spec = parse_tag_route(dict(start_tag_id=7, end_tag_id=8), 2)
    assert spec.complete_on_end_tag is False


@pytest.mark.parametrize('value', [None, 'false', 'true', 0, 1, {}, []])
def test_completion_policy_requires_a_json_boolean(value):
    with pytest.raises(ValueError, match='invalid_complete_on_end_tag'):
        parse_tag_route(dict(start_tag_id=7, end_tag_id=8, complete_on_end_tag=value), 2)


@pytest.mark.parametrize('payload,mask', [
    ({'start_tag_id': 7}, 2), ({'end_tag_id': 8}, 2),
    ({'start_tag_id': True, 'end_tag_id': 8}, 2),
    ({'start_tag_id': '7', 'end_tag_id': 8}, 2),
    ({'start_tag_id': -1, 'end_tag_id': 8}, 2),
    ({'start_tag_id': 7, 'end_tag_id': 7}, 2),
    ({'start_tag_id': 7, 'end_tag_id': 8}, 1),
    ({'start_tag_id': 7, 'end_tag_id': 8}, 0),
    ({'start_tag_id': 7, 'end_tag_id': 8, 'tag_size_m': float('nan')}, 2),
    ({'start_tag_id': 7, 'end_tag_id': 8, 'tag_size_m': True}, 2),
])
def test_invalid_or_half_configured_tag_routes_are_rejected(payload, mask):
    with pytest.raises(ValueError):
        parse_tag_route(payload, mask)


def test_payload_without_ids_retains_ordinary_tracking():
    assert parse_tag_route({'selected_mask': 2}, 2) is None
    route = ready_route()
    assert route.control(now=0., pose=pose()) is None


def test_initialization_waits_for_live_camera_and_pose_then_times_out():
    route = TagRouteController(TagRouteSpec(7, 8), now=0., duration_sec=60.)
    assert route.control(now=0., pose=None).reason == 'tag_route_inputs_pending'
    decision = route.control(now=5.1, pose=None)
    assert decision.vx == decision.yaw_rate == 0.
    assert route.phase == 'aborted'


def test_only_nearby_unambiguous_start_tag_takes_control_and_stops_for_confirmation():
    route = ready_route()
    for t, item in enumerate([tag(tag_id=8), tag(distance=4), tag(status='ambiguous_pose')], 1):
        observe(route, t * .1, [item])
        assert route.control(now=t * .1, pose=pose(t * .1)) is None
    observe(route, .4)
    decision = route.control(now=.4, pose=pose(.4))
    assert route.phase == 'start_confirm'
    assert decision.vx == decision.yaw_rate == 0.


def test_repeated_and_old_frames_cannot_confirm_a_start():
    route = ready_route()
    observe(route, .1)
    for _ in range(10):
        observe(route, .1)
    assert route.start_hits == 1
    observe(route, .05)
    assert route.start_hits == 1
    observe(route, .2)
    observe(route, .3)
    assert route.phase == 'align'


def test_disagreeing_start_normals_restart_confirmation():
    route = ready_route()
    observe(route, .1, [tag(error=10)])
    observe(route, .2, [tag(error=-10)])
    assert route.start_hits == 1
    observe(route, .3, [tag(error=-10)])
    observe(route, .4, [tag(error=-10)])
    assert route.phase == 'align'


def test_parallel_turn_has_no_forward_speed_and_uses_imu_not_published_yaw_integration():
    route = ready_route()
    for t in (.1, .2, .3):
        observe(route, t, [tag(error=20)], pose(t, yaw=math.radians(20)))
    decision = route.control(now=.3, pose=pose(.3, yaw=math.radians(20)))
    assert decision.vx == 0 and decision.yaw_rate < 0
    observe(route, .4, [])
    for _ in range(10):
        route.control(now=.4, pose=pose(.4))
    assert route.align_hits == 1 and route.phase == 'align'
    for t in (.5, .6):
        observe(route, t, [])
        route.control(now=t, pose=pose(t))
    assert route.phase == 'straight'


def test_parallel_plane_normal_is_transformed_with_captured_attitude():
    route = ready_route()
    angle = math.radians(20)
    for t in (.1, .2, .3):
        item = dict(tag(error=20), normal_body=[math.sin(angle), math.cos(angle), 0.])
        observe(route, t, [item], pose(t, yaw=angle))
    assert route.target_yaw == pytest.approx(0., abs=1e-8)


def test_empty_live_frames_after_start_are_allowed_and_line_drift_is_corrected():
    route = straight_route()
    observe(route, .7, [])
    decision = route.control(now=.7, pose=pose(.7, x=.1, y=.2))
    assert decision.vx == .3
    assert decision.yaw_rate < 0  # Drift left -> steer right toward original line.
    assert abs(decision.yaw_rate) <= .18


@pytest.mark.parametrize('fault', ['camera', 'pose', 'cross_track'])
def test_faults_stop_a_straight_segment_instead_of_falling_back_to_sidewalk(fault):
    route = straight_route()
    if fault == 'camera':
        now, body = 2., pose(2.)
    elif fault == 'pose':
        observe(route, 1.2, [])
        now, body = 1.2, pose(.6)
    else:
        observe(route, .7, [])
        now, body = .7, pose(.7, y=.6)
    decision = route.control(now=now, pose=body)
    assert route.phase == 'aborted'
    assert decision.vx == decision.yaw_rate == 0


def test_arrival_requires_correct_id_near_the_side_not_a_distant_tag_ahead():
    route = straight_route()
    for t, item in [(.7, tag(tag_id=9)), (.8, tag(tag_id=8, x=2.)),
                    (.9, tag(tag_id=8, distance=4.))]:
        observe(route, t, [item])
        assert route.phase == 'straight'
    observe(route, 1., [tag(tag_id=8, x=.3)])
    assert route.phase == 'end_confirm'
    assert route.control(now=1., pose=pose(1.)).vx == 0
    observe(route, 1.1, [tag(tag_id=8, x=.3)])
    observe(route, 1.2, [tag(tag_id=8, x=.3)])
    assert route.phase == 'complete'
    assert route.control(now=1.2, pose=pose(1.2)).reason == 'tag_route_complete'


def test_destination_seen_before_start_is_not_cached_as_an_arrival():
    route = ready_route()
    for t in (.1, .2, .3):
        observe(route, t, [tag(tag_id=8)])
    assert route.phase == 'sidewalk' and route.end_hits == 0


def test_route_duration_is_always_a_failure_bound_not_early_completion():
    route = ready_route()
    route.duration = .5
    observe(route, .6, [])
    decision = route.control(now=.6, pose=pose(.6))
    assert route.phase == 'aborted'
    assert decision.reason == 'tag_route_task_timeout'


def test_duplicate_physical_tags_never_supply_start_confirmation():
    route = ready_route()
    for t in (.1, .2, .3):
        observe(route, t, [tag(), tag()])
    assert route.phase == 'sidewalk' and route.start_hits == 0


def test_unconfirmed_start_and_unseen_end_have_bounded_waits():
    route = ready_route()
    observe(route, .1)
    observe(route, 5.2, [])
    assert route.control(now=5.2, pose=pose(5.2)).reason == 'tag_route_confirmation_timeout'
    route = straight_route()
    route.policy = replace(route.policy, straight_timeout_sec=.5)
    observe(route, 1.2, [])
    assert route.control(now=1.2, pose=pose(1.2)).reason == 'tag_route_straight_timeout'


def message(yaw=0., source_sec=1):
    return NS(imu_state=NS(quaternion=[math.cos(yaw / 2), 0., 0., math.sin(yaw / 2)],
                          rpy=[0., 0., 0.]), position=[1., 2., 0.],
              stamp=NS(sec=source_sec, nanosec=0))


def test_real_unitree_wxyz_quaternion_is_used_when_rpy_is_zero():
    history = BodyPoseHistory()
    assert history.observe(message(math.radians(40)), 100., 0.)
    assert history.latest().yaw == pytest.approx(math.radians(40))
    assert not history.observe(message(0.), 100.1, .1)  # Same source sample.
    bad = message(source_sec=2)
    bad.imu_state.quaternion = [0., 0., 0., 0.]
    assert not history.observe(bad, 100.2, .2)


def test_capture_pose_interpolation_wraps_yaw_and_rejects_large_clock_skew():
    history = BodyPoseHistory()
    history.observe(message(math.radians(179), 1), 100., 0.)
    history.observe(message(math.radians(-179), 2), 100.2, .2)
    assert abs(abs(history.at(100.1).yaw) - math.pi) < 1e-8
    assert history.at(100.5) is None


def test_camera_stop_discards_inflight_old_session_and_releases_device():
    entered, release, closed = threading.Event(), threading.Event(), threading.Event()
    class Source:
        def read(self):
            entered.set()
            assert release.wait(2.)
            return np.zeros((2, 2, 3), np.uint8), 100
        def close(self):
            closed.set()
    worker = TagRouteCamera('unused', hz=30, source_factory=Source,
        detector_factory=lambda *a, **k: NS(detect=lambda image: []))
    try:
        worker.start('old', TagRouteSpec(7, 8))
        assert entered.wait(2.)
        worker.stop()
        release.set()
        assert worker.idle.wait(2.)
        assert closed.is_set() and worker.latest() is None
    finally:
        release.set()
        worker.close()


def test_fixed_line_controller_converges_from_lateral_offset_in_kinematic_simulation():
    route = straight_route()
    x, y, yaw, t = 0., .2, 0., .6
    for i in range(240):
        t += .05
        observe(route, t, [])
        command = route.control(now=t, pose=pose(t, yaw=yaw, x=x, y=y, sequence=i + 100))
        assert route.phase == 'straight'
        assert 0 <= command.vx <= .3 and abs(command.yaw_rate) <= .18
        yaw = wrap(yaw + command.yaw_rate * .05)
        x += math.cos(yaw) * command.vx * .05
        y += math.sin(yaw) * command.vx * .05
    assert x > 3.
    assert abs(y) < .02
