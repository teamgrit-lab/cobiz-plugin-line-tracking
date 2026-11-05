"""Closed-loop corner behavior and geometric false-positive guards."""
from dataclasses import replace
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))

from local_path import RoadPathResult, LocalPathEstimate
from road_corner import CornerConfig, CornerNavigator, detect_corner, _world, _body
from drive_control import DriveConfig, DriveDecision, decide_drive


def scene(sign=1, pose=(0., 0., 0.), *, straight=False, observed=True):
    xs = np.linspace(-2., 8., 160)
    ys = np.linspace(7., -7., 224)
    x, y = np.meshgrid(xs, ys, indexing='ij')
    world = _world(np.column_stack((x.ravel(), y.ravel())), pose)
    wx, wy = world[:, 0].reshape(x.shape), world[:, 1].reshape(x.shape)
    mask = ((wx < 5.) & (abs(wy) < 1.5)) | ((wx > 3.) & (wx < 5.) & (sign*wy >= 0) & (sign*wy < 12))
    if straight:
        mask = abs(wy) < 1.5
    corner = detect_corner(mask, xs, ys, 'right', np.ones_like(mask))
    reference = (_body([[4., pose[1]+sign*3.], [4., pose[1]+sign*7.]], pose)
                 if not straight and abs(pose[2]) > math.pi/4
                 else _body([[pose[0]+3., 0.], [pose[0]+7., 0.]], pose))
    estimate = LocalPathEstimate(reference, .95, 1., 3.,
        reference, road_center_tracking=True,
        road_geometry={'direction_heading_rad': sign*math.pi/2-pose[2]})
    return RoadPathResult(estimate, 'regenerated', 'test', {}, mask.astype(np.uint8), xs, ys,
                          corner, np.ones_like(mask) if observed else np.zeros_like(mask),
                          mask.astype(np.uint8))


@pytest.mark.parametrize('sign', [-1, 1])
def test_detects_connected_left_and_right_but_not_a_straight_road(sign):
    result = scene(sign)
    assert result.corner['direction'] == ('left' if sign == 1 else 'right')
    assert abs(result.corner['exit_heading_rad']) == pytest.approx(math.pi/2, abs=.02)
    assert scene(sign, straight=True).corner is None
    # A side island separated from the incoming road cannot become an exit.
    mask = result.allowed_mask.copy()
    mask[:, abs(result.y_values-2*sign) < .3] = 0
    assert detect_corner(mask, result.x_values, result.y_values) is None


def test_image_crop_does_not_invent_a_corner_and_t_exit_needs_preference():
    left, right = scene(1), scene(-1)
    mask = left.allowed_mask | right.allowed_mask
    result = detect_corner(mask, left.x_values, left.y_values, 'center')
    assert result['ambiguous']
    assert detect_corner(mask, left.x_values, left.y_values, 'right')['direction'] == 'right'
    support = np.ones_like(mask)
    support[left.x_values > 4.9] = 0
    assert detect_corner(mask, left.x_values, left.y_values, observed=support) is None


def test_perspective_wedge_and_offset_forward_continuation_are_not_side_exits():
    xs, ys = np.linspace(3., 8., 160), np.linspace(8., -8., 240)
    x, y = np.meshgrid(xs, ys, indexing='ij')
    wedge = (abs(y) < 7.-.8*(x-3)) & (x < 6.8)
    assert detect_corner(wedge, xs, ys, observed=np.ones_like(wedge)) is None
    sloping = abs(y-.7*(x-3)) < 1.
    assert detect_corner(sloping, xs, ys, observed=np.ones_like(sloping)) is None


def test_confirmation_uses_new_frames_and_missing_pose_never_turns():
    nav = CornerNavigator()
    observation = scene()
    nav.observe(observation, 0., frame_key=1)
    for t in np.arange(.1, 1., .1):
        nav.observe(observation, t, frame_key=1)
    assert nav.frames == 1
    nav.observe(observation, 1., frame_key=2)
    assert nav.reason == 'corner_observed_no_pose' and not nav.active
    assert nav.decide(DriveDecision(.8, 0, 0, 'tracking'), None,
                      now=1., pose=None, drive_config=DriveConfig()).vx == .8


@pytest.mark.parametrize('sign', [-1, 1])
def test_closed_loop_approach_rotate_and_exit_confirmation(sign):
    # Measured pose feedback from the plant, with a conservative command lag.
    # This is a synthetic world test, not measured A2 locomotion.
    nav = CornerNavigator()
    cfg = DriveConfig(max_forward_mps=.8)
    pose = np.zeros(3)
    velocity = np.zeros(3)
    phases, commands = [], []
    for i in range(700):
        t = i*.05
        if i % 5 == 0:
            nav.observe(scene(sign, pose), t, pose=pose.copy(), frame_key=i+1)
        decision = nav.decide(DriveDecision(.8, 0., 0., 'tracking'), None,
                              now=t, pose=pose, drive_config=cfg)
        if nav.completed:
            break
        phases.append(nav.phase); commands.append(decision)
        nav.record_command(decision, t)
        target = np.array([decision.vx, decision.vy, decision.yaw_rate])
        velocity += np.clip(target-velocity, [-.025, -.01, -.015], [.025, .01, .015])
        c, s = math.cos(pose[2]), math.sin(pose[2])
        pose[:2] += .05*np.array([c*velocity[0]-s*velocity[1], s*velocity[0]+c*velocity[1]])
        pose[2] += .05*velocity[2]
    assert nav.completed == 1, (nav.metrics(), pose, commands[-1])
    assert set(('approach', 'rotate', 'exit_confirm')).issubset(phases)
    assert abs(pose[2]-sign*math.pi/2) <= math.radians(8)
    assert abs(pose[0]-4.) < .3 and abs(pose[1]) < .3
    assert max(d.vx for d in commands) <= cfg.max_forward_mps
    assert max(abs(d.vy) for d in commands) <= .25
    assert max(abs(d.yaw_rate) for d in commands) <= .18
    assert all(d.vx == d.vy == 0 for d in commands if d.reason == 'corner_rotate')


def armed():
    nav = CornerNavigator()
    nav.observe(scene(pose=(3.9, 0., 0.)), 0., pose=(3.9, 0., 0.), frame_key=1)
    nav.observe(scene(pose=(3.9, 0., 0.)), .25, pose=(3.9, 0., 0.), frame_key=2)
    return nav


@pytest.mark.parametrize('reason', ['camera_stale', 'road_clearance_blocked', 'branch_path_lost',
                                  'path_execution_pose_discontinuity', 'inference_stale'])
def test_corner_cannot_override_safety_stops(reason):
    nav = armed()
    decision = nav.decide(DriveDecision.stop(reason), None, now=.25, pose=(0., 0., 0.),
                          drive_config=DriveConfig())
    assert decision.reason == reason and decision.vx == decision.yaw_rate == 0


def test_unknown_space_fresh_exclusion_and_stale_input_stop_motion():
    nav = armed()
    cfg = DriveConfig()
    command = DriveDecision(.2, 0., 0., 'tracking')
    assert nav.decide(command, None, now=3., pose=(3.9, 0., 0.), drive_config=cfg).reason == 'corner_inputs_stale'
    observation = scene(pose=(3.9, 0., 0.), observed=False)
    nav.observations.clear()
    nav.observe(observation, .5, pose=(3.9, 0., 0.), frame_key=3)
    assert nav.decide(command, None, now=.5, pose=(3.9, 0., 0.), drive_config=cfg).reason == 'corner_space_unverified'
    nav = armed()
    blocked = scene(pose=(3.9, 0., 0.))
    blocked = replace(blocked, footprint_mask=np.zeros_like(blocked.allowed_mask))
    nav.observe(blocked, .5, pose=(3.9, 0., 0.), frame_key=3)
    assert nav.decide(command, None, now=.5, pose=(3.9, 0., 0.), drive_config=cfg).reason == 'corner_space_unverified'


def test_pose_jump_and_timeout_do_not_complete_a_turn():
    nav = armed()
    nav.observe(scene(), .5, pose=(100., 0., 0.), frame_key=3)
    assert nav.phase == 'failed' and nav.completed == 0
    nav = armed()
    nav.started = -100.
    assert nav.decide(DriveDecision(.2, 0, 0, 'tracking'), None, now=.5, pose=(0., 0., 0.),
                      drive_config=DriveConfig()).reason == 'corner_timeout'


def test_road_turning_slows_without_changing_ceiling_or_forcing_straight():
    from local_path import SmoothedPath
    points = np.column_stack((np.linspace(3, 8, 60), np.linspace(3, 8, 60)*math.tan(.5)))
    path = SmoothedPath(points, .95, 0., 'test', road_center_tracking=True,
                        road_geometry={'direction_heading_rad': .5, 'direction_confidence': 1.,
                                       'max_curvature_per_m': .3})
    cfg = DriveConfig(max_forward_mps=.8)
    decision = decide_drive(path, camera_age_sec=0, inference_age_sec=0, config=cfg)
    assert 0 < decision.vx < .8 and 0 < decision.yaw_rate <= .18
    straight = replace(path, points_xy=np.column_stack((points[:, 0], np.zeros(60))),
                       road_geometry={'direction_heading_rad': 0., 'direction_confidence': 1.,
                                      'max_curvature_per_m': 0.})
    assert decide_drive(straight, camera_age_sec=0, inference_age_sec=0, config=cfg).vx == .8


def test_fractional_committed_endpoint_cannot_skip_a_new_exclusion():
    from path_execution import PathEvidence
    xs = np.array([3., 4., 5.])
    ys = np.linspace(1., -1., 21)
    mask = np.ones((3, 21), np.uint8)
    mask[2, 10] = 0
    observation = RoadPathResult(None, 'observed', 'test', {}, mask, xs, ys)
    assert PathEvidence(observation).corridor(np.array([[3., 0.], [4.9, 0.]])) is None


def test_camera_relative_phantom_cannot_arm_but_a_fixed_world_corner_can():
    phantom, actual = CornerNavigator(), CornerNavigator()
    for i in range(21):
        pose = (.05*i, 0., 0.)
        phantom.observe(scene(), i*.2, pose=pose, frame_key=i+1)
        actual.observe(scene(pose=pose), i*.2, pose=pose, frame_key=i+1)
    assert not phantom.active
    assert actual.phase == 'approach'
    assert actual.pivot[0] == pytest.approx(4., abs=.1)


def test_pose_regime_change_restarts_candidate_confirmation():
    nav = CornerNavigator()
    nav.observe(scene(), 0., frame_key=1)
    nav.observe(scene(), .25, frame_key=2)
    nav.observe(scene(), .5, pose=(0., 0., 0.), frame_key=3)
    assert nav.frames == 1 and not nav.active


def test_straight_motion_uses_observed_footprint_and_rejects_new_exclusions():
    nav = CornerNavigator()
    cfg = DriveConfig(max_forward_mps=.8)
    observation = scene(straight=True)
    nav.observe(observation, 0., pose=(0., 0., 0.), frame_key=1)
    nominal = DriveDecision(.8, 0., 0., 'tracking')
    assert nav.decide(nominal, None, now=0., pose=(0., 0., 0.), drive_config=cfg) == nominal
    assert nav.trajectory_guard == 'allowed'
    nav.observe(replace(observation, footprint_mask=np.zeros_like(observation.allowed_mask)),
                .25, pose=(0., 0., 0.), frame_key=2)
    assert nav.decide(nominal, None, now=.25, pose=(0., 0., 0.),
                      drive_config=cfg).reason == 'road_footprint_outside'


def test_exit_confirmation_rejects_an_unrelated_parallel_corridor():
    nav = armed()
    nav.phase = 'exit_confirm'
    pose = (4., 0., math.pi/2)
    correct = scene(pose=pose)
    wrong = replace(correct.estimate, points_xy=correct.estimate.points_xy+np.array([0., 2.]))
    for i in range(2):
        nav.observe(replace(correct, estimate=wrong), 2.+i*.25, pose=pose, frame_key=10+i)
    assert nav.exit_frames == 0


@pytest.mark.parametrize('sign', [-1, 1])
def test_three_meter_camera_blind_zone_keeps_pivot_ground_until_turn_finishes(sign):
    nav, cfg = CornerNavigator(), DriveConfig(max_forward_mps=.8)
    pose, velocity = np.array([-4., 0., 0.]), np.zeros(3)
    for i in range(1300):
        t = i*.05
        if i % 5 == 0:
            full = scene(sign, pose)
            keep = full.x_values >= 3.
            camera = replace(full, allowed_mask=full.allowed_mask[keep], x_values=full.x_values[keep],
                             observed_mask=full.observed_mask[keep], footprint_mask=full.footprint_mask[keep])
            camera = replace(camera, corner=detect_corner(camera.allowed_mask, camera.x_values,
                camera.y_values, 'right', camera.observed_mask))
            nav.observe(camera, t, pose=pose.copy(), frame_key=i+1)
        decision = nav.decide(DriveDecision(.8, 0., 0., 'tracking'), None,
                              now=t, pose=pose, drive_config=cfg)
        if nav.completed:
            break
        nav.record_command(decision, t)
        target = np.array([decision.vx, decision.vy, decision.yaw_rate])
        velocity += np.clip(target-velocity, [-.025, -.01, -.015], [.025, .01, .015])
        c, s = math.cos(pose[2]), math.sin(pose[2])
        pose[:2] += .05*np.array([c*velocity[0]-s*velocity[1], s*velocity[0]+c*velocity[1]])
        pose[2] += .05*velocity[2]
    assert nav.completed == 1, (nav.metrics(), pose, decision)


def test_real_curve_does_not_extrapolate_the_far_heading_into_opposite_lateral_motion():
    from local_path import SmoothedPath
    from drive_control import road_path_errors
    x = np.linspace(3., 8., 160)
    p = SmoothedPath(np.column_stack((x, .3*(x-3.)**2)), .95, 0., 'test',
        road_center_tracking=True, road_geometry={'direction_heading_rad': .5,
            'direction_source': 'edges', 'road_curved': True})
    offset, heading, _ = road_path_errors(p, 4.)
    assert abs(offset) < .1 and heading == .5
