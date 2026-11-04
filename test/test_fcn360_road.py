import json

import numpy as np
import pytest

from fcn360_road import FcnRoadPlanner
from local_path import LocalPathConfig
from test_apriltag_task_stop_ros import RosHarness, SPORT, ZERO, debug
from road_test_support import FcnInput, start, cancel
from test_task_inference_lifecycle_ros import image, wait_for_count

SHA = 'b' * 64
ARGS = ('--fcn-sha256', SHA, '--fcn-input-width', '512', '--path-mask-class', '1', '--path-planner', 'local')


def lane():
    grids = np.full((4, 8, 16), 13, np.uint8)
    grids[0, 4:] = 3
    grids[1, 4:] = grids[3, 4:] = 11
    grids[1, 5:] = 3
    grids[3, 6:] = 3
    return grids


def test_sides_confirm_then_correct_towards_more_space_rear_never_steers():
    planner = FcnRoadPlanner(LocalPathConfig(), 1.6)
    grids = lane()
    first, metrics = planner.plan(grids)
    assert first is not None and metrics['correction_left_m'] == 0
    planner.plan(grids)
    corrected, metrics = planner.plan(grids)
    assert metrics['center_status'] == 'right_of_center'
    assert metrics['correction_left_m'] > 0
    # Bearing hysteresis may keep already-saturated near points unchanged.
    assert np.all(corrected.points_xy[:, 1] >= first.points_xy[:, 1] - 1e-6)
    assert corrected.points_xy[:, 1].mean() > first.points_xy[:, 1].mean()
    grids[3] = 3  # compost yard: no observable boundary
    unknown, metrics = planner.plan(grids)
    assert metrics['center_status'] == 'unknown' and metrics['correction_left_m'] == 0
    grids[2] = 3
    later, rear = planner.plan(grids)
    np.testing.assert_array_equal(later.points_xy, unknown.points_xy)
    assert rear['rear_center_road_fraction'] != metrics['rear_center_road_fraction']
    grids[0] = 11
    assert planner.plan(grids)[0] is None


def test_only_robot_gap_can_join_front_road_and_history_expires():
    from fcn360_road import front_robot_mask

    planner = FcnRoadPlanner(LocalPathConfig(), 1.6)
    grids = lane()
    grids[0, front_robot_mask()] = 1
    for t in (1., 1.25, 1.5):
        path, metrics = planner.plan(grids, stamp=t)
    assert path is not None and metrics['correction_left_m'] > 0
    assert planner.plan(grids, stamp=4.)[1]['correction_left_m'] == 0
    grids[0, 4, 7:9] = 1  # outside the smaller rig: do not bridge unseen ground
    truncated, _ = planner.plan(grids)
    # Keep only the currently observed prefix; a distant unknown patch must
    # not discard a valid near path or get bridged by the history.
    from fcn360_road import BLIND, ground_sample, observed_classes, ROAD
    assert truncated is not None and truncated.points_xy[-1, 0] < path.points_xy[-1, 0]
    visible = truncated.points_xy[truncated.points_xy[:, 0] >= 3.2]
    assert np.isin(ground_sample(observed_classes(grids)[0], visible[:, 0], visible[:, 1], 1.6), (ROAD, BLIND)).all()
    grids[0, 4:, 6:10] = 11
    assert planner.plan(grids)[0] is None
    grids = lane()
    grids[0, 7] = 11  # Road visible at 3 m, grass immediately in front.
    assert planner.plan(grids)[0] is None


def test_arm_is_unobserved_independent_of_fcn_class_and_needs_visible_shoulders():
    from fcn360_road import BLIND, FRONT_BLIND, observed_classes

    masks = np.full((4, 8, 16), 3, np.uint8)
    masks[0, 4, 7:9] = (14, 5)
    masks[0, 4, 11] = 14
    saved = masks.copy()
    observed = observed_classes(masks)
    assert np.all(observed[0, FRONT_BLIND] == BLIND)
    assert observed[0, 4, 11] == 14
    np.testing.assert_array_equal(masks, saved)
    planner = FcnRoadPlanner(LocalPathConfig(), 1.6)
    assert planner.plan(masks, stamp=1.)[0] is not None
    # Arm pixels classified as road must never supply the only forward evidence.
    masks[0] = 11
    masks[0, FRONT_BLIND] = 3
    for i in range(4):
        assert planner.plan(masks, stamp=1.1 + i * .1)[0] is None


@pytest.mark.parametrize('open_side,expected_sign', [('left',1), ('right',-1)])
def test_side_recovery_can_rotate_before_forward_turn_confirmation(open_side, expected_sign):
    from dangjin_fcn import FcnDrive

    masks = np.full((4, 8, 16), 13, np.uint8)
    masks[:, 4:] = 3
    if open_side == 'left':
        masks[0, 4:, 9:] = 11
        masks[3, 4:] = 11
    else:
        masks[0, 4:, :7] = 11
        masks[1, 4:] = 11
    drive = FcnDrive(1.6, 2.)
    for index,t in enumerate([1., 1.1]):
        cmd, info, _ = drive.command(masks,t,2.)
        assert cmd.vx == cmd.vy == 0.
        if index == 0:
            assert cmd.yaw_rate == 0. and info['maneuver'] == 'confirm_' + open_side
        else:
            assert cmd.yaw_rate == expected_sign*.18
            assert info['maneuver'] == 'recover_' + open_side
            assert not info['front_path_valid']
    cmd, info, _ = drive.command(masks,1.2,2.)
    assert 0 < cmd.vx < 2 and np.sign(cmd.yaw_rate) == expected_sign
    assert info['maneuver'] == 'turn_' + open_side
    cmd, _, _ = drive.command(np.zeros_like(masks),1.3,2.)
    assert (cmd.vx,cmd.yaw_rate) == (0.,0.)
    assert drive.command(masks,1.4,2.)[1]['maneuver'] == 'confirm_' + open_side


def test_two_missing_sides_or_a_blocked_near_strip_stop_with_distant_road():
    planner = FcnRoadPlanner(LocalPathConfig(), 1.6)
    masks = np.full((4,8,16), 3, np.uint8)
    masks[[1,3], 4:] = 11
    assert planner.plan(masks)[0] is None
    masks[:] = 3
    masks[0, 6:] = 11
    assert planner.plan(masks)[0] is None


def test_direct_road_applies_turn_speed_and_stops_on_following_loss(monkeypatch):
    ros = RosHarness(monkeypatch)
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_STOP_ON_' + check.upper(), 'false')
    inputs = FcnInput(monkeypatch, ros)

    def scenario(node):
        start(ros, 1, 'arm-control')
        grids = np.full((4, 8, 16), 13, np.uint8)
        grids[:, 4:] = 3
        grids[0, 4:, :7] = grids[1, 4:] = 11
        for count in (1, 2, 3, 4):
            ros.now += .2
            inputs.send(grids if count < 4 else np.full_like(grids, 11))
            wait_for_count(ros, count)
            command = json.loads(ros.published[SPORT][-1].parameter)
            if count == 3:
                assert ros.metrics()['fcn360']['maneuver'] == 'turn_right'
                assert 0 < command['x'] <= node.drive_config.max_forward_mps * .45
                assert command['y'] == 0 and command['z'] < 0
            elif count == 2:
                assert command == {'x': 0., 'y': 0., 'z': -.18}
                assert ros.metrics()['drive_reason'] == 'fcn_side_recovery'
                assert not ros.metrics()['fcn360']['front_path_valid']
            else:
                assert command == ZERO
                if count < 3:
                    assert ros.metrics()['drive_reason'] == 'fcn_turn_confirmation'
                    assert ros.metrics()['fcn360']['candidate_path_valid']
                    assert not ros.metrics()['fcn360']['front_path_valid']
    ros.run(scenario, *ARGS)


def test_inflight_fcn_plan_is_discarded_on_switch_to_sidewalk(monkeypatch):
    import threading
    ros = RosHarness(monkeypatch)
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_STOP_ON_' + check.upper(), 'false')
    inputs = FcnInput(monkeypatch, ros)
    entered, release = threading.Event(), threading.Event()
    original_plan = FcnRoadPlanner.plan

    def blocked(planner, masks, **kwargs):
        entered.set()
        assert release.wait(4)
        return original_plan(planner, masks, **kwargs)
    monkeypatch.setattr(FcnRoadPlanner, 'plan', blocked)

    def scenario(node):
        try:
            start(ros, 1, 'old-road')
            ros.now += .2
            inputs.send(lane())
            assert entered.wait(4)
            cancel(ros, 'old-road')
            start(ros, 2, 'new-sidewalk')
            ros.now += 2.1
            image(ros)
            release.set()
            wait_for_count(ros, 1)
            assert ros.metrics()['inference_backend'] == 'legacy'
            assert ros.metrics()['fcn360'] is None
        finally:
            release.set()
    ros.run(scenario, *ARGS)
