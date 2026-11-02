import json
from types import SimpleNamespace

import numpy as np
import pytest

from fcn360_road import FRAME_ID, FcnRoadPlanner, mask_topic, read_masks
from local_path import LocalPathConfig
from test_apriltag_task_stop_ros import Message, RosHarness, SPORT, ZERO, debug
from test_surface_model_routing import start, cancel
from test_task_inference_lifecycle_ros import image, wait_for_count

SHA = 'b' * 64
ARGS = ('--fcn-sha256', SHA, '--path-mask-class', '1', '--path-planner', 'local')


def lane():
    grids = np.full((4, 8, 16), 13, np.uint8)
    grids[0, 4:] = 3
    grids[1, 4:] = grids[3, 4:] = 11
    grids[1, 5:] = 3
    grids[3, 6:] = 3
    return grids


def packet(ros, grids=None):
    grids = lane() if grids is None else grids
    msg = Message(grids.tobytes())
    msg.height, msg.width, msg.step, msg.encoding = 32, 16, 16, 'mono8'
    msg.header.stamp, msg.header.frame_id = ros.stamp(), FRAME_ID
    return msg


def test_sides_confirm_then_correct_towards_more_space_rear_never_steers():
    planner = FcnRoadPlanner(LocalPathConfig(), 1.6)
    grids = lane()
    first, metrics = planner.plan(grids)
    assert first is not None and metrics['correction_left_m'] == 0
    planner.plan(grids)
    corrected, metrics = planner.plan(grids)
    assert metrics['center_status'] == 'right_of_center'
    assert metrics['correction_left_m'] > 0
    assert np.all(corrected.points_xy[:, 1] > first.points_xy[:, 1])
    grids[3] = 3  # compost yard: no observable boundary
    unknown, metrics = planner.plan(grids)
    assert metrics['center_status'] == 'unknown' and metrics['correction_left_m'] == 0
    grids[2] = 3
    later, rear = planner.plan(grids)
    np.testing.assert_array_equal(later.points_xy, unknown.points_xy)
    assert rear['rear_center_road_fraction'] != metrics['rear_center_road_fraction']
    grids[0] = 11
    assert planner.plan(grids)[0] is None


def test_tasks_use_only_current_camera_and_stop_on_lost_front(monkeypatch):
    ros = RosHarness(monkeypatch)
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_STOP_ON_' + check.upper(), 'false')
    # An old deployment .env must not reactivate the removed sensor pipeline.
    monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_AVOIDANCE_ENABLED', 'true')
    monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_PATH_LOSS_RECOVERY_ENABLED', 'true')
    side_calls = []
    def segment(frame, path_mask_class, **kwargs):
        side_calls.append(path_mask_class)
        return SimpleNamespace(selected_mask=np.full((360, 640), 2, np.uint8), inference_seconds=.01)
    monkeypatch.setattr(debug, 'BestSoFarSegmenter', lambda _: SimpleNamespace(
        device=SimpleNamespace(type='cuda'), reset=lambda: None, segment=segment))

    def drive():
        return json.loads(ros.published[SPORT][-1].parameter)

    def scenario(node):
        node.destroy_subscription = lambda sub: ros.subscriptions.pop(mask_topic(SHA))
        start(ros, 1, 'fcn')
        road = ros.subscriptions[mask_topic(SHA)]
        old_packet = packet(ros)
        ros.now += 2.1
        image(ros)  # front ROS camera is silent for the Road worker
        node.publish_state()
        assert drive() == ZERO and not side_calls and ros.metrics()['inference_count'] == 0
        road(packet(ros))
        wait_for_count(ros, 1)
        assert ros.metrics()['inference_backend'] == 'fcn512-360'
        assert drive()['x'] > 0 and drive()['y'] == 0
        assert not {'/livox/lidar', '/livox/imu'} & ros.subscriptions.keys()
        assert 'lidar_obstacles' not in ros.metrics()
        ros.now += 1.2
        node.publish_state()
        assert drive() == ZERO
        grass = lane()
        grass[0] = 11
        road(packet(ros, grass))
        wait_for_count(ros, 2)
        assert drive() == ZERO and not ros.metrics()['fcn360']['front_path_valid']
        cancel(ros, 'fcn')
        assert mask_topic(SHA) not in ros.subscriptions
        start(ros, 2, 'sidewalk')
        road(old_packet)
        ros.now += 2.1
        image(ros)
        wait_for_count(ros, 3)
        assert side_calls == [2] and ros.metrics()['inference_backend'] == 'legacy'
        cancel(ros, 'sidewalk')
        start(ros, 1, 'new-fcn')
        ros.subscriptions[mask_topic(SHA)](old_packet)
        node.publish_state()
        assert drive() == ZERO and ros.metrics()['inference_count'] == 3
    ros.run(scenario, *ARGS)


def test_invalid_ids_are_not_a_road_mask():
    msg = Message(bytes(512))
    msg.header.frame_id = FRAME_ID
    msg.encoding, msg.height, msg.width, msg.step = 'mono8', 32, 16, 16
    with pytest.raises(ValueError, match='class ID'):
        read_masks(msg)


def test_only_robot_gap_can_join_front_road_and_history_expires():
    planner = FcnRoadPlanner(LocalPathConfig(), 1.6)
    grids = lane()
    grids[0, 4:, 7:9] = 1
    for t in (1., 1.25, 1.5):
        path, metrics = planner.plan(grids, stamp=t)
    assert path is not None and metrics['correction_left_m'] > 0
    assert planner.plan(grids, stamp=4.)[1]['correction_left_m'] == 0
    grids[0, 4:, 6:10] = 11
    assert planner.plan(grids)[0] is None
    grids = lane()
    grids[0, 7] = 11  # Road visible at 3 m, grass immediately in front.
    assert planner.plan(grids)[0] is None


def test_inflight_road_result_is_discarded_on_switch_to_sidewalk(monkeypatch):
    import threading
    ros = RosHarness(monkeypatch)
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_" + check.upper(), "false")
    entered = threading.Event()
    release = threading.Event()
    original_plan = FcnRoadPlanner.plan
    def blocked(planner, masks, **kwargs):
        entered.set()
        assert release.wait(4)
        return original_plan(planner, masks, **kwargs)
    monkeypatch.setattr(FcnRoadPlanner, 'plan', blocked)
    def scenario(node):
        node.destroy_subscription = lambda sub: ros.subscriptions.pop(mask_topic(SHA))
        try:
            start(ros, 1, 'old-road')
            ros.subscriptions[mask_topic(SHA)](packet(ros))
            assert entered.wait(4)
            cancel(ros, 'old-road')
            start(ros, 2, 'new-sidewalk')
            release.set()
            ros.now += 2.1
            image(ros)
            wait_for_count(ros, 1)
            assert ros.metrics()['inference_backend'] == 'legacy'
            assert ros.metrics()['fcn360'] is None
        finally:
            release.set()
    ros.run(scenario, *ARGS)
