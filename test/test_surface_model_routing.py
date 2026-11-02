"""Exercise real task/worker routing with simulated models and ROS messages."""
import json
import threading
import time
from dataclasses import asdict
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_apriltag_task_stop_ros import Message, RosHarness, SPORT, ZERO, debug
from test_task_inference_lifecycle_ros import image, wait_for_count
from surface_segmenter import SurfaceSegmenter, FrontRoadInferenceError
from swin_l_drive_control import DriveDecision


FRONT_ARGS = ('--front-connectome-url', 'http://127.0.0.1:8092/v1/front-road',
              '--front-connectome-sha256', '0' * 64, '--path-planner', 'local',
              '--path-mask-class', '1')
ROAD_COMMAND = DriveDecision(.12, 0., -.04, 'tracking')


def result(mode):
    value = NS(selected_mask=np.full((360, 640), mode, np.uint8), inference_seconds=.01)
    if mode == 1:
        value.front_command = asdict(ROAD_COMMAND)
        value.front_metrics = {'test_fixture': True}
    return value


def start(ros, mode, task_id):
    ros.node.on_task_event(Message(json.dumps({
        'type': 'TASK_REGISTERED', 'action_name': 'LINE_TRACKING', 'task_id': task_id,
        'payload': {'selected_mask': mode, 'duration_sec': 60},
    })))
    assert ros.task_state()['type'] == 'TASK_STARTED'
    assert ros.node.tasks.active.selected_mask == mode


def cancel(ros, task_id):
    ros.node.on_task_event(Message(json.dumps({'type': 'TASK_ABORTED', 'task_id': task_id})))
    assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
    ros.now += 1.1
    ros.node.publish_state()


def setup_models(monkeypatch, ros, road_segment=None, road_factory=None):
    import front_connectome_client
    calls = []
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_STOP_ON_' + check.upper(), 'false')
    monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_MAX_FORWARD_MPS', '.5')

    def legacy_segment(frame, **kwargs):
        assert kwargs['path_mask_class'] == 2
        assert kwargs['color_order'] == 'rgb'
        assert 'source_stamp' not in kwargs
        calls.append('sidewalk')
        return result(2)

    def front_segment(frame, **kwargs):
        assert kwargs['path_mask_class'] == 1
        assert kwargs['source_stamp'] > 0
        calls.append('road')
        return road_segment(frame, **kwargs) if road_segment else result(1)

    def legacy_factory(_config):
        calls.append('load_sidewalk')
        return NS(device=NS(type='cuda'), reset=lambda: None, segment=legacy_segment)

    def front_factory(*args):
        calls.append('load_road')
        if road_factory:
            road_factory()
        return NS(device=NS(type='cuda'), reset=lambda: None, segment=front_segment)

    monkeypatch.setattr(debug, 'BestSoFarSegmenter', legacy_factory)
    monkeypatch.setattr(front_connectome_client, 'FrontConnectomeSegmenter', front_factory)
    return calls


@pytest.mark.parametrize('default_mode', [1, 2])
def test_sidewalk_road_sidewalk_tasks_route_both_inference_and_control(monkeypatch, default_mode):
    ros = RosHarness(monkeypatch)
    calls = setup_models(monkeypatch, ros)

    def scenario(node):
        assert calls == ['load_sidewalk']
        for count, mode in enumerate((2, 1, 2, 1), 1):
            task_id = f'mode-{count}'
            start(ros, mode, task_id)
            # No path or learned command may carry over to the new task.
            node.publish_state()
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
            ros.now += 2.1
            image(ros)
            wait_for_count(ros, count)
            move = json.loads(ros.published[SPORT][-1].parameter)
            metrics = ros.metrics()
            if mode == 1:
                assert move == {'x': .12, 'y': 0., 'z': -.04}
                assert metrics['inference_backend'] == 'front-connectome'
            else:
                assert move['x'] > 0 and move['z'] == pytest.approx(0.)
                assert metrics['inference_backend'] == 'legacy'
                assert metrics['front_connectome'] is None
            cancel(ros, task_id)
        assert calls == ['load_sidewalk', 'sidewalk', 'load_road', 'road', 'sidewalk', 'road']

    ros.run(scenario, *FRONT_ARGS, '--path-mask-class', str(default_mode))


def test_front_service_failure_aborts_only_road_and_can_reconnect(monkeypatch):
    ros = RosHarness(monkeypatch)
    unavailable = [True]

    def connect():
        if unavailable[0]:
            raise OSError('service unavailable')

    calls = setup_models(monkeypatch, ros, road_factory=connect)

    def scenario(node):
        start(ros, 1, 'road-down')
        image(ros)
        deadline = time.perf_counter() + 3
        while node.tasks.active is not None:
            assert time.perf_counter() < deadline
            time.sleep(.001)
        assert ros.task_state()['type'] == 'TASK_ABORTED'
        assert ros.task_state()['reason'] == 'front_connectome_unavailable'
        assert ros.running
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        ros.now = 1.1
        node.publish_state()
        start(ros, 2, 'sidewalk-after-failure')
        ros.now += 2.1
        image(ros)
        wait_for_count(ros, 1)
        assert json.loads(ros.published[SPORT][-1].parameter)['x'] > 0
        cancel(ros, 'sidewalk-after-failure')
        unavailable[0] = False
        start(ros, 1, 'road-recovered')
        ros.now += 2.1
        image(ros)
        wait_for_count(ros, 2)
        assert json.loads(ros.published[SPORT][-1].parameter) == {'x': .12, 'y': 0., 'z': -.04}
        assert calls.count('load_road') == 2

    ros.run(scenario, *FRONT_ARGS)


@pytest.mark.parametrize('old_request_fails', [False, True])
def test_late_road_reply_or_failure_cannot_affect_next_sidewalk(monkeypatch, old_request_fails):
    ros = RosHarness(monkeypatch)
    entered, release = threading.Event(), threading.Event()

    def blocked(*args, **kwargs):
        entered.set()
        assert release.wait(3)
        if old_request_fails:
            raise TimeoutError('old Road request timed out')
        return result(1)

    setup_models(monkeypatch, ros, road_segment=blocked)

    def scenario(node):
        try:
            start(ros, 1, 'old-road')
            image(ros)
            assert entered.wait(3)
            cancel(ros, 'old-road')
            start(ros, 2, 'new-sidewalk')
            ros.now += 2.1
            image(ros)
            release.set()
            wait_for_count(ros, 1)
            assert node.tasks.active.task_id == 'new-sidewalk'
            assert ros.metrics()['front_connectome'] is None
            move = json.loads(ros.published[SPORT][-1].parameter)
            assert move['x'] > 0 and move['z'] == pytest.approx(0.)
            assert ros.running
        finally:
            release.set()

    ros.run(scenario, *FRONT_ARGS)


def test_old_motion_env_does_not_gate_road_or_sidewalk(monkeypatch):
    from test_tracking_without_odometry_ros import setup
    ros = setup(monkeypatch)
    setup_models(monkeypatch, ros)

    def scenario(node):
        for mode, task_id in [(1, 'road-without-odom'), (2, 'sidewalk-without-odom')]:
            start(ros, mode, task_id)
            ros.now += 2.1
            before = ros.metrics()['inference_count'] if ros.published.get('/line_tracking/swin_l/metrics') else 0
            image(ros)
            wait_for_count(ros, before + 1)
            command = json.loads(ros.published[SPORT][-1].parameter)
            assert command['x'] > 0
            if mode == 1:
                assert command == {'x': .12, 'y': 0., 'z': -.04}
            assert 'motion_tracking' not in ros.metrics()
            ros.now += .4
            node.publish_state()
            assert json.loads(ros.published[SPORT][-1].parameter) == command
            assert '/dog_odom' not in ros.subscriptions
            assert '/dog_imu_raw' not in ros.subscriptions
            cancel(ros, task_id)

    ros.run(scenario, *FRONT_ARGS)


def test_sidewalk_model_pin_is_still_required_with_front_routing(monkeypatch):
    RosHarness(monkeypatch)
    args = debug.parse_args(['task-drive', *FRONT_ARGS])
    args.model_id = 'unexpected-unpinned-model'
    with pytest.raises(ValueError, match='pinned checkpoint'):
        debug._validate_task_drive_preflight(args)


def test_router_resets_model_history_on_direct_mode_change_and_never_falls_back():
    resets = []
    sidewalk = NS(device=NS(type='cuda'), reset=lambda: resets.append(2),
                  segment=lambda *a, **k: result(2))
    road = NS(device=NS(type='cuda'), reset=lambda: resets.append(1),
              segment=lambda *a, **k: result(1))
    router = SurfaceSegmenter(sidewalk, lambda: road)
    router.segment(None, path_mask_class=1)
    router.segment(None, path_mask_class=2)
    assert resets == [2, 1]
    road.segment = lambda *a, **k: (_ for _ in ()).throw(ValueError('bad model identity'))
    with pytest.raises(FrontRoadInferenceError, match='bad model identity'):
        router.segment(None, path_mask_class=1)
    assert router.road is None
    assert router.segment(None, path_mask_class=2).selected_mask.max() == 2
