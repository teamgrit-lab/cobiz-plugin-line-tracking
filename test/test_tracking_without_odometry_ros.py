"""Retired odom/gyro compensation cannot gate the live command publisher."""
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_apriltag_task_stop_ros import RosHarness, SPORT, ZERO, debug
from test_lidar_obstacles_ros import complete_inference
from local_path import LocalPathEstimate


def setup(monkeypatch):
    ros = RosHarness(monkeypatch)
    # Deployment overrides used to force these values on. Even malformed old
    # compensation settings must no longer create subscriptions or stop gates.
    for key, value in {
        'ENABLED': 'true', 'SENSOR_MAX_AGE_SEC': 'invalid',
        'BASE_FROM_CHILD_TRANSFORM': 'invalid', 'BASE_FROM_IMU_TRANSFORM': 'invalid',
    }.items():
        monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_MOTION_' + key, value)
    monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_MAX_FORWARD_MPS', '.80')
    monkeypatch.setattr(debug, 'BestSoFarSegmenter', lambda _config: NS(
        device=NS(type='cuda'), reset=lambda: None,
        segment=lambda _frame, **kwargs: NS(
            selected_mask=np.full((360, 640), kwargs['path_mask_class'] or 2, np.uint8),
            inference_seconds=.01)))
    for name in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_STOP_ON_' + name.upper(), 'false')

    class Client:
        metrics = {'ready': True, 'union_surface': 1}

        def __init__(self, *_a, **_k):
            self.config = debug.PlannerConfig()

        def plan(self, *_a, **_k):
            p = np.column_stack((np.linspace(3., 8., 60), np.zeros(60)))
            return {m: LocalPathEstimate(p, .95, 1., 3., p) for m in (0, 1, 2)}

    monkeypatch.setattr(debug, 'ConnectomeClient', Client)
    return ros


@pytest.mark.parametrize('planner', ['local', 'connectome'])
@pytest.mark.parametrize('mode', [0, 1, 2])
def test_commands_do_not_require_motion_sensors_or_restart_acceleration(monkeypatch, planner, mode):
    ros = setup(monkeypatch)

    def scenario(node):
        ros.start()
        ros.now = 2.1
        result = complete_inference(node, ros)
        first = json.loads(ros.published[SPORT][-1].parameter)
        assert first['x'] > .015  # No former 0.30 m/s² restart ramp.
        assert 'motion_tracking' not in result
        assert '/dog_odom' not in ros.subscriptions
        assert '/dog_imu_raw' not in ros.subscriptions
        source_stamp = debug._stamp_ns(ros.published['/line_tracking/swin_l/local_path'][-1].header)
        ros.now += .4  # Beyond former sensor/extrapolation windows, with no new input.
        node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter) == first
        assert ros.metrics()['inference_count'] == result['inference_count']
        assert debug._stamp_ns(ros.published['/line_tracking/swin_l/local_path'][-1].header) == source_stamp
        node.release_task_control('cancel')
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(scenario, '--path-planner', planner, '--path-mask-class', str(mode))


@pytest.mark.parametrize('mask', [1, 2])
def test_navigation_worker_does_not_require_motion_sensors_and_keeps_stop(monkeypatch, mask):
    from test_navigation_worker_ros import run, send, velocity
    ros = setup(monkeypatch)

    def scenario(node):
        ros.now = 2.1
        send(ros, mask=mask)
        complete_inference(node, ros)
        first = velocity(ros)
        assert first['vx'] > 0
        assert '/dog_odom' not in ros.subscriptions
        assert '/dog_imu_raw' not in ros.subscriptions
        ros.now += .4
        node.publish_state()
        assert velocity(ros)['vx'] == first['vx']
        send(ros, 'STOP', mask=mask)
        assert velocity(ros)['vx'] == velocity(ros)['yaw_rate'] == 0
        assert SPORT not in ros.published

    run(ros, scenario)


def test_process_results_keep_parent_cancellation_without_motion_gates(monkeypatch):
    ros = setup(monkeypatch)
    closed = []
    monkeypatch.setattr(debug, '_connectome_client_from_args', lambda *_: NS(
        url='http://127.0.0.1:8091/v1/plan', config=debug.PlannerConfig(), timeout=.8,
        metrics={'ready': False}))

    class Process:
        device = NS(type='cuda')
        cuda_memory = {}

        def __init__(self, *_):
            pass

        def reset(self):
            pass

        def close(self):
            closed.append(True)

        def infer(self, rgb, stamp, generation, sequence, *, mode):
            assert mode == 2
            p = np.array([[3., 0.], [8., 0.]])
            estimate = LocalPathEstimate(p, .9, 1., 3., p)
            return {'result': NS(selected_mask=np.ones((360, 640), np.uint8), inference_seconds=.01),
                    'estimates': {m: estimate if m == mode else None for m in (0, 1, 2)},
                    'planner': {'backend': 'local', 'ready': True, 'surface_class': 2},
                    'segmentation_finished_at': debug.time.monotonic()}

    monkeypatch.setattr(debug, 'PerceptionProcess', Process)

    def scenario(node):
        ros.start()
        ros.now = 2.1
        result = complete_inference(node, ros)
        assert result['pipeline_timing']['inference_process']
        assert json.loads(ros.published[SPORT][-1].parameter)['x'] > 0
        ros.now += .4
        node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter)['x'] > 0
        node.release_task_control('cancel')
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(scenario, '--path-planner', 'connectome', '--inference-process')
    assert closed
