"""Exercise the execution policy through task, inference and Sport outputs."""
import json
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_apriltag_task_stop_ros import RosHarness, Message, SPORT, ZERO, debug


@pytest.mark.parametrize('mode', [1, 2])
@pytest.mark.parametrize('with_odom', [False, True])
def test_live_confirmation_segment_handoff_and_task_reset(monkeypatch, mode, with_odom):
    ros = RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_PATH_EXECUTION_ENABLED', 'true')
    monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_MAX_FORWARD_MPS', '.8')
    monkeypatch.setitem(debug.ENV, 'SWIN_L_BRANCH_PREFERENCE', 'center')
    monkeypatch.setitem(debug.ENV, 'SWIN_L_SIDEWALK_VEGETATION_BIAS_M', '0')
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_STOP_ON_'+check.upper(), 'false')
    calls = []
    def segment(_frame, **kwargs):
        calls.append(kwargs)
        return NS(selected_mask=np.full((360, 640), mode, np.uint8),
                  road_hazard_mask=np.zeros((360, 640), bool), inference_seconds=.01)
    monkeypatch.setattr(debug, 'BestSoFarSegmenter', lambda _: NS(
        device=NS(type='cuda'), reset=lambda: None, segment=segment))

    def scenario(node):
        node.road_pose = (lambda header=None: ((header.stamp.sec+header.stamp.nanosec/1e9)*.8
                          if header is not None else ros.now*.8, 0., 0.)) if with_odom else lambda header=None: None
        ros.start(duration=200)
        def feed(dt=.5):
            ros.now += dt
            previous = ros.metrics()['inference_count'] if ros.published.get(debug.DEFAULT_METRICS_TOPIC) else 0
            image = Message(); image.header.stamp = ros.stamp(); node.on_image(image)
            deadline = time.perf_counter()+5
            while True:
                node.publish_state()
                if ros.metrics()['inference_count'] > previous:
                    return ros.metrics()
                assert time.perf_counter() < deadline, ros.errors
                time.sleep(.001)
        first = feed()
        if mode == 2:
            # Execution is enabled globally, but sidewalk starts on its FIRST
            # inference and keeps the old full-range camera-relative reference.
            assert first['drive_reason'] == 'tracking'
            assert first['path_execution'] == {'enabled': False}
            first_path = ros.published[debug.DEFAULT_LOCAL_PATH_TOPIC][-1]
            assert len(first_path.poses) == 20
            assert first_path.poses[-1].pose.position.x == 8.
            second = feed()
            assert second['drive_reason'] == 'tracking'
            assert second['path_execution'] == {'enabled': False}
            assert not second['road_motion']['control_compensated']
            second_path = ros.published[debug.DEFAULT_LOCAL_PATH_TOPIC][-1]
            np.testing.assert_array_equal(
                [p.pose.position.y for p in first_path.poses],
                [p.pose.position.y for p in second_path.poses])
            assert not any(c.get('detect_road_hazards') for c in calls)
            node.release_task_control('test_cancel')
            node.publish_state()
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
            return
        assert first['drive_reason'] == 'path_execution_waiting_initial_candidate'
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        second = feed()
        assert second['path_execution']['generation'] == 1
        assert second['drive_reason'] == 'tracking'
        assert json.loads(ros.published[SPORT][-1].parameter)['x'] == .8
        before = feed()
        assert before['path_execution']['generation'] == 1
        result = before
        for _ in range(4):
            result = feed()
            if result['path_execution']['generation'] >= 2:
                break
        assert result['path_execution']['generation'] >= 2
        assert result['drive_reason'] == 'tracking'
        assert result['path_execution']['progress_source'] == ('odom' if with_odom else 'time')
        assert result['path_execution']['motion_compensation_enabled'] == (mode == 1)
        assert not result['road_motion']['control_compensated'] if mode == 2 else True
        assert any(c.get('detect_road_hazards') for c in calls) == (mode == 1)
        node.release_task_control('test_cancel')
        node.publish_state()
        assert ros.metrics()['path_execution']['generation'] == 0
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
    ros.run(scenario, '--path-mask-class', str(mode))


def test_sidewalk_preserves_existing_path_loss_recovery_with_execution_enabled(monkeypatch):
    from test_path_loss_recovery_ros import run_recovery
    def scenario(ros, node, feed):
        assert feed(visible=True)['path_execution'] == {'enabled': False}
        for expected in ('tracking_path_recovery', 'tracking_path_recovery', 'path_recovery_waiting'):
            result = feed()
            assert result['drive_reason'] == expected
            assert result['path_execution'] == {'enabled': False}
    run_recovery(monkeypatch, scenario,
                 extra_args=('--path-execution-enabled', '--path-mask-class', '2'))
