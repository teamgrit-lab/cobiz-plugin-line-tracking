"""Both task modes reach corner control; cancellation and guards still win."""
import json
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_apriltag_task_stop_ros import RosHarness, SPORT, ZERO, debug
from ros_camera_fixture import complete_inference
from test_road_corner import scene


@pytest.mark.parametrize('mode', [0, 1])
@pytest.mark.parametrize('sign', [-1, 1])
def test_task_corner_uses_measured_yaw_and_resets_on_cancel(monkeypatch, mode, sign):
    ros = RosHarness(monkeypatch)
    ros.pose = np.array([3.9, 0., 0.])
    monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_PATH_EXECUTION_ENABLED', 'true')
    monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_MAX_FORWARD_MPS', '.8')
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_STOP_ON_'+check.upper(), 'false')
    monkeypatch.setattr(debug, 'SegmentationModel', lambda _: NS(device=NS(type='cuda'), reset=lambda: None,
        segment=lambda *_a, **_k: NS(selected_mask=np.ones((360, 640), np.uint8), inference_seconds=.01)))

    def estimates(selected, classes, config, vegetation=None, road_hazard_mask=None, road_analysis=None):
        result = scene(sign, ros.pose)
        if road_analysis is not None:
            road_analysis.update({0: result, 1: result, 'result': result})
        return {key: result.estimate if key in (0, 1) else None for key in classes}

    monkeypatch.setattr(debug, 'extract_path_estimates', estimates)

    def scenario(node):
        node.road_pose = lambda header=None: tuple(ros.pose)
        ros.start(duration=200)
        ros.now = 2.1
        complete_inference(node, ros)
        ros.now += .5
        result = complete_inference(node, ros)
        assert result['selected_mask'] == mode
        assert result['corner_navigation']['state'] == 'rotate'
        assert result['drive_reason'] == 'corner_rotate'
        for _ in range(18):
            ros.now += .5
            ros.pose[2] += sign*.09
            result = complete_inference(node, ros)
            move = json.loads(ros.published[SPORT][-1].parameter)
            assert abs(move['z']) <= .18 and abs(move['y']) <= .25 and 0 <= move['x'] <= .8
            if result['drive_reason'] == 'corner_rotate':
                assert move['x'] == move['y'] == 0
            if result['corner_navigation']['completed']:
                break
        for _ in range(3):
            ros.now += .3
            result = complete_inference(node, ros)
        assert result['corner_navigation']['completed'] == 1
        assert not result['path_recovery']['active']
        node.release_task_control('test_cancel')
        node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert node.corner_navigators[mode].phase == 'idle'
        assert not node.corner_navigators[mode].observations

    ros.run(scenario, '--path-mask-class', str(mode), '--branch-preference', 'center')
