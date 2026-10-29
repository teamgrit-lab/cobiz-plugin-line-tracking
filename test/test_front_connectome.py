import json
import sys
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from connectome_server import encode_image
from front_connectome_client import front_drive_decision, limit_front_command, validated_command
from front_connectome_service import FrontService, PROTOCOL
from swin_l_drive_control import DriveDecision


def test_single_image_protocol_rejects_pose_masks_surround_and_wrong_mode():
    calls = []
    def infer(frame):
        calls.append(frame.shape)
        return np.ones((270, 480), np.uint8), DriveDecision(.2, 0., -.05, 'tracking'), {}
    service = FrontService(NS(reset=lambda: None, infer=infer, details={'test_fixture': True}))
    req = dict(schema=PROTOCOL, stream_id='a', request_id='a:1', stamp_sec=1., mode=1,
               image_png=encode_image(np.full((360, 640, 3), 130, np.uint8)))
    for key in ['pose', 'target_xy', 'selected_png', 'views_png']:
        with pytest.raises(ValueError): service.process(dict(req, **{key: []}))
    with pytest.raises(ValueError): service.process(dict(req, mode=2))
    reply = service.process(req)
    assert calls == [(360, 640, 3)]
    assert reply['command'] == asdict(DriveDecision(.2, 0., -.05, 'tracking'))
    with pytest.raises(ValueError): service.process(req)


def test_no_command_on_stale_invalid_or_wrong_mode_and_caps_preserve_curvature():
    command = asdict(DriveDecision(.8, 0., .32, 'tracking'))
    assert front_drive_decision(command, 1, [.1, .2], 1.).vx == .8
    for ages in [[None, .1], [.1, 2.], [-1., .1], [float('nan'), .1]]:
        assert front_drive_decision(command, 1, ages, 1.).vx == 0
    assert front_drive_decision(command, 0, [.1, .1], 1.).vx == 0
    assert front_drive_decision(None, 1, [.1, .1], 1.).vx == 0
    capped = limit_front_command(validated_command(command), .2, .3)
    assert capped.vx == .2 and capped.yaw_rate == pytest.approx(.08)
    for bad in [dict(command, vx=float('nan')), dict(command, vy=.1), dict(command, yaw_rate=.5), dict(command, reason='neural_wait')]:
        with pytest.raises(ValueError): validated_command(bad)


def test_road_task_uses_model_command_then_stops_on_stale_and_cancellation(monkeypatch):
    from test_apriltag_task_stop_ros import RosHarness, SPORT, ZERO, debug
    from test_lidar_obstacles_ros import complete_inference
    ros = RosHarness(monkeypatch)
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_STOP_ON_' + check.upper(), 'false')
    def segment(_frame, **kwargs):
        assert kwargs['color_order'] == 'rgb' and kwargs['path_mask_class'] == 1
        assert kwargs['source_stamp'] > 0
        return NS(selected_mask=np.ones((270, 480), np.uint8), road_hazard_mask=np.zeros((270, 480), np.uint8),
            inference_seconds=.01, front_command=asdict(DriveDecision(.12, 0., -.04, 'tracking')), front_metrics={})
    monkeypatch.setattr(debug, '_segmenter_from_args', lambda _: NS(device=NS(type='cuda'), reset=lambda: None, segment=segment))
    def scenario(node):
        ros.start(); ros.now = 2.1
        complete_inference(node, ros)
        move = json.loads(ros.published[SPORT][-1].parameter)
        assert move == {'x': .12, 'y': 0., 'z': -.04}
        ros.now += 1.1; node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert ros.metrics()['drive_reason'] == 'front_connectome_source_stale'
        node.release_task_control('cancel')
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
    ros.run(scenario, '--path-mask-class', '1', '--path-planner', 'local',
            '--front-connectome-url', 'http://127.0.0.1:8092/v1/front-road', '--front-connectome-sha256', '0' * 64)


def test_surface_routing_rejects_ambiguous_union_task(monkeypatch):
    from test_apriltag_task_stop_ros import RosHarness, Message, debug
    ros = RosHarness(monkeypatch)
    monkeypatch.setattr(debug, '_segmenter_from_args', lambda _: NS(device=NS(type='cuda'), reset=lambda: None))
    def scenario(node):
        node.on_task_event(Message(json.dumps({'type': 'TASK_REGISTERED', 'action_name': 'LINE_TRACKING',
            'task_id': 'wrong-surface', 'payload': {'duration_sec': 30, 'selected_mask': 0}})))
        assert ros.task_state()['type'] == 'TASK_REJECTED'
        assert ros.task_state()['reason'] == 'front_connectome_requires_road_or_sidewalk'
        assert node.tasks.active is None
    ros.run(scenario, '--path-mask-class', '1', '--path-planner', 'local',
            '--front-connectome-url', 'http://127.0.0.1:8092/v1/front-road', '--front-connectome-sha256', '0' * 64)


def test_saturated_neural_output_respects_exact_transport_limits():
    import torch
    from front_connectome import FrontConnectome
    model = FrontConnectome.__new__(FrontConnectome)
    model.device = torch.device('cpu')
    model.config = {'max_speed_mps': .8, 'max_yaw_rate_radps': .35, 'stop_threshold': .5}
    model.actor = lambda _: (torch.tensor([[1., -1.]]), torch.tensor([[20., -20.]]))
    command, _ = model.act(np.ones((270, 480), np.uint8))
    assert validated_command(asdict(command)) == DriveDecision(.8, 0., -.35, 'tracking')


def test_lidar_overlay_survives_front_output_limit_but_raw_model_cannot_request_lateral():
    command = DriveDecision(.2, -.6, .04, 'lidar_avoiding')
    assert limit_front_command(command, .5, .18, max_lateral=.8) == command
    assert limit_front_command(command, .5, .18).vy == 0
    capped = limit_front_command(command, .5, .18, max_lateral=.3)
    assert capped.vy == -.3 and capped.vx == .1 and capped.yaw_rate == .02
    with pytest.raises(ValueError):
        validated_command(asdict(DriveDecision(.2, -.6, .04, 'tracking')))
