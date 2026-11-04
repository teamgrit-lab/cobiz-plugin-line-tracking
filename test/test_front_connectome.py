import json
import sys
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from front_connectome_client import front_drive_decision, limit_front_command, validated_command
from swin_l_drive_control import DriveDecision


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
