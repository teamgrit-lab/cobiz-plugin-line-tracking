"""ROS task owns the only movement publisher; connectome faults always stop."""
import json
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_apriltag_task_stop_ros import RosHarness, SPORT, ZERO, debug
from test_lidar_obstacles_ros import complete_inference
from local_path import LocalPathEstimate


@pytest.mark.parametrize('mode', [0, 1, 2])
def test_connectome_path_is_followed_and_failure_never_enters_legacy_recovery(monkeypatch, mode):
    ros = RosHarness(monkeypatch)
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_STOP_ON_'+check.upper(), 'false')
    monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_PATH_LOSS_RECOVERY_ENABLED', 'true')
    selected = np.full((360, 640), 2 if mode == 2 else 1, np.uint8)
    requests, enabled = [], [True]
    class Client:
        metrics = {'ready': True, 'backend': 'connectome', 'union_surface': 1}
        def __init__(self, *_a, **_k): pass
        def plan(self, frame, mask, hazard, stamp, generation, sequence):
            requests.append((mask.copy(), hazard.copy(), stamp, generation, sequence))
            self.metrics = {'ready': enabled[0], 'backend': 'connectome', 'union_surface': 1}
            p = np.column_stack((np.linspace(3., 8., 40), np.zeros(40)))
            return {m: LocalPathEstimate(p, .95, 1., 3., p) if enabled[0] else None for m in (0, 1, 2)}
    monkeypatch.setattr(debug, 'ConnectomeClient', Client)
    calls = []
    def segment(frame, **kwargs):
        calls.append(kwargs)
        return NS(selected_mask=selected.copy(), road_hazard_mask=np.zeros_like(selected), inference_seconds=.01)
    monkeypatch.setattr(debug, 'BestSoFarSegmenter', lambda _: NS(device=NS(type='cuda'), reset=lambda: None, segment=segment))
    def scenario(node):
        ros.start(); ros.now = 2.1
        first = complete_inference(node, ros)
        assert first['drive_reason'] in ('tracking', 'tracking_slow_turn')
        move = json.loads(ros.published[SPORT][-1].parameter)
        assert 0 < move['x'] <= node.drive_config.max_forward_mps
        assert calls[-1]['detect_road_hazards'] is True
        assert requests[-1][0].shape == requests[-1][1].shape == (360, 640)
        assert requests[-1][2] > 0 and first['path_planner']['ready']
        ros.now += 1.1; node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert ros.metrics()['drive_reason'].startswith('connectome_')
        enabled[0] = False
        for _ in range(5):
            ros.now += .5
            result = complete_inference(node, ros)
            assert result['drive_reason'] == 'connectome_unavailable'
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
            assert not result['path_recovery']['active']
        enabled[0] = True; ros.now += .3
        complete_inference(node, ros)
        assert json.loads(ros.published[SPORT][-1].parameter)['x'] > 0
        node.release_task_control('cancel')
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
    ros.run(scenario, '--path-planner', 'connectome', '--path-mask-class', str(mode))


def test_cancelled_inflight_planner_reply_cannot_repopulate_path_or_move(monkeypatch):
    from test_lidar_obstacles_ros import image
    ros = RosHarness(monkeypatch)
    returned = threading.Event()
    class Client:
        metrics = {'ready': True, 'backend': 'connectome', 'union_surface': 1}
        def __init__(self, *_a, **_k): pass
        def plan(self, *_a):
            # Simulate a cancellation while HTTP inference is outstanding.
            ros.node.release_task_control('cancel_during_request')
            p = np.column_stack((np.linspace(3., 8., 40), np.zeros(40)))
            returned.set()
            return {m: LocalPathEstimate(p, .95, 1., 3., p) for m in (0, 1, 2)}
    monkeypatch.setattr(debug, 'ConnectomeClient', Client)
    monkeypatch.setattr(debug, 'BestSoFarSegmenter', lambda _: NS(device=NS(type='cuda'), reset=lambda: None,
        segment=lambda *_a, **_k: NS(selected_mask=np.ones((360, 640), np.uint8), inference_seconds=.01)))
    def scenario(node):
        ros.start(); ros.now = 2.1
        image(node, ros)
        assert returned.wait(2)
        threading.Event().wait(.05)
        node.publish_state()
        assert ros.metrics()['inference_count'] == 0
        assert not ros.metrics()['path_planner']['ready']
        moves = [json.loads(m.parameter) for m in ros.published[SPORT] if m.header.identity.api_id == 1008]
        assert moves and all(m == ZERO for m in moves)
    ros.run(scenario, '--path-planner', 'connectome', '--path-mask-class', '1')


def test_current_hazard_output_overrides_the_surface_ema_without_extra_forward():
    import torch
    from test_accelerator_postprocess import make_segmenter
    model = make_segmenter(torch.device('cpu')); model.temporal_alpha = .1
    model.road_hazard_ids = (0, 5)
    scores = torch.zeros((6, 32, 48)); scores[1] = 10
    calls = []
    def semantic(*_a, **_k): calls.append(1); return scores
    model._semantic_scores = semantic
    frame = np.zeros((32, 48, 3), np.uint8)
    assert not model.segment(frame, detect_road_hazards=True).road_hazard_mask.any()
    scores[5, 5:15, 5:15] = 20
    result = model.segment(frame, detect_road_hazards=True)
    assert len(calls) == 2 and result.road_hazard_mask[5:15, 5:15].all()
    assert np.all(result.selected_mask[5:15, 5:15] == 1)
