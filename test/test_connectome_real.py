"""Opt-in real pretrained circuit -> HTTP -> R50 masks -> Sport encoding smoke.

CONNECTOME_TEST_BUNDLE and CONNECTOME_TEST_SHA256 are required. Optional
CONNECTOME_TEST_CACHE points to an existing frames/masks replay cache.
No ROS or robot command is published by this test.
"""
import hashlib
import json
import os
from pathlib import Path
import sys
import threading

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))


@pytest.mark.skipif(not os.getenv('CONNECTOME_TEST_BUNDLE'), reason='opt-in pretrained Flyvis bundle required')
def test_real_pretrained_connectome_mask_path_and_command_contract():
    from connectome_client import ConnectomeClient, ConnectomePathTracker
    from connectome_flow import FlyvisFlow
    from connectome_server import PlannerService, make_server
    from local_path import LocalPathConfig
    from swin_l_drive_control import DriveConfig, decide_drive
    from unitree_sport_api import drive_to_sport_move
    from test_connectome import strip

    bundle = Path(os.environ['CONNECTOME_TEST_BUNDLE'])
    expected = os.environ['CONNECTOME_TEST_SHA256']
    assert hashlib.sha256(bundle.read_bytes()).hexdigest() == expected
    encoder = FlyvisFlow(bundle, expected, 'cpu')
    service = PlannerService(encoder)
    server = make_server(service, ('127.0.0.1', 0))
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    client = ConnectomeClient(f'http://127.0.0.1:{server.server_port}/v1/plan', LocalPathConfig(), timeout=5.)
    tracker = ConnectomePathTracker(1.)
    valid = 0
    cache = Path(os.environ['CONNECTOME_TEST_CACHE']) if os.getenv('CONNECTOME_TEST_CACHE') else None
    try:
        for i in range(4):
            if cache:
                image = cv2.imread(str(cache/'frames'/f'frame-{i+1:05}.png'))
                with np.load(cache/'masks'/f'mask-{i:05}.npz') as data:
                    selected, hazard = data['selected'], data['hazard']
            else:
                image = np.zeros((360, 640, 3), np.uint8); image[:, 250+i*3:300+i*3] = 200
                selected = strip(); hazard = np.zeros_like(selected)
            # This test exercises the wiring, not an ego-motion replay.
            stamp = 1.+i*.25
            estimates = client.plan(image, selected, hazard, stamp, 0, i+1)
            assert client.metrics['ready'], client.metrics
            assert client.metrics['model']['nodes'] == 45669
            assert client.metrics['model']['inferred_frames'] == i+1
            tracker.update(estimates[1], stamp)
            path = tracker.current(stamp)
            if path is not None:
                valid += 1
                command = decide_drive(path, camera_age_sec=0., inference_age_sec=0.,
                                       config=DriveConfig(max_forward_mps=.8))
                move = json.loads(drive_to_sport_move(vx=command.vx, vy=command.vy, yaw_rate=command.yaw_rate).parameter_json())
                assert 0 <= move['x'] <= .8 and abs(move['y']) <= .25 and abs(move['z']) <= .18
        assert valid > 0
    finally:
        server.shutdown(); server.server_close(); thread.join(2)


@pytest.mark.skipif(not os.getenv('CONNECTOME_TEST_BUNDLE'), reason='opt-in pretrained Flyvis bundle required')
def test_native_cuda_recurrent_replay_matches_eager_across_dt_and_reset():
    import torch
    if not torch.cuda.is_available():
        pytest.skip('CUDA required for circuit replay')
    from connectome_flow import FlyvisFlow, FlyvisCudaStep
    encoder = FlyvisFlow(os.environ['CONNECTOME_TEST_BUNDLE'], os.environ['CONNECTOME_TEST_SHA256'], 'cuda')
    rng = np.random.default_rng(13)
    frames = [rng.integers(0, 256, (360, 640, 3), dtype=np.uint8) for _ in range(6)]
    stamps = [1., 1.25, 1.38, 2.13, 2.33, 2.58]
    expected = []
    for image, stamp in zip(frames, stamps):
        flow = encoder.advance(image, stamp)
        expected.append((encoder.state.nodes.activity.clone(), flow.relative_motion.copy()))
    with torch.inference_mode(): encoder.cuda_step = FlyvisCudaStep(encoder)
    for _ in range(2):
        encoder.reset()
        for image, stamp, (activity, motion) in zip(frames, stamps, expected):
            actual = encoder.advance(image, stamp)
            torch.testing.assert_close(encoder.cuda_step.activity, activity, rtol=1e-4, atol=1e-4)
            np.testing.assert_allclose(actual.relative_motion, motion, rtol=.005, atol=.005)
