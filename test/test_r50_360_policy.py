"""Detector replacement keeps actor weights, task isolation and physical caps."""
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from connectome360_client import Connectome360Segmenter
from connectome360_runtime import package_config, sha256
from connectome_policy import ConnectomeDrivePolicy, DriveReadout, policy_pixels
from direct_camera_client import DirectCameraClient
from direct_camera_service import DirectCameraService
from r50_360_policy_runtime import R50RoadPerception, R50Policy360Runtime
from surround_camera import load_profile, profile_sha256
from test_direct_camera import Source, running

DETECTOR_PIN = 'e'*64
PACKAGE_PIN = 'a'*64


def identity():
    return dict(backend='r50-360-road-policy', weights_sha256=PACKAGE_PIN,
        detector_sha256=DETECTOR_PIN, policy_sha256='c'*64,
        training_road_sha256='b'*64, training_encoder_sha256='d'*64,
        detector_override=True, runtime_planner=False,
        camera_views=['front', 'left', 'rear', 'right'],
        camera_profile_sha256=profile_sha256(load_profile()),
        road_mask_shape=[4,270,480], actor_input_chw=[4,32,64], actor_crop_rows=[135,257],
        trained_max_speed_mps=10., head_device='cpu', uses_ground_truth_at_runtime=False,
        pose_input=False, goal_input=False)


def source_package(root):
    (root/'policy').mkdir()
    (root/'road').mkdir()
    (root/'flyvis_encoder.pt').write_bytes(b'training encoder, not loaded')
    (root/'road/road-readout.pt').write_bytes(b'training detector, not loaded')
    road = sha256(root/'road/road-readout.pt')
    torch.manual_seed(917)
    torch.save(dict(schema='dangjin.drive_readout.vision.v2', road_sha256=road,
                    state_dict=DriveReadout().state_dict()), root/'policy/drive-readout.pt')
    (root/'policy/training.json').write_text('{}')
    config = dict(schema='connectome360.camera_only.pair.v1', simulation_max_speed_mps=10.,
        input_contract=dict(panorama_shape_wh=[2880,1440], camera_views=['front','left','rear','right'],
            view_shape_wh=[480,270], forward_heading_deg=90, actor_input_chw=[4,32,64], actor_crop_rows=[135,257]),
        camera_profile_sha256=profile_sha256(load_profile()),
        encoder_sha256=sha256(root/'flyvis_encoder.pt'), road_sha256=road,
        policy_sha256=sha256(root/'policy/drive-readout.pt'))
    (root/'model.json').write_text(json.dumps(config))
    return config, sha256(root/'model.json')


def test_r50_batch_excludes_curb_grass_and_robot_without_cross_view_history():
    received = []
    classes = [13, 2, 30, 64]  # Road, Curb, Vegetation, Ego Vehicle.
    def processor(*, images, return_tensors):
        received.append([v[0,0].tolist() for v in images])
        return {'pixel_values': torch.zeros((4,3,384,640))}
    def scores(_):
        values = torch.zeros((4,65,1,1))
        for i, label in enumerate(classes):
            values[i,label] = 1
        return values.expand(4,65,360,640)
    detector = R50RoadPerception.__new__(R50RoadPerception)
    detector.details = {}
    detector.runner = NS(processor=processor, model=lambda **_: None,
                         _move_inputs=lambda x: x, _r50_output_scores=scores,
                         road_ids=[13,23,24,41])
    views = [np.full((270,480,3), [10+i,20+i,30+i], np.uint8) for i in range(4)]
    masks, _ = detector.predict_views(views)
    assert masks.shape == (4,270,480) and masks.dtype == np.uint8
    assert [float(v.mean()) for v in masks] == [1.,0.,0.,0.]
    assert received == [[[30+i,20+i,10+i] for i in range(4)]]
    assert policy_pixels(masks).shape == (4,32,64)
    classes[:] = [2,13,13,13]
    masks, _ = detector.predict_views(views)
    assert [float(v.mean()) for v in masks] == [0.,1.,1.,1.]
    with pytest.raises(ValueError, match='aligned'):
        detector.predict_views(views[:3])


def test_runtime_retains_exact_actor_and_training_pair_pin(tmp_path, monkeypatch):
    import r50_360_policy_runtime as runtime
    config, pin = source_package(tmp_path)
    masks = np.zeros((4,270,480), np.uint8)
    masks[:,:,75:405] = 1
    fake = NS(details={'manifest_sha256': DETECTOR_PIN}, reset=lambda: None,
              predict_views=lambda views: (masks, {'road_fraction': masks.mean(axis=(1,2)).tolist()}))
    monkeypatch.setattr(runtime, 'R50RoadPerception', lambda *a: fake)
    actor_before = (tmp_path/'policy/drive-readout.pt').read_bytes()
    model = R50Policy360Runtime('unused', DETECTOR_PIN, tmp_path, pin, device='cpu')
    original = ConnectomeDrivePolicy(tmp_path/'policy', NS(details=dict(
        weights_sha256=config['road_sha256'], encoder_sha256=config['encoder_sha256'])),
        max_speed_mps=10.)
    _, command, metrics = model.infer_views([])
    expected, _ = original.infer(masks)
    assert asdict(command) == asdict(expected)
    assert metrics['policy']['raw_action'] == original.infer(masks)[1]['raw_action']
    assert actor_before == (tmp_path/'policy/drive-readout.pt').read_bytes()
    assert all(torch.equal(v, original.model.state_dict()[k])
               for k,v in model.policy.model.state_dict().items())
    assert model.details['detector_sha256'] == DETECTOR_PIN
    assert model.details['policy_sha256'] == config['policy_sha256']
    assert model.details['training_road_sha256'] == config['road_sha256']
    assert model.details['trained_max_speed_mps'] == 10.
    assert model.policy.details['detector_override'] is True
    with pytest.raises(ValueError, match='trained Flyvis'):
        ConnectomeDrivePolicy(tmp_path/'policy', NS(details=dict(
            weights_sha256=DETECTOR_PIN, encoder_sha256=config['encoder_sha256'])))
    wrong = dict(config, road_sha256='f'*64)
    with pytest.raises(ValueError, match='trained Flyvis'):
        ConnectomeDrivePolicy.with_detector_override(tmp_path/'policy', wrong, fake.details)
    wrong = dict(config, policy_sha256='f'*64)
    with pytest.raises(ValueError, match='retained policy'):
        ConnectomeDrivePolicy.with_detector_override(tmp_path/'policy', wrong, fake.details)
    (tmp_path/'policy/drive-readout.pt').write_bytes(b'wrong actor')
    with pytest.raises(ValueError, match='weight SHA-256'):
        package_config(tmp_path, pin)


def test_direct_http_requires_both_pins_and_preserves_learned_curvature():
    model = NS(details=identity(), reset=lambda: None)
    source = Source(lambda: (np.zeros((2,4,3), np.uint8), time.time_ns()))
    def predict(_):
        return dict(command=dict(vx=10.,vy=0.,yaw_rate=.35,reason='neural_drive'), metrics={})
    service = DirectCameraService(model, source, predict)
    with running(service) as url:
        with pytest.raises(ValueError, match='identity'):
            DirectCameraClient(url, 'connectome360', PACKAGE_PIN, 2.)
        with pytest.raises(ValueError, match='override'):
            DirectCameraClient(url, 'connectome360', PACKAGE_PIN, 2., detector_sha256='f'*64)
        client = DirectCameraClient(url, 'connectome360', PACKAGE_PIN, 2., detector_sha256=DETECTOR_PIN)
        assert source.reads == 0  # Health/identity requests never acquire a camera.
        started = time.time_ns()
        result = client.infer('task', started, 0, 1.5, time.time_ns)
        assert result.front_command == dict(vx=.8,vy=0.,yaw_rate=pytest.approx(.028),reason='tracking')
        model.details['policy_sha256'] = 'f'*64
        with pytest.raises(ValueError, match='weights changed'):
            client.infer('task', started, 0, 1.5, time.time_ns)


@pytest.mark.parametrize('change', [dict(pose_input=True), dict(goal_input=True),
    dict(runtime_planner=True), dict(actor_input_chw=[4,16,32]), dict(road_mask_shape=[4,360,640]),
    dict(detector_override=False)])
def test_detector_override_contract_rejects_hidden_inputs_or_wrong_shapes(change):
    client = Connectome360Segmenter.__new__(Connectome360Segmenter)
    client.expected_sha256, client.detector_sha256 = PACKAGE_PIN, DETECTOR_PIN
    with pytest.raises(ValueError):
        client.validate_model(dict(identity(), **change))


@pytest.mark.parametrize('arguments', [
    ['--connectome360-detector-sha256', DETECTOR_PIN],
    ['--connectome360-url', 'http://127.0.0.1:8093/v1/direct-road',
     '--connectome360-sha256', PACKAGE_PIN, '--connectome360-detector-sha256', DETECTOR_PIN],
    ['--road-direct-input', '--connectome360-url', 'http://127.0.0.1:8093/v1/direct-road',
     '--connectome360-sha256', PACKAGE_PIN, '--connectome360-detector-sha256', 'invalid'],
])
def test_r50_override_requires_explicit_direct_road_and_valid_pin(monkeypatch, arguments):
    import line_tracking_runtime as runtime
    monkeypatch.setattr(runtime, 'ENV', {})
    with pytest.raises(SystemExit):
        runtime.parse_args(['task-drive', '--path-mask-class', '1', *arguments])
