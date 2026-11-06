"""No side-dependent STOP, exact action retention and source-time isolation."""
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from connectome_policy import DriveReadout
from connectome360_client import Connectome360Segmenter
from direct_camera_service import DirectCameraService
from direct_camera_client import DirectCameraClient
from r50_front_stop_policy import (FrontStopReadout, FrontStopPolicy, OBSERVATION_SPEC,
                                   TIME_SPEC, observation_pixels)
import r50_policy_package as package
from test_direct_camera import Source, request, running
from test_r50_360_policy import retrained_package, DETECTOR_PIN


@pytest.mark.parametrize('architecture', ['front', 'temporal_front'])
def test_stop_ignores_side_rear_and_unobserved_robot_pixels_but_retains_action(architecture):
    torch.manual_seed(32)
    actor = DriveReadout().eval()
    model = FrontStopReadout(architecture).eval()
    model.retain_action(actor)
    x = torch.rand((3, 5, 4, 32, 64))
    dt = torch.full((3, 5), .4); dt[:, 0] = 0
    changed = x.clone()
    changed[:, :, 1:] = torch.rand_like(changed[:, :, 1:])
    unseen = torch.from_numpy(observation_pixels() == 0)
    changed[:, :, 0, unseen] = 1. - changed[:, :, 0, unseen]
    with torch.inference_mode():
        action, stop = model(x, dt)
        assert torch.equal(action, actor(x[:, -1])[0])
        assert torch.equal(stop, model(changed, dt)[1])
    # Nonrobot pixels remain observed even when Road is absent.
    known = observation_pixels()
    np.testing.assert_allclose(known[0], 1., atol=1e-7)
    np.testing.assert_allclose(known[:, :20], 1., atol=1e-7)
    assert known[-1, 30:34].max() == 0


def test_temporal_context_is_bounded_and_resets_on_gaps_missing_time_and_task_reset():
    torch.manual_seed(11)
    policy = FrontStopPolicy(FrontStopReadout('temporal_front'))
    pixels = np.random.default_rng(11).random((4, 32, 64), dtype=np.float32)
    cold = policy.infer_pixels(pixels, source_time_sec=0.)[1]
    for k in range(1, 9):
        result = policy.infer_pixels(pixels, source_time_sec=k * .4)[1]
    assert result['context_captures'] == 5
    reset = policy.infer_pixels(pixels, source_time_sec=4.)[1]
    assert reset['context_reset'] == 'source_gap_or_nonmonotonic'
    assert reset['context_captures'] == 1 and reset['stop_probability'] == cold['stop_probability']
    reset = policy.infer_pixels(pixels, source_time_sec=3.)[1]
    assert reset['context_captures'] == 1
    missing = policy.infer_pixels(pixels)[1]
    assert missing['context_reset'] == 'missing_timestamp' and missing['context_captures'] == 1
    policy.reset()
    assert policy.infer_pixels(pixels, source_time_sec=5.)[1]['context_captures'] == 1


def front_package(root):
    config, _ = retrained_package(root)
    model = FrontStopReadout('temporal_front')
    torch.save(dict(schema=package.FRONT_POLICY_SCHEMA, architecture='temporal_front',
        detector_manifest_sha256=DETECTOR_PIN, action_normalization=config['action_normalization'],
        observation_spec=OBSERVATION_SPEC, time_spec=TIME_SPEC,
        state_dict=model.state_dict()), root / 'policy/drive-readout.pt')
    config.update(schema=package.FRONT_PAIR_SCHEMA, policy_schema=package.FRONT_POLICY_SCHEMA,
        architecture='temporal_front', stop_input_chw=[2, 32, 64], stop_uses_side_views=False,
        observation_spec=OBSERVATION_SPEC, time_spec=TIME_SPEC,
        policy_sha256=package.sha256(root / 'policy/drive-readout.pt'))
    (root / 'model.json').write_text(json.dumps(config))
    return config, package.sha256(root / 'model.json')


def test_front_package_runtime_client_contract_and_new_stream_reset(tmp_path, monkeypatch):
    import r50_360_policy_runtime as runtime
    config, pin = front_package(tmp_path)
    masks = np.zeros((4, 270, 480), np.uint8)
    masks[:, 135:, 40:400] = 1
    fake = NS(details={}, reset=lambda: None, predict_views=lambda views: (masks, {}))
    monkeypatch.setattr(runtime, 'R50RoadPerception', lambda *args: fake)
    model = runtime.R50Policy360Runtime('unused', DETECTOR_PIN, tmp_path, pin, device='cpu')
    client = Connectome360Segmenter.__new__(Connectome360Segmenter)
    client.expected_sha256, client.detector_sha256 = pin, DETECTOR_PIN
    client.validate_model(model.details)
    assert model.infer_views([], source_time_sec=2.)[2]['policy']['context_captures'] == 1
    assert model.infer_views([], source_time_sec=2.4)[2]['policy']['context_captures'] == 2
    model.reset()
    assert model.infer_views([], source_time_sec=2.8)[2]['policy']['context_captures'] == 1
    source = Source(lambda: (np.zeros((2, 4, 3), np.uint8), time.time_ns()))
    def stamped(frame, stamp):
        from dataclasses import asdict
        _, command, metrics = model.infer_views([], source_time_sec=stamp / 1e9)
        return dict(command=asdict(command), metrics=metrics)
    service = DirectCameraService(model, source, lambda _: pytest.fail('unstamped HTTP path'),
                                  predict_stamped=stamped)
    with running(service) as url:
        http = DirectCameraClient(url, 'connectome360', pin, 2., detector_sha256=DETECTOR_PIN)
        assert source.reads == 0
        started = time.time_ns()
        first = http.infer('new-task', started, 0, 1.5, time.time_ns)
        second = http.infer('new-task', started, first.capture_stamp_ns, 1.5, time.time_ns)
        assert first.front_metrics['policy']['context_captures'] == 1
        assert second.front_metrics['policy']['context_captures'] == 2
        assert second.capture_stamp_ns > first.capture_stamp_ns
    with pytest.raises(ValueError, match='observation/time'):
        client.validate_model(dict(model.details, stop_uses_side_views=True))
    config['time_spec'] = dict(TIME_SPEC, max_gap_sec=10.)
    (tmp_path / 'model.json').write_text(json.dumps(config))
    with pytest.raises(ValueError, match='input/time'):
        package.load_config(tmp_path, package.sha256(tmp_path / 'model.json'), DETECTOR_PIN)


def test_stamped_inference_uses_capture_time_and_discards_stale_or_failed_context():
    clock = [5_000_000_000]
    calls = []
    model = NS(details={'source_timed_policy': True}, reset=lambda: calls.append('reset'))
    source = Source(lambda: (np.zeros((2, 4, 3), np.uint8), clock[0]))
    def stamped(frame, stamp):
        calls.append(stamp)
        assert stamp == 5_000_000_000
        clock[0] += 2_000_000_000
        return dict(command={})
    service = DirectCameraService(model, source, lambda _: pytest.fail('unstamped path'),
        now_ns=lambda: clock[0], predict_stamped=stamped)
    assert service.process(request(5_000_000_000))['reason'] == 'inference_source_stale'
    assert calls == ['reset', 5_000_000_000, 'reset']
    def failed(frame, stamp):
        raise RuntimeError('inference failed after partial state update')
    service.predict_stamped = failed
    with pytest.raises(RuntimeError):
        service.process(request(5_000_000_000))
    assert calls[-1] == 'reset'
