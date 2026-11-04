"""Input-contract parity and capture ownership for the accelerated Road path."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import cv2
import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from surround_tensor import SurroundTensorProjector, eye_canvases, area_weights
from surround_camera import project_panorama
from surround_capture import HardwareCamera, upload_bgrx
from fcn360_road import camera_maps, front_robot_mask

DEVICES = ['cpu', pytest.param('cuda', marks=pytest.mark.skipif(
    not torch.cuda.is_available(), reason='CUDA device required for native GPU parity'))]


@pytest.mark.parametrize('device', DEVICES)
def test_flyvis_projection_matches_reference_and_wraps_without_mirroring(device):
    rng = np.random.default_rng(9027)
    # Includes high-frequency edges, horizontal seam, poles, and channel order.
    panorama = rng.integers(0, 256, (1440, 2880, 3), dtype=np.uint8)
    projector = SurroundTensorProjector('flyvis', device=device)
    actual = projector.project(torch.from_numpy(panorama).to(device)).permute(0, 2, 3, 1).cpu().numpy()
    expected = np.stack(project_panorama(panorama))
    np.testing.assert_allclose(actual, expected, atol=1, rtol=0)
    landmarks = np.zeros_like(panorama)
    colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0]]
    for column, color in zip([720, 0, 2160, 1440], colors):
        landmarks[:, np.arange(column-40, column+40) % 2880] = color
    actual = projector.project(torch.from_numpy(landmarks).to(device))[:, :, 135, 240].cpu().numpy()
    np.testing.assert_array_equal(actual, colors)


@pytest.mark.parametrize('device', DEVICES)
@pytest.mark.parametrize('width', [512, 1024])
def test_fcn_projection_and_normalization_use_native_sampling(device, width):
    rng = np.random.default_rng(45)
    panorama = rng.integers(0, 256, (1440, 2880, 4), dtype=np.uint8)
    projector = SurroundTensorProjector('fcn', input_width=width, device=device)
    actual = projector.fcn_input(torch.from_numpy(panorama).to(device))
    # Explicit interpolation tables give the same reference on OpenCV builds
    # with or without an alternate full-float remap implementation (e.g. macOS).
    expected = []
    for x, y in camera_maps(width):
        maps, fractions = cv2.convertMaps(x, y, cv2.CV_16SC2)
        bgr = cv2.remap(panorama[..., :3], maps, fractions, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
        rgb = bgr[..., ::-1].astype(np.float32)/255
        rgb -= np.float32([.485, .456, .406])
        rgb /= np.float32([.229, .224, .225])
        expected.append(rgb.transpose(2, 0, 1))
    assert actual.device.type == device and actual.shape == (4, 3, width//2, width)
    np.testing.assert_allclose(actual.cpu(), expected, atol=1/255/.224 + 1e-5, rtol=0)


@pytest.mark.parametrize('device', DEVICES)
def test_flyvis_grayscale_letterbox_and_policy_area_match_training(device):
    from connectome_policy import policy_pixels
    rng = np.random.default_rng(7)
    frames = rng.integers(0, 256, (4, 270, 480, 3), dtype=np.uint8)
    actual = eye_canvases(torch.from_numpy(frames).to(device).permute(0, 3, 1, 2), 171).cpu().numpy()[:, 0]
    expected = []
    h = round(270*171/480)
    for frame in frames:
        canvas = np.full((171, 171), .5, np.float32)
        canvas[(171-h)//2:(171-h)//2+h] = cv2.resize(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)/255, (171, h))
        expected.append(canvas)
    np.testing.assert_allclose(actual, expected, atol=4e-5, rtol=0)
    masks = rng.integers(0, 2, (4, 270, 480), dtype=np.uint8)
    actual = (area_weights(122, 32, device) @ torch.from_numpy(masks[:, 135:257]).to(device).float()
              @ area_weights(480, 64, device).T)
    np.testing.assert_allclose(actual.cpu(), policy_pixels(masks), atol=2e-7, rtol=0)


def test_decoder_upload_respects_stride_offset_and_owns_its_memory():
    data = bytearray([99]*48)
    data[4:12] = bytes([1, 2, 3, 4, 5, 6, 7, 8])
    data[24:32] = bytes([9, 10, 11, 12, 13, 14, 15, 16])
    actual = upload_bgrx(data, 2, 2, 20, 4, device='cpu')
    data[:] = bytes(len(data))
    assert actual.tolist() == [[[1, 2, 3, 4], [5, 6, 7, 8]], [[9, 10, 11, 12], [13, 14, 15, 16]]]
    for stride, offset in [(7, 0), (20, -1), (50, 0)]:
        with pytest.raises(ValueError, match='layout'):
            upload_bgrx(data, 2, 2, stride, offset, device='cpu')


@pytest.mark.parametrize('fails', [False, True])
def test_hardware_capture_keeps_pts_and_unmaps_after_upload(monkeypatch, fails):
    import surround_capture
    events = []
    camera = HardwareCamera.__new__(HardwareCamera)
    info = NS(width=2880, height=1440, finfo=NS(name='BGRx'), stride=[11520], offset=[0])
    buffer = NS(map=lambda flags: (True, NS(data=b'frame')), unmap=lambda mapped: events.append('unmap'))
    sample = NS(get_buffer=lambda: buffer, get_caps=lambda: 'caps')
    camera._sample = lambda clock: (sample, 123456789)
    camera.video = NS(VideoInfo=NS(new_from_caps=lambda caps: info),
                      buffer_get_video_meta=lambda buf: NS(stride=[11584], offset=[128]))
    camera.gst = NS(MapFlags=NS(READ=1))
    def upload(data, width, height, stride, offset):
        assert (data, width, height, stride, offset) == (b'frame', 2880, 1440, 11584, 128)
        events.append('upload')
        if fails:
            raise RuntimeError('upload failed')
        return 'cuda frame'
    monkeypatch.setattr(surround_capture, 'upload_bgrx', upload)
    if fails:
        with pytest.raises(RuntimeError, match='upload failed'):
            camera.read()
    else:
        assert camera.read() == ('cuda frame', 123456789)
    assert events == ['upload', 'unmap']


def test_cuda_capture_stays_closed_before_road_and_closes_on_idle(monkeypatch):
    from direct_camera_service import DirectCameraService, UsbSource, SCHEMA
    calls = []
    class Capture:
        def __init__(self, device): calls.append(('open', device))
        def read(self): return 'cuda frame', 10_000_000_000
        def close(self): calls.append('close')
    source = UsbSource('/dev/video91', camera_factory=Capture)
    model = NS(details={}, reset=lambda: None)
    service = DirectCameraService(model, source, lambda frame: dict(result=frame), now_ns=lambda: 10_000_000_000)
    assert calls == []
    request = dict(schema=SCHEMA, stream_id='task', request_id='task:1', mode=1,
                   not_before_ns=9_000_000_000, after_stamp_ns=0, max_age_sec=1.5)
    assert service.process(request)['result'] == 'cuda frame'
    assert calls == [('open', '/dev/video91')]
    monkeypatch.setattr('direct_camera_service.time.monotonic', lambda: service.used_at+2.5)
    service.expire_idle()
    assert calls[-1] == 'close' and source.camera is None


@pytest.mark.parametrize('device', DEVICES)
def test_tensor_policy_matches_existing_commands_and_fcn_preserves_ego(device):
    from connectome_policy import ConnectomeDrivePolicy, DriveReadout
    from fcn360_detector import classify_cuda
    masks = np.random.default_rng(67).integers(0, 2, (4, 270, 480), dtype=np.uint8)
    actor = ConnectomeDrivePolicy.__new__(ConnectomeDrivePolicy)
    actor.device, actor.max_speed_mps = device, 10.
    torch.manual_seed(9027)
    actor.model = DriveReadout().to(device).eval()
    old, metrics = actor.infer(masks)
    new, tensor_metrics = actor.infer_tensor(torch.from_numpy(masks).to(device))
    assert new.reason == old.reason
    assert new.vx == pytest.approx(old.vx, abs=1e-5)
    assert new.yaw_rate == pytest.approx(old.yaw_rate, abs=1e-6)
    assert tensor_metrics['stop_probability'] == pytest.approx(metrics['stop_probability'], abs=1e-6)
    class Infer:
        def infer_tensor(self, view):
            assert view.device.type == device and view.shape == (1, 3, 512, 1024)
            logits = torch.zeros((1, 21, 16, 32), device=device)
            logits[:, 3] = 2.
            logits[:, 1, 0, 0] = 3.  # Off-prior ego is unknown, never road.
            return logits
    projector = SurroundTensorProjector('fcn', device=device)
    classes = classify_cuda(torch.zeros((1440, 2880, 4), dtype=torch.uint8, device=device), projector, Infer())
    assert classes.shape == (4, 16, 32) and classes[0, front_robot_mask(16, 32)].min() == 1
    assert np.all(classes[:, 0, 0] == 0) and classes[1, 8, 8] == 3


def test_live_hardware_path_has_no_implicit_cpu_fallback(monkeypatch):
    monkeypatch.setattr(torch.cuda, 'is_available', lambda: False)
    with pytest.raises(RuntimeError, match='no software fallback'):
        HardwareCamera.check_runtime()
    with pytest.raises(RuntimeError, match='no CPU fallback'):
        SurroundTensorProjector('flyvis')
