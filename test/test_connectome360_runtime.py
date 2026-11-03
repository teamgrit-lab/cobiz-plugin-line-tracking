"""Exercise the 360 live transport and task boundary without robot hardware."""
from contextlib import contextmanager
from dataclasses import asdict
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from connectome360_client import Connectome360Segmenter
from connectome360_runtime import PROTOCOL, VIEWS_TOPIC, package_config, sha256, unpack_views, views_frame_id
from connectome360_service import Connectome360Service
from connectome_server import encode_image
from front_connectome_service import make_server
from surround_camera import load_profile, profile_sha256
from swin_l_drive_control import DriveDecision


PIN = 'a'*64
ARGS = ('--connectome360-url', 'http://127.0.0.1:8093/v1/360-road',
        '--connectome360-sha256', PIN, '--path-mask-class', '1', '--path-planner', 'local')


def details():
    return dict(backend='flyvis360-road-policy', weights_sha256=PIN,
                road_sha256='b'*64, policy_sha256='c'*64, encoder_sha256='d'*64,
                camera_views=['front', 'left', 'rear', 'right'],
                camera_profile_sha256=profile_sha256(load_profile()),
                uses_ground_truth_at_runtime=False, pose_input=False, goal_input=False,
                trained_max_speed_mps=10., head_device='cpu')


@contextmanager
def http_model():
    received = []
    def infer(atlas):
        received.append(unpack_views(atlas))
        return np.ones((270, 480), np.uint8), DriveDecision(10., 0., .35, 'neural_drive'), {}
    model = NS(details=details(), reset=lambda: None, infer=infer)
    server = make_server(Connectome360Service(model), '127.0.0.1', 0, endpoint='/v1/360-road')
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f'http://127.0.0.1:{server.server_port}/v1/360-road', model, received
    finally:
        server.shutdown()
        server.server_close()
        thread.join(3)


def test_four_view_pixels_round_trip_and_physical_cap_preserves_curvature():
    atlas = np.zeros((540, 960, 3), np.uint8)
    for i, (y, x) in enumerate(((0, 0), (0, 480), (270, 0), (270, 480))):
        atlas[y:y+270, x:x+480] = [10+i, 20+i, 30+i]
    with http_model() as (url, model, received):
        client = Connectome360Segmenter(url, PIN)
        result = client.segment(atlas[..., ::-1].copy(), color_order='rgb', source_stamp=1.)
        assert [v[0, 0].tolist() for v in received[0]] == [[10+i, 20+i, 30+i] for i in range(4)]
        assert result.front_command['vx'] == .8
        assert result.front_command['yaw_rate'] == pytest.approx(.028)
        assert result.front_command['reason'] == 'tracking'
        assert result.selected_mask.shape == (270, 480)
        model.details['policy_sha256'] = 'e'*64
        with pytest.raises(ValueError, match='paired weights changed'):
            client.segment(atlas, source_stamp=2.)
        assert client.last_command is None


def test_360_protocol_rejects_masks_wrong_shape_modes_and_old_timestamps():
    service = Connectome360Service(NS(details=details(), reset=lambda: None,
        infer=lambda atlas: (unpack_views(atlas)[0][..., 0], DriveDecision(0., 0., 0., 'neural_wait'), {})))
    req = dict(schema=PROTOCOL, stream_id='x', request_id='x:1', stamp_sec=1., mode=1,
               image_png=encode_image(np.zeros((540, 960, 3), np.uint8)))
    with pytest.raises(ValueError): service.process(dict(req, masks=[]))
    with pytest.raises(ValueError): service.process(dict(req, mode=2))
    with pytest.raises(ValueError): service.process(dict(req, stamp_sec=float('nan')))
    with pytest.raises(ValueError):
        service.process(dict(req, image_png=encode_image(np.zeros((270, 480, 3), np.uint8))))
    service.process(req)
    with pytest.raises(ValueError, match='timestamp must increase'): service.process(req)


def test_model_manifest_binds_all_weights_and_the_projection(tmp_path):
    for name in ['flyvis_encoder.pt', 'road/road-readout.pt', 'policy/drive-readout.pt']:
        p = tmp_path/name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(name.encode())
    config = dict(schema='connectome360.camera_only.pair.v1', simulation_max_speed_mps=10.,
        input_contract=dict(panorama_shape_wh=[2880, 1440], camera_views=['front', 'left', 'rear', 'right'],
            view_shape_wh=[480, 270], forward_heading_deg=90, actor_input_chw=[4, 32, 64], actor_crop_rows=[135, 257]),
        camera_profile_sha256=profile_sha256(load_profile()),
        encoder_sha256=sha256(tmp_path/'flyvis_encoder.pt'), road_sha256=sha256(tmp_path/'road/road-readout.pt'),
        policy_sha256=sha256(tmp_path/'policy/drive-readout.pt'))
    (tmp_path/'model.json').write_text(json.dumps(config))
    pin = sha256(tmp_path/'model.json')
    assert package_config(tmp_path, pin) == config
    (tmp_path/'policy/drive-readout.pt').write_bytes(b'wrong policy')
    with pytest.raises(ValueError, match='weight SHA-256'): package_config(tmp_path, pin)
    with pytest.raises(ValueError, match='model.json SHA-256'): package_config(tmp_path, '0'*64)


@pytest.mark.parametrize('command', [
    dict(vx=float('nan'), vy=0., yaw_rate=0., reason='neural_drive'),
    dict(vx=10.1, vy=0., yaw_rate=0., reason='neural_drive'),
    dict(vx=.1, vy=.1, yaw_rate=0., reason='neural_drive'),
    dict(vx=.1, vy=0., yaw_rate=.4, reason='neural_drive'),
    dict(vx=.1, vy=0., yaw_rate=0., reason='neural_wait'),
])
def test_invalid_command_never_gets_clamped_into_a_valid_move(command):
    client = Connectome360Segmenter.__new__(Connectome360Segmenter)
    client.details = details()
    with pytest.raises(ValueError): client.decode_command(command)
    assert client.decode_command(dict(vx=0., vy=0., yaw_rate=0., reason='neural_wait')) == DriveDecision.stop('neural_wait')


def setup_ros(monkeypatch, road_infer=None):
    from test_apriltag_task_stop_ros import RosHarness, debug
    import connectome360_client
    ros = RosHarness(monkeypatch)
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_STOP_ON_' + check.upper(), 'false')
    monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_MAX_FORWARD_MPS', '.5')
    # The shared harness has no dynamic-subscription implementation; callbacks
    # still run the real generation, task, timestamp and frame-id guards.
    monkeypatch.setattr(sys.modules['rclpy.node'].Node, 'destroy_subscription', lambda *_: True, raising=False)
    calls = []
    def legacy(frame, **kwargs):
        assert kwargs['path_mask_class'] == 2
        calls.append('sidewalk')
        return NS(selected_mask=np.full((360, 640), 2, np.uint8), inference_seconds=.01)
    def road(frame, **kwargs):
        assert frame.shape == (540, 960, 3)
        assert kwargs['path_mask_class'] == 1 and kwargs['source_stamp'] > 0
        calls.append('road')
        if road_infer:
            return road_infer(frame, **kwargs)
        return NS(selected_mask=np.ones((270, 480), np.uint8), inference_seconds=.01,
                  front_command=asdict(DriveDecision(.8, 0., .08, 'tracking')), front_metrics={})
    monkeypatch.setattr(debug, 'BestSoFarSegmenter', lambda _: NS(
        device=NS(type='cuda'), reset=lambda: None, segment=legacy))
    monkeypatch.setattr(connectome360_client, 'Connectome360Segmenter', lambda *_: NS(
        device=NS(type='cuda'), reset=lambda: None, segment=road))
    return ros, calls


def atlas_image(ros, *, frame_id=None, stamp=None):
    from test_apriltag_task_stop_ros import Message
    msg = Message(bytes(540*960*3))
    msg.height, msg.width, msg.step, msg.encoding = 540, 960, 960*3, 'bgr8'
    msg.header.frame_id = frame_id or views_frame_id()
    msg.header.stamp = stamp or ros.stamp()
    ros.subscriptions[VIEWS_TOPIC](msg)


def test_road_uses_only_360_camera_then_sidewalk_uses_legacy_and_stale_input_stops(monkeypatch):
    from test_apriltag_task_stop_ros import SPORT, ZERO, debug
    from test_surface_model_routing import start, cancel
    from test_task_inference_lifecycle_ros import image, wait_for_count
    ros, calls = setup_ros(monkeypatch)
    def scenario(node):
        node.bridge.imgmsg_to_cv2 = lambda msg, **_: np.zeros((msg.height, msg.width, 3), np.uint8)
        start(ros, 1, 'road')
        ros.now += 2.1
        image(ros)  # Front camera must not feed this Road model.
        assert calls == []
        atlas_image(ros, frame_id='wrong-camera-profile')
        assert calls == []
        atlas_image(ros)
        wait_for_count(ros, 1)
        assert json.loads(ros.published[SPORT][-1].parameter) == {'x': .5, 'y': 0., 'z': .05}
        assert ros.metrics()['inference_backend'] == 'connectome360'
        assert ros.metrics()['camera_topic'] == VIEWS_TOPIC
        assert ros.metrics()['front_connectome'] is None
        ros.now += 1.1
        node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert ros.metrics()['drive_reason'] == 'connectome360_source_stale'
        cancel(ros, 'road')
        start(ros, 2, 'sidewalk')
        ros.now += 2.1
        atlas_image(ros)  # Old callback cannot feed the new Sidewalk task.
        image(ros)
        wait_for_count(ros, 2)
        assert calls == ['road', 'sidewalk']
        assert ros.metrics()['inference_backend'] == 'legacy'
        assert ros.metrics()['connectome360'] is None
        assert json.loads(ros.published[SPORT][-1].parameter)['x'] > 0
    ros.run(scenario, *ARGS)


@pytest.mark.parametrize('failure', [False, True])
def test_cancelled_360_reply_or_failure_cannot_drive_or_abort_the_next_sidewalk(monkeypatch, failure):
    from test_apriltag_task_stop_ros import SPORT, ZERO
    from test_surface_model_routing import start, cancel
    from test_task_inference_lifecycle_ros import image, wait_for_count
    entered, release = threading.Event(), threading.Event()
    def delayed(*_, **__):
        entered.set()
        assert release.wait(3)
        if failure:
            raise OSError('old Road request failed')
        return NS(selected_mask=np.ones((270, 480), np.uint8), inference_seconds=.01,
                  front_command=asdict(DriveDecision(.8, 0., .08, 'tracking')), front_metrics={})
    ros, calls = setup_ros(monkeypatch, delayed)
    def scenario(node):
        node.bridge.imgmsg_to_cv2 = lambda msg, **_: np.zeros((msg.height, msg.width, 3), np.uint8)
        try:
            start(ros, 1, 'old-road')
            ros.now += 2.1
            atlas_image(ros)
            assert entered.wait(3)
            cancel(ros, 'old-road')
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
            start(ros, 2, 'next-sidewalk')
            ros.now += 2.1
            image(ros)
            release.set()
            wait_for_count(ros, 1)
            assert node.tasks.active.key == 'next-sidewalk'
            assert ros.metrics()['inference_backend'] == 'legacy'
            assert json.loads(ros.published[SPORT][-1].parameter)['x'] > 0
        finally:
            release.set()
    ros.run(scenario, *ARGS)


def test_current_road_failure_aborts_then_sidewalk_still_runs(monkeypatch):
    from test_apriltag_task_stop_ros import SPORT, ZERO
    from test_surface_model_routing import start
    from test_task_inference_lifecycle_ros import image, wait_for_count
    def broken(*_, **__):
        raise TimeoutError('360 model service timeout')
    ros, calls = setup_ros(monkeypatch, broken)
    def scenario(node):
        node.bridge.imgmsg_to_cv2 = lambda msg, **_: np.zeros((msg.height, msg.width, 3), np.uint8)
        start(ros, 1, 'failed-road')
        ros.now += 2.1
        atlas_image(ros)
        deadline = time.perf_counter()+3
        while node.tasks.active is not None:
            assert time.perf_counter() < deadline
            time.sleep(.001)
        assert ros.task_state()['reason'] == 'connectome360_unavailable'
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        ros.now += 1.1
        node.publish_state()
        start(ros, 2, 'sidewalk-after-failure')
        ros.now += 2.1
        image(ros)
        wait_for_count(ros, 1)
        assert calls == ['road', 'sidewalk']
        assert json.loads(ros.published[SPORT][-1].parameter)['x'] > 0
    ros.run(scenario, *ARGS)


def test_float_roundoff_does_not_reject_a_command_on_the_transport_limit():
    client = Connectome360Segmenter.__new__(Connectome360Segmenter)
    client.details = details()
    for vx in np.random.default_rng(42).uniform(.8, 10, 1000):
        c = client.decode_command(dict(vx=float(vx), vy=0., yaw_rate=.35, reason='neural_drive'))
        assert 0 <= c.vx <= .8 and 0 <= c.yaw_rate <= .35
        assert c.yaw_rate/c.vx == pytest.approx(.35/vx)


def test_rejects_multiple_road_backends(monkeypatch):
    from test_apriltag_task_stop_ros import debug
    monkeypatch.setattr(debug, 'ENV', {})
    for extra in [('--front-connectome-url', 'http://127.0.0.1:8092/v1/front-road'),
                  ('--fcn-sha256', 'a'*64), ('--path-mask-class', '0')]:
        with pytest.raises(SystemExit): debug.parse_args(['task-drive', *ARGS, *extra])
