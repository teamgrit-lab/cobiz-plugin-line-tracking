"""Camera ownership, result-only HTTP and Road/Sidewalk task boundaries."""
from contextlib import contextmanager
import json
import threading
import time
from types import SimpleNamespace as NS
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from direct_camera_client import DirectCameraClient
from direct_camera_service import DirectCameraService, SCHEMA, make_server
from road_test_support import PIN, details


class Source:
    def __init__(self, capture):
        self.capture, self.reads, self.closes = capture, 0, 0

    def read(self):
        self.reads += 1
        return self.capture() if callable(self.capture) else self.capture

    def close(self):
        self.closes += 1


def request(now, **changes):
    return dict(dict(schema=SCHEMA, stream_id='task', request_id='task:1', mode=1,
                     not_before_ns=now-1_000_000_000, after_stamp_ns=0, max_age_sec=1.5), **changes)


@contextmanager
def running(service):
    server = make_server(service, 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f'http://127.0.0.1:{server.server_port}/v1/direct-road'
    finally:
        server.shutdown()
        service.close()
        server.server_close()
        thread.join(2.)


@pytest.mark.parametrize('backend', ['connectome360', 'front_connectome', 'fcn360'])
def test_real_http_transports_no_images_and_keeps_native_grids_and_caps(backend):
    identity = details() if backend == 'connectome360' else dict(
        backend='flyvis-front-road', weights_sha256=PIN, camera_views=['front'],
        uses_ground_truth_at_runtime=False, pose_input=False, goal_input=False)
    if backend == 'fcn360':
        identity = dict(backend='fcn1024-360', weights_sha256=PIN, input_width=1024, class_grid_shape=[4,16,32])
    source = Source(lambda: (np.full((3,4,3), 42, np.uint8), time.time_ns()))
    grids = np.full((4,16,32), 3, np.uint8)
    grids[1,4,9] = 11
    def predict(frame):
        assert frame[0,0].tolist() == [42,42,42]
        if backend == 'fcn360':
            return dict(classes=grids.tolist())
        return dict(command=dict(vx=10. if backend == 'connectome360' else .4,
                                 vy=0., yaw_rate=.2, reason='neural_drive' if backend == 'connectome360' else 'tracking'),
                    metrics={'perception': {'road_fraction': .4}})
    service = DirectCameraService(NS(details=identity, reset=lambda: None), source, predict)
    with running(service) as url:
        client = DirectCameraClient(url, backend, PIN, 2.)
        sent = []
        original = client._request
        def inspect(url, payload=None):
            sent.append(payload)
            return original(url, payload)
        client._request = inspect
        started = time.time_ns()
        result = client.infer('task-123', started, 0, 1.5, time.time_ns)
        assert set(sent[0]) == set(request(started))
        assert result.capture_stamp_ns >= started
        assert not hasattr(result, 'selected_mask')
        if backend == 'fcn360':
            assert result.masks.shape == (4,16,32)
            assert result.masks[1,4,9] == 11
        else:
            assert result.front_command['vx'] == (.8 if backend == 'connectome360' else .4)
            assert result.front_command['yaw_rate'] == pytest.approx(.016 if backend == 'connectome360' else .2)
        if backend == 'connectome360':
            identity['policy_sha256'] = 'e' * 64
            with pytest.raises(ValueError, match='paired weights changed'):
                client.infer('task-123', started, result.capture_stamp_ns, 1.5, time.time_ns)


def test_capture_freshness_is_checked_before_and_after_inference_and_on_task_switch(monkeypatch):
    clock = [5_000_000_000]
    source = Source((np.zeros((2,2,3), np.uint8), clock[0]))
    calls = []
    model = NS(details=details(), reset=lambda: calls.append('reset'))
    def predict(frame):
        calls.append('infer')
        return {'command': {}}
    service = DirectCameraService(model, source, predict, now_ns=lambda: clock[0])
    req = request(clock[0])
    assert service.process(req)['available']
    assert not service.process(req)['available']  # A duplicate never becomes a new frame.
    assert calls.count('infer') == 1
    clock[0] += 1_000_000_000
    assert not service.process(dict(req, stream_id='next-task', not_before_ns=clock[0]))['available']
    assert calls.count('infer') == 1  # Previous task's buffered capture was rejected.
    source.capture = (np.zeros((2,2,3), np.uint8), clock[0])
    def slow(_frame):
        clock[0] += 2_000_000_000
        return {'command': {}}
    service.predict = slow
    assert service.process(dict(req, stream_id='next-task', not_before_ns=6_000_000_000))['reason'] == 'inference_source_stale'
    source.capture = None
    assert service.process(dict(req, stream_id='next-task', not_before_ns=6_000_000_000))['reason'] == 'camera_pending'
    service.used_at = 100.
    idle_clock = [102.49]
    monkeypatch.setattr(time, 'monotonic', lambda: idle_clock[0])
    before = source.closes
    service.expire_idle()
    assert source.closes == before and service.stream == 'next-task'
    idle_clock[0] = 102.5
    service.expire_idle()
    assert source.closes == before+1 and service.stream is None


@pytest.mark.parametrize('change', [dict(image_png='no'), dict(mode=2), dict(max_age_sec=float('nan')),
    dict(not_before_ns=-1), dict(after_stamp_ns=1.2), dict(not_before_ns=6_000_000_000)])
def test_invalid_request_never_opens_camera(change):
    source = Source(None)
    service = DirectCameraService(NS(details={}, reset=lambda: None), source, lambda _: {}, now_ns=lambda: 5_000_000_000)
    with pytest.raises(ValueError):
        service.process(request(5_000_000_000, **change))
    assert source.reads == 0


def test_usb_capture_without_ros_keeps_original_buffer_time(monkeypatch):
    import cv2
    from surround_capture import Camera
    import surround_capture
    camera = Camera.__new__(Camera)
    encoded = cv2.imencode('.jpg', np.full((2,4,3), 120, np.uint8))[1].tobytes()
    buffer = NS(pts=100_000_000, extract_dup=lambda *a: encoded, get_size=lambda: len(encoded))
    sample = NS(get_buffer=lambda: buffer, get_segment=lambda: NS(to_running_time=lambda *a: buffer.pts))
    camera.gst = NS(SECOND=1_000_000_000, MSECOND=1_000_000, CLOCK_TIME_NONE=-1, Format=NS(TIME=0))
    camera.sink = NS(emit=lambda *a: sample)
    camera.pipeline = NS(get_clock=lambda: NS(get_time=lambda: 2_300_000_000), get_base_time=lambda: 2_000_000_000)
    monkeypatch.setattr(surround_capture.time, 'time_ns', lambda: 5_000_000_000)
    frame, stamp = camera.read()
    assert stamp == 4_800_000_000  # 200 ms buffer age, not decoding completion time.
    assert frame.shape == (2,4,3)


@pytest.mark.parametrize('change', [dict(capture_stamp_ns=1), dict(capture_stamp_ns=10**20),
    dict(stream_id='old-task'), dict(classes=[[[3.0]*32]*16]*4), dict(classes=[[[21]*32]*16]*4)])
def test_client_rejects_untrusted_stamp_identity_and_class_packets(change):
    identity = dict(backend='fcn1024-360', weights_sha256=PIN, input_width=1024, class_grid_shape=[4,16,32])
    service = DirectCameraService(NS(details=identity, reset=lambda: None), Source(None), lambda _: {})
    with running(service) as url:
        client = DirectCameraClient(url, 'fcn360', PIN, 1.)
        started = time.time_ns()-1000
        def forged(_url, req):
            return dict(dict(req, model=identity, available=True, capture_stamp_ns=started+1,
                             total_seconds=.01, classes=np.full((4,16,32),3).tolist()), **change)
        client._request = forged
        with pytest.raises(ValueError):
            client.infer('task', started, 0, 1.5, time.time_ns)


@pytest.mark.parametrize('backend', ['connectome360', 'front_connectome', 'fcn360'])
def test_direct_road_then_original_sidewalk_and_stale_stop(monkeypatch, backend):
    from test_apriltag_task_stop_ros import RosHarness, SPORT, ZERO, debug
    from road_test_support import start, cancel
    from test_task_inference_lifecycle_ros import image, wait_for_count
    from test_fcn360_road import lane
    import direct_camera_client
    monkeypatch.setattr(debug, 'ENV', {})
    ros = RosHarness(monkeypatch)
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_STOP_ON_'+check.upper(), 'false')
    side_calls, road_calls = [], []
    def sidewalk(frame, **kwargs):
        side_calls.append(kwargs['path_mask_class'])
        return NS(selected_mask=np.full((360,640),2,np.uint8), inference_seconds=.01)
    monkeypatch.setattr(debug, 'BestSoFarSegmenter', lambda _: NS(
        device=NS(type='cuda'), reset=lambda: None, segment=sidewalk))
    def infer(stream, not_before_ns, after_ns, max_age, now_ns):
        road_calls.append((stream, not_before_ns, after_ns))
        return NS(capture_stamp_ns=now_ns(), inference_seconds=.01,
            front_command=dict(vx=.8,vy=0.,yaw_rate=.08,reason='tracking'), front_metrics={},
            masks=lane().repeat(2,axis=1).repeat(2,axis=2))
    monkeypatch.setattr(direct_camera_client, 'DirectCameraClient', lambda *a, **kw: NS(infer=infer))
    if backend == 'fcn360':
        args = ('--fcn-sha256', PIN)
    else:
        name = 'connectome360' if backend == 'connectome360' else 'front-connectome'
        args = (f'--{name}-url', 'http://127.0.0.1:8093/v1/direct-road', f'--{name}-sha256', PIN)
    def scenario(node):
        start(ros, 1, 'road')
        ros.now += 2.1
        wait_for_count(ros, 1)
        assert not side_calls
        assert json.loads(ros.published[SPORT][-1].parameter)['x'] > 0
        assert ros.metrics()['camera_transport'] == 'direct'
        assert ros.metrics()['camera_topic'] is None
        assert '/line_tracking/connectome360/views' not in ros.subscriptions
        assert debug.DEFAULT_IMAGE_TOPIC not in ros.subscriptions
        assert not any('/line_tracking/fcn360/masks/' in topic for topic in ros.subscriptions)
        ros.now += 2.
        node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        cancel(ros, 'road')
        start(ros, 2, 'sidewalk')
        ros.now += 2.1
        image(ros)
        wait_for_count(ros, 2)
        assert side_calls == [2] and len(road_calls) == 1
        assert ros.metrics()['inference_backend'] == 'legacy'
        assert ros.metrics()['camera_transport'] == 'ros'
    ros.run(scenario, '--path-mask-class', '1', '--path-planner', 'local', *args)


@pytest.mark.parametrize('failure', [False, True])
def test_cancelled_direct_inference_cannot_move_or_abort_next_sidewalk(monkeypatch, failure):
    from test_apriltag_task_stop_ros import RosHarness, SPORT, ZERO, debug
    from road_test_support import start, cancel
    from test_task_inference_lifecycle_ros import image, wait_for_count
    import direct_camera_client
    monkeypatch.setattr(debug, 'ENV', {})
    ros = RosHarness(monkeypatch)
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_STOP_ON_'+check.upper(), 'false')
    entered, release = threading.Event(), threading.Event()
    def infer(*args, **kwargs):
        entered.set()
        assert release.wait(3)
        if failure:
            raise OSError('old request failed')
        return NS(capture_stamp_ns=ros.clock_ns(), inference_seconds=.01,
                  front_command=dict(vx=.8,vy=0.,yaw_rate=.1,reason='tracking'), front_metrics={})
    monkeypatch.setattr(direct_camera_client, 'DirectCameraClient', lambda *a, **kw: NS(infer=infer))
    def scenario(node):
        start(ros, 1, 'road')
        ros.now += 2.1
        assert entered.wait(3)
        cancel(ros, 'road')
        start(ros, 2, 'sidewalk')
        ros.now += 2.1
        image(ros)
        release.set()
        wait_for_count(ros, 1)
        assert node.tasks.active.task_id == 'sidewalk'
        assert ros.metrics()['inference_backend'] == 'legacy'
        assert ros.metrics()['connectome360'] is None
    ros.run(scenario, '--path-mask-class', '1', '--path-planner', 'local',
            '--connectome360-url', 'http://127.0.0.1:8093/v1/direct-road', '--connectome360-sha256', PIN)


def test_current_direct_service_failure_aborts_only_road(monkeypatch):
    from test_apriltag_task_stop_ros import RosHarness, SPORT, ZERO, debug
    from road_test_support import start
    from test_task_inference_lifecycle_ros import image, wait_for_count
    import direct_camera_client
    monkeypatch.setattr(debug, 'ENV', {})
    ros = RosHarness(monkeypatch)
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_STOP_ON_'+check.upper(), 'false')
    def unavailable(*args, **kwargs):
        raise OSError('inference service unavailable')
    monkeypatch.setattr(direct_camera_client, 'DirectCameraClient', unavailable)
    def scenario(node):
        start(ros, 1, 'road')
        ros.now += 2.1
        deadline = time.perf_counter()+3
        while node.tasks.active is not None:
            assert time.perf_counter() < deadline
            time.sleep(.001)
        assert ros.task_state()['type'] == 'TASK_ABORTED'
        assert ros.task_state()['reason'] == 'connectome360_unavailable'
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        ros.now += 1.1
        node.publish_state()
        start(ros, 2, 'sidewalk')
        ros.now += 2.1
        image(ros)
        wait_for_count(ros, 1)
        assert ros.metrics()['inference_backend'] == 'legacy'
    ros.run(scenario, '--path-mask-class', '1', '--path-planner', 'local',
            '--connectome360-url', 'http://127.0.0.1:8093/v1/direct-road', '--connectome360-sha256', PIN)


def test_sidewalk_model_pin_is_still_required_with_direct_road(monkeypatch):
    from test_apriltag_task_stop_ros import RosHarness, debug
    RosHarness(monkeypatch)
    args = debug.parse_args(['task-drive', '--path-mask-class', '1', '--path-planner', 'local',
        '--front-connectome-url', 'http://127.0.0.1:8092/v1/direct-road', '--front-connectome-sha256', PIN])
    args.model_id = 'unexpected-unpinned-model'
    with pytest.raises(ValueError, match='pinned checkpoint'):
        debug._validate_task_drive_preflight(args)
