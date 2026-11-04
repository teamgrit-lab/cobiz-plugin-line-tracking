"""Camera-owned inference: only requests, commands and small grids cross HTTP.

USB sources use the host system clock, matching live ROS system time. The
front-camera source keeps the A2 camera's original ROS stamp. No motion publisher.
"""
from dataclasses import asdict
import json
import math
import threading
import time

import numpy as np

SCHEMA = 'line_tracking.direct_camera.v1'
MAX_BODY = 128 * 1024


class UsbSource:
    def __init__(self, device):
        self.device = device
        self.camera = None

    def read(self):
        from surround_capture import Camera
        if self.camera is None:
            self.camera = Camera(self.device)
        return self.camera.read()

    def close(self):
        if self.camera is not None:
            self.camera.close()
            self.camera = None


class RosSource:
    """Subscribe at the inference process, retaining only the latest raw message."""
    def __init__(self, topic):
        self.topic = topic
        self.node = self.subscription = self.thread = None
        self.message = None
        self.lock = threading.Lock()

    def _open(self):
        import rclpy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        if not rclpy.ok():
            rclpy.init()
        self.bridge = CvBridge()
        self.node = rclpy.create_node('front_connectome_camera_input')
        if self.node.get_parameter('use_sim_time').value:
            raise RuntimeError('direct camera requires live system time')
        def receive(message):
            with self.lock:
                self.message = message
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.node.create_subscription(Image, self.topic, receive, qos)
        self.running = True
        def spin():
            while self.running and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=.1)
        self.thread = threading.Thread(target=spin, name='front-camera-input', daemon=True)
        self.thread.start()

    def read(self):
        if self.node is None:
            self._open()
        with self.lock:
            message, self.message = self.message, None
        if message is None:
            return None
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        # Reject old or future input before performing the image conversion.
        if not 0 <= time.time_ns() - stamp <= 2_000_000_000:
            return None
        return self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8'), stamp

    def close(self):
        if self.node is not None:
            self.running = False
            self.thread.join(2.)
            self.node.destroy_node()
            self.node = self.subscription = self.thread = None
        with self.lock:
            self.message = None


class DirectCameraService:
    def __init__(self, model, source, predict, *, idle_sec=1.5, now_ns=time.time_ns):
        self.model, self.source, self.predict = model, source, predict
        self.now_ns, self.idle_sec = now_ns, idle_sec
        self.lock = threading.RLock()
        self.stream = None
        self.not_before_ns = None
        self.last_stamp = 0
        self.used_at = None
        self.closed = threading.Event()

    def process(self, request):
        fields = {'schema', 'stream_id', 'request_id', 'mode', 'not_before_ns',
                  'after_stamp_ns', 'max_age_sec'}
        if (set(request) != fields or request['schema'] != SCHEMA
                or type(request['mode']) is not int or request['mode'] != 1):
            raise ValueError('explicit Road request required; images, masks and goals are not accepted')
        for key in ('stream_id', 'request_id'):
            if not isinstance(request[key], str) or not 1 <= len(request[key]) <= 160:
                raise ValueError('invalid direct-camera request identity')
        for key in ('not_before_ns', 'after_stamp_ns'):
            if type(request[key]) is not int or request[key] < 0:
                raise ValueError('integer system-clock timestamps required')
        age = request['max_age_sec']
        if type(age) not in (float, int) or not math.isfinite(age) or not 0 < age <= 5:
            raise ValueError('bounded camera age required')
        if not 0 <= self.now_ns() - request['not_before_ns'] <= 86_400_000_000_000:
            raise ValueError('invalid task start timestamp')
        with self.lock:
            start = time.perf_counter()
            reply = {key: request[key] for key in ('schema', 'stream_id', 'request_id', 'mode', 'not_before_ns')}
            reply.update(model=self.model.details, available=False)
            self.used_at = time.monotonic()
            try:
                if request['stream_id'] != self.stream:
                    self.source.close()  # Never reuse a queued image from the previous task.
                    self.model.reset()
                    self.stream, self.last_stamp = request['stream_id'], 0
                    self.not_before_ns = request['not_before_ns']
                elif request['not_before_ns'] != self.not_before_ns:
                    raise ValueError('task start changed within a stream')
                capture = self.source.read()
                if capture is None:
                    return dict(reply, reason='camera_pending')
                frame, stamp = capture
                if (type(stamp) is not int or stamp < request['not_before_ns']
                        or stamp <= max(self.last_stamp, request['after_stamp_ns'])
                        or not 0 <= self.now_ns() - stamp <= age * 1e9):
                    return dict(reply, reason='camera_stale_or_repeated')
                self.last_stamp = stamp
                output = self.predict(frame)
                if not 0 <= self.now_ns() - stamp <= age * 1e9:
                    return dict(reply, reason='inference_source_stale')
                return dict(reply, available=True, capture_stamp_ns=stamp,
                            total_seconds=time.perf_counter()-start, **output)
            finally:
                self.used_at = time.monotonic()

    def expire_idle(self):
        with self.lock:
            if self.used_at is not None and time.monotonic()-self.used_at >= self.idle_sec:
                self.source.close()
                self.stream, self.last_stamp, self.used_at = None, 0, None

    def close(self):
        self.closed.set()
        with self.lock:
            self.source.close()


def make_server(service, port):
    from http.server import BaseHTTPRequestHandler, HTTPServer
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(5.)

        def reply(self, code, value):
            data = json.dumps(value, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if self.path != '/health':
                self.reply(404, {'error': 'unknown endpoint'})
                return
            self.reply(200, dict(ready=True, schema=SCHEMA, model=service.model.details,
                                 camera_input='direct', camera_open_on_road_request=True))

        def do_POST(self):
            if self.path != '/v1/direct-road':
                self.reply(404, {'error': 'unknown endpoint'})
                return
            try:
                count = int(self.headers.get('Content-Length', '0'))
                if self.headers.get_content_type() != 'application/json' or not 0 < count <= MAX_BODY:
                    raise ValueError('bounded JSON required')
                self.reply(200, service.process(json.loads(self.rfile.read(count))))
            except Exception as error:
                self.reply(422, {'error': str(error)[:240]})

        def log_message(self, *_):
            pass
    server = HTTPServer(('127.0.0.1', port), Handler)
    server.request_queue_size = 1
    return server


def serve(service, port):
    server = make_server(service, port)
    def reap():
        while not service.closed.wait(.2):
            service.expire_idle()
    thread = threading.Thread(target=reap, name='camera-idle-release', daemon=True)
    thread.start()
    try:
        server.serve_forever()
    finally:
        service.close()
        server.server_close()
        thread.join(2.)


def learned_output(model, frame):
    _, command, metrics = model.infer(frame)
    return dict(command=asdict(command), metrics=metrics)
