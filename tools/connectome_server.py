"""Single-camera, serialized Flyvis + R50-mask planner HTTP service. No ROS/control."""
import argparse
import base64
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import math
import os
import time

import cv2
import numpy as np

from connectome_surface import LocalPathConfig, PlannerConfig, SurfacePlanner

SCHEMA = 'line_tracking.connectome.v1'
MAX_BODY = 4*1024*1024


def encode_image(image):
    ok, data = cv2.imencode('.png', image)
    if not ok:
        raise ValueError('image encoding failed')
    return base64.b64encode(data).decode('ascii')


def decode_image(value, color):
    raw = base64.b64decode(value, validate=True)
    # Reject oversized PNG dimensions before OpenCV allocates the image.
    if (raw[:8] != b'\x89PNG\r\n\x1a\n' or len(raw) < 24
            or not 1 <= int.from_bytes(raw[16:20], 'big') <= 1280
            or not 1 <= int.from_bytes(raw[20:24], 'big') <= 720):
        raise ValueError('bounded PNG camera/mask required')
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), color)
    if image is None:
        raise ValueError('invalid PNG')
    return image


class PlannerService:
    def __init__(self, encoder):
        self.encoder, self.stream, self.last_stamp, self.signature = encoder, None, None, None
        self.planner = SurfacePlanner()

    def process(self, request):
        if request.get('schema') != SCHEMA:
            raise ValueError('unsupported planner schema')
        for key in ('stream_id', 'request_id'):
            if not isinstance(request.get(key), str) or not 1 <= len(request[key]) <= 160:
                raise ValueError('invalid planner identity')
        stamp = request['stamp_sec']
        if isinstance(stamp, bool) or not isinstance(stamp, (float, int)) or not math.isfinite(stamp):
            raise ValueError('finite source timestamp required')
        local_values = dict(request['local_config'])
        local_values['roi_polygon'] = tuple(local_values['roi_polygon'])
        local = LocalPathConfig(**local_values)
        local.validate()
        if (local.bev_width_px > 512 or local.bev_height_px > 256
                or not .2 <= local.far_distance_m-local.near_distance_m <= 15
                or not .1 <= local.search_half_width_m <= 10 or local.far_distance_m > 20):
            raise ValueError('planner geometry exceeds bounded grid')
        config = PlannerConfig(**request['planner_config']); config.validate()
        signature = json.dumps([local_values, request['planner_config']], sort_keys=True)
        frame = decode_image(request['image_png'], cv2.IMREAD_COLOR)
        selected = decode_image(request['selected_png'], cv2.IMREAD_GRAYSCALE)
        hazard = decode_image(request['hazard_png'], cv2.IMREAD_GRAYSCALE)
        if frame.shape[:2] != selected.shape or hazard.shape != selected.shape or not np.isin(selected, [0, 1, 2]).all():
            raise ValueError('aligned original image and road/sidewalk labels required')
        if request['stream_id'] != self.stream:
            self.encoder.reset(); self.planner = SurfacePlanner()
            self.stream, self.last_stamp, self.signature = request['stream_id'], None, signature
        if signature != self.signature:
            raise ValueError('geometry cannot change within a camera/task stream')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            raise ValueError('duplicate/backwards camera stamp')
        started = time.perf_counter()
        try:
            flow = self.encoder.advance(frame, stamp)
            flow_finished = time.perf_counter()
            paths = self.planner.plan(selected, hazard, local, config, flow)
        except Exception:
            # A flow interruption does not authorize Mode 0 to switch surface.
            self.encoder.reset(); self.last_stamp = None
            raise
        self.last_stamp = stamp
        return {'schema': SCHEMA, 'request_id': request['request_id'], 'stream_id': self.stream,
                'stamp_sec': stamp, 'paths': paths, 'model': dict(self.encoder.details),
                'timing': {'flow_ms': (flow_finished-started)*1000,
                           'surface_ms': (time.perf_counter()-flow_finished)*1000}}


def make_server(service, address):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path != '/health':
                self.send_error(404); return
            self.reply(200, {'ready': True, 'model': service.encoder.details})

        def reply(self, status, body):
            data = json.dumps(body, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            if self.path != '/v1/plan':
                self.send_error(404); return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= MAX_BODY:
                    raise ValueError('bounded request body required')
                request = json.loads(self.rfile.read(length))
                self.reply(200, service.process(request))
            except Exception as error:
                self.reply(422, {'error': str(error)[:300]})

        def log_message(self, *_args):
            pass

        def setup(self):
            super().setup(); self.connection.settimeout(3.)

    server = HTTPServer(address, Handler)
    server.request_queue_size = 1
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8091)
    parser.add_argument('--bundle', default=os.getenv('CONNECTOME_BUNDLE', '/models/flyvis_encoder.pt'))
    parser.add_argument('--sha256', default=os.getenv('CONNECTOME_BUNDLE_SHA256', ''))
    parser.add_argument('--device', choices=['cpu', 'cuda'], default=os.getenv('CONNECTOME_DEVICE', 'cuda'))
    parser.add_argument('--cuda-graph', action=argparse.BooleanOptionalAction,
                        default=os.getenv('CONNECTOME_CUDA_GRAPH', 'false').lower() == 'true')
    args = parser.parse_args()
    from connectome_flow import FlyvisFlow
    cv2.setNumThreads(2)
    if args.cuda_graph and args.device != 'cuda':
        parser.error('CUDA replay requires --device cuda')
    encoder = FlyvisFlow(args.bundle, args.sha256, args.device, cuda_graph=args.cuda_graph)
    make_server(PlannerService(encoder), (args.host, args.port)).serve_forever()


if __name__ == '__main__':
    main()
