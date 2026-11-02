#!/usr/bin/env python3
"""Front-image-only model service. Never publishes robot commands."""
import argparse
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import math
import time

import cv2
import numpy as np

from connectome_server import MAX_BODY, decode_image, encode_image

PROTOCOL = 'line_tracking.front_connectome.v1'


class FrontService:
    def __init__(self, model):
        self.model = model; self.stream = None; self.stamp = None

    def process(self, request):
        allowed = {'schema', 'stream_id', 'request_id', 'stamp_sec', 'mode', 'image_png'}
        if set(request) != allowed or request['schema'] != PROTOCOL or type(request['mode']) is not int or request['mode'] != 1:
            raise ValueError('front camera image and explicit Road mode 1 required; no masks, map or goals')
        for key in ['stream_id', 'request_id']:
            if not isinstance(request[key], str) or not 1 <= len(request[key]) <= 160:
                raise ValueError('invalid request identity')
        stamp = request['stamp_sec']
        if type(stamp) not in (int, float) or not math.isfinite(stamp):
            raise ValueError('finite camera timestamp required')
        if request['stream_id'] != self.stream:
            self.model.reset(); self.stream = request['stream_id']; self.stamp = None
        if self.stamp is not None and stamp <= self.stamp:
            raise ValueError('camera timestamp must increase')
        frame = decode_image(request['image_png'], cv2.IMREAD_COLOR)
        start = time.perf_counter()
        mask, command, metrics = self.model.infer(frame)
        self.stamp = stamp
        return {'schema': PROTOCOL, 'stream_id': self.stream, 'request_id': request['request_id'],
            'stamp_sec': stamp, 'mode': 1, 'model': self.model.details,
            'selected_png': encode_image(mask), 'command': asdict(command),
            'metrics': metrics, 'total_seconds': time.perf_counter() - start}


def make_server(service, host, port):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, data):
            body = json.dumps(data, allow_nan=False).encode()
            self.send_response(status); self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body))); self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if self.path == '/health':
                self.reply(200, {'ready': True, 'schema': PROTOCOL, 'model': service.model.details})
            else:
                self.reply(404, {'error': 'unknown endpoint'})

        def do_POST(self):
            if self.path != '/v1/front-road':
                self.reply(404, {'error': 'unknown endpoint'}); return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if self.headers.get_content_type() != 'application/json' or not 0 < size <= MAX_BODY:
                    raise ValueError('bounded JSON required')
                self.reply(200, service.process(json.loads(self.rfile.read(size))))
            except Exception as error:
                service.model.reset(); self.reply(422, {'error': str(error)[:240]})

        def setup(self):
            super().setup(); self.connection.settimeout(5.)

        def log_message(self, *_args):
            pass
    server = HTTPServer((host, port), Handler); server.request_queue_size = 1
    return server


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', required=True); p.add_argument('--port', type=int, default=8092)
    p.add_argument('--device', choices=['cpu', 'cuda'], default='cuda')
    p.add_argument('--head-device', choices=['cpu', 'mps', 'cuda'])
    args = p.parse_args()
    from front_connectome import FrontConnectome
    model = FrontConnectome(args.model, encoder_device=args.device, head_device=args.head_device or args.device)
    # Compile/allocate before advertising readiness, not on the first live frame.
    model.infer(np.zeros((270, 480, 3), np.uint8)); model.reset()
    print(json.dumps({'ready': True, 'model': model.details}), flush=True)
    make_server(FrontService(model), '127.0.0.1', args.port).serve_forever()
