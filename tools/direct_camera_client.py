"""Pull camera-owned Road inference without sending camera pixels over HTTP."""
import json
import math
from types import SimpleNamespace
import urllib.request

import numpy as np

from direct_camera_service import MAX_BODY, SCHEMA


class DirectCameraClient:
    def __init__(self, url, backend, pin, timeout, input_width=1024, *, detector_sha256=''):
        if not url.startswith(('http://127.0.0.1:', 'http://localhost:')):
            raise ValueError('direct camera service must be loopback')
        if (len(pin) != 64 or any(c not in '0123456789abcdef' for c in pin)
                or not math.isfinite(timeout) or timeout <= 0 or input_width not in (512, 1024)
                or backend not in ('connectome360', 'front_connectome', 'fcn360')):
            raise ValueError('pinned supported model, resolution and positive timeout required')
        if detector_sha256 and (backend != 'connectome360' or len(detector_sha256) != 64
                                or any(c not in '0123456789abcdef' for c in detector_sha256)):
            raise ValueError('R50 detector override requires a pinned 360 learned policy')
        self.url, self.backend, self.pin = url, backend, pin
        self.timeout, self.input_width, self.sequence = timeout, input_width, 0
        self.validator = None
        if backend in ('connectome360', 'front_connectome'):
            if backend == 'connectome360':
                from connectome360_client import Connectome360Segmenter
                cls = Connectome360Segmenter
            else:
                from front_connectome_client import FrontConnectomeSegmenter
                cls = FrontConnectomeSegmenter
            self.validator = cls.__new__(cls)
            self.validator.expected_sha256 = pin
            self.validator.detector_sha256 = detector_sha256
        health = self._request(url.rsplit('/', 2)[0]+'/health')
        if health.get('schema') != SCHEMA or health.get('ready') is not True:
            raise ValueError('camera-owned inference service is not ready')
        self.validate_model(health['model'])

    def _request(self, url, payload=None):
        request = urllib.request.Request(url, data=None if payload is None else json.dumps(payload, allow_nan=False).encode(),
                                         headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            raw = response.read(MAX_BODY+1)
        if len(raw) > MAX_BODY:
            raise ValueError('oversized direct camera response')
        return json.loads(raw)

    def validate_model(self, model):
        if self.validator is not None:
            self.validator.validate_model(model)
            self.validator.details = model
        elif (model.get('backend') != f'fcn{self.input_width}-360'
                or model.get('weights_sha256') != self.pin
                or model.get('input_width') != self.input_width
                or model.get('class_grid_shape') != [4, self.input_width//64, self.input_width//32]):
            raise ValueError('FCN model identity/resolution mismatch')

    def infer(self, stream, not_before_ns, after_stamp_ns, max_age_sec, now_ns):
        self.sequence += 1
        request = dict(schema=SCHEMA, stream_id=stream, request_id=f'{stream}:{self.sequence}',
                       mode=1, not_before_ns=not_before_ns, after_stamp_ns=after_stamp_ns,
                       max_age_sec=max_age_sec)
        reply = self._request(self.url, request)
        if any(reply.get(k) != request[k] for k in ('schema', 'stream_id', 'request_id', 'mode', 'not_before_ns')):
            raise ValueError('direct response does not match the current task/request')
        self.validate_model(reply['model'])
        if type(reply.get('available')) is not bool:
            raise ValueError('invalid camera availability')
        if not reply['available']:
            return None
        stamp, seconds = reply.get('capture_stamp_ns'), reply.get('total_seconds')
        if (type(stamp) is not int or stamp < not_before_ns or stamp <= after_stamp_ns
                or not 0 <= now_ns()-stamp <= max_age_sec*1e9):
            # Discard this frame without aborting the task or refreshing the
            # last usable result. The next request can supply a current frame.
            return None
        if type(seconds) not in (float, int) or not math.isfinite(seconds) or seconds < 0:
            raise ValueError('invalid inference duration')
        result = SimpleNamespace(capture_stamp_ns=stamp, inference_seconds=seconds)
        if self.validator is not None:
            from dataclasses import asdict
            result.front_command = asdict(self.validator.decode_command(reply['command']))
            result.front_metrics = reply['metrics']
        else:
            from fcn360_road import apply_front_robot_prior
            masks = np.asarray(reply['classes'])
            if (masks.shape != (4, self.input_width//64, self.input_width//32)
                    or masks.dtype.kind not in 'iu' or masks.min() < 0 or masks.max() > 20):
                raise ValueError('invalid native FCN class grids')
            result.masks = apply_front_robot_prior(masks.astype(np.uint8))
        return result
