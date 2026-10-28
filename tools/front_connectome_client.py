"""Opt-in Road-only adapter for the existing LINE_TRACKING camera worker."""
from dataclasses import asdict
import json
import math
import time
from types import SimpleNamespace
import urllib.request
import uuid

import cv2
import numpy as np
import torch

from connectome_server import MAX_BODY, decode_image, encode_image
from front_connectome_service import PROTOCOL
from swin_l_drive_control import DriveDecision


def validated_command(value):
    if not isinstance(value, dict) or set(value) != {'vx', 'vy', 'yaw_rate', 'reason'}:
        raise ValueError('invalid learned command shape')
    for key in ['vx', 'vy', 'yaw_rate']:
        if type(value[key]) not in (int, float) or not math.isfinite(value[key]):
            raise ValueError('nonfinite learned command')
    if not 0 <= value['vx'] <= .8 or value['vy'] != 0 or abs(value['yaw_rate']) > .35:
        raise ValueError('learned command outside model contract')
    if value['reason'] not in ['tracking', 'neural_wait']:
        raise ValueError('unknown learned decision')
    if value['reason'] == 'neural_wait' and (value['vx'] or value['yaw_rate']):
        raise ValueError('learned stop must be zero')
    return DriveDecision(**value)


def limit_front_command(command, max_speed, max_yaw, *, max_lateral=0.):
    factor = min(1., max_speed / command.vx if command.vx else 1.,
                 max_yaw / abs(command.yaw_rate) if command.yaw_rate else 1.)
    factor = min(factor, max_lateral / abs(command.vy) if command.vy else 1.)
    return DriveDecision(command.vx * factor, command.vy * factor, command.yaw_rate * factor, command.reason)


def front_drive_decision(command, mode, ages, max_age, camera_fault=None):
    if mode != 1:
        return DriveDecision.stop('front_connectome_requires_road')
    if camera_fault:
        return DriveDecision.stop(camera_fault)
    if any(age is None or not math.isfinite(age) or not 0 <= age <= max_age for age in ages):
        return DriveDecision.stop('front_connectome_source_stale')
    if command is None:
        return DriveDecision.stop('front_connectome_unavailable')
    return validated_command(command)


class FrontConnectomeSegmenter:
    def __init__(self, url, expected_sha256, timeout=1.):
        if not url.startswith(('http://127.0.0.1:', 'http://localhost:')):
            raise ValueError('front model endpoint must be loopback')
        if len(expected_sha256) != 64 or any(c not in '0123456789abcdef' for c in expected_sha256):
            raise ValueError('pin the trained front checkpoint SHA-256')
        self.url = url; self.expected_sha256 = expected_sha256; self.timeout = timeout
        health_url = url.rsplit('/', 2)[0] + '/health'
        with urllib.request.urlopen(health_url, timeout=timeout) as response:
            health = json.loads(response.read(MAX_BODY))
        if not health.get('ready') or health.get('schema') != PROTOCOL:
            raise ValueError('front model service not ready')
        self.details = health['model']; self.validate_model(self.details)
        self.device = torch.device(self.details['head_device'])
        self.reset()

    def validate_model(self, model):
        if (model.get('backend') != 'flyvis-front-road' or model.get('camera_views') != ['front']
                or model.get('weights_sha256') != self.expected_sha256
                or model.get('uses_ground_truth_at_runtime') is not False
                or model.get('pose_input') is not False or model.get('goal_input') is not False):
            raise ValueError('front model identity/input contract mismatch')

    def reset(self):
        self.stream = uuid.uuid4().hex; self.sequence = 0; self.last_command = None

    def segment(self, frame, *, color_order='bgr', path_mask_class=1,
                detect_road_hazards=False, source_stamp=None):
        if color_order not in ['bgr', 'rgb'] or path_mask_class != 1:
            raise ValueError('front model supports explicit Road mode 1 only')
        self.last_command = None; self.sequence += 1; start = time.monotonic()
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if color_order == 'rgb' else frame
        # Bound transport dimensions; the same 480x270 resize runs in training.
        bgr = cv2.resize(bgr, (480, 270), interpolation=cv2.INTER_AREA)
        stamp = float(source_stamp if source_stamp is not None else start)
        identity = f'{self.stream}:{self.sequence}'
        request = {'schema': PROTOCOL, 'stream_id': self.stream, 'request_id': identity,
                   'stamp_sec': stamp, 'mode': 1, 'image_png': encode_image(bgr)}
        req = urllib.request.Request(self.url, data=json.dumps(request).encode(), headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=self.timeout) as response:
            payload = response.read(MAX_BODY + 1)
        if len(payload) > MAX_BODY:
            raise ValueError('oversized front model response')
        reply = json.loads(payload)
        if any(reply.get(k) != request[k] for k in ['schema', 'stream_id', 'request_id', 'stamp_sec', 'mode']):
            raise ValueError('front reply does not match the camera frame')
        self.validate_model(reply['model'])
        command = validated_command(reply['command'])
        mask = decode_image(reply['selected_png'], cv2.IMREAD_GRAYSCALE)
        if mask.shape != (270, 480) or not np.isin(mask, [0, 1]).all():
            raise ValueError('invalid front road mask')
        self.last_command = asdict(command)
        elapsed = time.monotonic() - start
        return SimpleNamespace(selected_mask=mask, road_hazard_mask=np.zeros_like(mask),
            road_building_mask=np.zeros_like(mask), inference_seconds=reply['total_seconds'],
            postprocess_seconds=0., total_seconds=elapsed, hysteresis_hold_ratio=0.,
            road_area_ratio=float(mask.mean()), sidewalk_area_ratio=0.,
            front_command=self.last_command, front_metrics=reply['metrics'])

    def metadata(self):
        return {**self.details, 'service_url': self.url, 'remote_model': True}

    def render_overlay(self, frame_bgr, selected_mask, **_kwargs):
        output = frame_bgr.copy()
        road = cv2.resize(selected_mask, (output.shape[1], output.shape[0]), interpolation=cv2.INTER_NEAREST) == 1
        output[road] = (output[road] * .55 + np.array([140, 210, 70]) * .45).astype(np.uint8)
        return output
