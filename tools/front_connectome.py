"""Single front-camera Flyvis road and driving readouts.

The pretrained biological circuit is frozen. Learned heads receive only its
activity / predicted road pixels. No pose, map, goals or LiDAR enter inference.
"""
from pathlib import Path
import hashlib
import json
import time

import cv2
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F

from connectome_flow import FlyvisFlow
from connectome_road import CircuitFeatures, RoadReadout
from swin_l_drive_control import DriveDecision

SCHEMA = 'connectum.front.road.v1'


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def front_pixels(masks):
    masks = np.asarray(masks)
    if masks.ndim == 2:
        masks = masks[None]
    if masks.ndim != 3 or masks.shape[1:] != (270, 480):
        raise ValueError('front masks must be [N,270,480]')
    return np.stack([cv2.resize(m[135:257].astype(np.float32), (96, 40),
                               interpolation=cv2.INTER_AREA) for m in masks])[:, None]


class FrontDriveReadout(nn.Module):
    def __init__(self):
        super().__init__()
        self.vision = nn.Sequential(nn.Conv2d(1, 16, 5, stride=2, padding=2), nn.SiLU(),
            nn.Conv2d(16, 32, 3, stride=2, padding=1), nn.SiLU(),
            nn.Conv2d(32, 32, 3, stride=2, padding=1), nn.SiLU(), nn.Flatten(),
            nn.Linear(32 * 5 * 12, 128), nn.SiLU())
        self.action = nn.Sequential(nn.Linear(128, 64), nn.SiLU(), nn.Linear(64, 2))
        self.stop = nn.Sequential(nn.Linear(128, 32), nn.SiLU(), nn.Linear(32, 2))

    def forward(self, pixels):
        visual = self.vision(pixels)
        raw = self.action(visual)
        return torch.stack([raw[:, 0].sigmoid(), raw[:, 1].tanh()], 1), self.stop(visual)


class FrontConnectome:
    def __init__(self, directory, *, encoder_device='cpu', head_device='cpu'):
        self.directory = Path(directory)
        self.config = json.loads((self.directory / 'model.json').read_text())
        if self.config['schema'] != SCHEMA or self.config['camera_views'] != ['front']:
            raise ValueError('single front-camera model required')
        for name, digest in self.config['weights_sha256'].items():
            if Path(name).name != name or sha256(self.directory / name) != digest:
                raise ValueError('model hash mismatch: ' + name)
        self.encoder = FlyvisFlow(self.directory / 'flyvis_encoder.pt',
            self.config['weights_sha256']['flyvis_encoder.pt'], encoder_device)
        self.features = CircuitFeatures(self.encoder)
        self.device = torch.device(head_device)
        self.road = RoadReadout().to(self.device).eval().requires_grad_(False)
        self.actor = FrontDriveReadout().to(self.device).eval().requires_grad_(False)
        checkpoint = torch.load(self.directory / 'front_model.pt', map_location='cpu', weights_only=True)
        if checkpoint['schema'] != SCHEMA:
            raise ValueError('front checkpoint schema mismatch')
        if checkpoint['encoder_sha256'] != self.config['weights_sha256']['flyvis_encoder.pt']:
            raise ValueError('checkpoint encoder mismatch')
        self.road.load_state_dict(checkpoint['road_state_dict'], strict=True)
        self.actor.load_state_dict(checkpoint['actor_state_dict'], strict=True)
        if not all(torch.isfinite(p).all() for model in (self.road, self.actor) for p in model.state_dict().values()):
            raise ValueError('nonfinite front model parameters')
        if not torch.all(self.road.std > 0):
            raise ValueError('invalid front feature normalization')
        self.details = {'backend': 'flyvis-front-road', 'schema': SCHEMA, 'camera_views': ['front'],
            'model_id': self.directory.name,
            'views': 1, 'encoder_sha256': self.config['weights_sha256']['flyvis_encoder.pt'],
            'weights_sha256': self.config['weights_sha256']['front_model.pt'],
            'encoder_frozen': True, 'uses_ground_truth_at_runtime': False,
            'map_assistance': False, 'goal_input': False, 'pose_input': False,
            'runtime_planner': False, 'runtime_pure_pursuit': False, 'fault_flag_input': False,
            'encoder_device': encoder_device, 'head_device': head_device,
            'trained_parameters': 'front road readout + front drive/stop readout'}

    def reset(self):
        self.encoder.reset()

    def perceive(self, frame_bgr):
        if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3 or frame_bgr.dtype != np.uint8:
            raise ValueError('uint8 BGR front image required')
        start = time.perf_counter()
        frame = cv2.resize(frame_bgr, (480, 270), interpolation=cv2.INTER_AREA)
        activity = self.features([frame])
        circuit_ms = (time.perf_counter() - start) * 1000
        with torch.inference_mode():
            logits = self.road(torch.as_tensor(activity, device=self.device))
            probability = F.interpolate(logits, size=(270, 480), mode='bilinear',
                                        align_corners=False).sigmoid()[0, 0].cpu().numpy()
        mask = (probability >= self.config['road_threshold']).astype(np.uint8)
        return mask, {'ms': (time.perf_counter() - start) * 1000, 'circuit_ms': circuit_ms,
                      'views': 1, 'road_fraction': float(mask.mean())}

    def act(self, mask):
        start = time.perf_counter()
        with torch.inference_mode():
            action, logits = self.actor(torch.as_tensor(front_pixels(mask), device=self.device))
            action = action[0].cpu().numpy()
            probability = logits.softmax(1)[0].cpu().numpy()
        stop = bool(probability[1] >= self.config.get('stop_threshold', .5))
        # Convert before scaling: float32(0.8) slightly exceeds the strict
        # transport bound when the sigmoid saturates exactly at one.
        command = DriveDecision(0. if stop else float(action[0]) * self.config['max_speed_mps'],
            0., 0. if stop else float(action[1]) * self.config['max_yaw_rate_radps'],
            'neural_wait' if stop else 'tracking')
        return command, {'ms': (time.perf_counter() - start) * 1000,
            'stop_probability': float(probability[1]), 'raw_action': action.tolist(), 'override': False}

    def infer(self, frame_bgr):
        mask, perception = self.perceive(frame_bgr)
        command, policy = self.act(mask)
        return mask, command, {'perception': perception, 'policy': policy}


class FrontSimulationPerception:
    def __init__(self, model):
        self.model = model; self.details = dict(model.details)

    def infer(self, frame):
        mask, metrics = self.model.perceive(frame)
        return mask, np.zeros_like(mask), metrics


class FrontSimulationPolicy:
    def __init__(self, model):
        self.model = model
        self.details = dict(model.details, backend='flyvis-front-drive',
            emergency_stop_override=False, simulation_only=True)

    def infer(self, masks):
        if np.shape(masks) != (1, 270, 480):
            raise ValueError('exactly one front mask is required')
        return self.model.act(masks[0])
