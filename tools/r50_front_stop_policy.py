"""Separate Road STOP learning from the retained four-view velocity readout.

Only the fixed robot box is unobserved. A zero Road score elsewhere is NOT
silently converted to unknown or filled with Road. No geometric planner runs.
"""
from collections import deque
import math
import time

import cv2
import numpy as np
import torch
from torch import nn

from connectome_policy import DriveReadout, policy_pixels
from drive_control import DriveDecision

POLICY_SCHEMA = 'r50_360.drive_readout.front_stop.v2'
PAIR_SCHEMA = 'r50_360.camera_only.front_stop.v2'
ARCHITECTURES = ('reset_shared', 'front', 'temporal_front')
OBSERVATION_SPEC = dict(reference_grid_hw=[8, 16], unknown_columns=[7, 9],
                        unknown_rows=[5, 8], direction='front',
                        outside_box='observed', road_gap_filling=False)
TIME_SPEC = dict(clock='source_capture_seconds', context_frames=5,
                 nominal_dt_sec=.4, max_gap_sec=.65,
                 reset_on_missing_timestamp=True, fixed_inference_timer=False)


def observation_pixels():
    """The user's central two columns, bottom three rows, in camera coordinates."""
    known = np.ones((270, 480), np.float32)
    known[math.ceil(270 * 5 / 8):, 210:270] = 0
    return cv2.resize(known[135:257], (64, 32), interpolation=cv2.INTER_AREA)


class FrontStopReadout(nn.Module):
    def __init__(self, architecture='temporal_front'):
        super().__init__()
        if architecture not in ARCHITECTURES:
            raise ValueError('unknown STOP architecture')
        self.architecture = architecture
        actor = DriveReadout()
        self.vision, self.action = actor.vision, actor.action
        self.vision.requires_grad_(False)
        self.action.requires_grad_(False)
        self.register_buffer('known_front', torch.from_numpy(observation_pixels())[None, None])
        if architecture == 'reset_shared':
            self.stop = nn.Sequential(nn.Linear(64, 32), nn.SiLU(), nn.Linear(32, 2))
        else:
            self.front = nn.Sequential(
                nn.Conv2d(2, 8, 5, stride=2, padding=2), nn.SiLU(),
                nn.Conv2d(8, 16, 3, stride=2, padding=1), nn.SiLU(),
                nn.Conv2d(16, 16, 3, stride=2, padding=1), nn.SiLU(),
                nn.Flatten(), nn.Linear(16 * 4 * 8, 32), nn.SiLU())
            if architecture == 'temporal_front':
                self.memory = nn.GRUCell(33, 32)
            self.stop = nn.Sequential(nn.Linear(32, 16), nn.SiLU(), nn.Linear(16, 2))

    def retain_action(self, original):
        self.vision.load_state_dict(original.vision.state_dict(), strict=True)
        self.action.load_state_dict(original.action.state_dict(), strict=True)

    def forward(self, pixels, dt=None):
        """B,T,4,32,64, source dt B,T -> final action and STOP logits.

        B,4,32,64 is also accepted for a cold single observation. The temporal
        branch is deliberately bounded to five captures, in training and live.
        """
        if pixels.ndim == 4:
            pixels = pixels[:, None]
        if pixels.ndim != 5 or tuple(pixels.shape[2:]) != (4, 32, 64):
            raise ValueError('policy requires B,T,4,32,64 Road crop pixels')
        batch, steps = pixels.shape[:2]
        if not 1 <= steps <= TIME_SPEC['context_frames']:
            raise ValueError('bounded source-capture context required')
        visual = self.vision(pixels[:, -1])
        raw = self.action(visual)
        action = torch.stack([raw[:, 0].sigmoid(), raw[:, 1].tanh()], 1)
        if self.architecture == 'reset_shared':
            return action, self.stop(visual)
        front = pixels[:, :, 0].reshape(batch * steps, 1, 32, 64)
        known = self.known_front.expand(batch * steps, -1, -1, -1)
        features = self.front(torch.cat([front * known, known], 1)).reshape(batch, steps, 32)
        if self.architecture == 'front':
            return action, self.stop(features[:, -1])
        if dt is None or tuple(dt.shape) != (batch, steps):
            raise ValueError('temporal STOP requires source capture deltas')
        memory = features.new_zeros((batch, 32))
        for k in range(steps):
            elapsed = dt[:, k:k + 1].clamp(0, TIME_SPEC['max_gap_sec']) / TIME_SPEC['nominal_dt_sec']
            memory = self.memory(torch.cat([features[:, k], elapsed], 1), memory)
        return action, self.stop(memory)


class FrontStopPolicy:
    def __init__(self, model, device='cpu', *, details=None):
        self.model = model.to(device).eval().requires_grad_(False)
        self.device, self.max_speed_mps = device, 10.
        self.details = details or {}
        self.reset()

    def reset(self):
        self.history = deque(maxlen=TIME_SPEC['context_frames'])
        self.last_time = None

    def infer(self, masks, *, source_time_sec=None):
        return self.infer_pixels(policy_pixels(masks), source_time_sec=source_time_sec)

    def infer_pixels(self, pixels, *, source_time_sec=None):
        start = time.perf_counter()
        pixels = np.asarray(pixels, dtype=np.float32)
        if pixels.shape != (4, 32, 64) or not np.isfinite(pixels).all() or pixels.min() < 0 or pixels.max() > 1:
            raise ValueError('finite Road probabilities in [0,1] required')
        temporal = self.model.architecture == 'temporal_front'
        valid_time = (type(source_time_sec) in (int, float)
                      and math.isfinite(source_time_sec) and source_time_sec >= 0)
        reset_reason = None
        delta = 0.
        if not valid_time:
            self.reset()
            reset_reason = 'missing_timestamp'
        elif self.last_time is None:
            reset_reason = 'new_stream'
        else:
            delta = float(source_time_sec - self.last_time)
            if not 0 < delta <= TIME_SPEC['max_gap_sec']:
                self.reset()
                reset_reason = 'source_gap_or_nonmonotonic'
                delta = 0.
        if not temporal:
            self.history.clear()
        self.history.append((pixels.copy(), delta))
        self.last_time = float(source_time_sec) if valid_time else None
        sequence = np.stack([p for p, _ in self.history])
        dt = np.array([[d for _, d in self.history]], np.float32)
        # The first retained capture starts a new bounded context.
        dt[0, 0] = 0.
        with torch.inference_mode():
            action, logits = self.model(torch.as_tensor(sequence[None], device=self.device),
                                        torch.as_tensor(dt, device=self.device))
            action = action[0].cpu().numpy()
            prob = logits.softmax(1)[0].cpu().numpy()
        stopped = bool(prob[1] > prob[0])
        command = DriveDecision(0. if stopped else 10. * float(action[0]), 0.,
                                0. if stopped else .35 * float(action[1]),
                                'neural_wait' if stopped else 'neural_drive')
        return command, dict(ms=(time.perf_counter() - start) * 1000,
            stop_probability=float(prob[1]), raw_action=action.tolist(), override=False,
            stop_architecture=self.model.architecture, stop_uses_side_views=self.model.architecture == 'reset_shared',
            context_captures=len(sequence), source_dt_sec=delta, context_reset=reset_reason,
            unknown_region='fixed_robot_box_only', steering_retrained=False)
