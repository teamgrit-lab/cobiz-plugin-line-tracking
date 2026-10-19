"""Verified Flyvis 1.2.0 optical-flow circuit, isolated in the planner service.

Uses the same flynav.encoder.v1 bundle as the referenced fly_connectom project.
The input remains the original camera image; semantic masks never enter the
pretrained circuit. There is no untrained/random decoder or optical-flow fallback.
"""
import hashlib
import math
import os
from pathlib import Path

import cv2
import numpy as np

from connectome_surface import FlowField


class FlyvisFlow:
    def __init__(self, bundle_path, expected_sha256, device='cuda', max_gap=1.):
        import torch
        os.environ.setdefault('FLYVIS_ROOT_DIR', '/tmp/line-tracking-flyvis')
        import flyvis
        from flyvis import Network
        from flyvis.datasets.rendering import BoxEye
        from flyvis.task.decoder import DecoderGAVP

        if flyvis.__version__ != '1.2.0':
            raise ValueError('Flyvis 1.2.0 required')
        actual = hashlib.sha256(Path(bundle_path).read_bytes()).hexdigest()
        if len(expected_sha256) != 64 or actual != expected_sha256.lower():
            raise ValueError('connectome bundle SHA-256 mismatch')
        self.torch, self.device, self.max_gap = torch, torch.device(device), max_gap
        if device == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('CUDA PyTorch required; no CPU fallback')
        torch.set_num_threads(2)
        torch.set_default_device(self.device)
        flyvis.device = self.device
        bundle = torch.load(bundle_path, map_location=self.device, weights_only=True)
        if (bundle.get('format') != 'flynav.encoder.v1' or not bundle.get('pretrained_checkpoint')
                or bundle.get('pretrained_task') != 'optical_flow'
                or not bundle.get('source', {}).get('checkpoint_sha256')):
            raise ValueError('verified pretrained optical-flow bundle required')
        self.network = Network(**bundle['network_config'])
        self.network.load_state_dict(bundle['state_dict'], strict=True)
        self.network.to(self.device).eval().requires_grad_(False)
        decoder_config = dict(bundle['flow_decoder_config'])
        if decoder_config.pop('type') != 'DecoderGAVP':
            raise ValueError('unsupported pretrained flow decoder')
        self.decoder = DecoderGAVP(self.network.connectome, **decoder_config)
        self.decoder.load_state_dict(bundle['flow_decoder_state_dict'], strict=True)
        self.decoder.to(self.device).eval().requires_grad_(False)
        for model in (self.network, self.decoder):
            if not all(torch.isfinite(p).all() for p in model.state_dict().values()):
                raise ValueError('nonfinite pretrained parameters')
        self.eye = BoxEye(extent=bundle['extent'], kernel_size=13)
        self.eye.conv.to(self.device)
        self.size = int(self.eye.min_frame_size.max().item())
        with torch.inference_mode():
            self.grey = self.network.steady_state(1., .02, batch_size=1)
        self.details = {'backend': 'flyvis', 'version': flyvis.__version__, 'bundle_sha256': actual,
                        'pretrained_task': 'optical_flow', 'flow_decoder_loaded': True,
                        'trained_waypoint_head': False, 'nodes': self.network.n_nodes,
                        'source': bundle['source'], 'inferred_frames': 0}
        self.reset()

    def reset(self):
        self.state = self.last_stamp = self.last_input = None

    def advance(self, frame, stamp):
        if (not math.isfinite(stamp) or self.last_stamp is not None
                and not 0 < stamp-self.last_stamp <= self.max_gap):
            self.reset()
            raise ValueError('connectome stream discontinuity')
        torch = self.torch
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)/255
        scale = min(self.size/frame.shape[1], self.size/frame.shape[0])
        w, h = round(frame.shape[1]*scale), round(frame.shape[0]*scale)
        ox, oy = (self.size-w)//2, (self.size-h)//2
        canvas = np.full((self.size, self.size), .5, np.float32)
        canvas[oy:oy+h, ox:ox+w] = cv2.resize(gray, (w, h))
        with torch.inference_mode():
            receptors = self.eye(torch.as_tensor(canvas, device=self.device)[None, None])
            steps = 1 if self.last_stamp is None else max(1, math.ceil((stamp-self.last_stamp)/.02))
            dt = .02 if self.last_stamp is None else (stamp-self.last_stamp)/steps
            inputs = receptors if self.last_input is None else self.last_input.expand(1, steps, 1, -1).clone()
            inputs[:, -1:] = receptors
            self.network.stimulus.zero(1, steps)
            self.network.stimulus.add_input(inputs)
            states = self.network(self.network.stimulus(), dt=dt,
                                  state=self.grey if self.state is None else self.state, as_states=True)
            self.state = states[-1]
            flow = self.decoder(self.state.nodes.activity[:, None, :])[0, 0].cpu().numpy().copy()
        centers = self.eye.receptor_centers.cpu().numpy()[:, ::-1].astype(float)
        # BoxEye centers are relative to the square input centre, in pixels.
        pixels = (centers+self.size//2-[ox, oy])/scale
        if flow.shape != (2, len(pixels)) or not np.isfinite(flow).all():
            self.reset()
            raise ValueError('invalid pretrained flow output')
        visible = ((pixels[:, 0] >= 0) & (pixels[:, 0] < frame.shape[1])
                   & (pixels[:, 1] >= 0) & (pixels[:, 1] < frame.shape[0]))
        vectors = flow[:, visible].T
        # Remove global image translation; these values are relative motion,
        # never metric obstacle distance, collision probability or odometry.
        motion = np.linalg.norm(vectors-np.median(vectors, axis=0), axis=1)
        motion = np.clip(motion/max(float(np.percentile(motion, 90)), 1e-8), 0, 1)
        self.last_stamp, self.last_input = stamp, receptors
        self.details['inferred_frames'] += 1
        return FlowField(pixels[visible], motion)
