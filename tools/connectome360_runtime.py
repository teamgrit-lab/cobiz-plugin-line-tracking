"""Load a camera-only Flyvis road/policy pair without its simulation server."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from surround_camera import load_profile, profile_sha256


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def unpack_views(atlas):
    if atlas.dtype != np.uint8 or atlas.shape != (540, 960, 3):
        raise ValueError('lossless 960x540 BGR atlas in front/left/rear/right order required')
    return [np.ascontiguousarray(atlas[y:y+270, x:x+480])
            for y, x in ((0, 0), (0, 480), (270, 0), (270, 480))]


def package_config(directory, expected_sha256):
    root = Path(directory)
    if (len(expected_sha256) != 64 or any(c not in '0123456789abcdef' for c in expected_sha256)
            or sha256(root/'model.json') != expected_sha256):
        raise ValueError('360 model.json SHA-256 mismatch')
    config = json.loads((root/'model.json').read_text())
    if config.get('schema') != 'connectome360.camera_only.pair.v1':
        raise ValueError('camera-only 360 road/policy package required')
    contract = config.get('input_contract', {})
    required = dict(panorama_shape_wh=[2880, 1440], camera_views=['front', 'left', 'rear', 'right'],
                    view_shape_wh=[480, 270], forward_heading_deg=90,
                    actor_input_chw=[4, 32, 64], actor_crop_rows=[135, 257])
    if (any(contract.get(k) != v for k, v in required.items())
            or config.get('camera_profile_sha256') != profile_sha256(load_profile())):
        raise ValueError('360 camera profile/input contract mismatch')
    scale = config.get('simulation_max_speed_mps')
    if type(scale) not in (int, float) or not math.isfinite(scale) or not .1 <= scale <= 20:
        raise ValueError('explicit trained policy speed scale required')
    for name, key in [('flyvis_encoder.pt', 'encoder_sha256'), ('road/road-readout.pt', 'road_sha256'),
                      ('policy/drive-readout.pt', 'policy_sha256')]:
        if sha256(root/name) != config.get(key):
            raise ValueError('360 model weight SHA-256 mismatch: ' + name)
    return config


class Connectome360Runtime:
    def __init__(self, directory, expected_sha256, *, encoder_device='cuda', head_device='cuda'):
        import torch
        from connectome_flow import FlyvisFlow
        from connectome_policy import ConnectomeDrivePolicy
        from connectome_road import ConnectomeRoadPerception

        root = Path(directory)
        config = package_config(root, expected_sha256)
        self.encoder = FlyvisFlow(root/'flyvis_encoder.pt', config['encoder_sha256'], encoder_device)
        self.road = ConnectomeRoadPerception(self.encoder, root/'road', head_device)
        # Preserve the trained speed/yaw ratio. Physical caps are applied by the
        # tracking adapter, never by changing the actor's normalization scale.
        self.policy = ConnectomeDrivePolicy(root/'policy', self.road, head_device,
                                          max_speed_mps=config['simulation_max_speed_mps'])
        for model in (self.road.model, self.policy.model):
            if not all(torch.isfinite(v).all() for v in model.state_dict().values()):
                raise ValueError('nonfinite 360 model parameters')
        self.details = dict(backend='flyvis360-road-policy', weights_sha256=expected_sha256,
            road_sha256=config['road_sha256'], policy_sha256=config['policy_sha256'],
            encoder_sha256=config['encoder_sha256'], camera_views=['front', 'left', 'rear', 'right'],
            camera_profile_sha256=config['camera_profile_sha256'], road_threshold=self.road.threshold,
            trained_max_speed_mps=float(config['simulation_max_speed_mps']),
            head_device=str(head_device), encoder_device=str(encoder_device),
            uses_ground_truth_at_runtime=False, pose_input=False, goal_input=False,
            runtime_planner=False, real_robot_validated=False,
            source_package_simulation_only=config.get('simulation_only', True))

    def reset(self):
        self.encoder.reset()

    def infer(self, atlas):
        return self.infer_views(unpack_views(atlas))

    def infer_views(self, views):
        masks, metrics = self.road.predict_views(views)
        command, policy = self.policy.infer(masks)
        return masks[0], command, dict(perception=metrics, policy=policy)
