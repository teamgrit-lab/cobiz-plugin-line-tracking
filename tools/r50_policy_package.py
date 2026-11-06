"""Hash-pinned R50 detector + retrained CNN actor; no Flyvis encoder at runtime."""
import hashlib
import json
from pathlib import Path

from surround_camera import load_profile, profile_sha256

SCHEMA = 'r50_360.camera_only.pair.v1'
POLICY_SCHEMA = 'r50_360.drive_readout.vision.v1'


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def load_config(directory, expected_sha256, detector_sha256):
    root = Path(directory)
    if len(expected_sha256) != 64 or sha256(root / 'model.json') != expected_sha256:
        raise ValueError('R50 paired model.json SHA-256 mismatch')
    config = json.loads((root / 'model.json').read_text())
    required = dict(panorama_shape_wh=[2880, 1440],
                    camera_views=['front', 'left', 'rear', 'right'],
                    view_shape_wh=[480, 270], forward_heading_deg=90,
                    actor_input_chw=[4, 32, 64], actor_crop_rows=[135, 257])
    if (config.get('schema') != SCHEMA
            or any(config.get('input_contract', {}).get(k) != v for k, v in required.items())
            or config.get('camera_profile_sha256') != profile_sha256(load_profile())
            or config.get('detector_manifest_sha256') != detector_sha256
            or config.get('policy_trained_detector_sha256') != detector_sha256
            or config.get('action_normalization') != dict(vx_mps=10., yaw_radps=.35)):
        raise ValueError('R50 detector/policy training pair or input contract mismatch')
    for filename, key in [('policy/drive-readout.pt', 'policy_sha256'),
                          ('policy/training.json', 'policy_training_sha256')]:
        if sha256(root / filename) != config.get(key):
            raise ValueError('R50 paired artifact SHA-256 mismatch: ' + filename)
    return config


def load_policy(directory, config, device='cpu'):
    import torch
    from connectome_policy import ConnectomeDrivePolicy, DriveReadout

    root = Path(directory)
    checkpoint = torch.load(root / 'policy/drive-readout.pt', map_location='cpu', weights_only=True)
    if (checkpoint.get('schema') != POLICY_SCHEMA
            or checkpoint.get('detector_manifest_sha256') != config['detector_manifest_sha256']
            or checkpoint.get('action_normalization') != config['action_normalization']):
        raise ValueError('Actor checkpoint does not match its trained R50 detector')
    # Reuse the unchanged architecture and command conversion, never the old
    # Flyvis pairing assertions or its training-detector identity.
    policy = ConnectomeDrivePolicy.__new__(ConnectomeDrivePolicy)
    policy.model = DriveReadout()
    policy.model.load_state_dict(checkpoint['state_dict'], strict=True)
    if not all(torch.isfinite(v).all() for v in policy.model.state_dict().values()):
        raise ValueError('Nonfinite retrained actor parameters')
    policy.model.to(device).eval().requires_grad_(False)
    policy.device, policy.max_speed_mps = device, 10.
    policy.details = dict(backend='r50-trained-cnn-drive-readout', schema=POLICY_SCHEMA,
                         weights_sha256=config['policy_sha256'],
                         trained_detector_sha256=config['detector_manifest_sha256'],
                         detector_override=False, runtime_planner=False,
                         input='four R50 binary camera road masks only',
                         max_speed_mps=10., real_robot_validated=False,
                         training=json.loads((root / 'policy/training.json').read_text()))
    return policy
