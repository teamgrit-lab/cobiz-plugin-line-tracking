"""Road adapter for the paired 360 detector/policy; retain Sidewalk routing."""
import math

import numpy as np

from connectome360_runtime import PROTOCOL
from front_connectome_client import FrontConnectomeSegmenter, limit_front_command, validated_command
from surround_camera import load_profile, profile_sha256
from swin_l_drive_control import DriveDecision


class Connectome360Segmenter(FrontConnectomeSegmenter):
    protocol = PROTOCOL

    def validate_model(self, model):
        scale = model.get('trained_max_speed_mps')
        if (model.get('backend') != 'flyvis360-road-policy'
                or model.get('camera_views') != ['front', 'left', 'rear', 'right']
                or model.get('weights_sha256') != self.expected_sha256
                or model.get('camera_profile_sha256') != profile_sha256(load_profile())
                or model.get('uses_ground_truth_at_runtime') is not False
                or model.get('pose_input') is not False or model.get('goal_input') is not False
                or type(scale) not in (int, float) or not math.isfinite(scale) or not .1 <= scale <= 20):
            raise ValueError('360 model identity/input contract mismatch')
        for key in ['road_sha256', 'policy_sha256', 'encoder_sha256']:
            value = model.get(key, '')
            if len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
                raise ValueError('360 paired weight identity required')
        # A response cannot silently swap the detector or actor behind the pin.
        if hasattr(self, '_paired_hashes') and self._paired_hashes != [model[k] for k in ['road_sha256', 'policy_sha256', 'encoder_sha256']]:
            raise ValueError('360 paired weights changed during stream')
        self._paired_hashes = [model[k] for k in ['road_sha256', 'policy_sha256', 'encoder_sha256']]

    def prepare_frame(self, bgr):
        if bgr.dtype != np.uint8 or bgr.shape != (540, 960, 3):
            raise ValueError('360 adapter requires the original four-view 960x540 atlas')
        return bgr

    def decode_command(self, value):
        if not isinstance(value, dict) or set(value) != {'vx', 'vy', 'yaw_rate', 'reason'}:
            raise ValueError('invalid 360 learned command')
        if any(type(value[k]) not in (int, float) or not math.isfinite(value[k])
               for k in ['vx', 'vy', 'yaw_rate']):
            raise ValueError('nonfinite 360 learned command')
        if (not 0 <= value['vx'] <= self.details['trained_max_speed_mps']
                or value['vy'] != 0 or abs(value['yaw_rate']) > .35 + 1e-6
                or value['reason'] not in ['neural_drive', 'neural_wait']):
            raise ValueError('360 command exceeds trained model contract')
        if value['reason'] == 'neural_wait' and (value['vx'] or value['yaw_rate']):
            raise ValueError('360 learned stop must be zero')
        command = DriveDecision(value['vx'], 0., float(np.clip(value['yaw_rate'], -.35, .35)),
                                'tracking' if value['reason'] == 'neural_drive' else 'neural_wait')
        # The simulation's 10 m/s scale is never a physical robot speed setting.
        # Apply both caps with ONE factor so curvature is preserved. The live
        # node applies its existing (default 0.5 m/s) cap again at publication.
        limited = limit_front_command(command, .8, .35)
        return validated_command(dict(vx=limited.vx, vy=limited.vy,
                                      yaw_rate=limited.yaw_rate, reason=limited.reason))
