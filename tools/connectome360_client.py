"""Road adapter for the paired 360 detector/policy; retain Sidewalk routing."""
import math

import numpy as np

from connectome360_runtime import PROTOCOL
from front_connectome_client import FrontConnectomeSegmenter, limit_front_command, validated_command
from surround_camera import load_profile, profile_sha256
from drive_control import DriveDecision


class Connectome360Segmenter(FrontConnectomeSegmenter):
    protocol = PROTOCOL

    def __init__(self, url, expected_sha256, timeout=1., *, detector_sha256=''):
        self.detector_sha256 = detector_sha256
        super().__init__(url, expected_sha256, timeout)

    def validate_model(self, model):
        scale = model.get('trained_max_speed_mps')
        detector_pin = getattr(self, 'detector_sha256', '')
        backend = 'r50-360-road-policy' if detector_pin else 'flyvis360-road-policy'
        if (model.get('backend') != backend
                or model.get('camera_views') != ['front', 'left', 'rear', 'right']
                or model.get('weights_sha256') != self.expected_sha256
                or model.get('camera_profile_sha256') != profile_sha256(load_profile())
                or model.get('uses_ground_truth_at_runtime') is not False
                or model.get('pose_input') is not False or model.get('goal_input') is not False
                or type(scale) not in (int, float) or not math.isfinite(scale) or not .1 <= scale <= 20):
            raise ValueError('360 model identity/input contract mismatch')
        keys = ['road_sha256', 'policy_sha256', 'encoder_sha256']
        if detector_pin:
            if (model.get('detector_sha256') != detector_pin
                    or model.get('runtime_planner') is not False
                    or model.get('road_mask_shape') != [4, 270, 480]
                    or model.get('actor_input_chw') != [4, 32, 64]
                    or model.get('actor_crop_rows') != [135, 257]):
                raise ValueError('R50 detector override/input contract mismatch')
            if model.get('detector_override') is True:
                keys = ['detector_sha256', 'policy_sha256', 'training_road_sha256', 'training_encoder_sha256']
            elif (model.get('detector_override') is False
                  and model.get('policy_schema') in ('r50_360.drive_readout.vision.v1',
                                                     'r50_360.drive_readout.front_stop.v2')
                  and model.get('policy_trained_detector_sha256') == detector_pin):
                keys = ['detector_sha256', 'policy_sha256', 'policy_trained_detector_sha256']
                if model['policy_schema'] == 'r50_360.drive_readout.front_stop.v2':
                    # Constants only; importing torch/model implementations in
                    # the lightweight ROS client would defeat process isolation.
                    architecture = model.get('stop_architecture')
                    observation = dict(reference_grid_hw=[8, 16], unknown_columns=[7, 9],
                        unknown_rows=[5, 8], direction='front', outside_box='observed', road_gap_filling=False)
                    timing = dict(clock='source_capture_seconds', context_frames=5,
                        nominal_dt_sec=.4, max_gap_sec=.65, reset_on_missing_timestamp=True, fixed_inference_timer=False)
                    if (architecture not in ('reset_shared', 'front', 'temporal_front')
                            or model.get('stop_input_chw') != [4 if architecture == 'reset_shared' else 2, 32, 64]
                            or model.get('stop_uses_side_views') is not (architecture == 'reset_shared')
                            or model.get('observation_spec') != observation
                            or model.get('time_spec') != timing
                            or model.get('source_timed_policy') is not (architecture == 'temporal_front')):
                        raise ValueError('R50 front STOP observation/time contract mismatch')
            else:
                raise ValueError('R50 policy must identify its actual training detector')
        for key in keys:
            value = model.get(key, '')
            if len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
                raise ValueError('360 paired weight identity required')
        # A response cannot silently swap the detector or actor behind the pin.
        if hasattr(self, '_paired_hashes') and self._paired_hashes != [model[k] for k in keys]:
            raise ValueError('360 paired weights changed during stream')
        self._paired_hashes = [model[k] for k in keys]

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
