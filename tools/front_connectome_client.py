"""Learned Road model identity, command limits and source-age checks."""
import math

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


def limit_front_command(command, max_speed, max_yaw):
    factor = min(1., max_speed / command.vx if command.vx else 1.,
                 max_yaw / abs(command.yaw_rate) if command.yaw_rate else 1.)
    return DriveDecision(min(max_speed, command.vx * factor), 0.,
                         max(-max_yaw, min(max_yaw, command.yaw_rate * factor)), command.reason)


def front_drive_decision(command, mode, ages, max_age, camera_fault=None, *, backend='front_connectome'):
    if mode != 1:
        return DriveDecision.stop(backend + '_requires_road')
    if camera_fault:
        return DriveDecision.stop(camera_fault)
    if any(age is None or not math.isfinite(age) or not 0 <= age <= max_age for age in ages):
        return DriveDecision.stop(backend + '_source_stale')
    if command is None:
        return DriveDecision.stop(backend + '_unavailable')
    return validated_command(command)


class FrontConnectomeSegmenter:
    """Validate the front model and decode commands; contains no image transport."""

    def __init__(self, expected_sha256):
        self.expected_sha256 = expected_sha256
        self.details = None

    def validate_model(self, model):
        if (model.get('backend') != 'flyvis-front-road' or model.get('camera_views') != ['front']
                or model.get('weights_sha256') != self.expected_sha256
                or model.get('uses_ground_truth_at_runtime') is not False
                or model.get('pose_input') is not False or model.get('goal_input') is not False):
            raise ValueError('front model identity/input contract mismatch')

    def decode_command(self, value):
        return validated_command(value)
