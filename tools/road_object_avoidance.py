"""Road-only opposite-side sidestep on measured person/vehicle positions."""
from dataclasses import replace
import math
import numpy as np

from local_avoidance import AvoidanceController


class RoadObjectAvoidanceController(AvoidanceController):
    def __init__(self, config, obstacles, objects):
        objects.validate()
        self.object_config = objects
        self.legacy_config = config
        super().__init__(replace(config, max_lateral_mps=objects.max_lateral_mps), obstacles)

    def reset(self):
        super().reset()
        self.objects = []
        self.object_stamp = None
        self.object_side = None

    def set_objects(self, objects, stamp):
        self.objects = objects
        if stamp != self.object_stamp:
            self.target_scan_stamp = None  # New camera evidence invalidates cached plans.
        self.object_stamp = stamp

    def _choose_target(self, nominal, path, scan, region, age, now, dt):
        if not self.objects:
            self.object_side = None
            self.metrics.update(object_label=None, object_x_m=None, object_y_m=None,
                                avoidance_side=None, requested_lateral_mps=0., target_lateral_mps=0.)
            # Keep ordinary obstacle avoidance/return speeds unchanged.
            current = self.config
            self.config = self.legacy_config
            try:
                return super()._choose_target(nominal, path, scan, region, age, now, dt)
            finally:
                self.config = current
        a, o = self.config, self.object_config
        target = min(self.objects, key=lambda p: math.hypot(p['x'], p['y']))
        side = (-1 if target['y'] > 0 else 1) if abs(target['y']) > o.center_deadband_m else (
            self.object_side or (-1 if a.preference == 'right' else 1))
        # Never select the object's side merely because the opposite side is blocked.
        self.object_side = side
        self.side = side
        self.avoiding = True
        distance = math.hypot(target['x'], target['y'])
        urgency = float(np.clip(1-distance/o.max_range_m, 0, 1))
        speed = o.min_lateral_mps + (o.max_lateral_mps-o.min_lateral_mps)*urgency
        self.metrics.update(object_label=target['label'], object_x_m=target['x'], object_y_m=target['y'],
                            avoidance_side='left' if side > 0 else 'right', requested_lateral_mps=side*speed)
        for magnitude in sorted({speed, (speed+o.min_lateral_mps)/2, o.min_lateral_mps}, reverse=True):
            for target_vx in [min(nominal.vx, a.max_forward_mps), 0.]:
                target_vy = side*magnitude
                vx, vy = self._ramp(target_vx, target_vy, dt)
                if vy*side < -1e-8:
                    continue  # Stop before a requested reversal; no motion toward the object.
                if self.trajectory_check(vx, vy, nominal.yaw_rate, scan, region, age,
                                         (target_vx, target_vy))[0]:
                    self.metrics['target_lateral_mps'] = target_vy
                    return target_vx, target_vy
        self.metrics['target_lateral_mps'] = 0.
        return None
