"""Timestamped planar ego-motion and bounded upper-level path following.

Odometry owns pose. The body gyro only propagates the short interval AFTER the
last odometry sample; it is not fused again as an independent orientation.
Transforms are explicit target-from-source matrices or fixed TF lookups.
"""
from bisect import bisect_left
from collections import deque
from dataclasses import dataclass, replace
import math
import threading

import numpy as np

from lidar_common import stamp_ns, transform_matrix, validate_transform
from swin_l_drive_control import DriveDecision


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def se2(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0., 0., 1.]])


def lift(matrix):
    result = np.eye(4)
    result[:2, :2], result[:2, 3] = matrix[:2, :2], matrix[:2, 2]
    return result


def move_points(points, matrix):
    return np.asarray(points) @ matrix[:2, :2].T + matrix[:2, 2]


@dataclass(frozen=True)
class MotionConfig:
    control_hz: float = 20.
    sensor_max_age_sec: float = .20
    history_sec: float = 4.
    max_sample_gap_sec: float = .10
    max_extrapolation_sec: float = .05
    max_reference_age_sec: float = 1.
    max_translation_m: float = .80
    max_rotation_deg: float = 25.
    max_odom_speed_mps: float = 2.
    max_odom_yaw_rps: float = 4.
    max_accel_mps2: float = .30
    max_yaw_accel_rps2: float = .50
    point_spacing_m: float = .10
    odom_frame: str = 'odom'
    odom_child_frame: str = 'robot_center'
    imu_frame: str = 'dog_imu_link'
    base_frame: str = 'base_link'

    def validate(self):
        for name, value in vars(self).items():
            if isinstance(value, float) and (not math.isfinite(value) or value <= 0):
                raise ValueError('motion '+name+' must be finite and positive')
        if not 10 <= self.control_hz <= 50 or self.max_reference_age_sec > self.history_sec:
            raise ValueError('motion rate must be 10..50 Hz; history must cover references')
        if self.max_extrapolation_sec > .10 or self.sensor_max_age_sec > .50:
            raise ValueError('motion extrapolation <= .10 s and sensor age <= .50 s required')
        if not .05 <= self.point_spacing_m <= .5 or self.max_rotation_deg > 30:
            raise ValueError('motion spacing must be .05.. .5 m and rotation <= 30 degrees')
        if not all(getattr(self, k) for k in ('odom_frame', 'odom_child_frame', 'imu_frame', 'base_frame')):
            raise ValueError('motion frame ids must be nonempty')


@dataclass(frozen=True)
class Pose:
    stamp: float
    arrival: float
    x: float
    y: float
    yaw: float
    vx: float
    vy: float

    def matrix(self):
        return se2(self.x, self.y, self.yaw)


class MotionBuffer:
    def __init__(self, config=MotionConfig(), base_from_child=None, base_from_imu=None, lookup=None):
        config.validate()
        self.config, self.lookup = config, lookup
        self.child_transform, self.imu_transform = base_from_child, base_from_imu
        for matrix in (base_from_child, base_from_imu):
            if matrix is not None:
                validate_transform(matrix)
        self.lock = threading.RLock()
        self.odom, self.imu = deque(maxlen=8192), deque(maxlen=8192)
        self.epoch = 0
        self.fault_counts = {}
        self.gap_counts = {'odom': 0, 'imu': 0}
        self.last_seen = {'odom': None, 'imu': None}
        self.faults = {'odom': 'motion_waiting_for_odom', 'imu': 'motion_waiting_for_imu'}
        self.metrics = {'ready': False, 'reason': 'motion_waiting_for_odom'}

    def _transform(self, name, source, header):
        matrix = self.child_transform if name == 'odom' else self.imu_transform
        if matrix is None:
            if source == self.config.base_frame:
                matrix = np.eye(4)
            elif self.lookup:
                try:
                    matrix = self.lookup(self.config.base_frame, source, header)
                except Exception as error:
                    raise ValueError('motion_'+name+'_transform_unavailable') from error
            else:
                raise ValueError('motion_'+name+'_transform_unavailable')
            validate_transform(matrix)
            if name == 'odom': self.child_transform = matrix
            else: self.imu_transform = matrix
        return matrix

    def _fault(self, name, reason):
        if self.faults[name] is None:
            # Invalidate camera packets already queued before the discontinuity.
            self.epoch += 1
        if self.faults[name] != reason:
            self.fault_counts[reason] = self.fault_counts.get(reason, 0)+1
        self.faults[name] = reason
        (self.odom if name == 'odom' else self.imu).clear()
        self.last_seen[name] = None

    def _time(self, message, arrival, clock_ns, name):
        t = stamp_ns(message.header)/1e9
        if t <= 0 or not -.02 <= clock_ns/1e9-t <= self.config.sensor_max_age_sec:
            raise ValueError('motion_'+name+'_timestamp_invalid')
        previous = self.last_seen[name]
        if previous is not None:
            if t <= previous:
                raise ValueError('motion_'+name+'_discontinuity')
            if t-previous > self.config.max_sample_gap_sec:
                self.gap_counts[name] += 1
        self.last_seen[name] = t
        # A dropped interval does not reset the odom frame. Fresh measured
        # endpoints remain usable; _pose_at still forbids interpolation INTO
        # the missing interval, and _fresh forbids driving during the outage.
        return t

    def on_odom(self, message, arrival, clock_ns):
        with self.lock:
            try:
                c = self.config
                if message.header.frame_id != c.odom_frame or message.child_frame_id != c.odom_child_frame:
                    raise ValueError('motion_odom_frame_mismatch')
                t = self._time(message, arrival, clock_ns, 'odom')
                p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
                values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                          v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z]
                if not np.isfinite(values).all() or not .9 <= np.linalg.norm(values[3:7]) <= 1.1:
                    raise ValueError('motion_odom_invalid')
                # Firmware publishes ~650Hz. Keep at most 100Hz of pose history
                # so deserialization cannot starve the 20Hz command timer.
                if self.odom and t-self.odom[-1].stamp < .01:
                    return
                base_child = self._transform('odom', message.child_frame_id, message.header)
                # T_odom_base = T_odom_child @ T_child_base, including lever arm.
                from types import SimpleNamespace
                odom_child = transform_matrix(SimpleNamespace(translation=p, rotation=q))
                child_base = np.linalg.inv(base_child)
                world = odom_child @ child_base
                if world[2, 2] < math.cos(math.radians(20)):
                    raise ValueError('motion_tilt_exceeded')
                linear = base_child[:3, :3] @ (np.array(values[7:10]) + np.cross(values[10:13], child_base[:3, 3]))
                pose = Pose(t, arrival, world[0, 3], world[1, 3], math.atan2(world[1, 0], world[0, 0]), *linear[:2])
                if math.hypot(pose.vx, pose.vy) > c.max_odom_speed_mps:
                    raise ValueError('motion_odom_velocity_invalid')
                if self.odom:
                    last = self.odom[-1]; dt = t-last.stamp
                    if (math.hypot(pose.x-last.x, pose.y-last.y) > c.max_odom_speed_mps*dt+.05
                            or abs(wrap(pose.yaw-last.yaw)) > c.max_odom_yaw_rps*dt+.08):
                        raise ValueError('motion_odom_jump')
                self.odom.append(pose)
                while self.odom[0].stamp < t-c.history_sec: self.odom.popleft()
                self.faults['odom'] = None
            except (ValueError, AttributeError, TypeError, np.linalg.LinAlgError) as error:
                reason = str(error) if str(error).startswith('motion_') else 'motion_odom_invalid'
                self._fault('odom', reason)

    def on_imu(self, message, arrival, clock_ns):
        with self.lock:
            try:
                c = self.config
                if message.header.frame_id != c.imu_frame:
                    raise ValueError('motion_imu_frame_mismatch')
                t = self._time(message, arrival, clock_ns, 'imu')
                if message.angular_velocity_covariance[0] == -1:
                    raise ValueError('motion_gyro_unavailable')
                w = message.angular_velocity
                values = (w.x, w.y, w.z)
                if not all(math.isfinite(v) for v in values) or math.sqrt(sum(v*v for v in values)) > c.max_odom_yaw_rps:
                    raise ValueError('motion_gyro_invalid')
                if self.imu and t-self.imu[-1][0] < .01:
                    return
                matrix = self._transform('imu', message.header.frame_id, message.header)
                gyro = matrix[:3, :3] @ values
                self.imu.append((t, arrival, float(gyro[2])))
                while self.imu[0][0] < t-c.history_sec: self.imu.popleft()
                self.faults['imu'] = None
            except (ValueError, AttributeError, TypeError, IndexError) as error:
                reason = str(error) if str(error).startswith('motion_') else 'motion_imu_invalid'
                self._fault('imu', reason)

    def _fresh(self, now, clock_ns):
        for name in ('odom', 'imu'):
            if self.faults[name]: raise ValueError(self.faults[name])
            value = self.odom[-1] if name == 'odom' else self.imu[-1]
            stamp, arrival = (value.stamp, value.arrival) if name == 'odom' else value[:2]
            if not (0 <= now-arrival <= self.config.sensor_max_age_sec
                    and -.02 <= clock_ns/1e9-stamp <= self.config.sensor_max_age_sec):
                raise ValueError('motion_'+name+'_stale')

    def snapshot(self, now, clock_ns):
        with self.lock:
            try:
                self._fresh(now, clock_ns)
                reason = 'ok'
            except ValueError as error:
                reason = str(error)
            return {**self.metrics, 'sensor_reason': reason, 'epoch': self.epoch,
                    'fault_counts': dict(self.fault_counts),
                    'gap_counts': dict(self.gap_counts),
                    'odom_samples': len(self.odom), 'imu_samples': len(self.imu),
                    'latest_odom_stamp_sec': self.odom[-1].stamp if self.odom else None,
                    'latest_imu_stamp_sec': self.imu[-1][0] if self.imu else None}

    def _pose_at(self, stamp):
        rows = self.odom
        i = bisect_left([p.stamp for p in rows], stamp)
        if i < len(rows) and abs(rows[i].stamp-stamp) < 1e-7: return rows[i].matrix()
        if i == 0: raise ValueError('motion_history_unavailable')
        a = rows[i-1]
        if i < len(rows):
            b = rows[i]
            if b.stamp-a.stamp > self.config.max_sample_gap_sec: raise ValueError('motion_history_gap')
            f = (stamp-a.stamp)/(b.stamp-a.stamp)
            return se2(a.x+f*(b.x-a.x), a.y+f*(b.y-a.y), a.yaw+f*wrap(b.yaw-a.yaw))
        dt = stamp-a.stamp
        if not 0 <= dt <= self.config.max_extrapolation_sec:
            raise ValueError('motion_extrapolation_exceeded')
        # Gyro applies only to the unobserved tail, after the last odom stamp.
        gyros = [v for v in self.imu if v[0] <= stamp+1e-7]
        if not gyros or stamp-gyros[-1][0] > self.config.max_extrapolation_sec:
            raise ValueError('motion_gyro_unsynchronized')
        yaw = a.yaw+gyros[-1][2]*dt
        mid = a.yaw+gyros[-1][2]*dt/2
        return se2(a.x+dt*(math.cos(mid)*a.vx-math.sin(mid)*a.vy),
                   a.y+dt*(math.sin(mid)*a.vx+math.cos(mid)*a.vy), yaw)

    def relative(self, source_ns, target_ns, now, clock_ns):
        """T_target_base_from_source_base; never change the original source stamp."""
        with self.lock:
            self._fresh(now, clock_ns)
            return np.linalg.inv(self._pose_at(target_ns/1e9)) @ self._pose_at(source_ns/1e9)

    def path(self, path, source_ns, epoch, now, clock_ns):
        with self.lock:
            try:
                if path is None: raise ValueError('motion_path_unavailable')
                if epoch != self.epoch: raise ValueError('motion_reference_epoch_changed')
                age = clock_ns/1e9-source_ns/1e9
                if not 0 <= age <= self.config.max_reference_age_sec:
                    raise ValueError('motion_reference_stale')
                matrix = self.relative(source_ns, clock_ns, now, clock_ns)
                distance, yaw = np.linalg.norm(matrix[:2, 2]), math.atan2(matrix[1, 0], matrix[0, 0])
                if distance > self.config.max_translation_m or abs(yaw) > math.radians(self.config.max_rotation_deg):
                    raise ValueError('motion_reference_displacement_exceeded')
                # Keep all original vertices: resampling must not cut a corner.
                source = resample(path.points_xy, self.config.point_spacing_m)
                points = move_points(source, matrix)
                if len(points) < 2 or np.any(points[:, 0] <= 0) or np.any(np.diff(points[:, 0]) <= 0):
                    raise ValueError('motion_path_not_forward')
                self.metrics = {'ready': True, 'reason': 'ok', 'epoch': self.epoch,
                                'reference_age_sec': age, 'translation_m': float(distance),
                                'rotation_deg': math.degrees(yaw), 'points': len(points),
                                'odom_age_sec': clock_ns/1e9-self.odom[-1].stamp,
                                'imu_age_sec': clock_ns/1e9-self.imu[-1][0],
                                'gyro_yaw_rps': self.imu[-1][2]}
                return replace(path, points_xy=points, source='motion_compensated'), None
            except (ValueError, np.linalg.LinAlgError) as error:
                reason = str(error)
                self.metrics = {'ready': False, 'reason': reason, 'epoch': self.epoch}
                return None, reason


def resample(points, spacing):
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 2 or not 2 <= len(points) <= 512 or not np.isfinite(points).all():
        raise ValueError('motion_path_invalid')
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if np.any(lengths <= 1e-8): raise ValueError('motion_path_invalid')
    if np.max(lengths) <= spacing:
        return points.copy()
    s = np.r_[0., np.cumsum(lengths)]
    if s[-1]/spacing+len(s) > 1024: raise ValueError('motion_path_too_large')
    samples = np.unique(np.r_[s, np.arange(0., s[-1], spacing)])
    return np.column_stack([np.interp(samples, s, points[:, axis]) for axis in (0, 1)])


def follow_path(path, nominal, config):
    """Pure-pursuit upper-level velocity target; the robot owns joint control."""
    if nominal.reason not in ('tracking', 'tracking_slow_turn'):
        return DriveDecision.stop(nominal.reason)
    points = path.points_xy
    # A shortened observed path uses its actual endpoint, never a fictitious 4m x.
    x = float(np.clip(config.lookahead_m, points[0, 0], points[-1, 0]))
    y = float(np.interp(x, points[:, 0], points[:, 1]))
    curvature = 2*y/(x*x+y*y)
    speed = min(nominal.vx, config.max_forward_mps,
                config.max_yaw_rps/max(abs(curvature), 1e-8))
    yaw = curvature*speed
    return DriveDecision(speed, 0., yaw,
                         'tracking_slow_turn' if speed < config.max_forward_mps else 'tracking')


class CommandLimiter:
    def __init__(self, config=MotionConfig()):
        config.validate(); self.config = config; self.reset()

    def reset(self):
        self.last = DriveDecision.stop('motion_reset'); self.at = None

    def limit(self, decision, now):
        if not all(math.isfinite(v) for v in (decision.vx, decision.vy, decision.yaw_rate, now)):
            return DriveDecision.stop('motion_command_invalid')
        if not (decision.vx or decision.vy or decision.yaw_rate): return decision
        dt = 1/self.config.control_hz if self.at is None else now-self.at
        if not 0 < dt <= 3/self.config.control_hz:
            return DriveDecision.stop('motion_control_gap')
        # Speed decreases and explicit stops bypass the acceleration ramp.
        vx = min(decision.vx, self.last.vx+self.config.max_accel_mps2*dt)
        vy = float(np.clip(decision.vy, self.last.vy-self.config.max_accel_mps2*dt,
                           self.last.vy+self.config.max_accel_mps2*dt))
        desired_yaw = decision.yaw_rate
        if decision.vy == 0 and decision.vx > 0:
            curvature = decision.yaw_rate/decision.vx
            desired_yaw = curvature*vx
        yaw = float(np.clip(desired_yaw, self.last.yaw_rate-self.config.max_yaw_accel_rps2*dt,
                            self.last.yaw_rate+self.config.max_yaw_accel_rps2*dt))
        if decision.vy == 0 and decision.vx > 0 and abs(yaw-desired_yaw) > 1e-10:
            # Never drive a widening/opposite arc while braking an old yaw.
            # Retain the bounded yaw transition with zero forward speed, then
            # resume when the requested curvature can be achieved.
            if abs(curvature) > 1e-8 and 0 <= yaw/curvature <= vx:
                vx = yaw/curvature
            else:
                vx = 0.
        return replace(decision, vx=max(0., vx), vy=vy, yaw_rate=yaw)

    def record(self, decision, now):
        self.last, self.at = decision, now
        if not (decision.vx or decision.vy or decision.yaw_rate): self.at = None
