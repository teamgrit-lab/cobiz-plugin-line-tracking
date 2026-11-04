"""Payload validation and feedback control for a sidewalk tag-to-tag segment."""
from collections import deque
from dataclasses import dataclass
import math
import threading

from swin_l_drive_control import DriveDecision


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class TagRouteSpec:
    start_tag_id: int
    end_tag_id: int
    tag_size_m: float = .167


def parse_tag_route(payload, selected_mask):
    if not any(key in payload for key in ('start_tag_id', 'end_tag_id')):
        return None
    if selected_mask != 2:
        raise ValueError('tag_route_requires_sidewalk')
    ids = []
    for key in ('start_tag_id', 'end_tag_id'):
        value = payload.get(key)
        if type(value) is not int or not 0 <= value < 65536:
            raise ValueError('invalid_' + key)
        ids.append(value)
    if ids[0] == ids[1]:
        raise ValueError('start_and_end_tag_ids_must_differ')
    size = payload.get('tag_size_m', .167)
    if isinstance(size, bool) or not isinstance(size, (int, float)) or not math.isfinite(size) or not 0 < size <= 2:
        raise ValueError('invalid_tag_size_m')
    return TagRouteSpec(*ids, float(size))


@dataclass(frozen=True)
class BodyPose:
    stamp_sec: float
    received_at: float
    x: float
    y: float
    yaw: float
    sequence: int
    roll: float = 0.
    pitch: float = 0.


def world_vector(vector, pose):
    """Body vector -> gravity/world frame using the captured IMU attitude."""
    x, y, z = vector
    cr, sr, cp, sp, cy, sy = (math.cos(pose.roll), math.sin(pose.roll),
        math.cos(pose.pitch), math.sin(pose.pitch), math.cos(pose.yaw), math.sin(pose.yaw))
    x, y, z = x, cr * y - sr * z, sr * y + cr * z
    x, y, z = cp * x + sp * z, y, -sp * x + cp * z
    return cy * x - sy * y, sy * x + cy * y, z


class BodyPoseHistory:
    """Unitree wxyz attitude + leg odometry, independent of SLAM lifecycle."""
    def __init__(self):
        self.samples = deque(maxlen=600)
        self.lock = threading.Lock()
        self.last_source_stamp = None

    def observe(self, message, stamp_sec, now):
        try:
            q = tuple(float(v) for v in message.imu_state.quaternion)
            x, y = (float(v) for v in message.position[:2])
            if len(q) != 4 or not all(math.isfinite(v) for v in (*q, x, y, stamp_sec, now)):
                return False
            norm = math.sqrt(sum(v * v for v in q))
            if not .5 <= norm <= 1.5:
                return False
            w, qx, qy, qz = (v / norm for v in q)
            yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            roll = math.atan2(2 * (w * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
            pitch = math.asin(max(-1., min(1., 2 * (w * qy - qz * qx))))
            source = getattr(message, 'stamp', None)
            source_stamp = (int(source.sec) * 1_000_000_000 + int(source.nanosec)) if source else 0
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False
        with self.lock:
            if source_stamp > 0 and self.last_source_stamp is not None and source_stamp <= self.last_source_stamp:
                return False
            if self.samples and stamp_sec <= self.samples[-1].stamp_sec:
                return False
            if source_stamp > 0:
                self.last_source_stamp = source_stamp
            sequence = self.samples[-1].sequence + 1 if self.samples else 1
            self.samples.append(BodyPose(stamp_sec, now, x, y, yaw, sequence, roll, pitch))
        return True

    def latest(self):
        with self.lock:
            return self.samples[-1] if self.samples else None

    def at(self, stamp_sec, max_skew_sec=.15):
        with self.lock:
            samples = list(self.samples)
        if not samples:
            return None
        nearest = min(samples, key=lambda p: abs(p.stamp_sec - stamp_sec))
        if abs(nearest.stamp_sec - stamp_sec) > max_skew_sec:
            return None
        before = next((p for p in reversed(samples) if p.stamp_sec <= stamp_sec), None)
        after = next((p for p in samples if p.stamp_sec >= stamp_sec), None)
        if before is None or after is None or before is after or after.stamp_sec - before.stamp_sec > 2 * max_skew_sec:
            return nearest
        fraction = (stamp_sec - before.stamp_sec) / (after.stamp_sec - before.stamp_sec)
        return BodyPose(stamp_sec, before.received_at,
            before.x + fraction * (after.x - before.x), before.y + fraction * (after.y - before.y),
            wrap(before.yaw + fraction * wrap(after.yaw - before.yaw)), before.sequence,
            wrap(before.roll + fraction * wrap(after.roll - before.roll)),
            before.pitch + fraction * (after.pitch - before.pitch))


@dataclass(frozen=True)
class TagRoutePolicy:
    speed_mps: float = .30
    max_yaw_rps: float = .18
    max_age_sec: float = 1.
    pose_max_age_sec: float = .5
    tolerance_deg: float = 3.
    confirm_frames: int = 3
    start_range_m: float = 3.
    end_range_m: float = 3.
    arrival_ahead_m: float = .5
    startup_timeout_sec: float = 5.
    confirm_timeout_sec: float = 5.
    align_timeout_sec: float = 20.
    straight_timeout_sec: float = 120.
    max_cross_track_m: float = .5

    def validate(self):
        values = [v for key, v in self.__dict__.items() if key != 'confirm_frames']
        if (not all(math.isfinite(v) and v > 0 for v in values)
                or self.speed_mps > 1. or self.max_yaw_rps > 1.
                or self.tolerance_deg >= 45 or type(self.confirm_frames) is not int
                or self.confirm_frames < 1):
            raise ValueError('invalid_tag_route_policy')


class TagRouteController:
    """Normal sidewalk -> start confirmation -> parallel turn -> fixed-line drive.

    The line is anchored at the body's position after alignment. Unitree position
    and quaternion feedback correct both heading and cross-track drift. Camera
    heartbeat and feedback remain mandatory after the start tag disappears.
    """
    def __init__(self, spec, *, now, duration_sec, policy=None):
        self.spec, self.policy = spec, policy or TagRoutePolicy()
        self.policy.validate()
        self.started_at, self.duration = now, duration_sec
        self.phase, self.phase_at = 'sidewalk', now
        self.frame_at, self.last_stamp = None, None
        self.target_yaw, self.anchor, self.straight_at = None, None, None
        self.start_hits, self.end_hits, self.align_hits = 0, 0, 0
        self.last_pose_sequence = None
        self.failure = None
        self.cross_track_m = None

    def abort(self, reason):
        self.phase, self.failure = 'aborted', reason

    def observe(self, tags, *, stamp_sec, now, source_age_sec, captured_pose):
        if (not math.isfinite(stamp_sec) or not math.isfinite(source_age_sec)
                or not 0 <= source_age_sec <= self.policy.max_age_sec
                or (self.last_stamp is not None and stamp_sec <= self.last_stamp)):
            return
        self.last_stamp, self.frame_at = stamp_sec, now - source_age_sec
        if self.phase in ('complete', 'aborted', 'align'):
            return
        tag_id = self.spec.start_tag_id if self.phase in ('sidewalk', 'start_confirm') else self.spec.end_tag_id
        matches = [tag for tag in tags if tag.get('tag_id') == tag_id]
        tag = matches[0] if len(matches) == 1 else None
        if tag is None or captured_pose is None:
            self.start_hits = self.end_hits = 0
            return
        distance = tag.get('range_m')
        if not isinstance(distance, (int, float)) or not math.isfinite(distance) or distance <= 0:
            self.start_hits = self.end_hits = 0
            return
        if self.phase in ('sidewalk', 'start_confirm'):
            error = tag.get('parallel_error_deg')
            if (tag.get('pose_status') != 'ok' or distance > self.policy.start_range_m
                    or not isinstance(error, (int, float)) or not math.isfinite(error)
                    or abs(error) >= 85.):
                self.start_hits = 0
                return
            target = wrap(captured_pose.yaw - math.radians(error))
            normal = tag.get('normal_body')
            if isinstance(normal, list) and len(normal) == 3 and all(math.isfinite(v) for v in normal):
                nx, ny, _ = world_vector(normal, captured_pose)
                if math.hypot(nx, ny) < .3:
                    self.start_hits = 0
                    return
                tangent = math.atan2(ny, nx) + math.pi / 2
                delta = (tangent - captured_pose.yaw + math.pi / 2) % math.pi - math.pi / 2
                if abs(delta) >= math.radians(85.):
                    self.start_hits = 0
                    return
                target = wrap(captured_pose.yaw + delta)
            if self.phase == 'sidewalk':
                self.phase, self.phase_at = 'start_confirm', now
            consistent = self.target_yaw is not None and abs(wrap(target - self.target_yaw)) <= math.radians(self.policy.tolerance_deg)
            self.start_hits = self.start_hits + 1 if consistent else 1
            self.target_yaw = target
            if self.start_hits >= self.policy.confirm_frames:
                self.phase, self.phase_at = 'align', now
                self.align_hits = 0
        elif self.phase in ('straight', 'end_confirm'):
            position = tag.get('position_body_m')
            if (distance > self.policy.end_range_m or not isinstance(position, list) or len(position) != 3
                    or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in position)):
                self.end_hits = 0
                return
            px, py, _ = world_vector(position, captured_pose)
            along = math.cos(self.target_yaw) * px + math.sin(self.target_yaw) * py
            if abs(along) > self.policy.arrival_ahead_m:
                self.end_hits = 0
                return
            if self.phase == 'straight':
                self.phase, self.phase_at = 'end_confirm', now
            self.end_hits += 1
            if self.end_hits >= self.policy.confirm_frames:
                self.phase = 'complete'

    def control(self, *, now, pose):
        if self.phase in ('aborted', 'complete'):
            return DriveDecision.stop(self.failure or 'tag_route_complete')
        if now - self.started_at >= self.duration:
            self.abort('tag_route_task_timeout')
        frame_ready = self.frame_at is not None and 0 <= now - self.frame_at <= self.policy.max_age_sec
        pose_ready = pose is not None and 0 <= now - pose.received_at <= self.policy.pose_max_age_sec
        if self.phase != 'aborted' and not (frame_ready and pose_ready):
            if self.phase == 'sidewalk' and now - self.started_at < self.policy.startup_timeout_sec:
                return DriveDecision.stop('tag_route_inputs_pending')
            self.abort('tag_route_camera_stale' if not frame_ready else 'tag_route_pose_stale')
        if self.phase == 'aborted':
            return DriveDecision.stop(self.failure)
        if self.phase == 'sidewalk':
            return None
        if self.phase in ('start_confirm', 'end_confirm'):
            if now - self.phase_at >= self.policy.confirm_timeout_sec:
                self.abort('tag_route_confirmation_timeout')
            return DriveDecision.stop(self.failure or 'tag_route_' + self.phase)
        error = wrap(self.target_yaw - pose.yaw)
        if self.phase == 'align':
            if now - self.phase_at >= self.policy.align_timeout_sec:
                self.abort('tag_route_alignment_timeout')
                return DriveDecision.stop(self.failure)
            within = abs(error) <= math.radians(self.policy.tolerance_deg)
            if pose.sequence != self.last_pose_sequence:
                self.align_hits = self.align_hits + 1 if within else 0
                self.last_pose_sequence = pose.sequence
            if self.align_hits >= self.policy.confirm_frames:
                self.phase, self.phase_at, self.straight_at = 'straight', now, now
                self.anchor = pose.x, pose.y
                return DriveDecision.stop('tag_route_aligned')
            yaw = 0. if within else max(-self.policy.max_yaw_rps, min(self.policy.max_yaw_rps, error))
            return DriveDecision(0., 0., yaw, 'tag_route_align')
        if now - self.straight_at >= self.policy.straight_timeout_sec:
            self.abort('tag_route_straight_timeout')
            return DriveDecision.stop(self.failure)
        dx, dy = pose.x - self.anchor[0], pose.y - self.anchor[1]
        self.cross_track_m = -math.sin(self.target_yaw) * dx + math.cos(self.target_yaw) * dy
        if abs(self.cross_track_m) > self.policy.max_cross_track_m:
            self.abort('tag_route_cross_track_limit')
            return DriveDecision.stop(self.failure)
        desired = self.target_yaw - math.atan(self.cross_track_m)
        correction = wrap(desired - pose.yaw)
        yaw = max(-self.policy.max_yaw_rps, min(self.policy.max_yaw_rps, correction))
        speed = self.policy.speed_mps if abs(error) <= math.radians(20) else 0.
        return DriveDecision(speed, 0., yaw, 'tag_route_straight')

    def metrics(self):
        return dict(phase=self.phase, start_tag_id=self.spec.start_tag_id, end_tag_id=self.spec.end_tag_id,
                    start_hits=self.start_hits, end_hits=self.end_hits, alignment_hits=self.align_hits,
                    target_yaw_rad=self.target_yaw, cross_track_m=self.cross_track_m, failure=self.failure)
