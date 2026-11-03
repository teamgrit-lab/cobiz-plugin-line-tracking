"""Observed dead-end corners, separate from blind path-loss search.

Only a connected side exit and an observed end of the incoming corridor can
arm a turn. A latched turn uses measured odom yaw, not integrated commands.
Metric geometry inherits the configured camera-to-ground calibration.
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np


def _runs(row):
    indices = np.flatnonzero(row)
    if not len(indices):
        return []
    groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
    return [(int(g[0]), int(g[-1])) for g in groups]


def detect_corner(mask, xs, ys, preference="center", observed=None):
    """Find an L/T exit; image crops and a still-open forward road are not corners.

    The return value is evidence, not permission to move. Detection operates on
    the clearance-eroded, connected mask. Footprint validation uses raw ground.
    """
    mask = np.asarray(mask, bool)
    xs, ys = np.asarray(xs), np.asarray(ys)
    if len(xs) < 8 or len(ys) < 8:
        return None
    seed_row = next((r for r in range(min(12, len(xs))) if mask[r].any()), None)
    if seed_row is None:
        return None
    # An off-center/curved forward continuation must not become a false dead
    # end just because it leaves the incoming center's constant-y ray.
    minimum_columns = max(2, int(math.ceil(.6/abs(ys[1]-ys[0]))))
    if any(b-a+1 >= minimum_columns for row in mask[-3:] for a, b in _runs(row)):
        return None
    a, b = min(_runs(mask[seed_row]), key=lambda ab: abs((ys[ab[0]] + ys[ab[1]]) / 2))
    center = float((ys[a] + ys[b]) / 2)
    band = abs(ys - center) <= max(abs(ys[1]-ys[0]), min(.25, abs(ys[a]-ys[b]) / 4))
    forward = np.mean(mask[:, band], axis=1) >= .8
    missing = np.flatnonzero(~forward & (np.arange(len(xs)) > seed_row + 3))
    # Three successive missing rows reject isolated segmentation holes.
    end = next((int(r) for r in missing if r + 2 < len(xs) and not forward[r:r+3].any()), None)
    if end is None or end >= len(xs)-3 or xs[end]-xs[seed_row] < .5:
        return None
    col = int(np.argmin(abs(ys-center)))
    if observed is not None:
        # Erosion also shrinks a cropped image's apparent road. Demand an
        # observed band beyond that shrinkage before calling it a road end.
        beyond = (xs >= xs[end]) & (xs <= xs[end]+.8)
        if xs[end]+.8 > xs[-1] or not np.all(np.asarray(observed, bool)[beyond][:, band]):
            return None
    candidates = []
    for sign, direction, edge in ((1, "left", ys[a]), (-1, "right", ys[b])):
        columns = np.flatnonzero(sign * (ys-edge) >= 1.5)
        columns = columns[np.argsort(-sign * ys[columns])]
        for c in columns:
            groups = [(u, v) for u, v in _runs(mask[:, c])
                      if u > 0 and v < len(xs)-1 and xs[v]-xs[u] >= .8
                      and xs[u] < xs[end] and xs[v] > xs[seed_row]+.5
                      and (observed is None or (observed[u-1, c] and observed[v+1, c]))]
            if not groups:
                continue
            u, v = min(groups, key=lambda ab: abs((xs[ab[0]]+xs[ab[1]])/2-xs[end]))
            pivot_x = float((xs[u]+xs[v])/2)
            row = int(np.argmin(abs(xs-pivot_x)))
            between = (ys >= min(center, ys[c])) & (ys <= max(center, ys[c]))
            if not np.all(mask[row, between]):
                continue
            # The centerline of the transverse exit determines its heading.
            columns_between = np.flatnonzero(between & (sign*(ys-edge) > .2))
            mids, lateral = [], []
            for cc in columns_between[::max(1, len(columns_between)//20)]:
                segments = [(i, j) for i, j in _runs(mask[:, cc]) if i <= row <= j]
                if segments:
                    i, j = segments[0]
                    mids.append(float((xs[i]+xs[j])/2)); lateral.append(float(ys[cc]))
            if len(mids) < 3:
                continue
            slope = float(np.polyfit(lateral, mids, 1)[0])
            heading = math.atan2(sign, sign*slope)
            if not math.radians(55) <= abs(heading) <= math.radians(125):
                continue
            candidates.append({"direction": direction, "pivot_xy": [pivot_x, center],
                               "exit_xy": [pivot_x, float(ys[c])],
                               "exit_heading_rad": heading,
                               "path_xy": [[float(xs[seed_row]), center], [pivot_x, center],
                                           [pivot_x, float(ys[c])]],
                               "ambiguous": False})
            break
    if not candidates:
        return None
    if len(candidates) > 1:
        if preference == "center":
            return {"ambiguous": True, "directions": [c["direction"] for c in candidates],
                    "candidates": candidates}
        return next(c for c in candidates if c["direction"] == preference)
    return candidates[0]


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _world(points, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + np.asarray(pose[:2])


def _body(points, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (np.asarray(points)-np.asarray(pose[:2])) @ np.array([[c, -s], [s, c]])


@dataclass(frozen=True)
class CornerConfig:
    enabled: bool = True
    confirm_frames: int = 2
    max_age_sec: float = 1.5
    # A 3m camera blind zone takes 15s to traverse at the approach speed;
    # retain that ground long enough for the subsequent ~9s 90-degree turn.
    history_sec: float = 40.0
    approach_mps: float = .20
    pivot_tolerance_m: float = .20
    heading_tolerance_deg: float = 8.0
    timeout_sec: float = 45.0
    prediction_sec: float = 1.0
    brake_mps2: float = .50
    confirm_sec: float = 1.0
    confirm_motion_m: float = .50
    max_pivot_drift_m: float = .25

    def validate(self):
        if type(self.enabled) is not bool or type(self.confirm_frames) is not int or self.confirm_frames < 2:
            raise ValueError("corner enabled/confirmation settings are invalid")
        for name, value in vars(self).items():
            if name not in ("enabled", "confirm_frames") and (not math.isfinite(value) or value <= 0):
                raise ValueError("corner " + name + " must be finite and positive")
        if self.heading_tolerance_deg >= 30 or self.history_sec < self.max_age_sec:
            raise ValueError("invalid corner heading/history limits")


class CornerNavigator:
    """Confirmed approach/rotate/exit with fresh sensors and swept ground checks.

    Calls share the ROS state lock. Observations are timestamped at capture;
    repeated control ticks cannot confirm a candidate or an exit. New observed
    exclusions override historical free space. Unknown footprint space stops.
    """
    def __init__(self, config=None, footprint=None):
        self.config = config or CornerConfig()
        self.config.validate()
        # Same defaults as ObstacleConfig; callers pass the configured values.
        self.footprint = footprint or (.50, .45, .30, .15)
        if len(self.footprint) != 4 or not all(math.isfinite(v) and v > 0 for v in self.footprint):
            raise ValueError('corner footprint dimensions must be finite and positive')
        self.reset()

    def reset(self):
        self.phase = "idle"
        self.reason = "idle"
        self.frames = self.exit_frames = self.completed = 0
        self.observations = deque(maxlen=256)
        self.last_frame = self.observed_at = self.last_pose = None
        self.candidate = self.target = self.pivot = None
        self.started = self.cooldown_until = 0.
        self.command = (0., 0., 0.)
        self.command_at = None
        self.last_path_heading = None
        self.trajectory_guard = 'not_evaluated'

    @property
    def active(self):
        return self.phase in ("approach", "rotate", "exit_confirm", "failed")

    def observe(self, result, now, *, pose=None, observed_at=None, frame_key=None):
        if not self.config.enabled:
            return
        observed_at = now if observed_at is None else observed_at
        key = now if frame_key is None else frame_key
        if self.last_frame is not None and key <= self.last_frame:
            return
        if not math.isfinite(observed_at) or not 0 <= now-observed_at <= self.config.max_age_sec:
            return
        self.last_frame, self.observed_at = key, observed_at
        pose = None if pose is None else np.asarray(pose, float)
        if pose is not None and (pose.shape != (3,) or not np.isfinite(pose).all()):
            pose = None
        if pose is not None and self.last_pose is not None:
            delta = np.linalg.norm(pose[:2]-self.last_pose[0][:2])
            dt = observed_at-self.last_pose[1]
            if dt <= 0 or delta > 3*dt+.25 or abs(_wrap(pose[2]-self.last_pose[0][2])) > 3*dt+.25:
                self.phase, self.reason = "failed", "corner_pose_discontinuity"
                self.observations.clear()
                return
        if pose is not None:
            self.last_pose = (pose, observed_at)
            observation = (observed_at, pose, result)
            # Preserve at most four snapshots/s, always keeping the newest
            # exclusion. High inference rates must not evict the needed ground
            # history before a slow approach and turn have finished.
            if (self.observations and int(observed_at*4) == int(self.observations[-1][0]*4)):
                self.observations[-1] = observation
            else:
                self.observations.append(observation)
        while self.observations and now-self.observations[0][0] > self.config.history_sec:
            self.observations.popleft()
        estimate = result.estimate
        self.last_path_heading = ((estimate.road_geometry or {}).get("direction_heading_rad")
                                  if estimate is not None else None)
        if self.phase == "exit_confirm":
            aligned = (self.last_path_heading is not None and estimate.confidence >= .49
                       and abs(self.last_path_heading) <= math.radians(self.config.heading_tolerance_deg))
            if aligned and pose is not None:
                points = _world(estimate.points_xy, pose)-self.pivot
                direction = np.array([math.cos(self.target), math.sin(self.target)])
                normal = np.array([-direction[1], direction[0]])
                aligned = (np.median(abs(points @ normal)) <= .75
                           and np.median(points @ direction) > .5)
            else:
                aligned = False
            self.exit_frames = self.exit_frames+1 if aligned else 0
            return
        if self.active or now < self.cooldown_until:
            return
        corner = result.corner
        if corner is None:
            self.candidate = None
            self.frames = 0
            self.reason = "idle"
            return
        if corner.get("ambiguous"):
            corner = {**corner['candidates'][0], **corner}
        self.candidate = {**corner, "pose": pose, "observed_at": observed_at,
                          "world_pivot": _world([corner["pivot_xy"]], pose)[0] if pose is not None else None,
                          "world_heading": pose[2]+corner["exit_heading_rad"] if pose is not None else None,
                          "previous": self.candidate}
        previous = self.candidate.pop("previous")
        consistent = bool(previous and previous["direction"] == corner["direction"]
                          and previous.get("ambiguous", False) == corner.get("ambiguous", False)
                          and (previous['pose'] is None) == (pose is None)
                          and observed_at-previous["observed_at"] <= self.config.max_age_sec)
        if consistent:
            if pose is not None and previous["world_pivot"] is not None:
                consistent = (np.linalg.norm(self.candidate["world_pivot"]-previous["anchor_pivot"]) <= self.config.max_pivot_drift_m
                              and abs(_wrap(self.candidate["world_heading"]-previous["world_heading"])) < math.radians(15))
            else:
                consistent = np.linalg.norm(np.array(corner["pivot_xy"])-previous["pivot_xy"]) < .5
        self.frames = self.frames+1 if consistent else 1
        self.candidate.update({
            'anchor_pivot': previous['anchor_pivot'] if consistent else self.candidate['world_pivot'],
            'anchor_pose': previous['anchor_pose'] if consistent else pose,
            'anchor_at': previous['anchor_at'] if consistent else observed_at})
        self.reason = "corner_confirming"
        if self.frames >= self.config.confirm_frames:
            if pose is None:
                self.reason = "corner_observed_no_pose"
                return
            # A horizon/projection artifact follows the camera. Confirm a
            # fixed world pivot across measured translation, using the FIRST
            # anchor so small framewise drift cannot accumulate unnoticed.
            moved = np.linalg.norm(pose[:2]-self.candidate['anchor_pose'][:2])
            parallax_confirmed = (observed_at-self.candidate['anchor_at'] >= self.config.confirm_sec
                                  and moved >= self.config.confirm_motion_m)
            at_pivot = np.linalg.norm(self.candidate['world_pivot']-pose[:2]) <= self.config.pivot_tolerance_m
            if not parallax_confirmed and not at_pivot:
                return
            if corner.get('ambiguous'):
                self.reason = "corner_exit_ambiguous"
                return
            # Do not enter an approach that must immediately stop on the
            # startup camera blind zone. Keep ordinary reference following
            # until measured observation history covers the current footprint.
            if not self._ground_allowed(_world(self._footprint_samples(), pose), now):
                return
            self.pivot = self.candidate["world_pivot"].copy()
            self.target = self.candidate["world_heading"]
            self.incoming_heading = pose[2]
            self.started, self.phase = now, "approach"
            self.reason = "corner_approach"

    def _ground_status(self, world_points, now):
        known, allowed = np.zeros(len(world_points), bool), np.zeros(len(world_points), bool)
        for stamp, pose, observation in reversed(self.observations):
            if now-stamp > self.config.history_sec:
                continue
            p = _body(world_points, pose)
            xs, ys = observation.x_values, observation.y_values
            inside = ((p[:, 0] >= xs[0]) & (p[:, 0] <= xs[-1]) &
                      (p[:, 1] >= ys[-1]) & (p[:, 1] <= ys[0]) & ~known)
            indices = np.flatnonzero(inside)
            if not len(indices):
                continue
            rows = np.rint(np.interp(p[indices, 0], xs, np.arange(len(xs)))).astype(int)
            cols = np.rint(np.interp(p[indices, 1], ys[::-1], np.arange(len(ys))[::-1])).astype(int)
            observed = observation.observed_mask
            visible = np.ones(len(indices), bool) if observed is None else observed[rows, cols] > 0
            indices, rows, cols = indices[visible], rows[visible], cols[visible]
            surface = observation.footprint_mask
            if surface is None:
                surface = observation.allowed_mask
            known[indices] = True
            allowed[indices] = surface[rows, cols] > 0
            if known.all():
                break
        return bool(np.all(known)), bool(np.all(known & allowed))

    def _ground_allowed(self, world_points, now):
        return self._ground_status(world_points, now)[1]

    def _footprint_samples(self):
        front, back, half_width, margin = self.footprint
        gx, gy = np.meshgrid(np.arange(-back-margin, front+margin+.05, .08),
                             np.arange(-half_width-margin, half_width+margin+.05, .08))
        return np.column_stack((gx.ravel(), gy.ravel()))

    def motion_allowed(self, command, pose, now, drive_config):
        footprint = self._footprint_samples()
        position = np.asarray(pose, float).copy()
        previous = np.array(self.command, float)
        requested = np.array(command, float)
        # Account for the previously published command while braking/turning.
        horizon = max(self.config.prediction_sec, np.linalg.norm(previous[:2])/self.config.brake_mps2+.25)
        points = [_world(footprint, position)]
        for _ in range(int(math.ceil(horizon/.05))):
            previous[:2] += np.clip(requested[:2]-previous[:2], -self.config.brake_mps2*.05,
                                    self.config.brake_mps2*.05)
            previous[2] += np.clip(requested[2]-previous[2], -drive_config.road_max_yaw_accel*.05,
                                   drive_config.road_max_yaw_accel*.05)
            c, s = math.cos(position[2]), math.sin(position[2])
            position[:2] += .05*np.array([c*previous[0]-s*previous[1], s*previous[0]+c*previous[1]])
            position[2] += previous[2]*.05
            points.append(_world(footprint, position))
        return self._ground_allowed(np.concatenate(points), now)

    def _guard_tracking(self, nominal, now, pose, drive_config):
        from swin_l_drive_control import DriveDecision
        self.trajectory_guard = 'not_evaluated'
        if nominal.reason not in ('tracking', 'tracking_slow_turn'):
            return nominal
        if pose is None or len(pose) != 3 or not np.isfinite(pose).all():
            self.trajectory_guard = 'no_odom'
            return nominal
        known, safe = self._ground_status(_world(self._footprint_samples(), pose), now)
        if not known:
            # The legacy camera calibration starts at 3m. Do not claim a
            # near-body check or invent free ground to bridge that blind area.
            # Ordinary reference following remains available until odom-aligned
            # observation history covers it. Planned rotations always require it.
            self.trajectory_guard = 'near_ground_unobserved'
            return nominal
        if not safe:
            self.trajectory_guard = 'body_outside_observed_surface'
            return DriveDecision.stop('road_footprint_outside')
        for scale in (1., .5, .25):
            candidate = (nominal.vx*scale, nominal.vy, nominal.yaw_rate)
            if self.motion_allowed(candidate, pose, now, drive_config):
                self.trajectory_guard = 'allowed' if scale == 1 else 'slowed'
                return DriveDecision(*candidate, nominal.reason if scale == 1 else 'tracking_slow_turn')
        self.trajectory_guard = 'no_verified_trajectory'
        return DriveDecision.stop('road_trajectory_unverified')

    def _limited(self, command, now, drive_config):
        dt = min(.2, max(0., now-self.command_at)) if self.command_at is not None else 0.
        vx, vy, yaw = command
        vy = float(np.clip(vy, self.command[1]-drive_config.road_max_lateral_accel*dt,
                            self.command[1]+drive_config.road_max_lateral_accel*dt))
        yaw = float(np.clip(yaw, self.command[2]-drive_config.road_max_yaw_accel*dt,
                             self.command[2]+drive_config.road_max_yaw_accel*dt))
        return vx, vy, yaw

    def decide(self, nominal, path, *, now, pose, drive_config, sensors_ready=True):
        from swin_l_drive_control import DriveDecision
        if not self.config.enabled:
            return nominal
        if not self.active:
            if self.reason in ("corner_pose_unavailable", "corner_exit_ambiguous"):
                return DriveDecision.stop(self.reason)
            return self._guard_tracking(nominal, now, pose, drive_config)
        # Never override an external/path/branch safety guard. A missing local
        # fit is expected when an observed L exit leaves y=f(x)'s domain.
        allowed_reasons = ("tracking", "tracking_slow_turn", "road_path_unconfirmed",
                           "waiting_for_path", "path_recovery_waiting")
        fit_wait = nominal.reason in (
            "path_execution_waiting_initial_candidate", "path_execution_waiting_next_candidate",
            "path_execution_waiting_connected_candidate", "path_execution_waiting_replan_candidate",
            "path_execution_invalidated", "path_execution_observation_expired")
        if nominal.reason not in allowed_reasons and not fit_wait:
            return nominal
        if self.phase == "failed":
            return DriveDecision.stop(self.reason)
        if not sensors_ready or self.observed_at is None or not 0 <= now-self.observed_at <= self.config.max_age_sec:
            return DriveDecision.stop("corner_inputs_stale")
        if pose is None or len(pose) != 3 or not np.isfinite(pose).all():
            return DriveDecision.stop("corner_pose_unavailable")
        if now-self.started > self.config.timeout_sec:
            self.phase, self.reason = "failed", "corner_timeout"
            return DriveDecision.stop(self.reason)
        pivot = _body([self.pivot], pose)[0]
        error = _wrap(self.target-pose[2])
        if self.phase == "approach":
            if np.linalg.norm(pivot) <= self.config.pivot_tolerance_m:
                self.phase = "rotate"
            elif pivot[0] < -.3:
                self.phase, self.reason = "failed", "corner_pivot_missed"
                return DriveDecision.stop(self.reason)
            else:
                vx = min(drive_config.max_forward_mps, self.config.approach_mps, max(0., pivot[0]*.5))
                vy = float(np.clip(pivot[1]*.4, -drive_config.road_max_lateral_mps, drive_config.road_max_lateral_mps))
                yaw = float(np.clip(_wrap(self.incoming_heading-pose[2]), -drive_config.max_yaw_rps, drive_config.max_yaw_rps))
                command = self._limited((vx, vy, yaw), now, drive_config)
                if not self.motion_allowed(command, pose, now, drive_config):
                    return DriveDecision.stop("corner_space_unverified")
                return DriveDecision(*command, "corner_approach")
        if self.phase == "rotate":
            if abs(error) <= math.radians(self.config.heading_tolerance_deg):
                self.phase, self.exit_frames = "exit_confirm", 0
            else:
                yaw = float(np.clip(drive_config.heading_gain*error, -drive_config.max_yaw_rps, drive_config.max_yaw_rps))
                _, _, yaw = self._limited((0., 0., yaw), now, drive_config)
                if not self.motion_allowed((0., 0., yaw), pose, now, drive_config):
                    return DriveDecision.stop("corner_space_unverified")
                return DriveDecision(0., 0., yaw, "corner_rotate")
        if self.phase == "exit_confirm":
            if abs(error) > math.radians(self.config.heading_tolerance_deg)*1.5:
                self.phase, self.exit_frames = "rotate", 0
            elif self.exit_frames >= self.config.confirm_frames:
                self.phase, self.reason = "idle", "corner_completed"
                self.completed += 1
                self.cooldown_until, self.candidate, self.frames = now+2., None, 0
                return nominal
            return DriveDecision.stop("corner_exit_confirming")
        return DriveDecision.stop("corner_exit_confirming")

    def record_command(self, command, now):
        self.command = (command.vx, command.vy, command.yaw_rate)
        self.command_at = now

    def metrics(self):
        candidate = ({k: v for k, v in self.candidate.items()
                      if k in ("direction", "pivot_xy", "exit_xy", "exit_heading_rad", "path_xy", "ambiguous")}
                     if self.candidate else None)
        return {"enabled": self.config.enabled, "state": self.phase, "reason": self.reason,
                "confirmations": self.frames, "exit_confirmations": self.exit_frames,
                "completed": self.completed, "candidate": candidate,
                "odom_required": True, "history_frames": len(self.observations),
                "trajectory_guard": self.trajectory_guard}
