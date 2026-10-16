"""Commit road-only path sections independently of perception and control."""
from dataclasses import dataclass, replace
import math
import threading

import numpy as np

from local_path import (LocalPathSmoother, RoadPathResult, SmoothedPath,
                        _birdseye_sidewalk, compensate_road_path)


@dataclass(frozen=True)
class PathExecutionConfig:
    enabled: bool = True
    segment_m: float = 1.0
    segment_sec: float = 1.5
    confirm_frames: int = 2
    candidate_max_age_sec: float = 1.5
    max_unverified_sec: float = 2.5
    join_length_m: float = 1.0
    max_join_offset_m: float = .6
    max_join_heading_deg: float = 25.
    replan_wait_sec: float = 1.0

    def validate(self):
        if type(self.enabled) is not bool:
            raise ValueError('path execution enabled must be boolean')
        if type(self.confirm_frames) is not int or self.confirm_frames < 2:
            raise ValueError('path execution confirm_frames must be an integer >= 2')
        for name in ('segment_m', 'segment_sec', 'candidate_max_age_sec',
                     'max_unverified_sec', 'join_length_m', 'max_join_offset_m',
                     'max_join_heading_deg', 'replan_wait_sec'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'path execution {name} must be finite and positive')
        if self.max_join_heading_deg >= 90:
            raise ValueError('path execution max_join_heading_deg must be below 90')


def _visible(points, xs):
    """Clip only to the observed forward range, without lateral clipping."""
    p = np.asarray(points, dtype=float)
    if (p.ndim != 2 or p.shape[1] != 2 or len(p) < 2 or
            not np.isfinite(p).all() or np.any(np.diff(p[:, 0]) <= 0)):
        return None
    lo, hi = max(xs[0], p[0, 0]), min(xs[-1], p[-1, 0])
    rows = np.asarray(xs)[(xs >= lo) & (xs <= hi)]
    if len(rows) < 2:
        return None
    lo, hi = rows[0], rows[-1]
    if hi - lo < .1:
        return None
    x = np.unique(np.r_[lo, p[(p[:, 0] > lo) & (p[:, 0] < hi), 0], hi])
    return np.column_stack((x, np.interp(x, p[:, 0], p[:, 1]))).astype(np.float32)


@dataclass(frozen=True)
class PathEvidence:
    region: RoadPathResult
    hazards: np.ndarray | None = None
    observed_at: float | None = None
    frame_key: int | None = None

    def corridor(self, points):
        visible = _visible(points, self.region.x_values)
        return self.region.corridor_for(visible) if visible is not None else None

    def blocked(self, points):
        if self.region.blocked:
            return True
        visible = _visible(points, self.region.x_values)
        if visible is None or self.hazards is None:
            return False
        xs, ys = self.region.x_values, self.region.y_values
        rows = np.flatnonzero((xs >= visible[0, 0]) & (xs <= visible[-1, 0]))
        y = np.interp(xs[rows], visible[:, 0], visible[:, 1])
        inside = (y >= ys[-1]) & (y <= ys[0])
        cols = np.rint(np.interp(y, ys[::-1], np.arange(len(ys))[::-1])).astype(int)
        return bool(np.any(self.hazards[rows[inside], cols[inside]]))


def surface_evidence(region, config, hazards=None, observed_at=None, frame_key=None):
    allowed, xs, ys = _birdseye_sidewalk(region, config)
    blocked = None
    if hazards is not None:
        blocked, _, _ = _birdseye_sidewalk(hazards, replace(config, close_kernel_px=0))
        allowed = np.where(blocked > 0, 0, allowed).astype(np.uint8)
    return PathEvidence(RoadPathResult(None, 'observed', 'surface_observation', {},
                                      allowed, xs, ys), blocked, observed_at, frame_key)


def _difference(a, b):
    lo, hi = max(a[0, 0], b[0, 0]), min(a[-1, 0], b[-1, 0])
    if hi - lo < .3:
        return math.inf, math.inf
    end = min(hi, lo + 1.)
    x = np.linspace(lo, end, 20)
    ay, by = np.interp(x, a[:, 0], a[:, 1]), np.interp(x, b[:, 0], b[:, 1])
    angle = abs(math.atan2(ay[-1]-ay[0], end-lo) -
                math.atan2(by[-1]-by[0], end-lo))
    return float(np.max(abs(ay-by))), math.degrees(angle)


class PathExecution:
    """Smoother-compatible active path and independently confirmed candidate.

    Fresh observations can invalidate a section immediately. Ordinary new
    candidates cannot replace it until its distance/time boundary is reached.
    All access is locked because inference and control run on different threads.
    """
    def __init__(self, local_config, execution_config, mask_class):
        execution_config.validate()
        if mask_class != 1:
            raise ValueError('path section execution is only supported for road mode 1')
        self.config = local_config
        self.execution = execution_config
        self.mask_class = mask_class
        self.smoother = LocalPathSmoother(local_config)
        self._lock = threading.RLock()
        self.reset()

    def reset(self, *, preserve_branch=False):
        with self._lock:
            self.smoother.reset(preserve_branch=preserve_branch)
            self._active = self._candidate = self._last_candidate = None
            self._evidence = None
            self._candidate_at = self._committed_at = self._verified_at = None
            self._last_offer_at = None
            self._last_frame_key = None
            self._hits = self._generation = 0
            self._progress = 0.
            self._last_pose = self._last_pose_at = None
            self._progress_source = 'time'
            self._guard = None
            self._path_loss_at = None
            self._waiting_at = None
            self._reason = 'waiting_initial_candidate'
            self._adoptions = 0

    def _relative(self, path, pose):
        return compensate_road_path(path, pose) if self.mask_class == 1 else path

    def update_road(self, result, timestamp_sec, observation_pose=None, *, observed_at=None, frame_key=None):
        with self._lock:
            candidate = self.smoother.update_road(result, timestamp_sec, observation_pose)
            return self._observe(candidate if result.estimate is not None else None,
                                 timestamp_sec, PathEvidence(result, observed_at=observed_at,
                                                            frame_key=frame_key), observation_pose)

    def update(self, estimate, timestamp_sec, *, evidence=None):
        with self._lock:
            candidate = self.smoother.update(estimate, timestamp_sec)
            return self._observe(candidate if estimate is not None else None,
                                 timestamp_sec, evidence, None)

    def _section(self, points):
        end = min(points[-1, 0], points[0, 0]+max(2., self.execution.segment_m+self.execution.join_length_m))
        x = np.unique(np.r_[points[points[:, 0] < end, 0], end])
        return np.column_stack((x, np.interp(x, points[:, 0], points[:, 1]))).astype(np.float32)

    def _prepare(self, candidate, evidence):
        if candidate is None or candidate.stop_reason is not None:
            return candidate
        if evidence is None:
            return candidate
        if evidence.blocked(self._section(candidate.points_xy)):
            return None
        if evidence.corridor(self._section(candidate.points_xy)) is not None:
            return candidate
        return None

    def _observe(self, candidate, now, evidence, pose):
        # Only inference completions count; output ticks never confirm a path.
        if self._last_offer_at is not None and now <= self._last_offer_at:
            return self.current(now)
        if evidence is not None and evidence.frame_key is not None:
            if self._last_frame_key is not None and evidence.frame_key <= self._last_frame_key:
                return self.current(now)
        self._last_offer_at = now
        observed_at = evidence.observed_at if evidence is not None and evidence.observed_at is not None else now
        if not math.isfinite(observed_at) or not 0 <= now-observed_at <= self.execution.candidate_max_age_sec:
            self._candidate = self._last_candidate = None
            self._hits = 0
            return self.current(now)
        if evidence is not None and evidence.frame_key is not None:
            self._last_frame_key = evidence.frame_key
        self._evidence = evidence
        self._guard = None
        self._path_loss_at = None
        if self.mask_class == 1 and self._active is not None:
            previous_pose = (self._active.road_geometry or {}).get('observation_pose')
            if (previous_pose is None) != (pose is None):
                # A TF outage/reappearance cannot align the old section with
                # this image. Confirm new observations in the new frame regime.
                self._active = self._candidate = self._last_candidate = None
                self._hits = 0
                self._reason = 'waiting_replan_candidate'
        active = self._relative(self._active, pose) if self._active is not None else None
        if candidate is not None and candidate.stop_reason is not None:
            self._active = self._candidate = self._last_candidate = None
            self._hits = 0
            self._guard = candidate
            return self.current(now)
        if self._active is not None and (active is None or
                evidence is not None and evidence.blocked(active.points_xy)):
            self._active = self._candidate = self._last_candidate = None
            self._hits = 0
            self._reason = 'invalidated'
            return self.current(now)
        if active is not None and evidence is not None:
            corridor = evidence.corridor(active.points_xy)
            if corridor is not None:
                self._verified_at = observed_at
                # Rebase only road history to the new image's robot frame.
                self._active = replace(active, road_corridor=corridor if self.mask_class == 1 else None)
            elif self.mask_class == 1:
                # A fresh allowed corridor excluded the committed road path.
                # Calculation failure never grants blind driving permission.
                self._active = None
                self._reason = 'invalidated'
        candidate = self._prepare(candidate, evidence)
        if candidate is None or len(candidate.points_xy) < 2:
            self._candidate = self._last_candidate = None
            self._hits = 0
            if self._active is None and (evidence is None or not evidence.region.blocked):
                # Only a fresh missing path can request a search. Positive
                # blockage, branch/pose guards and candidate waits cannot.
                self._path_loss_at = now
        else:
            previous = self._relative(self._last_candidate, pose) if self._last_candidate is not None else None
            distance, angle = _difference(previous.points_xy, candidate.points_xy) if previous is not None else (math.inf, math.inf)
            recent = (self._candidate_at is not None and
                      0 < observed_at-self._candidate_at <= self.execution.candidate_max_age_sec)
            self._hits = self._hits+1 if recent and distance <= .35 and angle <= 15. else 1
            self._candidate = self._last_candidate = candidate
            self._candidate_at = observed_at
        if self._active is None and self._ready(now):
            self._commit(candidate, now)
        return self.current(now)

    @property
    def path_loss_at(self):
        """Accepted inference completion that lost the usable road reference."""
        with self._lock:
            if self._active is not None or self._guard is not None or self._reason == 'pose_discontinuity':
                return None
            return self._path_loss_at

    def recovery_candidate(self, now):
        """A fresh safe observation for search confirmation, never for motion.

        The search may stop rotating on its first candidate. Motion still waits
        for both this executor's commitment and the search's fresh-frame checks.
        """
        with self._lock:
            if (self._active is not None or self._guard is not None or
                    self._reason == 'pose_discontinuity' or self._candidate is None or
                    not 0 <= now-self._candidate_at <= self.execution.candidate_max_age_sec):
                return None
            return self._candidate

    def _ready(self, now):
        return (self._candidate is not None and self._hits >= self.execution.confirm_frames
                and 0 <= now-self._candidate_at <= self.execution.candidate_max_age_sec)

    def _commit(self, path, now):
        self._active = replace(path, points_xy=self._section(path.points_xy))
        self._committed_at = now
        self._verified_at = self._candidate_at
        self._candidate = None
        self._progress = 0.
        self._generation += 1
        self._adoptions += 1
        self._reason = 'following'
        self._waiting_at = None

    def advance(self, now, pose=None):
        """Measure road progress from control-time odom, not commanded vx."""
        with self._lock:
            usable = pose is not None and len(pose) == 3 and np.isfinite(pose).all()
            if usable and self._last_pose is not None and self._last_pose_at is not None:
                dt = now-self._last_pose_at
                delta = np.asarray(pose[:2])-self._last_pose[:2]
                if dt > 0 and np.linalg.norm(delta) > 3*dt+.25:
                    self._active = self._candidate = self._last_candidate = None
                    self._hits = 0
                    self._reason = 'pose_discontinuity'
                elif self._active is not None and dt > 0:
                    yaw = self._last_pose[2]
                    self._progress += max(0., float(delta @ np.asarray([math.cos(yaw), math.sin(yaw)])))
            self._progress_source = 'odom' if usable else 'time'
            self._last_pose = np.asarray(pose, float) if usable else None
            self._last_pose_at = now if usable else None
            if self._active is not None and now-self._verified_at > self.execution.max_unverified_sec:
                self._active = None
                self._reason = 'observation_expired'
                # Reacquire from a fresh confirmed observation, never require a
                # connection to an expired camera-relative reference forever.
                if self._ready(now):
                    self._commit(self._candidate, now)
            self._maybe_switch(now, pose if usable else None)
            return self.current(now)

    def _maybe_switch(self, now, pose=None):
        if self._active is None:
            return
        due = (self._progress >= self.execution.segment_m if self._progress_source == 'odom'
               else now-self._committed_at >= self.execution.segment_sec)
        if not due:
            return
        if self._waiting_at is None:
            self._waiting_at = now
        elif now-self._waiting_at >= self.execution.replan_wait_sec:
            # The controller has been stopped at this boundary. Drop the old
            # anchor and require two NEW observations before starting again.
            self._active = self._candidate = self._last_candidate = None
            self._hits = 0
            self._reason = 'waiting_replan_candidate'
            return
        self._reason = 'waiting_next_candidate'
        if not self._ready(now):
            return
        active, candidate = self._relative(self._active, pose), self._relative(self._candidate, pose)
        if active is None or candidate is None:
            return
        a, b = active.points_xy, candidate.points_xy
        distance, angle = _difference(a, b)
        if distance > self.execution.max_join_offset_m or angle > self.execution.max_join_heading_deg:
            self._reason = 'waiting_connected_candidate'
            return
        start = max(a[0, 0], b[0, 0])
        end = min(a[-1, 0], b[-1, 0], start+self.execution.join_length_m)
        if end-start < .3:
            return
        x = np.unique(np.r_[b[:, 0], np.linspace(start, end, 40)])
        x = x[(x >= start) & (x <= b[-1, 0])]
        # Mixed grid/spline samples can collapse to the same float32 x. Those
        # zero-length segments would falsely appear as sharp steering changes.
        x = x[np.r_[True, np.diff(x) > 1e-4]]
        u = np.clip((x-start)/(end-start), 0., 1.)
        weight = u**3*(10-15*u+6*u*u)
        y = (1-weight)*np.interp(x, a[:, 0], a[:, 1])+weight*np.interp(x, b[:, 0], b[:, 1])
        points = np.column_stack((x, y)).astype(np.float32)
        # Evidence belongs to the candidate's capture frame, not current odom.
        check = points
        if self.mask_class == 1 and pose is not None and self._candidate.road_geometry:
            from road_curve import transform_points
            capture_pose = self._candidate.road_geometry.get('observation_pose')
            if capture_pose is not None:
                check = transform_points(points, pose, capture_pose)
        if self._evidence is not None and self._evidence.corridor(check) is None:
            self._reason = 'waiting_connected_candidate'
            return
        dx = np.diff(points[:, 0]); angles = np.unwrap(np.arctan2(np.diff(points[:, 1]), dx))
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        curvature = abs(np.diff(angles))/np.maximum((lengths[:-1]+lengths[1:])/2, 1e-6)
        if len(angles) > 1 and (np.max(abs(np.diff(angles))) > math.radians(25) or
                                np.max(curvature) > 4.):
            self._reason = 'waiting_connected_candidate'
            return
        self._commit(replace(candidate, points_xy=points, source='committed_handoff'), now)

    def current(self, now):
        with self._lock:
            if self._guard is not None:
                return self._guard
            if self._active is None:
                reason = self._reason if self._reason in (
                    'invalidated', 'pose_discontinuity', 'observation_expired',
                    'waiting_replan_candidate') else 'waiting_initial_candidate'
            elif now-self._verified_at > self.execution.max_unverified_sec:
                reason = 'observation_expired'
            elif self._reason.startswith('waiting_'):
                reason = self._reason
            else:
                geometry = dict(self._active.road_geometry) if self._active.road_geometry is not None else None
                if geometry is not None:
                    geometry.pop('control_motion_compensated', None)
                return replace(self._active, age_sec=max(0., now-self._verified_at),
                               source='committed_section', road_geometry=geometry)
            return SmoothedPath(np.empty((0, 2), np.float32), 0., 0.,
                                'path_execution_guard', stop_reason='path_execution_'+reason)

    def metrics(self, now):
        with self._lock:
            path = self.current(now)
            return {'enabled': True, 'generation': self._generation,
                    'state': path.stop_reason or 'following', 'candidate_confirmations': self._hits,
                    'candidate_ready': self._ready(now), 'segment_progress_m': self._progress,
                    'progress_source': self._progress_source,
                    'segment_elapsed_sec': max(0., now-self._committed_at) if self._committed_at is not None else None,
                    'segment_m': self.execution.segment_m, 'segment_sec': self.execution.segment_sec,
                    'adoptions': self._adoptions,
                    'motion_compensation_enabled': self.mask_class == 1}


def make_path_tracker(local_config, execution_config, mask_class):
    if execution_config.enabled and mask_class == 1:
        return PathExecution(local_config, execution_config, mask_class)
    return LocalPathSmoother(local_config)
