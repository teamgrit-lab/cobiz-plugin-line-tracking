"""Independent holonomic velocity overlay with swept-footprint validation."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from lidar_obstacles import fresh
from swin_l_drive_control import DriveDecision


@dataclass(frozen=True)
class AvoidanceConfig:
    preference: str = "right"
    control_hz: float = 20.0
    max_lateral_mps: float = 0.15
    max_forward_mps: float = 0.20
    max_accel_mps2: float = 0.30
    prediction_sec: float = 3.0
    max_path_age_sec: float = 2.0
    return_gain: float = 0.50
    brake_mps2: float = 0.50
    untracked_closing_mps: float = 1.0
    side_hold_sec: float = 1.0

    def validate(self):
        if self.preference not in ("right", "left"):
            raise ValueError("avoidance preference must be right or left")
        for name, value in vars(self).items():
            if isinstance(value, float) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"avoidance {name} must be finite and positive")
        if self.control_hz < 10 or self.control_hz > 100:
            raise ValueError("avoidance control_hz must be between 10 and 100")
        if max(self.max_forward_mps, self.max_lateral_mps) > 1:
            raise ValueError("avoidance velocity limits must not exceed 1 m/s")
        if self.prediction_sec > 10:
            raise ValueError("avoidance prediction horizon must not exceed 10 s")


class AvoidanceController:
    def __init__(self, config, obstacle_config):
        config.validate()
        obstacle_config.validate()
        self.config, self.obstacles = config, obstacle_config
        self.reset()

    def reset(self):
        self.last_time = None
        self.last_vy = 0.0
        self.last_vx = 0.0
        self.side = None
        self.clear_since = None
        self.planned_velocity = None
        self.target_velocity = None
        self.target_scan_stamp = None
        self.target_reason = None
        self.avoiding = False
        self.target_updates = 0
        self.metrics = {"state": "waiting", "side": None}

    def stop(self, reason):
        self.last_vx = self.last_vy = 0.0
        self.planned_velocity = None
        self.target_velocity = None
        self.target_scan_stamp = None
        self.last_time = None
        self.metrics.update(state="stopped", reason=reason, vx=0.0, vy=0.0)
        return DriveDecision.stop(reason)

    def _trajectory(self, vx, vy, yaw, duration, future_velocity=None):
        times = np.linspace(0, duration, max(2, int(math.ceil(duration / 0.10)) + 1))
        theta = yaw * times
        if future_velocity is not None:
            # The first emitted command already obeys the acceleration bound.
            # Predict subsequent control ticks accelerating toward this target.
            elapsed = np.maximum(
                0.0, (times[1:] + times[:-1]) / 2 - 1 / self.config.control_hz
            )
            initial = np.array([vx, vy])
            delta = np.asarray(future_velocity) - initial
            velocity = initial + np.sign(delta) * np.minimum(
                np.abs(delta), elapsed[:, None] * self.config.max_accel_mps2
            )
            mid = yaw * (times[1:] + times[:-1]) / 2
            steps = np.column_stack(
                (
                    np.cos(mid) * velocity[:, 0] - np.sin(mid) * velocity[:, 1],
                    np.sin(mid) * velocity[:, 0] + np.cos(mid) * velocity[:, 1],
                )
            )
            xy = np.vstack(
                (np.zeros(2), np.cumsum(steps * np.diff(times)[:, None], axis=0))
            )
        elif abs(yaw) < 1e-5:
            xy = times[:, None] * [vx, vy]
        else:
            xy = np.column_stack(
                (
                    (vx * np.sin(theta) + vy * (np.cos(theta) - 1)) / yaw,
                    (vx * (1 - np.cos(theta)) + vy * np.sin(theta)) / yaw,
                )
            )
        return times, theta, xy

    def trajectory_check(
        self, vx, vy, yaw, scan, region, age=0.0, future_velocity=None
    ):
        """Hard obstacles, observed space and selected semantic corridor gate all motion."""
        c, a = self.obstacles, self.config
        speed = math.hypot(vx, vy)
        duration = max(a.prediction_sec, speed / a.brake_mps2 + c.max_age_sec)
        times, theta, centers = self._trajectory(vx, vy, yaw, duration, future_velocity)
        clearance = math.inf
        if len(scan.points):
            # Latest body-frame returns are refreshed at LiDAR cadence. Do not
            # invent world velocities from successive moving body frames.
            delta = scan.points[None, :, :] - centers[:, None, :]
            co, si = np.cos(theta)[:, None], np.sin(theta)[:, None]
            x, y = (
                co * delta[..., 0] + si * delta[..., 1],
                -si * delta[..., 0] + co * delta[..., 1],
            )
            # Half-cell padding bounds quantization error; never inflate twice.
            age_padding = (
                a.untracked_closing_mps
                + math.hypot(a.max_forward_mps, a.max_lateral_mps)
            ) * age
            padding = c.margin_m + c.resolution_m * math.sqrt(2) / 2 + age_padding
            dx = np.maximum(np.maximum(x - c.front_m, -c.back_m - x), 0)
            dy = np.maximum(np.abs(y) - c.robot_half_width_m, 0)
            distances = np.hypot(dx, dy)
            clearance = float(distances.min())
            if clearance <= padding:
                return False, clearance, "obstacle_collision"
        # Sample the entire footprint, not just its center or goal.
        ox, oy = np.meshgrid(
            np.arange(
                -c.back_m - c.margin_m,
                c.front_m + c.margin_m + c.resolution_m / 2,
                c.resolution_m / 2,
            ),
            np.arange(
                -c.robot_half_width_m - c.margin_m,
                c.robot_half_width_m + c.margin_m + c.resolution_m / 2,
                c.resolution_m / 2,
            ),
            indexing="ij",
        )
        local = np.column_stack((ox.ravel(), oy.ravel()))
        co, si = np.cos(theta)[:, None], np.sin(theta)[:, None]
        swept = (
            np.stack(
                (
                    co * local[:, 0] - si * local[:, 1],
                    si * local[:, 0] + co * local[:, 1],
                ),
                axis=-1,
            )
            + centers[:, None, :]
        )
        indices = c.indices(swept)
        if (
            (indices[..., 0] < 0).any()
            or (indices[..., 0] >= c.shape[0]).any()
            or (indices[..., 1] < 0).any()
            or (indices[..., 1] >= c.shape[1]).any()
        ):
            return False, clearance, "avoidance_out_of_range"
        # Already occupied body space needs no new ground observation; newly
        # swept cells must be both observed and inside the chosen road/sidewalk.
        current = (
            (swept[..., 0] >= -c.back_m - c.margin_m)
            & (swept[..., 0] <= c.front_m + c.margin_m)
            & (np.abs(swept[..., 1]) <= c.robot_half_width_m + c.margin_m)
        )
        support = (
            scan.observed[indices[..., 0], indices[..., 1]]
            & region[indices[..., 0], indices[..., 1]]
        )
        if not np.all(support | current):
            return False, clearance, "avoidance_space_unobserved"
        return True, clearance, None

    def _ramp(self, target_vx, target_vy, dt):
        step = self.config.max_accel_mps2 * dt
        return (
            float(self.last_vx + np.clip(target_vx - self.last_vx, -step, step)),
            float(self.last_vy + np.clip(target_vy - self.last_vy, -step, step)),
        )

    def _blocked(self, reason, stamp):
        result = self.stop(reason)
        # A failed plan stays stopped until a new scan arrives. Do not repeatedly
        # search the same evidence at the faster command publication rate.
        self.target_scan_stamp, self.target_reason = stamp, reason
        return result

    def _choose_target(self, nominal, path, scan, region, age, now, dt):
        a, c = self.config, self.obstacles
        points = scan.points
        ahead = points[
            (points[:, 0] >= -c.back_m)
            & (np.abs(points[:, 1]) <= c.robot_half_width_m + c.margin_m + 0.4)
        ]
        front_gap = float(np.min(ahead[:, 0] - c.front_m)) if len(ahead) else math.inf
        closing = a.untracked_closing_mps
        influence = max(
            c.inflation_m,
            (a.prediction_sec + (c.robot_half_width_m + c.margin_m) / a.max_lateral_mps)
            * closing,
        )
        strength = float(np.clip(1 - front_gap / influence, 0, 1))
        self.avoiding = strength > 0
        self.metrics.update(
            front_clearance_m=front_gap if math.isfinite(front_gap) else None,
            influence_m=influence,
        )
        if strength > 0:
            self.clear_since = None
        elif self.clear_since is None:
            self.clear_since = now
        elif now - self.clear_since >= a.side_hold_sec:
            self.side = None
        preferred = self.side or (-1 if a.preference == "right" else 1)
        pts = np.asarray(path.points_xy)
        # Each fresh camera Path is already expressed in base_link. Previous
        # camera regions and paths are not warped or accumulated without odom.
        usable = pts[np.isfinite(pts).all(axis=1) & (pts[:, 0] > 0)]
        return_vy = 0.0
        if len(usable):
            target = usable[np.argmin(np.abs(usable[:, 0] - 1.0))]
            return_vy = float(
                np.clip(
                    a.return_gain * target[1], -a.max_lateral_mps, a.max_lateral_mps
                )
            )
        desired_vy = (
            preferred * a.max_lateral_mps * strength + (1 - strength) * return_vy
        )
        if len(ahead) and strength > 0:
            # Distance-only repulsion can ramp too slowly for an oncoming
            # object. Budget the remaining lateral clearance against a
            # conservative encounter time, without assigning object velocities.
            clearance = (
                c.robot_half_width_m
                + c.margin_m
                + c.resolution_m * math.sqrt(2) / 2
                + (closing + math.hypot(a.max_forward_mps, a.max_lateral_mps))
                * (age + 0.1)  # next 10 Hz LiDAR update
            )
            encounter = np.maximum(
                (ahead[:, 0] - c.front_m - c.margin_m) / (closing + a.max_forward_mps),
                0.1,
            )
            needed = np.maximum(clearance + preferred * ahead[:, 1], 0)
            urgent_vy = min(a.max_lateral_mps, float(np.max(needed / encounter)))
            desired_vy = preferred * max(preferred * desired_vy, urgent_vy)
        max_vx = min(nominal.vx, a.max_forward_mps)
        available = max(0.0, front_gap - c.margin_m - closing * c.max_age_sec)
        max_vx = min(max_vx, math.sqrt(2 * a.brake_mps2 * available))
        lateral = np.unique(
            [desired_vy, -a.max_lateral_mps, a.max_lateral_mps, 0, return_vy]
        )
        candidates = []
        for target_vx in np.unique([max_vx, max_vx / 2, 0.0]):
            for target_vy in lateral:
                vx, vy = self._ramp(target_vx, target_vy, dt)
                ok, clearance, _ = self.trajectory_check(
                    vx, vy, nominal.yaw_rate, scan, region, age, (target_vx, target_vy)
                )
                if not ok:
                    continue
                cost = (
                    8 * (target_vy - desired_vy) ** 2
                    + 2 * (max_vx - target_vx)
                    + 0.05 * math.exp(-clearance / c.inflation_m)
                )
                candidates.append((cost, target_vx, target_vy))
        if not candidates:
            return None
        if strength > 0:
            same_side = [item for item in candidates if item[2] * preferred > 1e-6]
            other_side = [item for item in candidates if item[2] * preferred < -1e-6]
            candidates = same_side or other_side or candidates
        _, target_vx, target_vy = min(candidates)
        if strength > 0 and abs(target_vy) > 1e-6:
            self.side = 1 if target_vy > 0 else -1
        return float(target_vx), float(target_vy)

    def apply(self, nominal, path, scan, region, *, now, clock_ns):
        a, c = self.config, self.obstacles
        self.metrics.update(
            preference=a.preference,
            obstacle_count=len(scan.points) if scan is not None else 0,
            obstacle_present=bool(scan is not None and len(scan.points)),
            nearest_obstacle_m=float(np.linalg.norm(scan.points, axis=1).min())
            if scan is not None and len(scan.points)
            else None,
        )
        reason = fresh(scan, now, clock_ns, c)
        if reason:
            return self.stop(reason)
        if (
            path is None
            or path.stop_reason is not None
            or nominal.reason not in ("tracking", "tracking_slow_turn")
        ):
            return self.stop(
                nominal.reason if path is not None else "avoidance_path_unavailable"
            )
        if region is None or region.shape != c.shape:
            return self.stop("avoidance_region_unavailable")
        dt = (
            min(0.1, max(0.0, now - self.last_time))
            if self.last_time is not None
            else 1 / a.control_hz
        )
        self.last_time = now
        age = max(0.0, (clock_ns - scan.stamp_ns) / 1e9)
        if self.target_scan_stamp != scan.stamp_ns:
            self.target_velocity = self._choose_target(
                nominal, path, scan, region, age, now, dt
            )
            self.target_scan_stamp = scan.stamp_ns
            self.target_updates += 1
            self.metrics.update(
                target_scan_stamp_ns=scan.stamp_ns, target_updates=self.target_updates
            )
            if self.target_velocity is None:
                return self._blocked("obstacle_no_safe_velocity", scan.stamp_ns)
        if self.target_velocity is None:
            return self._blocked(
                self.target_reason or "obstacle_blocked", scan.stamp_ns
            )
        target_vx, target_vy = self.target_velocity
        target_vx = min(target_vx, nominal.vx, a.max_forward_mps)
        vx, vy = self._ramp(target_vx, target_vy, dt)
        # The target changes on each LiDAR update; every command is still checked
        # against the latest mask and scan age, including between those updates.
        ok, _, reason = self.trajectory_check(
            vx, vy, nominal.yaw_rate, scan, region, age, (target_vx, target_vy)
        )
        if not ok:
            return self._blocked(reason, scan.stamp_ns)
        if vx <= 1e-6 and abs(vy) <= 1e-6:
            return self._blocked("obstacle_blocked", scan.stamp_ns)
        self.last_vx, self.last_vy = vx, vy
        self.planned_velocity = ((vx, vy, nominal.yaw_rate), (target_vx, target_vy))
        state = (
            "avoiding"
            if self.avoiding and abs(vy) > 1e-6
            else "returning"
            if abs(vy) > 1e-6
            else "tracking"
        )
        self.metrics.update(
            state=state,
            reason="lidar_" + state,
            side=self.side,
            vx=vx,
            vy=vy,
            height_min_m=c.min_height_m,
            height_max_m=c.max_height_m,
        )
        return DriveDecision(vx, vy, nominal.yaw_rate, "lidar_" + state)
