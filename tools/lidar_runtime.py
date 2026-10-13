"""Latest-frame LiDAR avoidance or optional Street Light corroboration."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading

import cv2
import numpy as np

from lidar_common import calibrated_transforms, cloud_xyz, parse_transform, stamp_ns
from lidar_obstacles import ObstacleDetector, camera_region, fresh
from local_avoidance import AvoidanceController
from swin_l_drive_control import decide_drive


@dataclass(frozen=True)
class ConfirmationScan:
    points: np.ndarray
    stamp_ns: int
    arrival: float


class LidarRuntime:
    def __init__(
        self, args, obstacle_config, avoidance_config, lookup=None, drive_config=None,
        *, confirmation_only=False,
    ):
        self.args, self.config, self.avoidance = args, obstacle_config, avoidance_config
        self.confirmation_only = confirmation_only
        if args.path_frame_id != "base_link":
            raise ValueError("local avoidance requires base_link paths")
        self.detector = ObstacleDetector(obstacle_config)
        self.controller = AvoidanceController(avoidance_config, obstacle_config)
        self.lookup = lookup
        self.drive_config = drive_config
        self.base = parse_transform(args.lidar_to_base_transform)
        self.camera = parse_transform(args.base_to_camera_transform)
        self.camera_frame = None
        if args.lidar_calibration_profile != "tf":
            base, camera = calibrated_transforms(
                args.lidar_calibration_profile,
                args.lidar_frame_id,
                args.path_frame_id,
                "camera_optical_frame",
            )
            self.base = self.base if self.base is not None else base
            self.camera = self.camera if self.camera is not None else camera
        self.lock = threading.RLock()
        self.imus, self.infos = (
            deque(maxlen=256),
            deque(maxlen=8),
        )
        self.scan = None
        self.reference = None
        self.fault = "lidar_waiting_for_cloud"
        self.metrics = {
            "state": "waiting",
            "mode": "street_light_confirmation" if confirmation_only else "base_link_local",
            "stops_enabled": not confirmation_only,
            "height_reference": obstacle_config.height_reference,
            "height_min_m": obstacle_config.min_height_m,
            "height_max_m": obstacle_config.max_height_m,
            "horizontal_range_m": obstacle_config.horizontal_range_m,
            "fov_deg": obstacle_config.fov_deg,
            "range_reference": "base_link",
        }

    def reset_reference(self):
        with self.lock:
            self.reference = None
            self.controller.reset()
            if self.confirmation_only:
                self.scan = None
                self.fault = "lidar_waiting_for_cloud"

    def on_imu(self, message, arrival):
        with self.lock:
            self.imus.append((message, arrival))

    def on_info(self, message):
        with self.lock:
            self.infos.append(message)

    def _transform(self, target, source, header):
        if self.lookup is None:
            raise ValueError("lidar_transform_unavailable")
        return self.lookup(target, source, header)

    def on_cloud(self, message, arrival, clock_ns):
        with self.lock:
            try:
                if message.header.frame_id != self.args.lidar_frame_id:
                    raise ValueError("lidar_frame_mismatch")
                source = stamp_ns(message.header)
                if not -0.05 <= (clock_ns - source) / 1e9 <= self.config.max_age_sec:
                    raise ValueError("lidar_stale")
                if self.confirmation_only:
                    base = self.base if self.base is not None else self._transform(
                        self.args.path_frame_id, message.header.frame_id, message.header)
                    raw = cloud_xyz(message)
                    points = raw @ base[:3, :3].T + base[:3, 3]
                    ranges = np.linalg.norm(points[:, :2], axis=1)
                    maximum = self.args.street_light_lidar_max_range_m
                    points = points[np.isfinite(points).all(axis=1) & (ranges <= maximum)]
                    self.scan = ConfirmationScan(points, source, arrival)
                    self.base, self.fault = base, None
                    self.metrics.update(state="ready", scan_stamp_ns=source,
                                        input_points=len(raw), confirmation_points=len(points),
                                        lidar_match_count=0, lidar_confirmed=False)
                    return
                if not self.imus:
                    raise ValueError("lidar_waiting_for_imu")
                imu, received = min(
                    self.imus, key=lambda v: abs(stamp_ns(v[0].header) - source)
                )
                if not 0 <= arrival - received <= self.config.max_age_sec:
                    raise ValueError("lidar_imu_stale")
                base = (
                    self.base
                    if self.base is not None
                    else self._transform(
                        self.args.path_frame_id, message.header.frame_id, message.header
                    )
                )
                scan = self.detector.update(message, imu, base, arrival)
                self.base = base  # Cache the fixed sensor extrinsic, including TF mode.
                self.scan, self.fault = scan, None
                self.metrics.update(
                    scan_stamp_ns=scan.stamp_ns,
                    input_points=scan.input_count,
                    obstacle_cells=len(scan.points),
                    obstacle_present=bool(len(scan.points)),
                    nearest_obstacle_m=float(np.linalg.norm(scan.points, axis=1).min())
                    if len(scan.points)
                    else None,
                    observed_ratio=float(scan.observed.mean()),
                    up_base=scan.up_base.tolist(),
                )
            except (ValueError, AttributeError, TypeError, cv2.error) as error:
                self.fault = (
                    str(error)
                    if str(error).startswith(("lidar_", "avoidance_"))
                    else "lidar_processing_error"
                )
                if self.confirmation_only:
                    self.metrics.update(state="unavailable", reason=self.fault,
                                        lidar_match_count=0, lidar_confirmed=False)

    def set_reference(self, mask, header, image_shape, arrival):
        with self.lock:
            if self.reference and stamp_ns(header) < stamp_ns(self.reference[1]):
                return
            self.reference = (mask.copy(), header, tuple(image_shape), arrival)

    def _context(self, mask_class, now, clock_ns):
        if self.fault:
            raise ValueError(self.fault)
        reason = fresh(self.scan, now, clock_ns, self.config)
        if reason:
            raise ValueError(reason)
        if self.reference is None:
            raise ValueError("avoidance_waiting_for_path_region")
        mask, header, shape, arrival = self.reference
        age = (clock_ns - stamp_ns(header)) / 1e9
        if not (
            0 <= now - arrival <= self.avoidance.max_path_age_sec
            and -0.05 <= age <= self.avoidance.max_path_age_sec
        ):
            raise ValueError("avoidance_path_stale")
        matching = [
            i
            for i in self.infos
            if i.header.frame_id == header.frame_id and (i.height, i.width) == shape
        ]
        if not matching:
            raise ValueError("avoidance_camera_info_unavailable")
        if self.camera_frame is not None and header.frame_id != self.camera_frame:
            raise ValueError("lidar_camera_frame_mismatch")
        if (
            self.args.lidar_calibration_profile != "tf"
            and header.frame_id != "camera_optical_frame"
        ):
            raise ValueError("lidar_calibration_frame_mismatch")
        camera = (
            self.camera
            if self.camera is not None
            else self._transform(header.frame_id, self.args.path_frame_id, header)
        )
        # With no ego-motion estimate, old camera regions cannot be carried
        # around the robot. Only the latest image constrains visible cells;
        # camera-blind sides still require current LiDAR evidence and clearance.
        allowed, visible = camera_region(
            mask,
            matching[-1],
            camera,
            self.scan,
            self.config,
            mask_class,
            return_visibility=True,
        )
        self.camera = camera
        self.camera_frame = header.frame_id
        self.metrics.update(
            path_age_sec=age, camera_visible_ratio=float(visible.mean())
        )
        return self.scan, allowed | ~visible

    def apply(self, nominal, path, mask_class, now, clock_ns):
        if self.confirmation_only:
            return nominal
        with self.lock:
            self.metrics["street_light_bias_blocked_reason"] = None
            if nominal.reason == "street_light_approaching":
                try:
                    scan, region = self._context(1, now, clock_ns)
                    if path is None or path.stop_reason is not None:
                        raise ValueError("avoidance_path_unavailable")
                    fallback = None
                    if self.drive_config is not None:
                        road = decide_drive(path, camera_age_sec=0.0, inference_age_sec=0.0,
                                            config=self.drive_config)

                        def fallback(reason):
                            self.metrics["street_light_bias_blocked_reason"] = reason
                            return self.controller.apply(
                                road, path, scan, region, now=now, clock_ns=clock_ns)

                    result = self.controller.apply_direct(
                        nominal, scan, region, now=now, clock_ns=clock_ns, fallback=fallback)
                except (ValueError, AttributeError, TypeError, cv2.error) as error:
                    result = self.controller.stop(str(error))
                self.metrics = {**self.metrics, **self.controller.metrics}
                return result
            if nominal.reason not in ("tracking", "tracking_slow_turn"):
                reason = nominal.reason
                if reason in ("tracking_path_recovery", "tracking_path_hold") or reason.startswith("path_recovery_"):
                    reason = "avoidance_path_unavailable"
                result = self.controller.stop(reason)
                self.metrics = {**self.metrics, **self.controller.metrics}
                return result

            try:
                scan, region = self._context(mask_class, now, clock_ns)
                if self.drive_config is not None:
                    nominal = decide_drive(
                        path,
                        camera_age_sec=0.0,
                        inference_age_sec=0.0,
                        config=self.drive_config,
                    )
                decision = self.controller.apply(
                    nominal, path, scan, region, now=now, clock_ns=clock_ns
                )
                self.metrics = {
                    **self.metrics,
                    **self.controller.metrics,
                    "scan_stamp_ns": scan.stamp_ns,
                    "input_points": scan.input_count,
                    "obstacle_cells": len(scan.points),
                    "observed_ratio": float(scan.observed.mean()),
                    "region_ratio": float(region.mean()),
                    "scan_age_sec": (clock_ns - scan.stamp_ns) / 1e9,
                    "up_base": scan.up_base.tolist(),
                }
                return decision
            except (ValueError, AttributeError, TypeError, cv2.error) as error:
                reason = (
                    str(error)
                    if str(error).startswith(("lidar_", "avoidance_"))
                    else "avoidance_processing_error"
                )
                result = self.controller.stop(reason)
                self.metrics = {**self.metrics, **self.controller.metrics}
                return result

    def locate_street_light(self, observation, now, clock_ns):
        """Visual bearing and LiDAR proximity in matching image columns.

        Column association is proximity evidence, not a 3-D object ID. Never
        project the elevated lamp head onto a ground plane to invent its range.
        """
        if self.confirmation_only:
            return self._confirm_street_light(observation, now, clock_ns)
        with self.lock:
            scan, _ = self._context(1, now, clock_ns)
            _, header, shape, _ = self.reference
            info = [i for i in self.infos if i.header.frame_id == header.frame_id
                    and (i.height, i.width) == shape][-1]
            k, d = np.asarray(info.k, float).reshape(3, 3), np.asarray(info.d, float)
            pixel = np.array([[[observation.center * info.width,
                                observation.bottom * info.height]]], dtype=float)
            ray_xy = cv2.undistortPoints(pixel, k, d).reshape(2)
            ray = self.camera[:3, :3].T @ np.r_[ray_xy, 1.0]
            if not np.isfinite(ray).all() or ray[0] <= 0.01:
                raise ValueError("street_light_geometry_invalid")
            bearing = float(np.arctan2(ray[1], ray[0]))
            distance = None
            if len(scan.points):
                points = np.column_stack((scan.points, np.zeros(len(scan.points))))
                camera = points @ self.camera[:3, :3].T + self.camera[:3, 3]
                uv, _ = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), k, d)
                columns = uv.reshape(-1, 2)[:, 0] / info.width
                match = ((camera[:, 2] > 0.01) & np.isfinite(columns)
                         & (columns >= observation.left - 0.03)
                         & (columns <= observation.right + 0.03))
                if np.any(match):
                    distance = float(np.linalg.norm(scan.points[match], axis=1).min())
            return bearing, distance

    def _confirm_street_light(self, observation, now, clock_ns):
        """Match current 3-D returns to the visual box without obstacle/IMU gates."""
        with self.lock:
            self.metrics.update(lidar_confirmed=False, lidar_match_count=0)
            if self.fault:
                raise ValueError(self.fault)
            scan = self.scan
            if scan is None or not (
                0 <= now - scan.arrival <= self.config.max_age_sec
                and -0.05 <= (clock_ns - scan.stamp_ns) / 1e9 <= self.config.max_age_sec
            ):
                raise ValueError("lidar_stale")
            if self.reference is None:
                raise ValueError("street_light_camera_reference_unavailable")
            _, header, shape, arrival = self.reference
            source_age = (clock_ns - stamp_ns(header)) / 1e9
            if not (0 <= now - arrival <= self.args.street_light_max_age_sec
                    and -0.05 <= source_age <= self.args.street_light_max_age_sec):
                raise ValueError("street_light_camera_reference_stale")
            if abs(scan.stamp_ns - stamp_ns(header)) / 1e9 > self.config.max_age_sec:
                raise ValueError("street_light_lidar_camera_unsynchronized")
            infos = [info for info in self.infos if info.header.frame_id == header.frame_id
                     and (info.height, info.width) == shape]
            if not infos:
                raise ValueError("street_light_camera_info_unavailable")
            if self.camera_frame is not None and header.frame_id != self.camera_frame:
                raise ValueError("lidar_camera_frame_mismatch")
            if self.args.lidar_calibration_profile != "tf" and header.frame_id != "camera_optical_frame":
                raise ValueError("lidar_calibration_frame_mismatch")
            camera = self.camera if self.camera is not None else self._transform(
                header.frame_id, self.args.path_frame_id, header)
            info = infos[-1]
            k, d = np.asarray(info.k, float).reshape(3, 3), np.asarray(info.d, float)
            points = scan.points
            camera_points = points @ camera[:3, :3].T + camera[:3, 3]
            front = (camera_points[:, 2] > 0.01) & np.isfinite(camera_points).all(axis=1)
            points, camera_points = points[front], camera_points[front]
            if not len(points):
                return 0.0, None
            pixels, _ = cv2.projectPoints(camera_points, np.zeros(3), np.zeros(3), k, d)
            uv = pixels.reshape(-1, 2) / [info.width, info.height]
            margin = 0.03
            match = (np.isfinite(uv).all(axis=1)
                     & (uv[:, 0] >= observation.left - margin)
                     & (uv[:, 0] <= observation.right + margin)
                     & (uv[:, 1] >= observation.top - margin)
                     & (uv[:, 1] <= observation.bottom + margin))
            count = int(np.count_nonzero(match))
            self.metrics["lidar_match_count"] = count
            if count < self.args.street_light_lidar_min_points:
                return 0.0, None
            target = np.median(points[match, :2], axis=0)
            self.camera, self.camera_frame = camera, header.frame_id
            self.metrics.update(lidar_confirmed=True, state="confirmed")
            return float(np.arctan2(target[1], target[0])), float(np.linalg.norm(target))

    def guard(self, decision, mask_class, now, clock_ns):
        """Guard avoidance commands; confirmation mode preserves the road decision."""
        if self.confirmation_only:
            return decision
        if decision.vx == decision.vy == decision.yaw_rate == 0:
            with self.lock:
                if not (
                    self.controller.metrics.get("state") == "stopped"
                    and self.controller.metrics.get("reason") == decision.reason
                ):
                    self.controller.stop(decision.reason)
                self.metrics = {**self.metrics, **self.controller.metrics}
            return decision
        with self.lock:
            try:
                if decision.reason in ("tracking_path_recovery", "tracking_path_hold") or decision.reason.startswith("path_recovery_"):
                    raise ValueError("avoidance_path_unavailable")
                scan, region = self._context(mask_class, now, clock_ns)
                plan = self.controller.planned_velocity
                target = (
                    plan[1]
                    if plan and plan[0] == (decision.vx, decision.vy, decision.yaw_rate)
                    else None
                )
                ok, _, reason = self.controller.trajectory_check(
                    decision.vx,
                    decision.vy,
                    decision.yaw_rate,
                    scan,
                    region,
                    max(0.0, (clock_ns - scan.stamp_ns) / 1e9),
                    target,
                )
                if not ok:
                    raise ValueError(reason)
                return decision
            except (ValueError, AttributeError, TypeError, cv2.error) as error:
                result = self.controller.stop(str(error))
                self.metrics = {**self.metrics, **self.controller.metrics}
                return result
