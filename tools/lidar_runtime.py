"""Latest-frame base_link avoidance, independent of segmentation cadence."""

from __future__ import annotations

from collections import deque
import threading

import cv2
import numpy as np

from lidar_common import calibrated_transforms, parse_transform, stamp_ns
from lidar_obstacles import ObstacleDetector, camera_region, fresh
from local_avoidance import AvoidanceController
from swin_l_drive_control import decide_drive


class LidarRuntime:
    def __init__(
        self, args, obstacle_config, avoidance_config, lookup=None, drive_config=None
    ):
        self.args, self.config, self.avoidance = args, obstacle_config, avoidance_config
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
            "mode": "base_link_local",
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
        with self.lock:
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

    def guard(self, decision, mask_class, now, clock_ns):
        """All positive commands pass here, including task acquisition paths."""
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
