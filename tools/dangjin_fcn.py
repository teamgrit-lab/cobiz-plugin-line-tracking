"""Mac/browser FCN perception and camera-only Road control. No ROS interfaces."""
from dataclasses import replace
import hashlib
import json
from time import perf_counter

import cv2
import numpy as np

from fcn360_road import (BINARY_NONROAD, FRONT_ROBOT_COLUMNS, FRONT_ROBOT_ROWS, FRONT_ROBOT_REVISION,
                         FcnRoadPlanner, PLANNER_REVISION, ROAD, apply_front_robot_prior, classify_view, model_shapes,
                         side_recovery_yaw)
from local_path import LocalPathConfig, LocalPathSmoother
from swin_l_drive_control import DriveConfig, DriveDecision, decide_drive


class FcnPerception:
    def __init__(self, model_dir):
        model = model_dir / "fcn_resnet18.onnx"
        expected = json.loads((model_dir / "complete.json").read_text())["sha256"][model.name]
        digest = hashlib.sha256(model.read_bytes()).hexdigest()
        if digest != expected:
            raise ValueError("FCN model does not match its training manifest")
        manifest = json.loads((model_dir / "complete.json").read_text())["sha256"]
        raw_config = (model_dir / "config.json").read_bytes()
        if hashlib.sha256(raw_config).hexdigest() != manifest.get("config.json"):
            raise ValueError("FCN config does not match its training manifest")
        config = json.loads(raw_config)
        height, width = config["dataset_contract"]["shape"]
        input_shape, output_shape = model_shapes(width)
        if height != input_shape[2]:
            raise ValueError("Invalid FCN model input shape")
        self.input_width, self.input_height = width, height
        self.net = cv2.dnn.readNetFromONNX(str(model))
        grid_scale = width // 512
        self.details = dict(backend=f"fcn{width}-360", views=4, weights_sha256=digest,
                            input_width=width, input_height=height,
                            class_grid_shape=[4, *output_shape[2:]],
                            bundle_sha256=digest, uses_ground_truth_at_runtime=False,
                            postprocessing_revision=FRONT_ROBOT_REVISION,
                            fixed_front_robot_columns=[v * grid_scale for v in FRONT_ROBOT_COLUMNS],
                            fixed_front_robot_rows=[v * grid_scale for v in FRONT_ROBOT_ROWS],
                            off_region_ego="unobserved")

    def logits(self, image):
        self.net.setInput(image)
        return self.net.forward()

    def predict_classes(self, views):
        if len(views) != 4 or any(v.shape != (self.input_height, self.input_width, 3) for v in views):
            raise ValueError("FCN views do not match the model package resolution")
        start = perf_counter()
        classes = apply_front_robot_prior(np.stack([classify_view(view, self.logits) for view in views]))
        return classes, (perf_counter() - start) * 1000


class FcnDrive:
    def __init__(self, height, max_speed):
        config = LocalPathConfig()
        self.planner = FcnRoadPlanner(config, height)
        self.smoother = LocalPathSmoother(config)
        self.drive = DriveConfig(max_forward_mps=1., stop_on_path_unavailable=True)
        self.max_speed_mps = max_speed
        self.details = dict(backend="fcn360-path-controller", planner_revision=PLANNER_REVISION,
                            fixed_front_robot_revision=FRONT_ROBOT_REVISION, map_assistance=False,
                            pose_input=False, goal_input=False, max_speed_mps=max_speed)

    def reset(self):
        self.planner.reset()
        self.smoother.reset()

    def command(self, classes, stamp, speed):
        start = perf_counter()
        estimate, metrics = self.planner.plan(classes, stamp=stamp)
        path = None
        if estimate is None:
            self.smoother.reset()
        else:
            path = self.smoother.update(estimate, stamp)
        decision = decide_drive(path, camera_age_sec=0., inference_age_sec=0., config=self.drive)
        if estimate is None:
            yaw = side_recovery_yaw(metrics, self.drive.max_yaw_rps)
            if yaw:
                decision = DriveDecision(0., 0., yaw, 'road_side_recovery')
            else:
                reason = {"turn_confirmation": "turn_confirmation", "reobserve": "road_reobserve"}.get(metrics["path_status"])
                if reason:
                    decision = replace(decision, reason=reason)
        elif decision.vx > 0 and metrics["path_status"] in ("turn_confirmation", "reobserve"):
            decision = replace(decision, reason="tracking_turn_confirmation")
        # Apply the requested simulation speed to the controller's normalized
        # forward output. The physical robot's hard limit stays in DriveConfig.
        decision = replace(decision, vx=decision.vx * speed * metrics["speed_fraction"])
        points = path.points_xy.tolist() if path is not None else []
        metrics.update(front_path_valid=estimate is not None, ms=(perf_counter()-start)*1000)
        return decision, metrics, points


def binary_road_grid(masks):
    """Match the controller's angular cells, pooling each native Flyvis view independently."""
    masks = np.asarray(masks)
    if masks.shape != (4, 270, 480) or not np.isin(masks, (0, 1)).all():
        raise ValueError("Expected four binary Connectome road masks at 480x270")
    # Each cell covers 30 columns and 33.75 rows. Count in quarter-row
    # units so exactly 50% coverage never flips due to interpolation roundoff.
    row_votes = masks.reshape(4, 270, 16, 30).sum(axis=3, dtype=np.uint16)
    votes = row_votes.repeat(4, axis=1).reshape(4, 8, 135, 16).sum(axis=2)
    return np.where(2 * votes > 135 * 30, ROAD, BINARY_NONROAD).astype(np.uint8)


def apply_class_faults(classes, faults, elapsed):
    # Apply the same image-space holes as Flyvis, then sample each FCN cell at
    # its centre. Only removed road becomes unobserved (0); preserve ego labels.
    road = (classes == ROAD).astype(np.uint8)
    display = np.stack([cv2.resize(m, (480,270), interpolation=cv2.INTER_NEAREST) for m in road])
    masked, info = faults.apply(display, elapsed)
    rows, columns = classes.shape[1:]
    observed = masked[:, ((np.arange(rows)+.5)*270/rows).astype(int)][:, :, ((np.arange(columns)+.5)*480/columns).astype(int)]
    classes = classes.copy()
    classes[(road != 0) & (observed == 0)] = 0
    return classes, info
