"""R50 four-view Road detection feeding a pinned retained or retrained actor."""
from pathlib import Path
import json
import time

import cv2
import numpy as np
import torch
from torch.nn import functional as F

from connectome360_runtime import package_config
from connectome_policy import ConnectomeDrivePolicy
from segmentation_runtime import SegmentationConfig, SegmentationModel
from r50_checkpoint import FINETUNED_PROFILE
import r50_policy_package


class R50RoadPerception:
    """Batch four directions, without sharing single-camera temporal history."""

    def __init__(self, directory, manifest_sha256, device='cuda'):
        if (len(manifest_sha256) != 64
                or any(c not in '0123456789abcdef' for c in manifest_sha256)):
            raise ValueError('pin the R50 checkpoint manifest SHA-256')
        self.runner = SegmentationModel(SegmentationConfig(
            profile=FINETUNED_PROFILE, model_id=str(directory),
            checkpoint_manifest_sha256=manifest_sha256,
            device=device, evaluation_height=360, evaluation_width=640))
        self.details = dict(backend='r50-finetuned', manifest_sha256=manifest_sha256,
                            profile=FINETUNED_PROFILE, input_size_wh=[640, 360],
                            evaluation_shape=[4, 360, 640], temporal_filter=False,
                            road_labels=list(self.runner.road_labels),
                            precision='fp16' if self.runner.use_fp16 else 'fp32')

    def reset(self):
        self.runner.reset()

    def predict_views(self, views):
        if (len(views) != 4 or any(v.dtype != np.uint8 or v.shape != (270, 480, 3)
                                    for v in views)):
            raise ValueError('four aligned 480x270 BGR camera views required')
        start = time.perf_counter()
        # Use the same processor and semantic-score computation as Sidewalk's
        # validated R50 loader, but never mix directions through its one EMA.
        inputs = self.runner.processor(
            images=[cv2.cvtColor(v, cv2.COLOR_BGR2RGB) for v in views], return_tensors='pt')
        with torch.inference_mode():
            outputs = self.runner.model(**self.runner._move_inputs(inputs))
            scores = self.runner._r50_output_scores(outputs)
            if scores.shape != (4, 65, 360, 640) or not torch.isfinite(scores).all():
                raise ValueError('invalid R50 four-view semantic scores')
            labels = scores.argmax(1)
            # Road is 1; grass, curb, sidewalk, robot and all other classes are
            # 0. No arm-gap filling or geometric path replaces the learned actor.
            road = torch.zeros_like(labels, dtype=torch.bool)
            for label in self.runner.road_ids:
                road |= labels == label
            masks = F.interpolate(road[:, None].float(), size=(270, 480), mode='nearest')
            masks = masks[:, 0].to(torch.uint8).cpu().numpy()
        return masks, dict(ms=(time.perf_counter()-start)*1000,
                           road_fraction=[float(mask.mean()) for mask in masks],
                           detector=self.details)


class R50Policy360Runtime:
    def __init__(self, detector_directory, detector_sha256, policy_directory, package_sha256,
                 *, device='cuda'):
        is_retrained = json.loads((Path(policy_directory)/'model.json').read_text()).get('schema') in r50_policy_package.PAIR_SCHEMAS
        config = (r50_policy_package.load_config(policy_directory, package_sha256, detector_sha256)
                  if is_retrained else package_config(policy_directory, package_sha256))
        self.road = R50RoadPerception(detector_directory, detector_sha256, device)
        self.policy = (r50_policy_package.load_policy(policy_directory, config, device)
                       if is_retrained else ConnectomeDrivePolicy.with_detector_override(
                           Path(policy_directory)/'policy', config, self.road.details, device))
        if not all(torch.isfinite(v).all() for v in self.policy.model.state_dict().values()):
            raise ValueError('nonfinite retained policy parameters')
        self.details = dict(backend='r50-360-road-policy', weights_sha256=package_sha256,
            detector_sha256=detector_sha256, policy_sha256=config['policy_sha256'],
            detector_override=not is_retrained, detector=self.road.details,
            camera_views=['front', 'left', 'rear', 'right'],
            camera_profile_sha256=config['camera_profile_sha256'],
            road_mask_shape=[4, 270, 480], actor_input_chw=[4, 32, 64], actor_crop_rows=[135, 257],
            trained_max_speed_mps=10. if is_retrained else float(config['simulation_max_speed_mps']), head_device=str(device),
            uses_ground_truth_at_runtime=False, pose_input=False, goal_input=False,
            runtime_planner=False, real_robot_validated=False,
            source_package_simulation_only=config.get('simulation_only', True))
        if is_retrained:
            self.details.update(policy_schema=self.policy.details['schema'],
                                policy_trained_detector_sha256=config['policy_trained_detector_sha256'])
            if config['schema'] == r50_policy_package.FRONT_PAIR_SCHEMA:
                self.details.update(stop_architecture=config['architecture'],
                    stop_input_chw=config['stop_input_chw'], stop_uses_side_views=config['stop_uses_side_views'],
                    observation_spec=config['observation_spec'], time_spec=config['time_spec'],
                    source_timed_policy=config['architecture'] == 'temporal_front', steering_retrained=False)
        else:
            self.details.update(training_road_sha256=config['road_sha256'],
                                training_encoder_sha256=config['encoder_sha256'])

    def reset(self):
        self.road.reset()
        if hasattr(self.policy, 'reset'):
            self.policy.reset()

    def infer_views(self, views, *, source_time_sec=None):
        masks, perception = self.road.predict_views(views)
        command, policy = (self.policy.infer(masks, source_time_sec=source_time_sec)
                           if self.details.get('policy_schema') == r50_policy_package.FRONT_POLICY_SCHEMA
                           else self.policy.infer(masks))
        return masks[0], command, dict(perception=perception, policy=policy)
