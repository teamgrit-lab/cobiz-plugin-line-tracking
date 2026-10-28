"""Front RGB person/vehicle detections associated with measured LiDAR returns.

This is a separate COCO detector, not a newly trained connectome head. Loading
never downloads weights; prepare_road_object_model.py prepares the local file.
"""
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path

import cv2
import numpy as np

CLASSES = {1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 6: 'bus', 8: 'truck'}


@dataclass(frozen=True)
class RoadObjectConfig:
    min_lateral_mps: float = .3
    max_lateral_mps: float = .8
    max_range_m: float = 6.
    min_score: float = .55
    max_age_sec: float = .8
    max_sync_sec: float = .25
    center_deadband_m: float = .15

    def validate(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('road object settings must be finite and positive')
        if not .3 <= self.min_lateral_mps <= self.max_lateral_mps <= .8:
            raise ValueError('road object target lateral speed must be within 0.3..0.8 m/s')
        if self.min_score > 1 or self.max_age_sec > 1 or self.max_sync_sec > .5:
            raise ValueError('invalid road object confidence or freshness limit')


class RoadObjectDetector:
    def __init__(self, weights, sha256, device='cpu', min_score=.55, exclusion_polygon=()):
        import torch
        from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_320_fpn
        path = Path(weights)
        if len(sha256) != 64 or hashlib.sha256(path.read_bytes()).hexdigest() != sha256:
            raise ValueError('road object weights SHA-256 mismatch')
        self.device = torch.device(device)
        self.model = fasterrcnn_mobilenet_v3_large_320_fpn(weights=None, weights_backbone=None, num_classes=91)
        self.model.load_state_dict(torch.load(path, map_location='cpu', weights_only=True))
        self.model.to(self.device).eval()
        self.min_score = min_score
        self.exclusion_polygon = exclusion_polygon

    def detect(self, rgb):
        import torch
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError('road object detector requires uint8 RGB')
        excluded = exclusion_mask(rgb.shape[:2], self.exclusion_polygon)
        visible_rgb = rgb.copy()
        visible_rgb[excluded] = 127
        with torch.inference_mode():
            image = torch.from_numpy(np.ascontiguousarray(visible_rgb.transpose(2, 0, 1))).to(self.device).float() / 255
            result = self.model([image])[0]
        detections = [{'class_id': int(label), 'label': CLASSES[int(label)],
                 'score': float(score), 'box': box.tolist()}
                for box, label, score in zip(result['boxes'].cpu(), result['labels'].cpu(), result['scores'].cpu())
                if int(label) in CLASSES and float(score) >= self.min_score]
        return [d for d in detections if box_visible_fraction(d['box'], excluded) >= .5]


def exclusion_mask(shape, polygon):
    mask = np.zeros(shape, np.uint8)
    if polygon:
        points = np.asarray(polygon, float).reshape(-1, 2)*[shape[1]-1, shape[0]-1]
        cv2.fillPoly(mask, [np.rint(points).astype(np.int32)], 1)
    return mask.astype(bool)


def box_visible_fraction(box, excluded):
    x0, y0, x1, y1 = box
    area = excluded[max(0,int(y0)):min(excluded.shape[0],int(y1)+1),
                    max(0,int(x0)):min(excluded.shape[1],int(x1)+1)]
    return float(1-area.mean()) if area.size else 0.


def associate_objects(detections, scan, info, camera_from_base, mask, config):
    """Use original 3D hits, not bounding-box width, to determine body-frame side.

Require Road support just below/alongside the object's feet. Bounding boxes
alone never authorize a manoeuvre. Missing depth for an on-road object stops.
"""
    xyz = scan.obstacle_xyz
    xyz = np.empty((0, 3)) if xyz is None else np.asarray(xyz)
    camera = xyz @ camera_from_base[:3, :3].T + camera_from_base[:3, 3]
    uv = np.empty((0, 2))
    if len(camera):
        uv = cv2.projectPoints(camera, np.zeros(3), np.zeros(3),
                               np.asarray(info.k, float).reshape(3, 3), np.asarray(info.d, float))[0].reshape(-1, 2)
    road = cv2.resize((mask == 1).astype(np.uint8), (info.width, info.height), interpolation=cv2.INTER_NEAREST)
    targets = []
    for detection in detections:
        if detection['class_id'] not in CLASSES or detection['score'] < config.min_score:
            continue
        box = np.asarray(detection['box'], float)
        if box.shape != (4,) or not np.isfinite(box).all():
            raise ValueError('road_object_invalid_box')
        x0, y0, x1, y1 = box
        if x1 <= x0 or y1 <= y0:
            raise ValueError('road_object_invalid_box')
        # Road immediately below the feet; include the sides of an occluding box.
        pad = max(3, round(info.width * .015))
        left, right = max(0, int(x0)-pad), min(info.width, int(x1)+pad)
        top, bottom = max(0, int(y1)-pad), min(info.height, int(y1)+2*pad)
        support = road[top:bottom, left:right]
        if not support.size or support.mean() < .15:
            continue
        hits = xyz[(camera[:, 2] > .01) & np.isfinite(uv).all(axis=1)
                   & (uv[:, 0] >= x0) & (uv[:, 0] <= x1)
                   & (uv[:, 1] >= y0) & (uv[:, 1] <= y1)]
        hits = hits[hits[:, 0] > 0]
        if len(hits) < 2:
            raise ValueError('road_object_depth_unavailable')
        # Reject background returns behind the nearest object surface.
        depth = np.linalg.norm(hits[:, :2], axis=1)
        order = np.argsort(depth)
        # A small pedestrian can have few returns in front of many wall returns.
        # A global percentile would let the distant background win that case.
        cluster = None
        for index in order:
            nearby = (depth >= depth[index]) & (depth <= depth[index]+.5)
            if np.count_nonzero(nearby) >= 2:
                cluster = hits[nearby]
                break
        if cluster is None:
            raise ValueError('road_object_depth_unavailable')
        hits = cluster
        x, y = np.median(hits[:, :2], axis=0)
        if math.hypot(x, y) > config.max_range_m:
            continue
        targets.append({'label': detection['label'], 'x': float(x), 'y': float(y),
                        'score': float(detection['score']), 'points': len(hits)})
    return targets
