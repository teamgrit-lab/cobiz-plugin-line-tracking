"""FCN camera geometry and Road planning. All distances are in the robot frame (x forward, y left)."""
from dataclasses import replace
from functools import lru_cache

import cv2
import numpy as np

from local_path import extract_metric_centerline

ROAD = 3
EGO = 1
ASPECT = 480 / 270  # optical field of the 480x270 view resampled to FCN's 512x256
GRID = (4, 8, 16)  # front, left, rear, right in a single capture
FRAME_ID = "fcn360_camera"


def mask_topic(sha):
    return "/line_tracking/fcn360/masks/model_" + sha


@lru_cache(maxsize=1)
def camera_maps():
    v, u = np.mgrid[:256, :512]
    right = ((u + .5) / 512 - .5) * 2 * ASPECT
    up = (.5 - (v + .5) / 256) * 2
    latitude = np.arctan2(up, np.sqrt(1 + right ** 2))
    map_y = ((.5 - latitude / np.pi) * 1440 - .5).astype(np.float32)
    return tuple(((((np.arctan2(right, 1) - np.deg2rad(yaw) + np.pi) % (2*np.pi))
                   * 2880 / (2*np.pi) - .5).astype(np.float32), map_y)
                 for yaw in (90, 180, 270, 360))


def classify_panorama(bgr, infer):
    if bgr.shape != (1440, 2880, 3) or bgr.dtype != np.uint8:
        raise ValueError("FCN requires a 2880x1440 stitched BGR panorama")
    views, classes = [], []
    for maps in camera_maps():
        view = cv2.remap(bgr, *maps, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
        rgb = cv2.cvtColor(view, cv2.COLOR_BGR2RGB).astype(np.float32) / 255
        rgb -= np.float32([.485, .456, .406])
        rgb /= np.float32([.229, .224, .225])
        logits = infer(np.ascontiguousarray(rgb.transpose(2, 0, 1)[None]))
        if logits.shape != (1, 21, 8, 16) or not np.isfinite(logits).all():
            raise ValueError("Invalid FCN scores")
        classes.append((1 + logits[0, 1:].argmax(0)).astype(np.uint8))
        views.append(view)
    return np.stack(classes), views


def read_masks(message):
    if (message.header.frame_id != FRAME_ID or message.encoding != "mono8"
            or (message.height, message.width, message.step) != (32, 16, 16)
            or len(message.data) != 512):
        raise ValueError("Invalid FCN four-view class packet")
    masks = np.frombuffer(message.data, np.uint8).reshape(GRID)
    if masks.min() < 1 or masks.max() > 20:
        raise ValueError("Invalid FCN class ID")
    return masks


def ground_sample(mask, forward, left, height):
    """Project a flat ground point into one level virtual camera. Zero = unobserved."""
    distance = np.maximum(forward, 1e-6)
    u = np.floor((.5 - left / (2 * ASPECT * distance)) * 16).astype(int)
    v = np.floor((.5 + height / (2 * distance)) * 8).astype(int)
    visible = (forward > 0) & (u >= 0) & (u < 16) & (v >= 0) & (v < 8)
    return np.where(visible, mask[v.clip(0, 7), u.clip(0, 15)], 0)


def side_edge(mask, height):
    """Accept the first ground boundary following a continuous near strip of road."""
    distances = np.arange(height * 1.02, 8.01, .1)
    edges = []
    for offset in (-.6, 0., .6):
        labels = ground_sample(mask, distances, offset, height)
        ends = np.flatnonzero(labels != ROAD)
        if ends.size and ends[0] >= 2 and labels[ends[0]] in (4, 5, 6, 7, 11, 12):
            edges.append(float(distances[ends[0]]))
    if len(edges) < 2 or np.ptp(edges) > 1.0:
        return None
    return float(np.median(edges))


def rear_offset(mask, height):
    """Estimate the rear corridor center, signed left in the *robot* frame."""
    lateral = np.linspace(-6., 6., 121)
    centers = []
    for distance in (3., 4., 5.):
        row = ground_sample(mask, distance, -lateral, height)
        mid = len(lateral) // 2
        if row[mid] != ROAD:
            continue
        left = np.flatnonzero(row[mid:] != ROAD)
        right = np.flatnonzero(row[:mid+1][::-1] != ROAD)
        if left.size and right.size:
            lo, hi = mid - right[0], mid + left[0]
            if row[lo] in (4, 5, 6, 7, 11, 12) and row[hi] in (4, 5, 6, 7, 11, 12):
                centers.append((lateral[lo] + lateral[hi]) / 2)
    return float(np.median(centers)) if len(centers) >= 2 else None


class FcnRoadPlanner:
    def __init__(self, config, height):
        self.config = replace(config, branch_preference="center", unrestricted_path_mode=False)
        self.height = height
        self.x = np.linspace(config.near_distance_m, config.far_distance_m, config.bev_height_px)
        self.y = np.linspace(config.search_half_width_m, -config.search_half_width_m, config.bev_width_px)
        # Anchor the component at the first visible ground in front of the robot;
        # a distant road beyond a grass verge must not start a path.
        first_visible = height * 1.02
        near_rows = np.arange(first_visible, self.x[0], .10)
        self.near_rows = len(near_rows)
        self.forward, self.left = np.meshgrid(np.r_[near_rows, self.x], self.y, indexing="ij")
        self.reset()

    def reset(self):
        self.stamp = None
        self.width = None
        self.bias = 0.
        self.direction = 0
        self.confirmations = 0

    def side_feedback(self, left, right):
        status, target = "unknown", 0.
        if left is not None and right is not None:
            width = left + right
            stable = self.width is None or abs(width-self.width) < max(1.2, .3*self.width)
            if stable and width <= 10.0:
                self.width = width if self.width is None else .8*self.width + .2*width
                error = (left-right)/2
                direction = int(np.sign(error)) if abs(error) > .35 else 0
                self.confirmations = self.confirmations + 1 if direction == self.direction else 1
                self.direction = direction
                status = "centered" if direction == 0 else ("right_of_center" if direction > 0 else "left_of_center")
                if self.confirmations >= 3:
                    target = np.clip(error * .45, -.45, .45)
        if status == "unknown":
            self.direction = self.confirmations = 0
            self.bias = 0.
        if self.confirmations < 3:
            self.bias = 0.
        self.bias += float(np.clip(target-self.bias, -.12, .12))
        return status, self.bias if status != "unknown" else 0.

    def plan(self, masks, *, stamp=None, max_gap=1.0):
        if stamp is not None and self.stamp is not None and not 0 < stamp - self.stamp <= max_gap:
            self.reset()
        self.stamp = stamp
        left, right = side_edge(masks[1], self.height), side_edge(masks[3], self.height)
        status, bias = self.side_feedback(left, right)
        back = ground_sample(masks[2], np.array([3., 4., 5.]), 0., self.height)
        metrics = dict(left_edge_m=left, right_edge_m=right, center_status=status,
                       correction_left_m=bias, rear_offset_left_m=rear_offset(masks[2], self.height),
                       rear_center_road_fraction=float(np.mean(back == ROAD)))
        # The mast has fixed angular extent, not fixed width in ground meters.
        # Interpolate a center *estimate* only across ego cells flanked by road
        # on the same image row. Keep the original class masks intact.
        front = masks[0].copy()
        for row in front[4:]:
            edges = np.flatnonzero(np.diff(np.r_[False, row == EGO, False]))
            for start, end in zip(edges[::2], edges[1::2]):
                if 6 <= start < end <= 10 and row[start-1] == ROAD and row[end] == ROAD:
                    row[start:end] = ROAD
        road = (ground_sample(front, self.forward, self.left, self.height) == ROAD).astype(np.uint8)
        _, components = cv2.connectedComponents(road, connectivity=8)
        near = components[0, np.abs(self.y) <= .9]
        candidates = near[near > 0]
        if not len(candidates):
            return None, metrics
        component = (components[self.near_rows:] == np.bincount(candidates).argmax()).astype(np.uint8)
        estimate = extract_metric_centerline(component, self.x, self.y, self.config)
        if estimate is None:
            return None, metrics
        points = estimate.points_xy.copy()
        # Shift only within the front-observed corridor. Rear never alters the path.
        for i, (x, center) in enumerate(points):
            row = component[np.abs(self.x - x).argmin()]
            center_idx = np.abs(self.y - center).argmin()
            if not row[center_idx]:
                return None, metrics
            proposed = center + bias
            target_idx = np.abs(self.y - proposed).argmin()
            lo, hi = sorted((center_idx, target_idx))
            if bias and row[lo:hi+1].all():
                points[i, 1] = proposed
        return replace(estimate, points_xy=points), metrics
