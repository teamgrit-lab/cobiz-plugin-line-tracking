"""R50-constrained centre candidates; Flyvis relative flow only ranks safe paths.

This is a geometric planner with a pretrained connectome auxiliary signal, not
a trained neural waypoint head. Coordinates and distances use the existing
line-tracking ROI homography, which still needs physical camera calibration.
"""
from dataclasses import dataclass
from functools import lru_cache
import math

import cv2
import numpy as np

from local_path import LocalPathConfig, ground_to_pixel, pixel_to_ground_homography


@dataclass(frozen=True)
class PlannerConfig:
    half_width_m: float = .30
    margin_m: float = .15
    min_length_m: float = 1.
    max_center_offset_m: float = .20
    flow_weight: float = .02

    def validate(self):
        for key, value in vars(self).items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{key} must be finite and nonnegative')
        if (not 0 < self.half_width_m <= 1 or self.margin_m > 1 or not 0 < self.min_length_m <= 10
                or self.max_center_offset_m > .30 or self.flow_weight > .1):
            raise ValueError('positive footprint/lookahead and centre offset <= .30m required')


@dataclass
class FlowField:
    pixels: np.ndarray
    relative_motion: np.ndarray

    def cost(self, pixels):
        if (self.pixels.ndim != 2 or self.pixels.shape[1] != 2
                or self.relative_motion.shape != (len(self.pixels),) or not len(self.pixels)
                or not np.isfinite(self.pixels).all() or not np.isfinite(self.relative_motion).all()
                or np.any(self.relative_motion < 0)):
            raise ValueError('invalid connectome flow field')
        nearest = np.argmin(np.sum((pixels[:, None]-self.pixels[None])**2, axis=2), axis=1)
        return float(np.mean(np.clip(self.relative_motion[nearest], 0, 1)))


def runs(row):
    edges = np.flatnonzero(np.diff(np.r_[0, np.asarray(row, np.uint8), 0]))
    return [(int(a), int(b)-1) for a, b in edges.reshape(-1, 2)]


@lru_cache(maxsize=8)
def surface_geometry(shape, local, config):
    """Cache projection/footprint only; never cache a camera's free space."""
    h = pixel_to_ground_homography(shape, local)
    xs = np.linspace(local.near_distance_m, local.far_distance_m, local.bev_height_px)
    ys = np.linspace(local.search_half_width_m, -local.search_half_width_m, local.bev_width_px)
    x, y = np.meshgrid(xs, ys, indexing='ij')
    pixels = ground_to_pixel(np.column_stack((x.ravel(), y.ravel())), h).reshape(*x.shape, 2)
    clearance = config.half_width_m+config.margin_m
    dx, dy = xs[1]-xs[0], abs(ys[1]-ys[0])
    kernel_shape = (2*math.ceil(clearance/dx)+1, 2*math.ceil(clearance/dy)+1)
    if max(kernel_shape) > 1025:
        raise ValueError('clearance kernel exceeds bounded geometry')
    arrays = (h, xs, ys, pixels[..., 0].astype(np.float32), pixels[..., 1].astype(np.float32),
              np.ones(kernel_shape, np.uint8))
    for a in arrays:
        a.setflags(write=False)
    return arrays


class SurfaceArea:
    def __init__(self, selected, hazard, surface, local: LocalPathConfig, config: PlannerConfig):
        local.validate(); config.validate()
        self.local, self.config = local, config
        self.h, self.xs, self.ys, mx, my, kernel = surface_geometry(selected.shape, local, config)
        raw = cv2.remap((selected == surface).astype(np.uint8), mx, my, cv2.INTER_NEAREST)
        excluded = cv2.remap(np.asarray(hazard, np.uint8), mx, my, cv2.INTER_NEAREST)
        # No closing/filling: semantic holes and unknown gaps are not free ground.
        # The near/far camera crop does not manufacture an obstacle. Lateral
        # crop edges are unknown and explicitly excluded by the padded mask.
        padded = np.pad(raw, ((0, 0), (kernel.shape[1]//2,)*2))
        inner = cv2.erode(padded, kernel, borderType=cv2.BORDER_REPLICATE)
        inner = inner[:, kernel.shape[1]//2:-(kernel.shape[1]//2)] if kernel.shape[1] > 1 else inner
        self.allowed = inner & (cv2.dilate(excluded, kernel) == 0)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(self.allowed, 8)
        candidates = []
        for label in range(1, n):
            first = int(stats[label, cv2.CC_STAT_TOP])
            if self.xs[first]-self.xs[0] <= .30:
                columns = np.flatnonzero(labels[first] == label)
                candidates.append(((first, float(np.min(abs(self.ys[columns])))), label))
        self.allowed = (labels == min(candidates)[1]).astype(np.uint8) if candidates else np.zeros_like(raw)
        self.seed_distance = (min(candidates)[0][1] if candidates else math.inf)
        self.centers, self.intervals, self.rows = [], [], []
        previous = 0.
        for row in range(len(self.xs)):
            intervals = runs(self.allowed[row])
            if not intervals:
                if self.rows:
                    break
                continue
            a, b = min(intervals, key=lambda ab: abs((self.ys[ab[0]]+self.ys[ab[1]])/2-previous))
            previous = float((self.ys[a]+self.ys[b])/2)
            self.rows.append(row); self.centers.append(previous)
            self.intervals.append((float(self.ys[b]), float(self.ys[a])))
        self.centers = np.asarray(self.centers)

    def accepts(self, points, *, require_center=True):
        p = np.asarray(points, float)
        if (p.ndim != 2 or p.shape[1] != 2 or not 2 <= len(p) <= 512
                or not np.isfinite(p).all() or np.any(np.diff(p[:, 0]) <= 0)
                or p[-1, 0]-p[0, 0] < self.config.min_length_m or not len(self.rows)):
            return False
        xs = self.xs[self.rows]
        if p[0, 0] < xs[0]-1e-6 or p[-1, 0] > xs[-1]+1e-6:
            return False
        # Sample every crossed grid cell, including diagonal segment interiors.
        spacing = min(self.xs[1]-self.xs[0], abs(self.ys[1]-self.ys[0]))/3
        dense = np.concatenate([np.linspace(a, b, max(2, math.ceil(np.linalg.norm(b-a)/spacing)+1))
                                for a, b in zip(p, p[1:])])
        rr = np.rint(np.interp(dense[:, 0], self.xs, np.arange(len(self.xs)))).astype(int)
        cc = np.rint(np.interp(dense[:, 1], self.ys[::-1], np.arange(len(self.ys))[::-1])).astype(int)
        if (np.any(dense[:, 1] < self.ys[-1]) or np.any(dense[:, 1] > self.ys[0])
                or not np.all(self.allowed[rr, cc])):
            return False
        if require_center and np.max(abs(dense[:, 1]-np.interp(dense[:, 0], xs, self.centers))) > self.config.max_center_offset_m+1e-5:
            return False
        return True

    def longest_prefix(self, points):
        """Same longest accepted vertex prefix without quadratic rechecking.

        Containment and centre checks apply to every segment independently.
        Once a prefix fails, adding segments cannot repair it. Search only
        prefixes meeting the minimum length, then revalidate the final one.
        """
        if len(points) < 2 or self.accepts(points):
            return points
        first = int(np.searchsorted(points[:, 0], points[0, 0]+self.config.min_length_m))+1
        lo, hi, best = max(2, first), len(points)-1, 0
        while lo <= hi:
            mid = (lo+hi)//2
            if self.accepts(points[:mid]):
                best, lo = mid, mid+1
            else:
                hi = mid-1
        return points[:best]

    def plan(self, flow):
        if len(self.rows) < 2:
            return None
        xs = self.xs[self.rows]
        if xs[-1]-xs[0] < self.config.min_length_m:
            return None
        center = self.centers
        smooth = np.convolve(np.pad(center, (2, 2), mode='edge'), np.ones(5)/5, mode='valid')
        candidates = []
        for base in (smooth, center):
            for offset in np.linspace(-self.config.max_center_offset_m, self.config.max_center_offset_m, 5):
                points = np.column_stack((xs, base+offset))
                slopes = np.diff(points[:, 1])/np.diff(xs)
                bounds = np.asarray(self.intervals)
                good = ((points[:, 1] >= bounds[:, 0]) & (points[:, 1] <= bounds[:, 1])
                        & (abs(points[:, 1]-center) <= self.config.max_center_offset_m+1e-6))
                good[1:] &= abs(slopes) <= math.tan(math.radians(60))
                bad = np.flatnonzero(~good)
                if len(bad):
                    points = points[:bad[0]]
                # A far ambiguous junction must not discard a verified near
                # straight section. Never extend across it or invent support.
                points = self.longest_prefix(points)
                if len(points) < 2 or points[-1, 0]-points[0, 0] < self.config.min_length_m:
                    continue
                slopes = np.diff(points[:, 1])/np.diff(points[:, 0])
                score = float(np.mean((points[:, 1]-center[:len(points)])**2)+.001*np.mean(slopes**2))
                score += self.config.flow_weight*flow.cost(ground_to_pixel(points, self.h))
                candidates.append((score, points))
        if not candidates:
            return None
        score, points = min(candidates, key=lambda c: (-len(c[1]), c[0]))
        ratio = float(len(points)/len(self.xs))
        return {'points_xy': points.tolist(), 'confidence': ratio, 'valid_ratio': ratio,
                'mean_sidewalk_width_m': float(np.mean([b-a for a, b in self.intervals[:len(points)]])),
                'score': score, 'candidate_count': len(candidates), 'support_truncated': len(points) < len(xs)}


class SurfacePlanner:
    def __init__(self):
        self.union_surface = None

    def plan(self, selected, hazard, local, config, flow):
        areas = {s: SurfaceArea(selected, hazard, s, local, config) for s in (1, 2)}
        if self.union_surface is None:
            available = [(a.seed_distance, -s, s) for s, a in areas.items() if len(a.rows)]
            if available:
                self.union_surface = min(available)[2]
        # Mode 0 selects one of these same surfaces. Re-evaluating its ten
        # candidates repeats the entire containment/flow calculation.
        paths = {s: a.plan(flow) for s, a in areas.items()}
        result = {}
        for mode in (0, 1, 2):
            surface = self.union_surface if mode == 0 else mode
            path = paths.get(surface)
            result[str(mode)] = {**path, 'surface_class': surface} if path else None
        return result
