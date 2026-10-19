"""R50-constrained centre candidates; Flyvis relative flow only ranks safe paths.

This is a geometric planner with a pretrained connectome auxiliary signal, not
a trained neural waypoint head. Coordinates and distances use the existing
line-tracking ROI homography, which still needs physical camera calibration.
"""
from dataclasses import dataclass, field
from functools import lru_cache
import math

import cv2
import numpy as np

from local_path import (LocalPathConfig, ground_to_pixel, pixel_to_ground_homography,
                        path_config_for_mode, roi_exclusion_mask)


@dataclass(frozen=True)
class PlannerConfig:
    half_width_m: float = .30
    margin_m: float = .15
    min_length_m: float = 1.
    max_center_offset_m: float = .20
    flow_weight: float = .02
    road_corridor_smoothing_m: float = .30
    road_building_cost_weight: float = 2.
    road_building_distance_m: float = 2.

    def validate(self):
        for key, value in vars(self).items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{key} must be finite and nonnegative')
        if (not 0 < self.half_width_m <= 1 or self.margin_m > 1 or not 0 < self.min_length_m <= 10
                or self.max_center_offset_m > .30 or self.flow_weight > .1
                or self.road_corridor_smoothing_m > 1.5
                or self.road_building_cost_weight > 2 or not 0 < self.road_building_distance_m <= 5):
            raise ValueError('positive footprint/lookahead and centre offset <= .30m required')


def road_corridor_reference(centers, intervals, spacing, length):
    """Current-frame main-corridor reference, not a learned building detector.

    A wide apron has an uncertain midpoint. Narrow cross-sections have more
    influence, while a second-difference cost discourages detours into short
    side openings. Anchor the near section and clip every point to its observed
    footprint-safe interval. Segment containment is still checked separately.
    No prior camera path is reused without motion compensation.
    """
    n = len(centers)
    if n < 3 or length == 0:
        return centers.copy()
    from scipy.linalg import solve_banded

    bounds = np.asarray(intervals)
    weights = 1 / (.1 + (bounds[:, 1] - bounds[:, 0]) / 2) ** 2
    weights[0] = max(weights[0], 100.)
    strength = (length / spacing) ** 4
    # Pentadiagonal D2.T @ D2, avoiding a dense solve in the live worker.
    diagonal = np.full(n, 6.); diagonal[[0, -1]] = 1.; diagonal[[1, -2]] = 5.
    adjacent = np.full(n - 1, -4.); adjacent[[0, -1]] = -2.
    if n == 3:
        diagonal[1] = 4.
    matrix = np.zeros((5, n))
    matrix[2] = weights + strength * diagonal
    matrix[1, 1:] = matrix[3, :-1] = strength * adjacent
    matrix[0, 2:] = matrix[4, :-2] = strength
    reference = solve_banded((2, 2), matrix, weights * centers, check_finite=False)
    if not np.isfinite(reference).all():
        raise ValueError('invalid road corridor reference')
    return np.clip(reference, bounds[:, 0], bounds[:, 1])


def building_ground_contacts(building, ground_support, body, homography):
    """Project only observed Building/ground footpoints, never walls or roofs.

    An open doorway is not filled or labelled forbidden by this heuristic.
    Image-border truncation, tiny components and unobserved ground bases are
    ignored. Distances retain the existing homography's calibration limits.
    """
    building = np.asarray(building)
    if building.shape != ground_support.shape or not np.isin(building, [0, 1]).all():
        raise ValueError('aligned binary building mask required')
    mask = ((building != 0) & (body == 0)).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    keep = np.zeros(n, np.uint8)
    keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= max(32, int(mask.size*.00025))
    mask = keep[labels]
    height, width = mask.shape
    columns = np.flatnonzero(mask.any(axis=0))
    if not len(columns):
        return np.empty((0, 2))
    bottoms = height-1-np.argmax(mask[::-1, columns], axis=0)
    valid = (bottoms >= 3) & (bottoms < height-2)
    valid &= mask[np.maximum(0, bottoms-2), columns] != 0
    ground_below = np.zeros(len(columns), bool)
    for step in range(1, max(3, round(height*.02))+1):
        rr = np.minimum(height-1, bottoms+step)
        ground_below |= np.asarray(ground_support[rr, columns], bool) & (body[rr, columns] == 0)
    valid &= ground_below
    pixels = np.column_stack((columns[valid], bottoms[valid])).astype(np.float32)
    if not len(pixels):
        return np.empty((0, 2))
    points = cv2.perspectiveTransform(pixels[None], homography)[0].astype(float)
    return points[np.isfinite(points).all(axis=1)]


@dataclass(frozen=True)
class FlowField:
    pixels: np.ndarray
    relative_motion: np.ndarray
    _tree: object = field(default=None, init=False, repr=False, compare=False)

    def __post_init__(self):
        # One immutable field per source frame. The spatial index must not
        # retain references to a buffer that a decoder/caller can overwrite.
        for key in ('pixels', 'relative_motion'):
            values = np.array(getattr(self, key), copy=True)
            values.setflags(write=False)
            object.__setattr__(self, key, values)

    def cost(self, pixels):
        if (self.pixels.ndim != 2 or self.pixels.shape[1] != 2
                or self.relative_motion.shape != (len(self.pixels),) or not len(self.pixels)
                or not np.isfinite(self.pixels).all() or not np.isfinite(self.relative_motion).all()
                or np.any(self.relative_motion < 0)):
            raise ValueError('invalid connectome flow field')
        if len(self.pixels) > 32 and np.result_type(pixels, self.pixels) == np.float64:
            from scipy.spatial import cKDTree
            if self._tree is None:
                object.__setattr__(self, '_tree', cKDTree(self.pixels))
            distances, indices = self._tree.query(pixels, k=2)
            nearest = indices[:, 0]
            # Match np.argmin's first-index rule at equidistant receptors.
            ties = np.isclose(distances[:, 0], distances[:, 1], rtol=1e-12, atol=1e-12)
            if np.any(ties):
                nearest[ties] = np.argmin(np.sum((pixels[ties, None]-self.pixels[None])**2, axis=2), axis=1)
        else:
            nearest = np.argmin(np.sum((pixels[:, None]-self.pixels[None])**2, axis=2), axis=1)
        return float(np.mean(np.clip(self.relative_motion[nearest], 0, 1)))


def runs(row):
    edges = np.flatnonzero(np.diff(np.r_[0, np.asarray(row, np.uint8), 0]))
    return [(int(a), int(b)-1) for a, b in edges.reshape(-1, 2)]


def grid_runs(grid):
    """The same exact runs for all rows in one NumPy operation."""
    rr, edges = np.nonzero(np.diff(np.pad(grid, ((0, 0), (1, 1))), axis=1))
    result = [[] for _ in range(len(grid))]
    for row, (a, end) in zip(rr[::2], edges.reshape(-1, 2)):
        result[row].append((int(a), int(end)-1))
    return result


def dense_segments(points, spacing):
    """Vectorized per-segment linspace, retaining each exact endpoint."""
    counts = np.maximum(2, np.ceil(np.linalg.norm(np.diff(points, axis=0), axis=1)/spacing).astype(int)+1)
    ends = np.cumsum(counts)
    segment = np.repeat(np.arange(len(counts)), counts)
    offset = np.arange(ends[-1])-np.repeat(ends-counts, counts)
    step = np.diff(points, axis=0)/(counts-1)[:, None]
    dense = points[:-1][segment]+step[segment]*offset[:, None]
    dense[ends-1] = points[1:]
    return dense


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
    def __init__(self, selected, hazard, surface, local: LocalPathConfig, config: PlannerConfig, *, building=None):
        local.validate(); config.validate()
        self.local, self.config = local, config
        self.h, self.xs, self.ys, mx, my, kernel = surface_geometry(selected.shape, local, config)
        body = roi_exclusion_mask(selected.shape, local)
        self.building_contacts = np.empty((0, 2))
        self._building_tree = None
        if surface == 1 and building is not None and config.road_building_cost_weight:
            contacts = building_ground_contacts(building, (selected != 0) | np.asarray(hazard, bool), body, self.h)
            # Far/behind-camera evidence cannot affect this local route.
            reach = 2*config.road_building_distance_m
            valid = ((contacts[:, 0] >= self.xs[0]-reach) & (contacts[:, 0] <= self.xs[-1]+reach)
                     & (abs(contacts[:, 1]) <= local.search_half_width_m+reach))
            self.building_contacts = contacts[valid]
            if len(self.building_contacts):
                from scipy.spatial import cKDTree
                self._building_tree = cKDTree(self.building_contacts)
        raw = cv2.remap(((selected == surface) & (body == 0)).astype(np.uint8), mx, my, cv2.INTER_NEAREST)
        excluded = cv2.remap((np.asarray(hazard, bool) | (body != 0)).astype(np.uint8), mx, my, cv2.INTER_NEAREST)
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
        for row, intervals in enumerate(grid_runs(self.allowed)):
            if not intervals:
                if self.rows:
                    break
                continue
            a, b = min(intervals, key=lambda ab: abs((self.ys[ab[0]]+self.ys[ab[1]])/2-previous))
            previous = float((self.ys[a]+self.ys[b])/2)
            self.rows.append(row); self.centers.append(previous)
            self.intervals.append((float(self.ys[b]), float(self.ys[a])))
        self.centers = np.asarray(self.centers)
        self.raw_centers = self.centers.copy()
        if surface == 1:
            self.centers = road_corridor_reference(self.centers, self.intervals,
                self.xs[1] - self.xs[0], config.road_corridor_smoothing_m)

    def excluded_path_centers(self):
        """Observed safe centers excluded by route policy, not semantic obstacles.

        The same continuous reference/band is enforced in accepts(). The raw
        footprint-safe mask stays separate so path smoothing cannot fill holes.
        """
        excluded = np.zeros_like(self.allowed)
        if len(self.rows):
            excluded[self.rows] = self.allowed[self.rows] & (
                abs(self.ys[None] - self.centers[:, None]) > self.config.max_center_offset_m + 1e-5)
        return excluded

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
        dense = dense_segments(p, spacing)
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

    def building_cost(self, points):
        if self._building_tree is None:
            return 0.
        distance, _ = self._building_tree.query(np.asarray(points))
        # Bounded, continuous soft cost. It cannot create road support or
        # veto the only valid corner route merely for being beside a barn.
        floor = math.exp(-2.)
        return float(np.mean(np.clip((np.exp(-distance/self.config.road_building_distance_m)-floor)/(1-floor), 0, 1)))

    def plan(self, flow):
        if len(self.rows) < 2:
            return None
        xs = self.xs[self.rows]
        if xs[-1]-xs[0] < self.config.min_length_m:
            return None
        center = self.centers
        smooth = np.convolve(np.pad(center, (2, 2), mode='edge'), np.ones(5)/5, mode='valid')
        candidates = []
        bases = [(smooth, False), (center, False)]
        if self._building_tree is not None:
            # A rigid parallel offset can be blocked by one narrow near row.
            # Also evaluate offsets clipped to each observed safe interval.
            # The original candidates remain, and every segment is rechecked.
            bases += [(smooth, True), (center, True)]
        for base, bounded in bases:
            for offset in np.linspace(-self.config.max_center_offset_m, self.config.max_center_offset_m, 9 if bounded else 5):
                bounds = np.asarray(self.intervals)
                lateral = base+offset
                if bounded:
                    lower = np.maximum(bounds[:, 0], center-self.config.max_center_offset_m+1e-6)
                    upper = np.minimum(bounds[:, 1], center+self.config.max_center_offset_m-1e-6)
                    lateral = np.clip(lateral, lower, upper)
                # Validate the exact coordinates the receiving client uses.
                # Float64 paths on a grid edge can otherwise change cells
                # when JSON coordinates are converted to float32 on receipt.
                points = np.column_stack((xs, lateral)).astype(np.float32)
                slopes = np.diff(points[:, 1])/np.diff(xs)
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
                score += self.config.road_building_cost_weight*self.building_cost(points)
                candidates.append((score, points))
        if not candidates:
            return None
        score, points = min(candidates, key=lambda c: (-len(c[1]), c[0]))
        ratio = float(len(points)/len(self.xs))
        return {'points_xy': points.tolist(), 'confidence': ratio, 'valid_ratio': ratio,
                'mean_sidewalk_width_m': float(np.mean([b-a for a, b in self.intervals[:len(points)]])),
                'score': score, 'candidate_count': len(candidates), 'support_truncated': len(points) < len(xs),
                'building_cost': self.building_cost(points), 'building_contact_count': len(self.building_contacts)}


class SurfacePlanner:
    def __init__(self):
        self.union_surface = None

    def plan(self, selected, hazard, local, config, flow, *, modes=(0, 1, 2), building=None):
        common_surfaces = {1, 2} if 0 in modes else ({2} if 2 in modes else set())
        areas = {s: SurfaceArea(selected, hazard, s, local, config) for s in common_surfaces}
        if 0 in modes and self.union_surface is None:
            available = [(a.seed_distance, -s, s) for s, a in areas.items() if len(a.rows)]
            if available:
                self.union_surface = min(available)[2]
        # Mode 0 keeps the common ROI even when it latches the road. Reuse
        # those candidates; only explicit Mode 1 gets the road ROI override.
        needed = ({self.union_surface} if 0 in modes else set()) | ({2} if 2 in modes else set())
        paths = {s: a.plan(flow) for s, a in areas.items() if s in needed}
        road_local = path_config_for_mode(local, 1)
        road_path = None
        if 1 in modes:
            road_area = areas.get(1) if road_local == local and building is None else None
            road_path = paths.get(1) if road_area is not None and 1 in paths else (
                road_area or SurfaceArea(selected, hazard, 1, road_local, config, building=building)).plan(flow)
        result = {str(mode): None for mode in (0, 1, 2)}
        for mode in modes:
            surface = self.union_surface if mode == 0 else mode
            path = road_path if mode == 1 else paths.get(surface)
            result[str(mode)] = {**path, 'surface_class': surface} if path else None
        return result
