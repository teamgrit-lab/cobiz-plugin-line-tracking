"""Small in-sidewalk path bias toward adjacent Mapillary Vegetation evidence."""

from __future__ import annotations

from dataclasses import replace
import math

import numpy as np

from local_path import LocalPathConfig, LocalPathEstimate, _birdseye_sidewalk, _runs


MIN_VEGETATION_WIDTH_M = 0.15
MIN_SUPPORT_RATIO = 0.30


def _bias_points(points, sidewalk, vegetation, xs, ys, config):
    """Vote on one side, then bound every offset by the observed corridor."""
    if len(points) < 2:
        return points
    spacing = float(abs(ys[1] - ys[0]))
    min_width = max(1, int(math.ceil(MIN_VEGETATION_WIDTH_M / spacing)))
    supports = np.zeros(len(points), dtype=np.float32)
    bounds = []
    for i, (x, y) in enumerate(points):
        if not (xs[0] <= x <= xs[-1] and ys[-1] <= y <= ys[0]):
            bounds.append(None)
            continue
        row = int(np.argmin(np.abs(xs - x)))
        column = int(np.argmin(np.abs(ys - y)))
        corridor = next(((start, end) for start, end in _runs(sidewalk[row] > 0)
                         if start <= column <= end), None)
        if corridor is None:
            bounds.append(None)
            continue
        start, end = corridor
        low = float(ys[end]) + config.sidewalk_vegetation_edge_margin_m
        high = float(ys[start]) - config.sidewalk_vegetation_edge_margin_m
        # Never use a bias to pull a legacy fit back into a narrow/off-path row.
        # Only change points which already have the configured edge clearance.
        if not low <= y <= high:
            bounds.append(None)
            continue
        bounds.append((low, high))
        left = right = False
        for a, b in _runs(vegetation[row] > 0):
            if b - a + 1 < min_width:
                continue
            if b < start:
                gap = max(0.0, float(ys[b] - ys[start]) - spacing)
                left |= gap <= config.sidewalk_vegetation_max_gap_m + 1e-6
            elif a > end:
                gap = max(0.0, float(ys[end] - ys[a]) - spacing)
                right |= gap <= config.sidewalk_vegetation_max_gap_m + 1e-6
        # Two-sided evidence does not prefer either edge.
        supports[i] = int(left) - int(right)

    if abs(float(np.sum(supports))) / len(points) < MIN_SUPPORT_RATIO:
        return points
    direction = float(np.sign(np.sum(supports)))
    weights = (supports == direction).astype(np.float32)
    # Taper patch boundaries spatially, including in unrestricted path mode.
    weights = np.convolve(np.pad(weights, (2, 2), mode="edge"),
                          np.asarray([1, 2, 3, 2, 1]) / 9, mode="valid")
    shifted = points.copy()
    for i, bound in enumerate(bounds):
        if bound is None or supports[i] != direction:
            continue
        low, high = bound
        shifted[i, 1] = np.clip(points[i, 1] + direction * weights[i]
                                * config.sidewalk_vegetation_bias_m,
                                max(low, -config.max_path_lateral_m),
                                min(high, config.max_path_lateral_m))
    return shifted


def _limit_branch_offset(original, shifted):
    """Reduce offsets near bends without exceeding the original branch slope."""
    from branch_path import MAX_SLOPE

    if len(original) < 2:
        return shifted
    low = np.minimum(original[:, 1], shifted[:, 1]).astype(float)
    high = np.maximum(original[:, 1], shifted[:, 1]).astype(float)
    steps = MAX_SLOPE * np.diff(original[:, 0]).astype(float) + 1e-7
    for i in range(len(original) - 2, -1, -1):
        low[i] = max(low[i], low[i + 1] - steps[i])
        high[i] = min(high[i], high[i + 1] + steps[i])
    if np.any(low > high):
        return original
    result = shifted.astype(float)
    result[0, 1] = np.clip(result[0, 1], low[0], high[0])
    for i in range(1, len(result)):
        lo = max(low[i], float(result[i - 1, 1]) - steps[i - 1])
        hi = min(high[i], float(result[i - 1, 1]) + steps[i - 1])
        if lo > hi:
            return original
        result[i, 1] = np.clip(result[i, 1], lo, hi)
    return result.astype(np.float32)


def _branch_segments_inside(points, mask, xs, ys):
    """Retain the branch planner's slope and unclosed-mask segment guards."""
    from branch_path import MAX_SLOPE

    if len(points) < 2:
        return False
    if np.any(np.abs(np.diff(points[:, 1])) > MAX_SLOPE * np.diff(points[:, 0]) + 1e-6):
        return False
    rows = np.interp(points[:, 0], xs, np.arange(len(xs)))
    columns = np.interp(points[:, 1], ys[::-1], np.arange(len(ys))[::-1])
    for i in range(len(points) - 1):
        count = max(3, int(math.ceil(abs(columns[i + 1] - columns[i]) * 2)) + 1)
        rr = np.rint(np.linspace(rows[i], rows[i + 1], count)).astype(int)
        cc = np.rint(np.linspace(columns[i], columns[i + 1], count)).astype(int)
        if not np.all(mask[rr, cc] > 0):
            return False
    return True


def bias_sidewalk_estimate(
    estimate: LocalPathEstimate | None,
    sidewalk_mask: np.ndarray,
    vegetation_mask: np.ndarray | None,
    config: LocalPathConfig,
) -> LocalPathEstimate | None:
    """Bias only existing sidewalk paths; branch identities stay unchanged.

    Vegetation is context, never a drivable surface. Distances depend on the
    configured camera homography and describe path-center clearance only.
    """
    if estimate is None or vegetation_mask is None or config.sidewalk_vegetation_bias_m == 0:
        return estimate
    vegetation_mask = np.asarray(vegetation_mask)
    if vegetation_mask.ndim != 2 or vegetation_mask.shape != sidewalk_mask.shape:
        raise ValueError("vegetation and sidewalk masks must share two-dimensional geometry")
    if not np.any(vegetation_mask):
        return estimate
    unclosed = replace(config, close_kernel_px=0)
    sidewalk, xs, ys = _birdseye_sidewalk(sidewalk_mask, unclosed)
    vegetation, _, _ = _birdseye_sidewalk(vegetation_mask, unclosed)
    # Surface hysteresis may retain sidewalk at a current vegetation pixel.
    vegetation[sidewalk > 0] = 0
    points = _bias_points(estimate.points_xy, sidewalk, vegetation, xs, ys, config)
    observation = estimate.branch_observation
    if observation is not None:
        paths = []
        for path in observation.paths:
            shifted = _bias_points(path.points, sidewalk, vegetation, xs, ys, config)
            shifted = _limit_branch_offset(path.points, shifted)
            if path.usable and _branch_segments_inside(shifted, sidewalk, xs, ys):
                path = replace(path, points=shifted, vegetation_bias_active=bool(
                    np.any(np.abs(shifted[:, 1] - path.points[:, 1]) > 1e-6)))
            paths.append(path)
        observation = replace(observation, paths=tuple(paths))
    return replace(estimate, points_xy=points, branch_observation=observation,
                   vegetation_bias_active=bool(np.any(
                       np.abs(points[:, 1] - estimate.points_xy[:, 1]) > 1e-6)))
