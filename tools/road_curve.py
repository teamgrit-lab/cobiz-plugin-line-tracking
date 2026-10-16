"""Small constrained cubic-spline planner for a forward, holonomic corridor.

Bounds constrain the *published segments*, not just the spline samples. No
pointwise clipping or jagged polyline fallback is permitted after optimization.
"""
from functools import lru_cache
import math

import numpy as np
from scipy.interpolate import BSpline
from scipy.optimize import LinearConstraint, minimize


@lru_cache(maxsize=32)
def _basis(count):
    knots = np.r_[np.zeros(4), np.linspace(0, 1, count - 2)[1:-1], np.ones(4)]
    return BSpline(knots, np.eye(count), 3)


def edge_direction(corridor, lookahead=4.):
    """Use agreement of two local boundaries to qualify a road direction.

    A widening crop, camera occlusion or one segmentation edge should not turn
    a lateral recentering request into a large yaw command.
    """
    xs = corridor[:, 0]
    start = np.clip(lookahead - .5, xs[0], max(xs[0], xs[-1] - 1.))
    rows = (xs >= start) & (xs <= start + 2.)
    if rows.sum() < 4 or np.ptp(xs[rows]) < .3:
        rows = np.ones(len(xs), bool)
    angles = []
    for column in (1, 2):
        # Median long-baseline secants resist isolated semantic-edge notches.
        x, y = xs[rows], corridor[rows, column]
        step = max(1, len(x) // 2)
        slopes = (y[step:] - y[:-step]) / (x[step:] - x[:-step])
        angles.append(float(np.arctan(np.median(slopes))))
    difference = abs(angles[0] - angles[1])
    reliability = float(np.clip(1 - difference / math.radians(30), 0, 1))
    heading = (angles[0] + angles[1]) / 2 * reliability if reliability >= .5 else 0.
    return heading, reliability


def _fit_curve(corridor, count=40, previous=None, history_weight=0., direction_hint=None):
    """Fit a C2 curve; shorten only the uncertain far end if infeasible.

    Current clearance bounds always win over a soft history penalty. A hard
    temporal point-speed cap would freeze a valid change caused by body motion.
    """
    c = np.asarray(corridor, np.float64)
    if (c.ndim != 2 or c.shape[1] != 3 or len(c) < 4 or
            not np.isfinite(c).all() or np.any(np.diff(c[:, 0]) <= 0) or
            np.any(c[:, 1] >= c[:, 2])):
        return None
    # Bounded attempts keep optimization cost predictable on the Jetson CPU.
    ends = sorted(set([len(c), int(len(c)*.85), int(len(c)*.65), int(len(c)*.45)]), reverse=True)
    for end in ends:
        cc = c[:end]
        span = float(np.ptp(cc[:, 0]))
        if len(cc) < 4 or span < .5:
            continue
        origin = cc[0, 0]
        n = min(10, max(4, int(span / .5) + 3))
        spline = _basis(n)
        # At least one output point per observed row: the interpolated path is
        # constrained at every corridor row as well as at all output points.
        xx = np.unique(np.r_[cc[:, 0], np.linspace(origin, cc[-1, 0], count)].astype(np.float32)).astype(np.float64)
        xx = xx[np.r_[True, np.diff(xx) > 1e-4]]
        tt = (xx - origin) / span
        design = spline(tt)
        evaluation = np.linspace(0, 1, max(40, len(cc)))
        b = spline(evaluation)
        d1 = spline.derivative(1)(evaluation) / span
        d2 = spline.derivative(2)(evaluation) / span**2
        d3 = spline.derivative(3)(evaluation) / span**3
        ex = origin + span * evaluation
        centers = np.interp(ex, cc[:, 0], cc[:, 1:].mean(axis=1))
        # Wide or cropped observations provide less precise center evidence.
        widths = np.interp(ex, cc[:, 0], cc[:, 2] - cc[:, 1])
        weights = 1 / np.maximum(1., widths / 3.)
        direction, reliable = edge_direction(cc)
        target_slope = math.tan(direction if direction_hint is None else direction_hint)
        matrices = [b * weights[:, None], d1 * 1.2, d2 * 2., d3 * .6]
        targets = [centers * weights, np.full(len(ex), target_slope * 1.2),
                   np.zeros(len(ex)), np.zeros(len(ex))]
        if previous is not None and history_weight > 0:
            overlap = (ex >= previous[0, 0]) & (ex <= previous[-1, 0])
            if overlap.sum() >= 2:
                w = math.sqrt(history_weight)
                matrices.append(b[overlap] * w)
                targets.append(np.interp(ex[overlap], previous[:, 0], previous[:, 1]) * w)
        a, target = np.vstack(matrices), np.concatenate(targets)
        h = a.T @ a / len(ex) + np.eye(n) * 1e-8
        g = a.T @ target / len(ex)
        initial = np.linalg.solve(h, g)
        inset = np.minimum(.01, (cc[:, 2] - cc[:, 1]) * .05)
        lower = np.interp(xx, cc[:, 0], cc[:, 1] + inset)
        upper = np.interp(xx, cc[:, 0], cc[:, 2] - inset)
        if np.all(design @ initial >= lower) and np.all(design @ initial <= upper):
            q = initial
        else:
            fit = minimize(lambda q: float(.5*q@h@q-g@q), initial,
                jac=lambda q: h@q-g, method='SLSQP',
                constraints=[LinearConstraint(design, lower, upper)],
                options={'maxiter': 60, 'ftol': 1e-8})
            if not fit.success:
                continue
            q = fit.x
        values = design @ q
        if np.any(values < lower - 1e-5) or np.any(values > upper + 1e-5):
            continue
        # Reject a corridor requiring a sudden steering reversal; never emit
        # the old adaptive polygon to squeeze around an isolated bad row.
        derivatives = d1 @ q
        second = d2 @ q
        curvature = abs(second) / (1 + derivatives**2)**1.5
        if np.max(curvature) > 4.:
            continue
        points = np.column_stack((xx, values)).astype(np.float32)
        # If the two semantic edges disagree, a corridor that still requires
        # a substantial smooth turn must remain followable. Unlike the old
        # global quadratic, this tangent comes from the constrained curve.
        local_t = float(np.clip((4.-origin)/span, 0., 1.))
        local_heading = float(np.arctan(spline.derivative(1)(local_t) @ q / span))
        source = 'edges' if reliable >= .5 else 'uncertain_parallel'
        if reliable < .5 and abs(local_heading) > math.radians(8):
            direction = local_heading
            source = 'corridor_tangent'
        return points, {'method': 'constrained_cubic_spline', 'truncated': end < len(c),
                        'direction_heading_rad': direction,
                        'direction_confidence': reliable,
                        'direction_source': source,
                        'max_curvature_per_m': float(np.max(curvature))}
    return None


def fit_curve(corridor, count=40, previous=None, history_weight=0., direction_hint=None):
    try:
        return _fit_curve(corridor, count, previous, history_weight, direction_hint)
    except (ValueError, FloatingPointError, np.linalg.LinAlgError):
        return None


def transform_points(points, old_pose, new_pose):
    """Move base-frame points between two timestamped planar odom poses."""
    old, new = np.asarray(old_pose), np.asarray(new_pose)
    angle = old[2] - new[2]
    rotation = np.array([[np.cos(angle), -np.sin(angle)],
                         [np.sin(angle), np.cos(angle)]])
    yaw = new[2]
    translation = np.array([[np.cos(yaw), np.sin(yaw)],
                            [-np.sin(yaw), np.cos(yaw)]]) @ (old[:2] - new[:2])
    return np.asarray(points) @ rotation.T + translation
