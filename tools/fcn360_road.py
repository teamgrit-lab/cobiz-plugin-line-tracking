"""FCN camera geometry and Road planning. All distances are in the robot frame (x forward, y left)."""
from functools import lru_cache

import cv2
import numpy as np

from local_path import LocalPathEstimate

ROAD = 3
EGO = 1
# Simulation-only token for a binary detector. It provides a boundary but no semantic class.
BINARY_NONROAD = 254
ROAD_BOUNDARIES = (4, 5, 6, 7, 11, 12, BINARY_NONROAD)
ASPECT = 480 / 270  # shared optical field, sampled at either native FCN resolution
GRID = (4, 8, 16)  # reference/legacy grid: front, left, rear, right
DEFAULT_INPUT_WIDTH = 1024
SUPPORTED_INPUT_WIDTHS = (512, 1024)
FRONT_ROBOT_COLUMNS = (7, 9)  # 0-based, end-exclusive, in the reference 8x16 grid
FRONT_ROBOT_ROWS = (5, 8)
FRONT_ROBOT_REVISION = "front-two-columns-bottom-three-v3"
PLANNER_REVISION = "native-fcn-grid-v6-side-recovery"
TURN_ENTER_SLOPE = .40
TURN_EXIT_SLOPE = .25
TURN_CONFIRM_FRAMES = 3
BEARING_SWITCH_COST_M = .12
SIDE_RECOVERY_MIN_ROAD = .25
SIDE_RECOVERY_MIN_DIFFERENCE = .10
SIDE_RECOVERY_CONFIRM_FRAMES = 2
SIDE_RECOVERY_YAW_RPS = .18
SIDE_RECOVERY_MAX_SECONDS = 10.


def side_recovery_yaw(metrics, max_yaw_rate=SIDE_RECOVERY_YAW_RPS, *, age_sec=0.):
    """Only a current, bounded recovery decision may request a pure rotation."""
    if metrics.get('recovery_status') != 'rotating':
        return 0.
    remaining = metrics.get('recovery_remaining_sec', 0.)
    yaw = metrics.get('recovery_yaw_rate', 0.)
    if (not all(np.isfinite(v) for v in (remaining, yaw, max_yaw_rate, age_sec))
            or age_sec < 0 or remaining <= age_sec or max_yaw_rate <= 0):
        return 0.
    return float(np.clip(yaw, -min(max_yaw_rate, SIDE_RECOVERY_YAW_RPS),
                         min(max_yaw_rate, SIDE_RECOVERY_YAW_RPS)))


def connected_side_road(mask):
    """Largest connected pavement component in the same near-side angular ROI."""
    scale = mask.shape[0] // 8
    roi = np.ascontiguousarray(mask[5*scale:8*scale, 2*scale:14*scale] == ROAD, dtype=np.uint8)
    count, _, stats, _ = cv2.connectedComponentsWithStats(roi, connectivity=4)
    return float(stats[1:, cv2.CC_STAT_AREA].max()/roi.size) if count > 1 else 0.


def model_shapes(input_width):
    """Fixed ONNX NCHW contracts; native class grids are never downsampled."""
    if input_width not in SUPPORTED_INPUT_WIDTHS:
        raise ValueError("FCN input width must be 512 or 1024")
    return (1, 3, input_width // 2, input_width), (1, 21, input_width // 64, input_width // 32)


@lru_cache(maxsize=4)
def front_robot_mask(height=8, width=16):
    """User-defined rig: central two columns and bottom three rows.

    The 512x256 perspective region is x=224..287, y=160..255; preserve
    the same angular region at higher native resolutions.
    """
    if height < 8 or height % 8 or width % 16 or height // 8 != width // 16:
        raise ValueError("Expected an 8x16 FCN grid or an integer-resolution multiple")
    scale = width // 16
    mask = np.zeros((height, width), dtype=bool)
    lo, hi = FRONT_ROBOT_COLUMNS
    top, bottom = FRONT_ROBOT_ROWS
    mask[top * scale:bottom * scale, lo * scale:hi * scale] = True
    mask.setflags(write=False)
    return mask


def apply_front_robot_prior(classes):
    """Camera-specific postprocessing, not a learned class prediction.

    The fixed front region is the only permitted ego label. Elsewhere, raw
    ego predictions become unknown (0), never fabricated road or another class.
    All other predictions and the caller's original array are preserved.
    """
    classes = np.asarray(classes)
    if classes.ndim != 3 or classes.shape[0] != 4 or classes.dtype != np.uint8:
        raise ValueError("Expected four uint8 semantic class grids")
    fixed = classes.copy()
    fixed[fixed == EGO] = 0
    fixed[0, front_robot_mask(*classes.shape[1:])] = EGO
    return fixed


@lru_cache(maxsize=2)
def camera_maps(input_width=DEFAULT_INPUT_WIDTH):
    _, _, height, width = model_shapes(input_width)[0]
    v, u = np.mgrid[:height, :width]
    right = ((u + .5) / width - .5) * 2 * ASPECT
    up = (.5 - (v + .5) / height) * 2
    latitude = np.arctan2(up, np.sqrt(1 + right ** 2))
    map_y = ((.5 - latitude / np.pi) * 1440 - .5).astype(np.float32)
    return tuple(((((np.arctan2(right, 1) - np.deg2rad(yaw) + np.pi) % (2*np.pi))
                   * 2880 / (2*np.pi) - .5).astype(np.float32), map_y)
                 for yaw in (90, 180, 270, 360))


def classify_view(view, infer):
    """One native 512×256 or 1024×512 BGR perspective from the panorama."""
    if (view.ndim != 3 or view.shape[1] not in SUPPORTED_INPUT_WIDTHS
            or view.shape != (view.shape[1] // 2, view.shape[1], 3) or view.dtype != np.uint8):
        raise ValueError("Invalid FCN view")
    rgb = cv2.cvtColor(view, cv2.COLOR_BGR2RGB).astype(np.float32) / 255
    rgb -= np.float32([.485, .456, .406])
    rgb /= np.float32([.229, .224, .225])
    logits = infer(np.ascontiguousarray(rgb.transpose(2, 0, 1)[None]))
    if logits.shape != model_shapes(view.shape[1])[1] or not np.isfinite(logits).all():
        raise ValueError("Invalid FCN scores")
    return (1 + logits[0, 1:].argmax(0)).astype(np.uint8)


def classify_panorama(bgr, infer, input_width=DEFAULT_INPUT_WIDTH):
    if bgr.shape != (1440, 2880, 3) or bgr.dtype != np.uint8:
        raise ValueError("FCN requires a 2880x1440 stitched BGR panorama")
    views, classes = [], []
    for maps in camera_maps(input_width):
        view = cv2.remap(bgr, *maps, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
        classes.append(classify_view(view, infer))
        views.append(view)
    return apply_front_robot_prior(np.stack(classes)), views


def ground_sample(mask, forward, left, height):
    """Project a flat ground point into one level virtual camera. Zero = unobserved."""
    distance = np.maximum(forward, 1e-6)
    rows, columns = mask.shape
    u = np.floor((.5 - left / (2 * ASPECT * distance)) * columns).astype(int)
    v = np.floor((.5 + height / (2 * distance)) * rows).astype(int)
    visible = (forward > 0) & (u >= 0) & (u < columns) & (v >= 0) & (v < rows)
    return np.where(visible, mask[v.clip(0, rows-1), u.clip(0, columns-1)], 0)


def side_edge(mask, height):
    """Accept the first ground boundary following a continuous near strip of road."""
    distances = np.arange(height * 1.02, 8.01, .1)
    edges = []
    for offset in (-.6, 0., .6):
        labels = ground_sample(mask, distances, offset, height)
        ends = np.flatnonzero(labels != ROAD)
        if ends.size and ends[0] >= 2 and labels[ends[0]] in ROAD_BOUNDARIES:
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
            if row[lo] in ROAD_BOUNDARIES and row[hi] in ROAD_BOUNDARIES:
                centers.append((lateral[lo] + lateral[hi]) / 2)
    return float(np.median(centers)) if len(centers) >= 2 else None


# Keep the fixed planning exclusion identical to the user-defined robot region.
# Cells outside it remain observed or unknown according to the actual detector.
FRONT_BLIND = front_robot_mask()
BLIND = 255


def observed_classes(masks):
    """Derive planning labels without altering the FCN packet or filling the arm."""
    observed = masks.copy()
    observed[observed == EGO] = 0
    observed[0, front_robot_mask(*masks.shape[1:])] = BLIND
    return observed


class FcnRoadPlanner:
    def __init__(self, config, height):
        self.config = config
        self.height = height
        # Forward slices start at the first reliably visible ground. Sample
        # both shoulders of a 12 m road; never extrapolate beyond vision.
        self.x = np.arange(height * 1.08, config.far_distance_m + .001, .2)
        self.y = np.arange(-7., 7.01, .1)
        self.forward, self.left = np.meshgrid(self.x, self.y, indexing="ij")
        self.slopes = np.linspace(-.9, .9, 37)
        self.rays = self.x[:, None] * self.slopes
        self.reset()

    def reset(self):
        self.stamp = None
        self.width = None
        self.bias = 0.
        self.direction = 0
        self.confirmations = 0
        self.turn = 0
        self.pending_turn = 0
        self.turn_frames = 0
        self.previous_slope = None
        self.reset_side_recovery()

    def reset_side_recovery(self):
        self.recovery_side = 0
        self.recovery_frames = 0
        self.recovery_started_at = None

    def no_path(self, masks, metrics, stamp):
        """Select a side from fresh camera evidence, never invent a forward path."""
        left, right = (connected_side_road(masks[v]) for v in (1, 3))
        metrics.update(recovery_left_road_fraction=left, recovery_right_road_fraction=right,
                       recovery_direction=None, recovery_confirmation_frames=0,
                       recovery_status='insufficient_side_road', recovery_yaw_rate=0.,
                       recovery_elapsed_sec=0., recovery_remaining_sec=0.)
        if stamp is None or not np.isfinite(stamp):
            self.recovery_side = self.recovery_frames = 0
            metrics['recovery_status'] = 'no_timestamp'
            return None, metrics
        if self.recovery_started_at is not None:
            elapsed = max(0., stamp-self.recovery_started_at)
            metrics.update(recovery_elapsed_sec=elapsed,
                           recovery_remaining_sec=max(0., SIDE_RECOVERY_MAX_SECONDS-elapsed))
            if elapsed >= SIDE_RECOVERY_MAX_SECONDS:
                self.recovery_side = self.recovery_frames = 0
                metrics['recovery_status'] = 'timeout'
                return None, metrics
        if max(left, right) < SIDE_RECOVERY_MIN_ROAD or abs(left-right) < SIDE_RECOVERY_MIN_DIFFERENCE:
            self.recovery_side = self.recovery_frames = 0
            if max(left, right) >= SIDE_RECOVERY_MIN_ROAD:
                metrics['recovery_status'] = 'ambiguous_sides'
            return None, metrics
        side = 1 if left > right else -1
        if self.recovery_started_at is None:
            self.recovery_started_at = stamp
            metrics['recovery_remaining_sec'] = SIDE_RECOVERY_MAX_SECONDS
        self.recovery_frames = self.recovery_frames+1 if side == self.recovery_side else 1
        self.recovery_side = side
        metrics.update(recovery_direction='left' if side > 0 else 'right',
                       recovery_confirmation_frames=self.recovery_frames, recovery_status='confirming')
        if self.recovery_frames >= SIDE_RECOVERY_CONFIRM_FRAMES:
            metrics.update(recovery_status='rotating', recovery_yaw_rate=side*SIDE_RECOVERY_YAW_RPS,
                           maneuver='recover_left' if side > 0 else 'recover_right')
        return None, metrics

    def clear_heading(self):
        self.turn = self.pending_turn = self.turn_frames = 0
        self.previous_slope = None

    def turn_direction(self, slope):
        """Small centring changes are not new turns; retain a turn in the dead band."""
        for previous in (self.pending_turn, self.turn):
            if previous * slope >= TURN_EXIT_SLOPE - 1e-6:
                return previous
        return int(np.sign(slope)) if abs(slope) >= TURN_ENTER_SLOPE - 1e-6 else 0

    def side_feedback(self, left, right):
        status, target = "unknown", 0.
        if left is not None and right is not None:
            width = left + right
            stable = self.width is None or abs(width-self.width) < max(1.2, .3*self.width)
            if stable and width <= 16.0:
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

    def corridor_bounds(self, masks):
        """Measured lateral limits of each reachable strip, in ground metres."""
        labels = ground_sample(masks[0], self.forward, self.left, self.height)
        for side, view in ((1, 1), (-1, 3)):
            # Use the more directly facing lens in the overlapping field of view.
            lateral = side * self.left
            use = lateral > self.forward
            labels[use] = ground_sample(masks[view], lateral[use],
                                        -side * self.forward[use], self.height)
        lo = np.full(self.rays.shape, np.inf)
        hi = np.full(self.rays.shape, -np.inf)
        for i, row in enumerate(labels):
            # A blind span can separate two observed shoulders. A component
            # made only of blind cells, or its unobserved tail, has no width.
            edges = np.flatnonzero(np.diff(np.r_[False, (row == ROAD) | (row == BLIND), False]))
            for start, end in zip(edges[::2], edges[1::2]):
                visible = np.flatnonzero(row[start:end] == ROAD)
                if len(visible) < 7:  # at least 0.7 m of actual pavement
                    continue
                lower, upper = self.y[start + visible[[0, -1]]]
                if self.x[i] < 3.2:
                    # The vehicle starts behind its own arm. To enter an
                    # observed shoulder it can start inside this blind span;
                    # the same ray must emerge on pavement farther ahead.
                    lower, upper = self.y[start], self.y[end-1]
                inside = (lower <= self.rays[i]) & (self.rays[i] <= upper)
                lo[i, inside] = lower
                hi[i, inside] = upper
        return lo, hi

    def plan(self, masks, *, stamp=None, max_gap=1.0):
        if masks.shape not in ((4, 8, 16), (4, 16, 32)):
            raise ValueError("FCN planner requires native 8x16 or 16x32 view grids")
        scale = masks.shape[1] // 8
        if stamp is not None and self.stamp is not None and not 0 < stamp - self.stamp <= max_gap:
            self.reset()
        self.stamp = stamp
        left, right = side_edge(masks[1], self.height), side_edge(masks[3], self.height)
        status, bias = self.side_feedback(left, right)
        back = ground_sample(masks[2], np.array([3., 4., 5.]), 0., self.height)
        metrics = dict(left_edge_m=left, right_edge_m=right, center_status=status,
                       correction_left_m=bias, rear_offset_left_m=rear_offset(masks[2], self.height),
                       rear_center_road_fraction=float(np.mean(back == ROAD)), maneuver="stop",
                       speed_fraction=0., observed_horizon_m=0.,
                       path_status="no_forward_samples", candidate_path_valid=False,
                       candidate_horizon_m=0., candidate_slope=None, selected_slope=None,
                       path_source=None, turn_confirmation_frames=0,
                       turn_confirmation_target=TURN_CONFIRM_FRAMES,
                       recovery_status='inactive', recovery_yaw_rate=0.,
                       planner_revision=PLANNER_REVISION)

        if len(self.x) < 2:
            self.clear_heading()
            self.reset_side_recovery()
            return None, metrics
        observed = observed_classes(masks)
        lo, hi = self.corridor_bounds(observed)
        # The first strip must touch the forward footprint. A remote side road
        # beyond a grass verge can never start a trajectory.
        reachable = ((self.rays - lo >= .65) & (hi - self.rays >= .65))
        reachable[0] &= (lo[0] <= .65) & (hi[0] >= -.65)
        # Do not join disjoint strips between slices. Only the contiguous
        # prefix of each ray counts; far pavement cannot bridge a near gap.
        reachable[1:] &= np.maximum(lo[:-1], lo[1:]) + 1.3 <= np.minimum(hi[:-1], hi[1:])
        prefix = np.logical_and.accumulate(reachable, axis=0)
        counts = prefix.sum(axis=0)
        lengths = np.where(counts, self.x[np.maximum(counts-1, 0)], 0.)
        eligible = lengths >= max(3.2, self.config.near_distance_m)
        # A turn also needs visible road on its near side. Upper-image sky or
        # a driveway at right angles cannot authorize forward motion.
        side_support = [float((masks[v, 5*scale:8*scale, 6*scale:10*scale] == ROAD).mean()) for v in (1, 3)]
        metrics.update(left_road_support=side_support[0], right_road_support=side_support[1])
        if max(side_support) < .25:
            self.clear_heading()
            metrics["path_status"] = "insufficient_side_road"
            return self.no_path(masks, metrics, stamp)
        for sign, support in zip((1, -1), side_support):
            if support < .25:
                eligible &= sign * self.slopes <= -.20 + 1e-6
        if not eligible.any():
            self.clear_heading()
            metrics["path_status"] = "no_connected_corridor"
            return self.no_path(masks, metrics, stamp)

        # Pick a supported bearing close to the centre of the forward corridor.
        # Prefer longer evidence, but avoid chasing a distant opening across
        # an intersection. Side feedback shifts the target only within a strip.
        mid = np.zeros_like(lo)
        np.add(lo, hi, out=mid, where=prefix)
        error = np.abs(.5 * mid + bias - self.rays)
        cost = (np.where(prefix, error, 0.).sum(axis=0) / np.maximum(counts, 1)
                + .7 * np.abs(self.slopes) + .75 * (self.x[-1] - lengths))
        best = int(np.argmin(np.where(eligible, cost, np.inf)))
        previous_index = None
        if self.previous_slope is not None:
            previous_index = int(np.argmin(abs(self.slopes-self.previous_slope)))
            # Both bearings are checked against this frame, not a cached mask.
            # Switch for a material gain in current corridor evidence.
            if eligible[previous_index] and cost[previous_index] <= cost[best] + BEARING_SWITCH_COST_M:
                best = previous_index
        slope = self.slopes[best]
        direction = self.turn_direction(slope)
        metrics.update(candidate_path_valid=True, candidate_horizon_m=float(lengths[best]),
                       observed_horizon_m=float(lengths[best]), candidate_slope=float(slope),
                       path_status="tracking", path_source="current_candidate")
        # A hole on the visible approach is not evidence that the road
        # turns away. Wait for a new observation before committing to a turn.
        uncertain_approach = (observed[0, 4*scale:, 3*scale:13*scale] == 0).any()
        uncertain_change = uncertain_approach and abs(slope) >= .20-1e-6 and (
            previous_index is None or not eligible[previous_index] or best != previous_index)
        waiting = None
        if (direction and direction != self.turn) or uncertain_change:
            self.turn_frames = self.turn_frames + 1 if direction == self.pending_turn else 1
            self.pending_turn = direction
            if uncertain_approach:
                self.turn_frames = 0
                waiting = "reobserve"
            elif self.turn_frames < TURN_CONFIRM_FRAMES:
                waiting = "confirm_left" if direction > 0 else "confirm_right"
            metrics["turn_confirmation_frames"] = self.turn_frames
            if waiting:
                metrics.update(maneuver=waiting, path_status="reobserve" if uncertain_approach else "turn_confirmation")
                # Continue a previously accepted bearing only if the current
                # connected corridor still clears its footprint. Rebuild the
                # points in the current camera frame; never replay old points.
                continuing = eligible & (abs(self.slopes) < TURN_ENTER_SLOPE - 1e-6 if self.turn == 0
                                         else self.turn*self.slopes >= TURN_EXIT_SLOPE - 1e-6)
                if previous_index is not None and continuing[previous_index]:
                    best = previous_index
                    metrics["path_source"] = "revalidated_previous_bearing"
                elif continuing.any() and not uncertain_approach:
                    best = int(np.argmin(np.where(continuing, cost, np.inf)))
                    metrics["path_source"] = "current_continuation"
                else:
                    self.previous_slope = None
                    metrics["path_source"] = None
                    return self.no_path(masks, metrics, stamp)
                direction = self.turn
            else:
                self.turn = direction
                self.pending_turn = 0
                self.turn_frames = 0
        else:
            self.turn = direction
            self.pending_turn = self.turn_frames = 0

        slope = self.slopes[best]
        self.previous_slope = float(slope)
        count = counts[best]
        forward = self.x[:count]
        # A small continuous correction avoids quantising centring steering to
        # the bearing bank. Clamp each point to its measured footprint margins.
        centers = .5 * mid[:count, best] + bias
        lateral = np.clip(self.rays[:count, best] + .25*np.clip(centers-self.rays[:count,best], -.5,.5),
                          lo[:count,best]+.65, hi[:count,best]-.65)
        points = np.column_stack((forward, lateral)).astype(np.float32)
        horizon = float(forward[-1])
        maneuver = waiting or {0:"straight", 1:"turn_left", -1:"turn_right"}[direction]
        fraction = .45 if waiting or direction or abs(slope) >= .20-1e-6 or horizon < 5.0 else .7 if horizon < 6.5 else 1.
        metrics.update(maneuver=maneuver, observed_horizon_m=horizon, speed_fraction=fraction,
                       selected_slope=float(slope))
        self.reset_side_recovery()
        return LocalPathEstimate(points_xy=points, raw_points_xy=points.copy(), confidence=float(count/len(self.x)),
                                 valid_ratio=float(count/len(self.x)), mean_sidewalk_width_m=float(
                                     np.mean(hi[:count,best]-lo[:count,best])), surface_constrained=True), metrics
