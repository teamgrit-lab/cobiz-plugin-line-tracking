"""Connected, forward-only fork selection in the configured BEV geometry.

This is not a general 2-D planner: paths must advance in x. Distances use the
input metric grid, supplied by the camera homography.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from local_path import LocalPathConfig, LocalPathEstimate


MIN_BRANCH_LENGTH_M = 0.75
MIN_BRANCH_SEPARATION_M = 0.60
MIN_BRANCH_ANGLE_RAD = math.radians(15)
MATCH_DISTANCE_M = 0.65
MAX_PATHS = 32
MAX_SLOPE = 1.5
EXIT_FRAMES = 3


@dataclass(frozen=True)
class Interval:
    row: int
    start: int
    end: int


@dataclass(frozen=True)
class BranchPath:
    intervals: tuple[Interval, ...]
    centers: np.ndarray
    points: np.ndarray
    min_width_m: float

    @property
    def usable(self) -> bool:
        return len(self.points) >= 2


@dataclass(frozen=True)
class BranchObservation:
    paths: tuple[BranchPath, ...]
    branches: tuple[int, ...]
    headings: tuple[float, ...]
    overflow: bool = False


@dataclass(frozen=True)
class BranchChoice:
    estimate: LocalPathEstimate | None
    stop_reason: str | None = None
    constrained: bool = False


def _inside_path(
    intervals: tuple[Interval, ...],
    mask: np.ndarray,
    xs: np.ndarray,
    ys: np.ndarray,
    config: LocalPathConfig,
) -> BranchPath:
    rows = np.array([item.row for item in intervals])
    starts = np.array([item.start for item in intervals])
    ends = np.array([item.end for item in intervals])
    x = xs[rows].astype(float)
    center = (ys[starts] + ys[ends]) / 2
    widths = ys[starts] - ys[ends]
    centers = np.column_stack((x, center))
    empty = np.empty((0, 2), dtype=np.float32)
    minimum = float(np.min(widths))
    if len(rows) < 2 or minimum < config.branch_min_width_m:
        return BranchPath(intervals, centers, empty, minimum)

    lo = np.maximum(
        ys[ends].astype(float) + config.branch_margin_m, -config.max_path_lateral_m
    )
    hi = np.minimum(
        ys[starts].astype(float) - config.branch_margin_m, config.max_path_lateral_m
    )
    # Propagate feasible bounds backwards before selecting points forwards.
    # This starts moving within the common corridor before reaching the fork.
    for i in range(len(x) - 2, -1, -1):
        step = MAX_SLOPE * (x[i + 1] - x[i])
        lo[i] = max(lo[i], lo[i + 1] - step)
        hi[i] = min(hi[i], hi[i + 1] + step)
    if np.any(lo > hi):
        return BranchPath(intervals, centers, empty, minimum)
    target = center.astype(float).copy()
    # Spatial regularization is projected into this corridor only. Never mix
    # the centers of two different branches or two unaligned camera frames.
    for _ in range(40):
        target[1:-1] = (target[:-2] + target[2:] + 0.15 * center[1:-1]) / 2.15
        target = np.clip(target, lo, hi)
    for i in range(1, len(x)):
        step = MAX_SLOPE * (x[i] - x[i - 1])
        low, high = max(lo[i], target[i - 1] - step), min(hi[i], target[i - 1] + step)
        if low > high + 1e-9:
            return BranchPath(intervals, centers, empty, minimum)
        target[i] = np.clip(target[i], low, high)
    # Keep every BEV row. Downsampling before checking segments could cut the
    # inside of a bend. Check segment interiors on the unclosed source mask.
    columns = np.interp(target, ys[::-1], np.arange(len(ys))[::-1])
    for i in range(len(rows) - 1):
        count = max(3, int(math.ceil(abs(columns[i + 1] - columns[i]) * 2)) + 1)
        rr = np.rint(np.linspace(rows[i], rows[i + 1], count)).astype(int)
        cc = np.rint(np.linspace(columns[i], columns[i + 1], count)).astype(int)
        if not np.all(mask[rr, cc] > 0):
            return BranchPath(intervals, centers, empty, minimum)
    return BranchPath(
        intervals, centers, np.column_stack((x, target)).astype(np.float32), minimum
    )


def observe_branches(
    mask: np.ndarray,
    xs: np.ndarray,
    ys: np.ndarray,
    config: LocalPathConfig,
    anchor_y: float,
) -> BranchObservation:
    """Trace interval overlap; only common-root, sustained splits are forks.

    No row gaps are bridged and no disconnected roots are joined. A split
    that reconnects is collapsed at its merge rather than treated as a fork.
    The bounded graph fails closed on combinatorial segmentation noise.
    """
    from local_path import _runs

    live: list[tuple[Interval, ...]] = []
    finished: list[tuple[Interval, ...]] = []
    seeded = False
    for row in range(len(xs)):
        runs = [
            Interval(row, start, end)
            for start, end in _runs(mask[row] > 0)
            if float(ys[start] - ys[end]) >= config.min_sidewalk_width_m
        ]
        if not seeded:
            if not runs:
                continue
            live = [(item,) for item in runs]
            seeded = True
            continue
        next_paths: dict[Interval, list[tuple[Interval, ...]]] = {}
        for path in live:
            tail = path[-1]
            children = [
                item
                for item in runs
                if max(tail.start, item.start) <= min(tail.end, item.end)
            ]
            if not children:
                finished.append(path)
            for child in children:
                next_paths.setdefault(child, []).append(path + (child,))
        # Reconverging paths share the same new interval. Retain one history;
        # their transient hole must not create two long-lived alternatives.
        live = [
            max(options, key=lambda p: sum(n.end - n.start for n in p))
            for options in next_paths.values()
        ]
        if len(live) + len(finished) > MAX_PATHS:
            return BranchObservation((), (), (), overflow=True)
        if not live:
            break
    paths = tuple(
        _inside_path(path, mask, xs, ys, config)
        for path in finished + live
        if len(path) >= 2
    )
    if not paths:
        return BranchObservation((), (), ())
    root = min(paths, key=lambda p: abs(float(p.centers[0, 1]) - anchor_y)).intervals[0]
    branch_indices: set[int] = set()
    splits: list[int] = []
    for i, left in enumerate(paths):
        if left.intervals[0] != root:
            continue
        for j in range(i + 1, len(paths)):
            right = paths[j]
            if right.intervals[0] != root:
                continue
            common = 0
            for a, b in zip(left.intervals, right.intervals):
                if a != b:
                    break
                common += 1
            if common == 0 or common >= min(len(left.intervals), len(right.intervals)):
                continue
            split_x, split_y = left.centers[common - 1]
            end_x = min(left.centers[-1, 0], right.centers[-1, 0])
            length = end_x - split_x
            if length < MIN_BRANCH_LENGTH_M:
                continue
            left_y = float(np.interp(end_x, left.centers[:, 0], left.centers[:, 1]))
            right_y = float(np.interp(end_x, right.centers[:, 0], right.centers[:, 1]))
            angle = abs(
                math.atan2(left_y - split_y, length)
                - math.atan2(right_y - split_y, length)
            )
            if (
                abs(left_y - right_y) >= MIN_BRANCH_SEPARATION_M
                and angle >= MIN_BRANCH_ANGLE_RAD
            ):
                branch_indices.update((i, j))
                splits.append(common - 1)
    indices = tuple(sorted(branch_indices))
    if not indices:
        return BranchObservation(paths, (), ())
    # Use one common distance and origin for every candidate being ranked.
    split = min(splits)
    split_x, split_y = paths[indices[0]].centers[split]
    end_x = min(paths[i].centers[-1, 0] for i in indices)
    headings = tuple(
        math.atan2(
            float(np.interp(end_x, paths[i].centers[:, 0], paths[i].centers[:, 1]))
            - split_y,
            end_x - split_x,
        )
        for i in indices
    )
    return BranchObservation(paths, indices, headings)


def _distance(previous: BranchPath, current: BranchPath) -> float:
    """Compare the informative far half; a shared trunk alone is not a match."""
    a, b = previous.centers, current.centers
    start = max(float((a[0, 0] + a[-1, 0]) / 2), float(b[0, 0]))
    end = min(float(a[-1, 0]), float(b[-1, 0]))
    if end - start < MIN_BRANCH_LENGTH_M or b[-1, 0] < a[-1, 0] - MIN_BRANCH_LENGTH_M:
        return math.inf
    x = np.linspace(start, end, 16)
    return float(
        np.mean(np.abs(np.interp(x, a[:, 0], a[:, 1]) - np.interp(x, b[:, 0], b[:, 1])))
    )


class BranchSelector:
    """One task/surface's confirmed branch identity, independent of the EMA."""

    def __init__(self, config: LocalPathConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._locked: BranchPath | None = None
        self._alternatives: tuple[BranchPath, ...] = ()
        self._pending: BranchPath | None = None
        self._hits = 0
        self._single_hits = 0
        self._locked_at = 0.0
        self._last_update = -math.inf
        self._status = {
            "preference": self.config.branch_preference,
            "state": "idle",
            "branch_detected": False,
            "candidate_headings_deg": [],
            "reason": "single_path",
            "confirmation_hits": 0,
            "usable_candidate_count": 0,
            "preference_applied": False,
        }

    def metrics(self) -> dict:
        return dict(self._status)

    def update(self, estimate: LocalPathEstimate | None, now: float) -> BranchChoice:
        if self.config.branch_preference == "center":
            return BranchChoice(estimate)
        fresh = math.isfinite(now) and now > self._last_update
        if fresh:
            self._last_update = now
        observation = estimate.branch_observation if estimate is not None else None
        detected = observation is not None and bool(observation.branches)
        self._status.update(
            branch_detected=detected,
            candidate_headings_deg=(
                [math.degrees(angle) for angle in observation.headings]
                if observation
                else []
            ),
            usable_candidate_count=(
                sum(observation.paths[i].usable for i in observation.branches)
                if observation
                else 0
            ),
        )

        def stop(reason: str) -> BranchChoice:
            self._status.update(
                state="locked" if self._locked is not None else "confirming",
                reason=reason,
                confirmation_hits=self._hits,
            )
            return BranchChoice(None, reason)

        def select(path: BranchPath, reason: str) -> BranchChoice:
            assert estimate is not None
            chosen = replace(
                estimate,
                points_xy=path.points,
                raw_points_xy=path.centers,
                branch_observation=None,
            )
            self._status.update(
                state="locked", reason=reason, confirmation_hits=self._hits
            )
            return BranchChoice(chosen, constrained=True)

        if observation is not None and observation.overflow:
            self._pending, self._hits, self._single_hits = None, 0, 0
            return stop("branch_graph_ambiguous")
        if self._locked is not None:
            matches = sorted(
                (
                    (_distance(self._locked, path), i, path)
                    for i, path in enumerate(observation.paths if observation else ())
                    if path.usable
                ),
                key=lambda item: item[0],
            )
            if not matches or matches[0][0] > MATCH_DISTANCE_M:
                self._single_hits = 0
                return stop("branch_path_lost")
            if len(matches) > 1 and matches[1][0] - matches[0][0] < 0.15:
                self._single_hits = 0
                return stop("branch_match_ambiguous")
            path = matches[0][2]
            # A short fork can share most of the comparison range. If its
            # other arm survives, proximity alone must not relabel it as the
            # selected arm just because their average separation is small.
            if any(
                _distance(other, path) + 0.10 < matches[0][0]
                for other in self._alternatives
            ):
                self._single_hits = 0
                return stop("branch_path_lost")
            if fresh:
                self._locked = path
                alternatives = []
                for other in self._alternatives:
                    possible = [
                        candidate
                        for candidate in observation.paths
                        if candidate is not path
                    ]
                    match = min(
                        possible,
                        key=lambda candidate: _distance(other, candidate),
                        default=None,
                    )
                    alternatives.append(
                        match
                        if match is not None
                        and _distance(other, match) <= MATCH_DISTANCE_M
                        else other
                    )
                self._alternatives = tuple(alternatives)
                self._single_hits = (
                    self._single_hits + 1
                    if len(observation.paths) == 1 and not detected
                    else 0
                )
            chosen = select(path, "locked_branch")
            if (
                self._single_hits >= EXIT_FRAMES
                and now - self._locked_at >= self.config.branch_hold_sec
            ):
                self._locked, self._pending, self._hits = None, None, 0
                self._alternatives = ()
                self._status.update(
                    state="idle", reason="branch_completed", confirmation_hits=0
                )
            return chosen
        if not detected:
            self._pending, self._hits = None, 0
            self._status.update(
                state="idle",
                reason="single_path",
                confirmation_hits=0,
                preference_applied=False,
            )
            return BranchChoice(estimate)
        candidates = [
            (angle, observation.paths[i])
            for i, angle in zip(observation.branches, observation.headings)
            if observation.paths[i].usable
        ]
        if not candidates:
            self._pending, self._hits = None, 0
            return stop("branch_no_usable_path")
        # Robot coordinates are y-left: larger bearings select the left arm.
        choose = max if self.config.branch_preference == "left" else min
        heading, preferred = choose(candidates, key=lambda item: item[0])
        if fresh:
            self._hits = (
                self._hits + 1
                if self._pending is not None
                and _distance(self._pending, preferred) <= MATCH_DISTANCE_M
                else 1
            )
            self._pending = preferred
        if self._hits < self.config.branch_confirm_frames:
            return stop("branch_confirming")
        self._locked, self._locked_at, self._single_hits = preferred, now, 0
        self._alternatives = tuple(
            observation.paths[i]
            for i in observation.branches
            if observation.paths[i] is not preferred
        )
        applied = heading == choose(observation.headings)
        self._status.update(
            preference_applied=applied, selected_heading_deg=math.degrees(heading)
        )
        return select(
            preferred,
            f"{self.config.branch_preference}_branch_selected"
            if applied
            else f"{self.config.branch_preference}_unusable_fallback",
        )
