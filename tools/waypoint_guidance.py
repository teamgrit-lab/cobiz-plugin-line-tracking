"""Waypoint-directed selection among observed, footprint-safe surface branches.

No waypoint creates free ground. The original branch confirmation/lock and
surface containment checks remain authoritative; no straight-line fallback.
"""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from branch_path import BranchSelector, observe_branches
from connectome_surface import PlannerConfig, SurfaceArea
from local_path import LocalPathEstimate, path_config_for_mode


class WaypointGuide:
    def __init__(self, local, config=PlannerConfig()):
        self.local, self.config = local, config
        self.generation = None
        self.selector = None
        self.metrics = {"ready": False, "reason": "waypoint_waiting"}

    def prepare(self, selected, hazard, mode, stamp, generation, target_xy, *, building=None, robot_pose=None):
        local = path_config_for_mode(self.local, mode)
        # SurfaceArea already erodes the entire robot footprint and margin.
        # Branch extraction needs only a small extra numerical cell margin.
        grid_step = max((local.far_distance_m-local.near_distance_m)/(local.bev_height_px-1),
                        2*local.search_half_width_m/(local.bev_width_px-1))
        branch_config = replace(local, branch_min_width_m=4*grid_step, branch_margin_m=1.5*grid_step)
        if generation != self.generation or self.selector is None:
            self.generation = generation
            self.selector = BranchSelector(branch_config)
        area = SurfaceArea(selected, hazard, mode, local, self.config, building=building)
        estimate = None
        if len(area.rows) >= 2:
            points = np.column_stack((area.xs[area.rows], area.centers)).astype(np.float32)
            observation = observe_branches(area.allowed, area.xs, area.ys, branch_config, 0.)
            estimate = LocalPathEstimate(points, 1., 1., 0., points, observation)
        choice = self.selector.update(estimate, stamp, target_xy=target_xy, robot_pose=robot_pose)
        reason = choice.stop_reason
        if target_xy[0] < 0 and np.hypot(*target_xy) > .6:
            reason = "waypoint_behind_robot"
        self.metrics = {**self.selector.metrics(), "ready": reason is None and estimate is not None,
                        "branch_reason": self.selector.metrics()["reason"],
                        "reason": reason or ("ok" if estimate is not None else "waypoint_path_unavailable"),
                        "target_xy": list(target_xy)}
        if not self.metrics["ready"]:
            return None, None
        corridor = choice.estimate.points_xy if choice.constrained else None
        if corridor is not None:
            area = SurfaceArea(selected, hazard, mode, local, self.config, building=building, corridor=corridor)
        return area, corridor

    def local_plan(self, selected, hazard, mode, stamp, generation, target_xy, *, robot_pose=None):
        area, _ = self.prepare(selected, hazard, mode, stamp, generation, target_xy, robot_pose=robot_pose)
        path = area.plan(SimpleNamespace(cost=lambda _pixels: 0.), target_xy=target_xy) if area else None
        if path is None:
            self.metrics.update(ready=False, reason=self.metrics["reason"] if area is None else "waypoint_path_unavailable")
            return None
        points = np.asarray(path["points_xy"], np.float32)
        # The current mask was checked for every segment. An EMA with another
        # camera/branch path could cut the fork gap, so publish these points intact.
        return LocalPathEstimate(points, path["confidence"], path["valid_ratio"],
            path["mean_sidewalk_width_m"], points.copy(), surface_constrained=True)
