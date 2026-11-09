"""Route sidewalks to the original local fit and roads to Connectome.

Mode 0 retains its per-task surface latch and common ROI. A sidewalk task
never makes an HTTP request, even when the road service is unavailable.
"""
from local_path import extract_sidewalk_centerline, path_config_for_mode, selected_path_region
from connectome_surface import SurfaceArea


def uses_connectome(mode, enabled, metrics=None):
    # Until Mode 0 has selected a surface, keep the road stop gates closed.
    return bool(enabled and (mode == 1 or (
        mode == 0 and (metrics or {}).get('union_surface') != 2)))


class SurfacePathPlanner:
    def __init__(self, client, local_config):
        self.client, self.local = client, local_config
        self.generation, self.union_surface = None, None
        self.metrics = {'backend': 'connectome', 'ready': False, 'reason': 'connectome_waiting'}

    def plan(self, frame, selected, hazard, stamp, generation, sequence, *, mode, building=None):
        if mode not in (0, 1, 2):
            raise ValueError('path mode must be 0, 1 or 2')
        if generation != self.generation:
            self.generation, self.union_surface = generation, None
        if mode == 0 and self.union_surface is None:
            areas = {s: SurfaceArea(selected, hazard, s, self.local, self.client.config) for s in (1, 2)}
            choices = [(a.seed_distance, -s, s) for s, a in areas.items() if len(a.rows)]
            if choices:
                self.union_surface = min(choices)[2]
        surface = self.union_surface if mode == 0 else mode
        estimates = {key: None for key in (0, 1, 2)}
        if surface == 1:
            estimates = self.client.plan(frame, selected, hazard, stamp, generation, sequence,
                                         modes=(mode,), union_surface=surface if mode == 0 else None,
                                         building=building if mode == 1 else None)
            self.metrics = {**self.client.metrics, 'backend': 'connectome',
                            'union_surface': self.union_surface, 'surface_class': surface}
        elif surface == 2:
            estimates[mode] = extract_sidewalk_centerline(
                selected_path_region(selected, 2), path_config_for_mode(self.local, mode))
            self.metrics = {'backend': 'local', 'ready': True, 'reason': 'ok',
                            'union_surface': self.union_surface, 'surface_class': surface}
        else:
            self.metrics = {'backend': 'connectome', 'ready': False,
                            'reason': 'surface_waiting', 'union_surface': None}
        return estimates
