"""Synchronous depth-one planner client, local mask revalidation and path lifetime."""
from dataclasses import asdict
import json
import math
import time
import urllib.request
import urllib.error
import uuid

import cv2
import numpy as np

from connectome_server import MAX_BODY, SCHEMA, encode_image
from connectome_surface import PlannerConfig, SurfaceArea
from local_path import LocalPathEstimate, SmoothedPath, path_config_for_mode


def reference_stop_reason(path, ready, ages, max_age):
    if not ready:
        return 'connectome_unavailable'
    if path is None:
        return 'connectome_path_unavailable'
    if any(age is None or not math.isfinite(age) or not 0 <= age <= max_age for age in ages):
        return 'connectome_source_stale'
    return None


class ConnectomeClient:
    def __init__(self, url, local_config, planner_config=PlannerConfig(), timeout=.8):
        if not url.startswith(('http://', 'https://')) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('planner HTTP URL and positive finite timeout required')
        self.url, self.local, self.config, self.timeout = url, local_config, planner_config, timeout
        self.local.validate(); self.config.validate()
        self.session, self.generation, self.union_surface = uuid.uuid4().hex, None, None
        self.metrics = {'backend': 'connectome', 'ready': False, 'reason': 'connectome_waiting'}

    def plan(self, frame_bgr, selected, hazard, stamp, generation, sequence, *, modes=(0, 1, 2), union_surface=None, building=None):
        if not modes or any(type(mode) is not int or mode not in (0, 1, 2) for mode in modes):
            raise ValueError('requested modes must contain 0, 1 or 2')
        if generation != self.generation:
            self.generation, self.union_surface = generation, None
        if union_surface is not None:
            if type(union_surface) is not int or union_surface not in (1, 2) or 0 not in modes:
                raise ValueError('a pinned union surface requires Mode 0 and surface 1 or 2')
            if self.union_surface not in (None, union_surface):
                raise ValueError('union surface cannot change within a task')
            self.union_surface = union_surface
        stream = f'{self.session}:{generation}'
        identity = f'{stream}:{sequence}'
        started = time.monotonic()
        try:
            frame = cv2.resize(frame_bgr, (selected.shape[1], selected.shape[0]))
            payload = {'schema': SCHEMA, 'stream_id': stream, 'request_id': identity, 'stamp_sec': stamp,
                       'requested_modes': list(modes),
                       'union_surface': self.union_surface if 0 in modes else None,
                       'local_config': asdict(self.local), 'planner_config': asdict(self.config),
                       # Flyvis's first operation is this exact conversion.
                       # Lossless mono PNG avoids sending unused colour planes.
                       'image_png': encode_image(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)),
                       'selected_png': encode_image(selected.astype(np.uint8)),
                       'hazard_png': encode_image(np.asarray(hazard, np.uint8))}
            if 1 in modes and building is not None:
                if np.shape(building) != selected.shape or not np.isin(building, [0, 1]).all():
                    raise ValueError('aligned binary building mask required')
                payload['building_png'] = encode_image(np.asarray(building, np.uint8))
            data = json.dumps(payload, allow_nan=False).encode()
            if len(data) > MAX_BODY:
                raise ValueError('planner request too large')
            request = urllib.request.Request(self.url, data=data, headers={'Content-Type': 'application/json'})
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                raw = response.read(MAX_BODY+1)
                if len(raw) > MAX_BODY:
                    raise ValueError('planner response too large')
            reply = json.loads(raw)
            if (reply.get('schema') != SCHEMA or reply.get('request_id') != identity
                    or reply.get('stream_id') != stream or reply.get('stamp_sec') != stamp):
                raise ValueError('planner response does not match camera/task frame')
            model = reply['model']
            if ('building_png' in payload and self.config.road_building_cost_weight
                    and reply.get('building_cost_applied') is not True):
                raise ValueError('planner does not support requested building cost')
            if (model.get('backend') != 'flyvis' or model.get('pretrained_task') != 'optical_flow'
                    or model.get('flow_decoder_loaded') is not True
                    or len(model.get('bundle_sha256', '')) != 64):
                raise ValueError('pretrained Flyvis inference required; no planner fallback')
            common_surfaces = {1, 2} if 0 in modes else ({2} if 2 in modes else set())
            areas = {surface: SurfaceArea(selected, hazard, surface, self.local, self.config)
                     for surface in common_surfaces}
            road_local = path_config_for_mode(self.local, 1)
            road_area = None
            if 1 in modes:
                road_area = areas.get(1) if road_local == self.local and building is None else None
                if road_area is None:
                    road_area = SurfaceArea(selected, hazard, 1, road_local, self.config, building=building)
            if 0 in modes and self.union_surface is None:
                available = [(a.seed_distance, -s, s) for s, a in areas.items() if len(a.rows)]
                if available:
                    self.union_surface = min(available)[2]
            estimates = {mode: None for mode in (0, 1, 2)}
            for mode in modes:
                path = reply['paths'][str(mode)]
                if path is None:
                    estimates[mode] = None
                    continue
                surface = self.union_surface if mode == 0 else mode
                points = np.asarray(path['points_xy'], np.float32)
                area = road_area if mode == 1 else areas.get(surface)
                if area is None or path['surface_class'] != surface or not area.accepts(points):
                    raise ValueError('planner path violates selected surface, centre or fresh mask')
                confidence = float(path['confidence'])
                if not math.isfinite(confidence) or not 0 < confidence <= 1:
                    raise ValueError('invalid planner confidence')
                # Geometry metadata also comes from our own current mask,
                # rather than trusting arbitrary remote confidence/width data.
                rows = np.asarray(area.rows)
                visible = (area.xs[rows] >= points[0, 0]-1e-5) & (area.xs[rows] <= points[-1, 0]+1e-5)
                ratio = float(np.count_nonzero(visible)/len(area.xs))
                widths = np.diff(np.asarray(area.intervals)[visible], axis=1)
                estimates[mode] = LocalPathEstimate(points, min(confidence, ratio), ratio,
                    float(np.mean(widths)), points.copy())
            if time.monotonic()-started > self.timeout:
                raise TimeoutError('planner roundtrip budget exceeded; discard late response')
            self.metrics = {'backend': 'connectome', 'ready': True, 'reason': 'ok', 'model': model,
                            'union_surface': self.union_surface,
                            'road_corridor': {'smoothing_m': self.config.road_corridor_smoothing_m,
                                              'center_band_half_width_m': self.config.max_center_offset_m},
                            'road_building': {'weight': self.config.road_building_cost_weight,
                                              'distance_m': self.config.road_building_distance_m,
                                              'mask_received': 'building_png' in payload,
                                              'contact_count': len(road_area.building_contacts) if road_area is not None else 0},
                            'timing': reply.get('timing', {}),
                            'roundtrip_ms': (time.monotonic()-started)*1000}
            return estimates
        except Exception as error:
            detail = f'{type(error).__name__}: {error}'
            if isinstance(error, urllib.error.HTTPError):
                try:
                    detail += ': '+error.read(512).decode('utf-8', errors='replace')
                except Exception:
                    pass
                finally:
                    error.close()
            self.metrics = {'backend': 'connectome', 'ready': False, 'reason': 'connectome_unavailable',
                            'detail': detail[:300],
                            'roundtrip_ms': (time.monotonic()-started)*1000}
            return {mode: None for mode in (0, 1, 2)}


class ConnectomePathTracker:
    """No EMA, extrapolation or hold across a failed fresh-mask validation."""
    def __init__(self, max_age):
        self.max_age = max_age
        self.reset()

    def reset(self, *, preserve_branch=False):
        self._reference = (None, None)

    def update(self, estimate, timestamp_sec):
        # The inference worker and ROS timer run concurrently. Publish the
        # estimate and its arrival as one snapshot, including a missing path.
        self._reference = (estimate, timestamp_sec)

    def current(self, timestamp_sec):
        estimate, at = self._reference
        if estimate is None or at is None or not 0 <= timestamp_sec-at <= self.max_age:
            return None
        return SmoothedPath(estimate.points_xy.copy(), estimate.confidence,
                            timestamp_sec-at, 'connectome_surface_center')
