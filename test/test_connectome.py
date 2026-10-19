"""Mask containment, actual HTTP exchange, failure and semantic policy contracts."""
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from connectome_client import ConnectomeClient, ConnectomePathTracker
from connectome_server import PlannerService, make_server
from connectome_surface import FlowField, LocalPathConfig, PlannerConfig, SurfaceArea, SurfacePlanner
from local_path import ground_to_pixel, pixel_to_ground_homography


def strip(left=1.5, right=-1.5, surface=1, near=3., far=8.):
    cfg = LocalPathConfig()
    p = ground_to_pixel(np.array([[near, left], [near, right], [far, right], [far, left]], np.float32),
                        pixel_to_ground_homography((360, 640), cfg))
    mask = np.zeros((360, 640), np.uint8)
    cv2.fillPoly(mask, [np.rint(p).astype(np.int32)], surface)
    return mask


FLOW = FlowField(np.array([[320., 280.]]), np.array([0.]))


@pytest.mark.parametrize('surface', [1, 2])
def test_both_surfaces_get_centred_current_mask_paths(surface):
    selected = strip(surface=surface)
    result = SurfacePlanner().plan(selected, np.zeros_like(selected), LocalPathConfig(), PlannerConfig(), FLOW)
    p = result[str(surface)]
    assert p is not None and result[str(3-surface)] is None
    points = np.array(p['points_xy'])
    assert abs(np.median(points[:, 1])) < .06
    assert SurfaceArea(selected, np.zeros_like(selected), surface, LocalPathConfig(), PlannerConfig()).accepts(points)


def test_narrow_surface_and_close_full_width_hazard_stop():
    cfg, pc = LocalPathConfig(), PlannerConfig()
    narrow = strip(.4, -.4)
    assert SurfacePlanner().plan(narrow, np.zeros_like(narrow), cfg, pc, FLOW)['1'] is None
    road = strip()
    hazard = strip(2., -2., near=3.6, far=4.1)
    assert SurfacePlanner().plan(road, hazard, cfg, pc, FLOW)['1'] is None


def test_path_never_extrapolates_over_a_future_gap_or_disconnected_patch():
    road = strip()
    road[strip(2., -2., near=5., far=5.4).astype(bool)] = 0
    p = SurfacePlanner().plan(road, np.zeros_like(road), LocalPathConfig(), PlannerConfig(), FLOW)['1']
    assert p is not None and np.array(p['points_xy'])[-1, 0] < 4.7


def test_union_latches_current_surface_until_new_task_instead_of_crossing_classes():
    planner = SurfacePlanner()
    road = strip()
    first = planner.plan(road, np.zeros_like(road), LocalPathConfig(), PlannerConfig(), FLOW)
    assert first['0']['surface_class'] == 1
    sidewalk = strip(surface=2)
    second = planner.plan(sidewalk, np.zeros_like(road), LocalPathConfig(), PlannerConfig(), FLOW)
    assert second['0'] is None and second['2'] is not None
    fresh = SurfacePlanner().plan(sidewalk, np.zeros_like(road), LocalPathConfig(), PlannerConfig(), FLOW)
    assert fresh['0']['surface_class'] == 2


def test_union_prefers_occupied_wide_road_over_a_closer_sidewalk_centroid():
    selected = strip(6., -.6)
    sidewalk = strip(-.65, -1.75, surface=2)
    selected[sidewalk == 2] = 2
    result = SurfacePlanner().plan(selected, np.zeros_like(selected), LocalPathConfig(), PlannerConfig(), FLOW)
    assert result['0']['surface_class'] == 1


def test_flow_can_rank_candidates_but_cannot_pull_them_out_of_the_centre_band():
    road, cfg, pc = strip(), LocalPathConfig(), PlannerConfig()
    area = SurfaceArea(road, np.zeros_like(road), 1, cfg, pc)
    xs = area.xs[area.rows]
    pixels, costs = [], []
    for offset in np.linspace(-.2, .2, 5):
        p = np.column_stack((xs, area.centers+offset))
        pixels.extend(ground_to_pixel(p, area.h))
        costs.extend(np.full(len(xs), 0. if offset > .05 else 1.))
    path = area.plan(FlowField(np.array(pixels), np.array(costs)))
    points = np.array(path['points_xy'])
    assert .05 < np.median(points[:, 1]) <= .20
    assert area.accepts(points)
    assert not area.accepts(points+[0, 1.])


class EncoderStub:
    """Only HTTP/geometry tests use this stub; real-model smoke is separate."""
    def __init__(self):
        self.resets, self.frames = 0, []
        self.details = {'backend': 'flyvis', 'pretrained_task': 'optical_flow',
                        'flow_decoder_loaded': True, 'bundle_sha256': 'a'*64}

    def reset(self):
        self.resets += 1

    def advance(self, image, stamp):
        self.frames.append((image.copy(), stamp))
        return FLOW


@pytest.fixture
def endpoint():
    service = PlannerService(EncoderStub())
    server = make_server(service, ('127.0.0.1', 0))
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    yield f'http://127.0.0.1:{server.server_port}/v1/plan', service
    server.shutdown(); server.server_close(); thread.join(2)


def invoke(client, selected=None, **kwargs):
    selected = strip() if selected is None else selected
    image = np.full((*selected.shape, 3), [5, 12, 88], np.uint8)
    return client.plan(image, selected, np.zeros_like(selected), kwargs.get('stamp', 1.),
                       kwargs.get('generation', 0), kwargs.get('sequence', 1))


def test_http_transmits_original_image_and_labels_then_revalidates_and_resets(endpoint):
    url, service = endpoint
    client = ConnectomeClient(url, LocalPathConfig())
    result = invoke(client)
    assert result[0] is not None and result[1] is not None and result[2] is None
    np.testing.assert_array_equal(service.encoder.frames[0][0][0, 0], [5, 12, 88])
    assert client.metrics['ready'] and service.encoder.resets == 1
    # A same-frame replay cannot advance the neural state or refresh a path.
    assert all(v is None for v in invoke(client).values())
    assert not client.metrics['ready']
    assert len(service.encoder.frames) == 1
    assert invoke(client, strip(surface=2), generation=1)[0] is not None
    assert client.union_surface == 2 and service.encoder.resets == 2


@pytest.mark.parametrize('mutation', ['wrong_frame', 'wrong_class', 'outside', 'not_flyvis'])
def test_bad_planner_response_cannot_become_a_reference(endpoint, mutation):
    url, service = endpoint
    process = service.process
    def corrupt(request):
        reply = process(request)
        if mutation == 'wrong_frame': reply['request_id'] = 'old-task'
        if mutation == 'wrong_class': reply['paths']['1']['surface_class'] = 2
        if mutation == 'outside': reply['paths']['1']['points_xy'][2][1] = 10.
        if mutation == 'not_flyvis': reply['model']['backend'] = 'classical'
        return reply
    service.process = corrupt
    client = ConnectomeClient(url, LocalPathConfig())
    assert all(v is None for v in invoke(client).values())
    assert not client.metrics['ready']


def test_offline_planner_drops_reference_instead_of_holding_or_searching(endpoint):
    url, _ = endpoint
    client = ConnectomeClient(url, LocalPathConfig())
    result = invoke(client)
    tracker = ConnectomePathTracker(.3)
    tracker.update(result[1], 1.)
    assert tracker.current(1.1) is not None
    assert tracker.current(1.31) is None
    client.url = 'http://127.0.0.1:1/v1/plan'
    tracker.update(invoke(client)[1], 1.2)
    assert tracker.current(1.21) is None


def test_slow_http_response_times_out_without_a_geometric_fallback(endpoint):
    url, service = endpoint
    advance = service.encoder.advance
    def delayed(image, stamp):
        time.sleep(.08)
        return advance(image, stamp)
    service.encoder.advance = delayed
    client = ConnectomeClient(url, LocalPathConfig(), timeout=.02)
    assert all(v is None for v in invoke(client).values())
    assert not client.metrics['ready'] and 'Timeout' in client.metrics['detail']


@pytest.mark.parametrize('kwargs', [{'margin_m': float('nan')}, {'half_width_m': 0}, {'flow_weight': 100}])
def test_invalid_geometry_is_rejected(kwargs):
    with pytest.raises(ValueError):
        PlannerConfig(**kwargs).validate()
