"""Side-opening suppression must not invent free ground or flatten real bends."""
from dataclasses import replace
import cv2
import numpy as np
import pytest
from test_connectome import endpoint, FLOW
from connectome_client import ConnectomeClient
from connectome_surface import PlannerConfig, SurfaceArea, road_corridor_reference
from local_path import LocalPathConfig, ground_to_pixel, pixel_to_ground_homography


def corridor(kind, sign=1, surface=1):
    xs = np.linspace(3., 8., 161)
    if kind == 'entrance':
        center = np.zeros_like(xs)
        left = center + 1.5
        right = center - 1.5 - 1.8*np.maximum(0, 1-abs(xs-5.6)/1.2)
    else:
        center = sign*.50*np.maximum(0, xs-4.5)
        left, right = center+1.35, center-1.35
    points = np.vstack((np.column_stack((xs, left)), np.column_stack((xs[::-1], right[::-1]))))
    xy = ground_to_pixel(points, pixel_to_ground_homography((360, 640), LocalPathConfig()))
    selected = np.zeros((360, 640), np.uint8)
    cv2.fillPoly(selected, [np.rint(xy).astype(np.int32)], surface)
    return selected


def area(mask, enabled=True, surface=1):
    return SurfaceArea(mask, np.zeros_like(mask), surface, LocalPathConfig(),
                       PlannerConfig(road_corridor_smoothing_m=.3 if enabled else 0.))


def test_entrance_midpoint_detour_is_rejected_even_though_the_floor_is_road():
    mask = corridor('entrance');old, new = area(mask, False), area(mask)
    before, after = old.plan(FLOW), new.plan(FLOW)
    assert before is not None and after is not None
    p, q = np.array(before['points_xy']), np.array(after['points_xy'])
    assert np.max(abs(q[:, 1])) < np.max(abs(p[:, 1]))*.85
    assert new.accepts(p, require_center=False)  # still observed road, including the apron
    assert not new.accepts(p)  # route-policy band excludes the side detour
    assert new.accepts(q) and np.any(new.excluded_path_centers())


@pytest.mark.parametrize('sign', [-1, 1])
def test_required_left_and_right_bends_remain_supported(sign):
    mask = corridor('bend', sign);old, new = area(mask, False), area(mask)
    before, after = old.plan(FLOW), new.plan(FLOW)
    assert before is not None and after is not None
    p, q = np.array(before['points_xy']), np.array(after['points_xy'])
    assert q[-1, 0] >= p[-1, 0]-.2
    assert sign*q[-1, 1] > 1.
    assert new.accepts(q)


def test_reference_respects_observed_bounds_and_can_be_disabled():
    centers = np.array([0., -.7, 0.]);bounds = [(-1., 1.), (-.9, -.4), (-1., 1.)]
    r = road_corridor_reference(centers, bounds, .1, .3)
    assert -.9 <= r[1] <= -.4
    np.testing.assert_array_equal(road_corridor_reference(centers, bounds, .1, 0), centers)


def test_sidewalk_geometry_and_paths_are_identical_with_the_road_policy_enabled():
    selected = corridor('bend', surface=2)
    old, new = area(selected, False, 2), area(selected, True, 2)
    np.testing.assert_array_equal(old.centers, new.centers)
    np.testing.assert_array_equal(old.allowed, new.allowed)
    assert old.plan(FLOW) == new.plan(FLOW)


def test_every_returned_path_remains_valid_after_client_float32_conversion():
    for phase in np.linspace(0, np.pi, 12):
        mask = corridor('entrance')
        # Change observed support at grid boundaries without adding free ground.
        mask[:180, int(290+15*np.sin(phase)):int(308+15*np.sin(phase))] = 0
        observed = area(mask)
        path = observed.plan(FLOW)
        if path is not None:
            points = np.asarray(path['points_xy'], np.float32)
            np.testing.assert_array_equal(points.astype(float), path['points_xy'])
            assert observed.accepts(points)


def test_client_rejects_an_http_server_that_returns_the_old_entrance_detour(endpoint):
    url, service = endpoint;process = service.process
    def ignore_policy(request):
        request['planner_config']['road_corridor_smoothing_m'] = 0.
        return process(request)
    service.process = ignore_policy
    selected = corridor('entrance');client = ConnectomeClient(url, LocalPathConfig())
    image = np.zeros((*selected.shape, 3), np.uint8)
    assert client.plan(image, selected, np.zeros_like(selected), 1., 0, 1, modes=(1,))[1] is None
    assert not client.metrics['ready'] and 'violates selected surface' in client.metrics['detail']


@pytest.mark.parametrize('value', [-.01, 2., float('nan')])
def test_invalid_corridor_length_is_rejected(value):
    with pytest.raises(ValueError):
        PlannerConfig(road_corridor_smoothing_m=value).validate()
