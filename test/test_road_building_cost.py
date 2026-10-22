"""Building costs rank existing safe routes; they do not invent entrance masks."""
from dataclasses import replace
import numpy as np
import pytest
from test_connectome import endpoint, FLOW
from test_road_corridor import corridor
from connectome_client import ConnectomeClient
from connectome_surface import PlannerConfig, SurfaceArea, SurfacePlanner
from local_path import LocalPathConfig
from surface_path_planner import SurfacePathPlanner


def scene():
    selected = np.ones((360, 640), np.uint8)
    building = np.zeros_like(selected)
    building[:300, 70:160] = 1
    selected[building != 0] = 0
    return selected, building


def test_building_cost_moves_path_away_without_changing_road_support():
    selected, building = scene(); hazard = np.zeros_like(selected)
    pc = PlannerConfig(road_building_distance_m=2.)
    before = SurfaceArea(selected, hazard, 1, LocalPathConfig(), pc)
    after = SurfaceArea(selected, hazard, 1, LocalPathConfig(), pc, building=building)
    p, q = before.plan(FLOW), after.plan(FLOW)
    assert p is not None and q is not None and len(after.building_contacts) > 0
    np.testing.assert_array_equal(before.allowed, after.allowed)
    pp, qq = np.asarray(p['points_xy']), np.asarray(q['points_xy'])
    assert len(pp) == len(qq) and before.accepts(qq)
    assert after.building_cost(qq) < after.building_cost(pp)
    assert np.mean(qq[:, 1]) < np.mean(pp[:, 1])  # away from left-hand building


def test_missing_empty_or_disabled_building_cost_preserves_points():
    selected, building = scene(); hazard = np.zeros_like(selected); lc = LocalPathConfig()
    pc = PlannerConfig(); baseline = SurfaceArea(selected, hazard, 1, lc, pc).plan(FLOW)
    for mask, config in [(np.zeros_like(building), pc), (building, replace(pc, road_building_cost_weight=0.))]:
        assert SurfaceArea(selected, hazard, 1, lc, config, building=mask).plan(FLOW)['points_xy'] == baseline['points_xy']


def test_roof_without_observed_road_at_its_base_has_no_ground_contacts():
    selected = np.ones((360,640),np.uint8); selected[:240] = 0
    building = np.zeros_like(selected); building[20:140,120:500] = 1
    area = SurfaceArea(selected,np.zeros_like(selected),1,LocalPathConfig(),PlannerConfig(),building=building)
    assert len(area.building_contacts) == 0


@pytest.mark.parametrize('sign', [-1, 1])
def test_true_bends_keep_their_direction_and_supported_length(sign):
    selected = corridor('bend',sign); building = np.zeros_like(selected)
    building[:180, :220] = selected[:180, :220] == 0
    hazard = np.zeros_like(selected); pc = PlannerConfig()
    p = SurfaceArea(selected,hazard,1,LocalPathConfig(),pc).plan(FLOW)
    area = SurfaceArea(selected,hazard,1,LocalPathConfig(),pc,building=building); q = area.plan(FLOW)
    assert p is not None and q is not None and len(q['points_xy']) == len(p['points_xy'])
    assert sign*q['points_xy'][-1][1] > 1. and area.accepts(np.asarray(q['points_xy'],np.float32))


def test_http_transports_building_and_only_explicit_mode1_uses_it(endpoint):
    url, service = endpoint; selected, building = scene(); hazard = np.zeros_like(selected)
    client = ConnectomeClient(url,LocalPathConfig(),PlannerConfig(road_building_distance_m=2.))
    p = client.plan(np.zeros((360,640,3),np.uint8),selected,hazard,1.,0,1,building=building)
    assert p[1] is not None and client.metrics['road_building']['contact_count'] > 0
    assert client.metrics['road_building']['mask_received']
    common = SurfacePlanner().plan(selected,hazard,LocalPathConfig(),client.config,FLOW)
    actual = SurfacePlanner().plan(selected,hazard,LocalPathConfig(),client.config,FLOW,building=building)
    assert common['0'] == actual['0'] and common['2'] == actual['2']
    assert common['1']['points_xy'] != actual['1']['points_xy']


def test_sidewalk_stays_local_even_with_buildings_and_offline_server():
    mask = np.full((360,640),2,np.uint8); lc = LocalPathConfig()
    client = ConnectomeClient('http://127.0.0.1:1/v1/plan',lc)
    planner = SurfacePathPlanner(client,lc)
    result = planner.plan(np.zeros((360,640,3),np.uint8),mask,np.zeros_like(mask),1.,0,1,mode=2,building=np.ones_like(mask))
    assert result[2] is not None and planner.metrics['backend'] == 'local'


def test_old_server_cannot_silently_ignore_requested_cost(endpoint):
    url, service = endpoint; process = service.process
    def old_reply(request):
        reply = process(request); reply.pop('building_cost_applied'); return reply
    service.process = old_reply
    selected, building = scene(); client = ConnectomeClient(url,LocalPathConfig())
    assert client.plan(np.zeros((360,640,3),np.uint8),selected,np.zeros_like(selected),1.,0,1,modes=(1,),building=building)[1] is None
    assert 'does not support' in client.metrics['detail']


@pytest.mark.parametrize('values', [dict(road_building_cost_weight=-.1),dict(road_building_cost_weight=float('nan')),
    dict(road_building_cost_weight=3),dict(road_building_distance_m=0),dict(road_building_distance_m=6)])
def test_invalid_cost_configuration_is_rejected(values):
    with pytest.raises(ValueError): PlannerConfig(**values).validate()
