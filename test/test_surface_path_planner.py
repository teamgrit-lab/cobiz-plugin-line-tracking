"""Road HTTP isolation, exact legacy sidewalk paths and per-task surface latch."""
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_connectome import endpoint, strip
from connectome_client import ConnectomeClient
from connectome_surface import PlannerConfig
from local_path import (LocalPathConfig, DEFAULT_ROAD_ROI_POLYGON,
                        extract_sidewalk_centerline, selected_path_region)
from surface_path_planner import SurfacePathPlanner


def request(planner, mask, mode, *, generation=0, stamp=1., sequence=1):
    return planner.plan(np.zeros((*mask.shape, 3), np.uint8), mask, np.zeros_like(mask),
                        stamp, generation, sequence, mode=mode)


@pytest.mark.parametrize('mode', [0, 2])
def test_sidewalk_is_exactly_the_legacy_fit_and_never_contacts_the_server(endpoint, mode):
    url, service = endpoint
    config = LocalPathConfig()
    client = ConnectomeClient(url, config)
    client.url = 'http://127.0.0.1:1/v1/plan'
    planner = SurfacePathPlanner(client, config)
    mask = strip(surface=2)
    expected = extract_sidewalk_centerline(selected_path_region(mask, 2), config)
    result = request(planner, mask, mode)[mode]
    assert expected is not None and result is not None
    np.testing.assert_array_equal(result.points_xy, expected.points_xy)
    assert result.confidence == expected.confidence
    assert planner.metrics['backend'] == 'local' and planner.metrics['ready']
    assert service.encoder.frames == []
    # A local task must not be blocked by stale failure metrics on the road client.
    client.metrics = {'ready': False, 'reason': 'connectome_unavailable'}
    assert request(planner, mask, mode, stamp=1.5, sequence=2)[mode] is not None
    assert planner.metrics['backend'] == 'local' and planner.metrics['ready']


def test_road_requests_only_its_mode_and_does_not_validate_an_unused_sidewalk_reply(endpoint):
    url, service = endpoint
    config = replace(LocalPathConfig(), road_roi_polygon=DEFAULT_ROAD_ROI_POLYGON)
    process = service.process
    requested = []
    def unrelated_bad_sidewalk(payload):
        requested.append(payload['requested_modes'])
        reply = process(payload)
        assert reply['paths']['0'] is None and reply['paths']['2'] is None
        # The client must not let an unrequested surface veto a road path.
        reply['paths']['2'] = {'points_xy': [[-99, 999]], 'surface_class': 2}
        return reply
    service.process = unrelated_bad_sidewalk
    planner = SurfacePathPlanner(ConnectomeClient(url, config), config)
    result = request(planner, np.ones((360, 640), np.uint8), 1)
    assert result[1] is not None and result[0] is result[2] is None
    assert requested == [[1]] and len(service.encoder.frames) == 1
    assert planner.metrics['backend'] == 'connectome' and planner.metrics['ready']


def test_mode0_pins_road_through_outage_and_does_not_switch_to_sidewalk(endpoint):
    url, service = endpoint
    client = ConnectomeClient('http://127.0.0.1:1/v1/plan', LocalPathConfig())
    planner = SurfacePathPlanner(client, LocalPathConfig())
    assert request(planner, strip(), 0)[0] is None
    assert planner.union_surface == 1 and not planner.metrics['ready']
    client.url = url
    assert request(planner, strip(surface=2), 0, stamp=1.5, sequence=2)[0] is None
    assert service.planner.union_surface == planner.union_surface == 1
    assert planner.metrics['backend'] == 'connectome'
    assert request(planner, strip(), 0, stamp=2., sequence=3)[0] is not None
    count = len(service.encoder.frames)
    assert request(planner, strip(surface=2), 0, generation=1, stamp=2.5, sequence=4)[0] is not None
    assert planner.union_surface == 2 and planner.metrics['backend'] == 'local'
    assert len(service.encoder.frames) == count
    # Losing the latched sidewalk cannot turn a local task into a road task.
    assert request(planner, strip(), 0, generation=1, stamp=3., sequence=5)[0] is None
    assert planner.metrics['backend'] == 'local' and len(service.encoder.frames) == count


def test_perception_child_routes_sidewalk_without_any_http(monkeypatch):
    import best_so_far_runtime
    import connectome_client
    from perception_process import perception_worker
    class Client:
        config = PlannerConfig()
        def __init__(self, **_kwargs): pass
        def plan(self, *_args, **_kwargs): pytest.fail('sidewalk reached Connectome HTTP')
    monkeypatch.setattr(connectome_client, 'ConnectomeClient', Client)
    segment_calls = []
    def segment(_frame, **kwargs):
        segment_calls.append(kwargs)
        return NS(selected_mask=strip(surface=2), road_hazard_mask=np.zeros((360, 640), np.uint8))
    monkeypatch.setattr(best_so_far_runtime, 'BestSoFarSegmenter', lambda _cfg: NS(
        device=NS(type='cpu'), reset=lambda: None,
        segment=segment))
    packets = iter([{'reset': True, 'rgb': np.zeros((360, 640, 3), np.uint8),
                     'stamp': 1., 'generation': 0, 'sequence': 7, 'mode': 2}, None])
    replies = []
    pipe = NS(send=replies.append, recv=lambda: next(packets), close=lambda: None)
    perception_worker(pipe, NS(cuda_graph=False), {'local_config': LocalPathConfig()})
    assert replies[0]['ready']
    assert 'error' not in replies[1]
    assert replies[1]['planner']['backend'] == 'local'
    assert replies[1]['estimates'][2] is not None
    assert segment_calls == [{'color_order': 'rgb', 'detect_road_hazards': True, 'path_mask_class': 2}]
