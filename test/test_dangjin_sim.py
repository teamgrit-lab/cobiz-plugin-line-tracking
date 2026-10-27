"""Simulation transport and stop contracts. Browser traces cover real Flyvis."""
import base64
from dataclasses import asdict
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from connectome_surface import FlowField, PlannerConfig
from local_path import LocalPathConfig
from dangjin_sim_server import SimulationService, forbidden_path
from export_dangjin_navigation import raster
from dangjin_patrol import Patrol,track_camera_path


class TestEncoder:
    __test__=False
    details={'backend':'test-only'}
    def reset(self):pass
    def advance(self,image,stamp):return FlowField(np.array([[240.,220.]]),np.zeros(1))


class TestPerception:
    __test__=False
    details={'backend':'test-only','uses_ground_truth_at_runtime':False}
    def __init__(self,road=True):self.road=road
    def infer(self,image):
        mask=np.full(image.shape[:2],int(self.road),np.uint8)
        return mask,np.zeros_like(mask),{'ms':0.,'road_fraction':float(self.road)}


def png(image):return base64.b64encode(cv2.imencode('.png',image)[1]).decode()


def request():
    mask=np.ones((270,480),np.uint8)
    return json.loads(json.dumps(dict(schema='line_tracking.connectome.v1',stream_id='test',request_id='test:1',stamp_sec=1.,
        requested_modes=[1],pose={'x':220,'y':184,'heading':0},speed_mps=.5,
        local_config=asdict(LocalPathConfig()),planner_config=asdict(PlannerConfig()),
        image_png=png(np.zeros((270,480,3),np.uint8)))))


def service(road=True):return SimulationService(TestEncoder(),json.loads((ROOT/'environments/dangjin/site.json').read_text()),TestPerception(road),json.loads((ROOT/'environments/dangjin/world.json').read_text()))


def test_json_lists_revalidate_successfully_and_produce_bounded_command():
    result=service().step(request())
    assert 0<result['command']['vx']<=.5
    assert abs(result['command']['yaw_rate'])<=.18
    assert result['simulation_only'] and not result['perception']['uses_ground_truth_at_runtime']


def test_no_go_overrides_a_falsely_drivable_mask():
    r=request();r['pose']={'x':145.65,'y':0,'heading':np.pi/2}
    result=service().step(r)
    assert result['command']['vx']==0 and result['command']['reason']=='no_go_zone'


def test_loss_of_surface_stops_instead_of_dead_reckoning():
    assert service(False).step(request())['command']['vx']==0


def test_simulator_cannot_sneak_a_ground_truth_route_into_the_model():
    r=request();r['target_xy']=[5,0]
    with pytest.raises(ValueError,match='RGB only'):service().step(r)


def test_runtime_rejects_injected_semantic_truth():
    r=request();r['selected_png']=png(np.ones((270,480),np.uint8))
    with pytest.raises(ValueError,match='RGB only'):service().step(r)


def test_360_detector_receives_four_images_without_pose_or_map_inputs():
    class Surround(TestPerception):
        def predict_views(self,frames):
            assert len(frames)==4 and all(f.shape==(270,480,3) for f in frames)
            self.received=True
            mask,_,metrics=self.infer(frames[0]);return np.repeat(mask[None],4,axis=0),metrics
    s=service();s.perception=Surround();r=request();r['views_png']=[r['image_png']]*4
    result=s.step(r)
    assert s.perception.received and result['surround_prediction_png']
    r['views_png']=r['views_png'][:3]
    with pytest.raises(ValueError,match='four ordered'):s.step(r)
    r['views_png']=[png(np.ones((270,480,3),np.uint8))]+[r['image_png']]*3
    with pytest.raises(ValueError,match='view 0'):s.step(r)


def test_dropout_modifies_the_planner_mask_stops_and_then_recovers():
    s=service();r=request();r['sensor_faults']={'enabled':True,'profile':'normal','seed':47}
    dropout=False;recovered=False;holes=False
    for i in range(70):
        r['stamp_sec']=1+i*.5;r['request_id']=f'fault:{i}'
        result=s.step(r);faults=result['sensor_faults'];command=result['command']
        holes|=faults['kind']=='holes' and faults['removed_fraction']>0
        if faults['kind']=='dropout':
            dropout=True;assert command['vx']==command['vy']==command['yaw_rate']==0
            assert command['reason']=='detection_dropout' and result['paths']['1'] is None
            assert result['perception_metrics']['raw_road_fraction']==1
            assert result['perception_metrics']['road_fraction']==0
        elif dropout and faults['kind']=='clear' and command['vx']>0:recovered=True
    assert dropout and recovered and holes
    assert s.telemetry()['sensor_faults']['seed']==47
    r['sensor_faults']['seed']=48
    with pytest.raises(ValueError,match='new test'):s.step(r)


def test_whole_lap_accounting_requires_ordered_observed_movement():
    world=json.loads((ROOT/'environments/dangjin/world.json').read_text());patrol=Patrol(world['perimeter_route'])
    for p in world['perimeter_route']:
        state=patrol.update({'x':p[0],'y':p[1],'heading':0})
        # Every intermediate state, including the final approach, must cross
        # the HTTP boundary without leaking a NumPy boolean.
        json.dumps(state,allow_nan=False)
    assert state['complete'] and state['fraction']==1 and state['checkpoints']>=48
    fresh=Patrol(world['perimeter_route']);p=world['perimeter_route'][0]
    for _ in range(100):state=fresh.update({'x':p[0],'y':p[1],'heading':0})
    assert not state['complete']
    with pytest.raises(ValueError,match='pose jump'):fresh.update({'x':250,'y':150,'heading':0})


def test_return_gate_accepts_lateral_tracking_error_after_an_ordered_lap():
    world=json.loads((ROOT/'environments/dangjin/world.json').read_text());patrol=Patrol(world['perimeter_route'])
    normal=np.array([-patrol.tangent[1],patrol.tangent[0]])
    for p in np.asarray(world['perimeter_route'])+normal:
        state=patrol.update({'x':float(p[0]),'y':float(p[1]),'heading':0})
    p=patrol.points[0]+normal+.4*patrol.tangent
    state=patrol.update({'x':float(p[0]),'y':float(p[1]),'heading':0})
    assert state['complete'] and state['fraction']==1
    assert abs(state['return_gate_lateral_m']-1)<1e-8
    json.dumps(state,allow_nan=False)


def test_pursuit_rotates_camera_path_back_to_robot_and_stops_on_loss():
    points=np.array([[2.5,0],[3.5,0],[6.,0]])
    assert track_camera_path(points,.3).yaw_rate>0
    assert track_camera_path(points,-.3).yaw_rate<0
    assert track_camera_path(None,0).vx==0


def test_swept_path_checks_between_waypoints_and_entire_footprint():
    zones=[dict(x=0,y=0,length=1,width=1)]
    assert forbidden_path([[-2,.5],[3,.5]],zones,.65)
    assert forbidden_path([[-.6,0]],zones,.65)
    assert not forbidden_path([[-2,-2],[-2,3]],zones,.65)


def test_neural_mode_does_not_override_actor_for_dropout_or_no_go(monkeypatch):
    from swin_l_drive_control import DriveDecision
    class Surround(TestPerception):
        def predict_views(self,frames):
            return np.ones((4,270,480),np.uint8),{'ms':1.,'road_fraction':1.}
    class Actor:
        details={'backend':'test-neural-actor'}
        def infer(self,masks):
            self.last_masks=masks.copy()
            return DriveDecision(.4,0.,.1,'neural_drive'),{'ms':1.,'override':False}
    s=service();s.perception=Surround();s.policy=Actor()
    def forbidden_call(*args,**kwargs):raise AssertionError('classical planner must not run')
    monkeypatch.setattr(s.service,'process',forbidden_call)
    monkeypatch.setattr('dangjin_patrol.track_camera_path',forbidden_call)
    r=request();r['pose']={'x':145.65,'y':94.,'heading':0.}
    r['views_png']=[r['image_png']]*4;r['sensor_faults']={'enabled':True,'profile':'severe','seed':47}
    saw_dropout=False
    for t in np.arange(1.,15.,.5):
        r['stamp_sec']=float(t);reply=s.step(r)
        assert reply['command']['vx']==.4 and reply['command']['yaw_rate']==.1
        assert not reply['emergency_stop_override'] and reply['paths']=={}
        if reply['sensor_faults']['kind']=='dropout':
            saw_dropout=True;assert not s.policy.last_masks.any()
    assert saw_dropout


def test_neural_patrol_records_route_deviation_without_correcting_pose():
    world=json.loads((ROOT/'environments/dangjin/world.json').read_text())
    patrol=Patrol(world['perimeter_route'],strict=False,guidance=False);p=patrol.points[0].copy()
    patrol.update({'x':float(p[0]),'y':float(p[1]),'heading':0.})
    for _ in range(15):
        p[0]-=.5
        state=patrol.update({'x':float(p[0]),'y':float(p[1]),'heading':0.})
    assert state['cross_track_m']>4 and not state['complete']
    assert 'target_site' not in state


def test_vision_only_patrol_never_computes_a_target(monkeypatch):
    world=json.loads((ROOT/'environments/dangjin/world.json').read_text())
    def forbidden(*args,**kwargs):raise AssertionError('map target generation forbidden')
    monkeypatch.setattr(np,'interp',forbidden)
    patrol=Patrol(world['perimeter_route'],strict=False,guidance=False)
    p=patrol.points[0]
    state=patrol.update({'x':float(p[0]),'y':float(p[1]),'heading':0.})
    assert 'target_site' not in state and not state['complete']


def test_navigation_export_uses_same_exclusions_and_radius():
    world=json.loads((ROOT/'environments/dangjin/world.json').read_text())
    free,origin=raster(world,radius=.65)
    def available(x,y):return free[free.shape[0]-1-int((y-origin[1])/.25),int((x-origin[0])/.25)]
    assert not available(145.65,94)
    assert not available(25,25)
    assert available(204,158)
    assert not available(-100,0)


def test_navigation_grid_keeps_subcell_silo_and_lamp_supports():
    world=json.loads((ROOT/'environments/dangjin/world.json').read_text())
    free,origin=raster(world,radius=0.)
    for box in world['obstacles']:
        if 'silo-' not in box['id'] and not box['id'].startswith('lamp-'):continue
        x,y=box['x']+box['length']/2,box['y']+box['width']/2
        row=free.shape[0]-1-int((y-origin[1])/.25);col=int((x-origin[0])/.25)
        assert not free[row,col],box['id']
