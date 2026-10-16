"""Smooth corridor geometry, road direction and capture-time pose contracts."""
from dataclasses import replace
from pathlib import Path
import sys,math
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import local_path as lp
import swin_l_local_path_debug as debug
from road_curve import fit_curve,edge_direction,transform_points
from swin_l_drive_control import DriveConfig,RoadCenterState,DriveDecision,decide_drive

def corridor(center,width=.3):
    x=np.linspace(3.,8.,161);y=center(x)
    return np.column_stack((x,y-width,y+width))

def path(c):
    p,g=fit_curve(c)
    return lp.SmoothedPath(p,.95,0.,'test',road_center_tracking=True,
                           road_corridor=c,road_geometry=g)

def corner(p):
    return np.rad2deg(abs(np.diff(np.unwrap(np.arctan2(np.diff(p[:,1]),np.diff(p[:,0])))))).max()

def test_real_bend_can_curve_and_change_direction_without_a_corner():
    c=corridor(lambda x:.7*np.sin((x-3)*.8),.12)
    p=path(c)
    assert lp.road_path_within_corridor(p.points_xy,c)
    assert np.ptp(p.points_xy[:,1])>1.0
    assert corner(p.points_xy)<10.
    assert np.any(np.diff(p.points_xy[:,1])>0) and np.any(np.diff(p.points_xy[:,1])<0)

def test_temporal_history_is_soft_and_cannot_push_curve_into_fresh_grass():
    c=corridor(lambda x:np.full_like(x,-1.),.15)
    previous=np.column_stack((c[:,0],np.ones(len(c))))
    p,g=fit_curve(c,previous=previous,history_weight=10.)
    assert lp.road_path_within_corridor(p,c)
    assert corner(p)<5.

def test_inconsistent_widening_edges_do_not_invent_body_yaw():
    x=np.linspace(3.,8.,161)
    c=np.column_stack((x,-2.-.8*(x-3),2.+1.1*(x-3)))
    p=path(c);heading,confidence=edge_direction(c)
    assert confidence==0 and heading==0.
    out=decide_drive(p,camera_age_sec=0.,inference_age_sec=0.,config=DriveConfig(max_forward_mps=.8))
    assert out.yaw_rate==0. and out.vx==.8 and abs(out.vy)<=.25

def test_wrong_body_heading_can_recover_with_yaw_and_lateral_together():
    p=path(corridor(lambda x:.45*x+.6,.5))
    out=decide_drive(p,camera_age_sec=0.,inference_age_sec=0.,config=DriveConfig(max_forward_mps=.8))
    assert out.yaw_rate>0 and out.vy>0 and out.vx==.8
    assert abs(out.vy)<=.25

def test_heading_transition_is_rate_limited_in_both_directions():
    cfg=DriveConfig(max_forward_mps=.8);state=RoadCenterState()
    state.record_command(DriveDecision(.8,0.,.18,'tracking'),now=1.)
    p=path(corridor(lambda x:np.zeros_like(x),1.))
    out=decide_drive(p,camera_age_sec=0.,inference_age_sec=0.,config=cfg,road_state=state,now=1.1)
    assert out.yaw_rate==pytest.approx(.15) and out.vx==.8

def test_pose_transform_corrects_translation_and_yaw_without_fabricated_motion():
    p=path(corridor(lambda x:np.ones_like(x),1.));p=replace(p,road_geometry={**p.road_geometry,'observation_pose':(0.,0.,0.)})
    current=(.2,.1,.05);moved=lp.compensate_road_path(p,current)
    np.testing.assert_allclose(moved.points_xy,transform_points(p.points_xy,(0,0,0),current),atol=1e-6)
    assert moved.road_geometry['direction_heading_rad']==pytest.approx(-.05)
    assert moved.road_corridor is None
    assert lp.compensate_road_path(p,None) is p
    assert lp.compensate_road_path(replace(p,road_geometry=None),current).points_xy is p.points_xy

def test_impossible_or_short_corridor_does_not_emit_jagged_fallback():
    c=corridor(lambda x:np.zeros_like(x),.01)
    c[80:,1:]+=10.
    out=fit_curve(c)
    # A verified near prefix is permitted, a sharp jump is not.
    assert out is None or (out[1]['truncated'] and lp.road_path_within_corridor(out[0],c))


def test_single_frame_direction_jump_does_not_change_reference_or_start_turning():
    c=corridor(lambda x:np.zeros_like(x),2.)
    points,g=fit_curve(c)
    e=lp.LocalPathEstimate(points,.95,1.,4.,points,road_center_tracking=True,
                           road_corridor=c,road_geometry=g)
    sm=lp.LocalPathSmoother(lp.LocalPathConfig(unrestricted_path_mode=True))
    sm.update(e,0.)
    jumped=replace(e,road_geometry={**g,'direction_heading_rad':-.5})
    p=sm.update(jumped,.5)
    assert p.road_geometry['direction_pending']
    assert p.road_geometry['direction_heading_rad']==0.
    assert abs(np.diff(p.points_xy[:,1])).max()<.001
    confirmed=sm.update(jumped,1.)
    assert -.5<confirmed.road_geometry['direction_heading_rad']<0.
    normal=sm.update(e,1.5)
    assert abs(normal.road_geometry['direction_heading_rad'])<=abs(confirmed.road_geometry['direction_heading_rad'])


def test_capture_to_capture_body_rotation_does_not_freeze_corrective_direction():
    c=corridor(lambda x:np.zeros_like(x),2.)
    points,g=fit_curve(c)
    e=lp.LocalPathEstimate(points,.95,1.,4.,points,road_center_tracking=True,
                           road_corridor=c,road_geometry=g)
    result=lp.RoadPathResult(e,'regenerated','test',{},np.ones((2,2),np.uint8),np.array([3.,8.]),np.array([2.,-2.]))
    sm=lp.LocalPathSmoother(lp.LocalPathConfig(unrestricted_path_mode=True))
    sm.update_road(result,0.,(0.,0.,0.))
    moved=replace(e,road_geometry={**g,'direction_heading_rad':-.3})
    p=sm.update_road(replace(result,estimate=moved),.5,(.1,0.,.3))
    assert p.road_geometry['direction_heading_rad']==pytest.approx(-.3)
    assert not p.road_geometry['direction_pending']


def test_live_pose_lookup_uses_capture_stamp_and_rejects_stale_or_future_tf(monkeypatch):
    from types import SimpleNamespace as NS
    from sensor_fixtures import header
    from test_lidar_obstacles_ros import enabled_ros
    import swin_l_local_path_debug as debug
    ros=enabled_ros(monkeypatch)
    monkeypatch.setitem(debug.ENV,'LINE_TRACKING_AVOIDANCE_ENABLED','false')
    class Time:
        def __init__(self,nanoseconds=0):self.nanoseconds=nanoseconds
        @classmethod
        def from_msg(cls,stamp):return cls(stamp.sec*10**9+stamp.nanosec)
    monkeypatch.setitem(sys.modules,'rclpy.time',NS(Time=Time))
    def scenario(node):
        ros.now=2.1
        calls=[];age=[.1]
        class Buffer:
            def lookup_transform(self,target,source,stamp):
                calls.append((target,source,stamp.nanoseconds))
                sec=stamp.nanoseconds/1e9 if stamp.nanoseconds else ros.clock_ns()/1e9-age[0]
                return NS(header=header(sec),transform=NS(translation=NS(x=sec,y=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
        node.road_tf_buffer=Buffer()
        capture=ros.clock_ns()/1e9-.6
        assert node.road_pose(header(capture))[0]==pytest.approx(capture)
        assert calls[-1][2]==round(capture*1e9)
        assert node.road_pose()[0]==pytest.approx(ros.clock_ns()/1e9-.1)
        assert calls[-1][2]==0
        age[0]=.3;assert node.road_pose() is None
        age[0]=-.1;assert node.road_pose() is None
    ros.run(scenario,'--path-mask-class','1')


def test_curved_corridor_can_turn_when_one_boundary_direction_is_uncertain():
    # Unequal boundary slopes suppress direct edge direction. The constrained
    # center curve still has a persistent left turn that cannot be ignored.
    x=np.linspace(3.,8.,161)
    center=.3*(x-3.)**2
    c=np.column_stack((x,center-.2,center+1.5+2.5*(x-3.)))
    p=path(c)
    assert p.road_geometry['direction_source']=='corridor_tangent'
    assert p.road_geometry['direction_heading_rad']>math.radians(8)
    out=decide_drive(p,camera_age_sec=0.,inference_age_sec=0.,config=DriveConfig(max_forward_mps=.8))
    assert out.yaw_rate>0. and out.vx==.8 and abs(out.vy)<=.25


def test_small_target_bearings_do_not_hide_a_confirmed_wrong_body_heading():
    # The centerline crosses ahead of the robot, so endpoint bearings are small
    # despite a 26-degree direction error. A lateral-only lock would persist.
    p=path(corridor(lambda x:.5*(x-5.),.5))
    state=RoadCenterState();state.translating=True
    state.record_command(DriveDecision(.8,0.,0.,'tracking'),now=0.)
    out=decide_drive(p,camera_age_sec=0.,inference_age_sec=0.,config=DriveConfig(max_forward_mps=.8),road_state=state,now=.1)
    assert out.yaw_rate>0. and out.vx==.8 and not state.translating


def test_pose_availability_change_does_not_mix_unaligned_history():
    def result(offset):
        c=corridor(lambda x:np.full_like(x,offset),2.)
        points,g=fit_curve(c)
        e=lp.LocalPathEstimate(points,.95,1.,4.,points,road_center_tracking=True,
                               road_corridor=c,road_geometry=g,road_clearance={})
        return lp.RoadPathResult(e,'regenerated','test',{},np.ones((2,2),np.uint8),
                                np.array([3.,8.]),np.array([2.,-2.]))
    sm=lp.LocalPathSmoother(lp.LocalPathConfig(unrestricted_path_mode=True))
    sm.update_road(result(1.),0.,(0.,0.,0.))
    unknown=sm.update_road(result(0.),.5,None)
    assert not unknown.road_clearance['history_replanned']
    assert np.median(unknown.points_xy[:,1])==pytest.approx(0.,abs=1e-4)
    known=sm.update_road(result(-.5),1.,(1.,1.,.3))
    assert not known.road_clearance['history_replanned']
    assert np.median(known.points_xy[:,1])==pytest.approx(-.5,abs=1e-4)
