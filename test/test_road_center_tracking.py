"""Road centering contracts: wide observed corridor, lateral recovery, mode 3 priority."""
from dataclasses import replace
from pathlib import Path
import sys
import math

import cv2
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import local_path as lp
import swin_l_local_path_debug as debug
from swin_l_drive_control import DriveConfig, DriveDecision, RoadCenterState, decide_drive, road_path_errors


def road_path(offset=0., slope=0., curve=0.):
    xs=np.linspace(3.,8.,20)
    return lp.SmoothedPath(np.column_stack((xs,offset+slope*xs+curve*(xs-3.)**2)),
                           .95,0.,'test',road_center_tracking=True)


def command(path,config=DriveConfig(),**kwargs):
    return decide_drive(path,camera_age_sec=0.,inference_age_sec=0.,config=config,**kwargs)


def projected_road(config,left,right,near=3.,far=8.):
    h=lp.pixel_to_ground_homography((360,640),config)
    polygon=lp.ground_to_pixel(np.array([[near,left],[near,right],[far,right],[far,left]],np.float32),h)
    mask=np.zeros((360,640),np.uint8)
    cv2.fillPoly(mask,[np.rint(polygon).astype(np.int32)],1)
    return mask


def test_road_midpoint_uses_edges_beyond_the_old_search_width():
    config=lp.LocalPathConfig(close_kernel_px=0,branch_preference='center')
    mask=projected_road(config,6.,-2.)
    before=lp.extract_sidewalk_centerline(mask,config)
    after=lp.extract_road_centerline(mask,config)
    assert before is not None and after is not None
    # The old +/-3.5m grid sees only part of the left half.
    assert np.median(before.points_xy[:,1]) < 1.0
    assert np.median(after.points_xy[:,1]) == pytest.approx(2.,abs=.10)
    assert after.road_center_tracking
    assert after.road_corridor.shape == (config.path_points, 3)
    assert np.all(after.road_corridor[:, 1] <= after.points_xy[:, 1])
    assert np.all(after.road_corridor[:, 2] >= after.points_xy[:, 1])
    assert after.mean_sidewalk_width_m > 7.8
    assert after.points_xy[0,0] >= after.raw_points_xy[:,0].min()
    assert after.points_xy[-1,0] <= after.raw_points_xy[:,0].max()


def test_road_roi_does_not_change_distance_calibration_or_sidewalk():
    config=lp.LocalPathConfig()
    narrow=replace(config,road_roi_polygon=config.roi_polygon)
    assert np.array_equal(lp.pixel_to_ground_homography((360,640),config),
                          lp.pixel_to_ground_homography((360,640),narrow))
    mask=projected_road(config,1.,-1.)
    assert np.array_equal(lp.extract_sidewalk_centerline(mask,config).points_xy,
                          lp.extract_sidewalk_centerline(mask,narrow).points_xy)
    selected=mask*2
    estimates=debug.extract_path_estimates(selected,(0,1,2),config)
    assert estimates[1] is None
    assert estimates[0] is not None and not estimates[0].road_center_tracking
    assert estimates[2] is not None and not estimates[2].road_center_tracking


@pytest.mark.parametrize('polygon',[(0.,)*8,(float('nan'),)*8,(0.,0.,1.,1.,0.,1.,1.,0.)])
def test_invalid_road_roi_is_rejected(polygon):
    with pytest.raises(ValueError):replace(lp.LocalPathConfig(),road_roi_polygon=polygon).validate()


def test_road_always_smooths_jitter_even_with_unrestricted_mode():
    config=lp.LocalPathConfig(unrestricted_path_mode=True)
    smoother=lp.LocalPathSmoother(config)
    def estimate(offset):
        p=road_path(offset).points_xy
        return lp.LocalPathEstimate(p,.95,1.,4.,p,road_center_tracking=True)
    smoother.update(estimate(0.),0.)
    path=smoother.update(estimate(1.),.25)
    assert 0. < path.points_xy[:,1].mean() < .35
    assert path.road_center_tracking and path.source=='smoothed_update'
    smoother.reset()
    assert smoother.current(.3) is None
    # Non-road unrestricted behavior still accepts the raw latest estimate.
    path=smoother.update(replace(estimate(1.),road_center_tracking=False),.4)
    assert np.allclose(path.points_xy[:,1],1.) and path.source=='raw_latest'


@pytest.mark.parametrize('offset',[-2.,-.2,.2,2.])
def test_parallel_road_recovers_laterally_without_rotation(offset):
    out=command(road_path(offset))
    assert out.vx==.5 and out.yaw_rate==0.
    assert np.sign(out.vy)==np.sign(offset)
    assert abs(out.vy)<=.10


def test_center_deadband_and_small_heading_feedforward():
    assert command(road_path(.02)).vy==0.
    slope=math.tan(math.radians(6.))
    out=command(road_path(slope=slope))
    assert out.yaw_rate==0. and out.vy==pytest.approx(.5*slope)
    errors=road_path_errors(road_path(.3,slope),4.)
    assert errors[0]==pytest.approx(.3) and math.degrees(errors[1])==pytest.approx(6.)


def test_small_targets_translate_despite_noisy_fitted_tangent():
    path=road_path()
    ys=.15*np.sin(np.linspace(0.,2*np.pi,20))
    path=replace(path,points_xy=np.column_stack((path.points_xy[:,0],ys)))
    out=command(path)
    assert out.yaw_rate==0.


@pytest.mark.parametrize('path',[road_path(slope=.5),road_path(curve=.15)])
def test_real_direction_change_or_bend_keeps_rotation(path):
    out=command(path)
    assert out.yaw_rate>0. and out.vy==0.
    assert 0. < out.vx <= .5


def test_hysteresis_and_limits_follow_published_commands():
    state=RoadCenterState();cfg=DriveConfig()
    assert state.choose_translation(math.radians(7),0.,cfg)
    assert state.choose_translation(math.radians(10),0.,cfg)
    assert not state.choose_translation(math.radians(13),0.,cfg)
    state.record_command(DriveDecision.stop('task_idle'),now=0.)
    out=command(road_path(2.),road_state=state,now=.1)
    assert out.vy==pytest.approx(.02) and out.yaw_rate==0.
    # Re-evaluating readiness without publishing must not ramp again.
    assert command(road_path(2.),road_state=state,now=.1)==out
    state.record_command(out,now=.1)
    assert command(road_path(2.),road_state=state,now=.2).vy==pytest.approx(.04)
    state.record_command(DriveDecision(.5,.15,0.,'street_light_approaching'),now=.2)
    assert command(road_path(),road_state=state,now=.3).vy==0.
    state.reset();assert state.command_at is None and not state.translating


def test_guards_stop_immediately_even_while_translating():
    cfg=replace(DriveConfig(),stop_on_camera_stale=True)
    state=RoadCenterState();state.record_command(DriveDecision(.5,.1,0.,'tracking'),now=0.)
    out=decide_drive(road_path(2.),camera_age_sec=10.,inference_age_sec=0.,config=cfg,road_state=state,now=.1)
    assert out==DriveDecision.stop('camera_stale')
    assert command(replace(road_path(2.),stop_reason='branch_selected_lost'),road_state=state,now=.1).vx==0.


@pytest.mark.parametrize('initial',[-1.,1.])
def test_position_loop_returns_to_center_after_a_disturbance(initial):
    # Kinematic translation simulation: no hardware or model accuracy claims.
    state=RoadCenterState();offset=initial;history=[]
    state.record_command(DriveDecision.stop('task_idle'),now=0.)
    for i in range(1,801):
        if i==400:offset+=.7*np.sign(initial)
        out=command(road_path(offset),road_state=state,now=i*.05)
        assert out.yaw_rate==0.
        offset-=out.vy*.05;state.record_command(out,now=i*.05);history.append(offset)
    assert abs(history[398])<.06 and abs(history[-1])<.06
    assert all(o*initial>=0. for o in history)  # No repeated side crossings.


def test_env_and_cli_road_settings_and_validation(monkeypatch):
    monkeypatch.setitem(debug.ENV,'LINE_TRACKING_ROAD_MAX_LATERAL_MPS','0.07')
    monkeypatch.setitem(debug.ENV,'SWIN_L_ROAD_ROI_POLYGON','0.01,1,0.99,1,0.99,0.43,0.01,0.43')
    args=debug.parse_args(['task-drive','--road-lateral-gain','0.3'])
    assert debug._drive_config_from_args(args).road_max_lateral_mps==.07
    assert debug._drive_config_from_args(args).road_lateral_gain==.3
    assert debug._local_path_config_from_args(args).road_roi_polygon[0]==.01
    with pytest.raises(SystemExit):debug.parse_args(['ros2','--road-max-lateral-mps','0.2'])


@pytest.mark.parametrize('selected',[1,3])
def test_runtime_confirmed_pole_shifts_only_mode_three_reference(monkeypatch,selected):
    import json
    from types import SimpleNamespace as NS
    from test_lidar_obstacles_ros import complete_inference, enabled_ros
    from test_street_light_task_ros import model, confirmed_points
    from test_apriltag_task_stop_ros import Message, SPORT
    ros=enabled_ros(monkeypatch)
    monkeypatch.setitem(debug.ENV,'LINE_TRACKING_AVOIDANCE_ENABLED','false')
    config=lp.LocalPathConfig(branch_preference='center')
    detection=NS(road=projected_road(config,3.,-1.),pole=np.zeros((360,640),bool))
    model(monkeypatch,detection)

    def tick(node):
        ros.now+=.1
        return complete_inference(node,ros)

    def scenario(node):
        node.on_task_event(Message(json.dumps({'type':'TASK_REGISTERED','action_name':'LINE_TRACKING',
                            'task_id':'center','payload':{'selected_mask':selected,'duration_sec':30}})))
        ros.now=2.1
        for _ in range(3):metrics=tick(node)
        cmd=json.loads(ros.published[SPORT][-1].parameter)
        assert cmd['x']>0 and cmd['y']<0 and cmd['z']==0.  # Center is to the left.
        assert metrics['road_center_control']['active']
        assert metrics['road_center_control']['mode']=='translate'
        assert (node.obstacles is not None)==(selected==3)
        detection.pole[70:100,395:415]=True  # Pole lies to the right, opposite to center.
        for _ in range(4):
            if selected==3:confirmed_points(node,ros,centers=(405,))
            metrics=tick(node)
        cmd=json.loads(ros.published[SPORT][-1].parameter)
        assert cmd['x']>0 and cmd['z']==0.
        if selected==3:
            assert cmd['y']<0  # The shifted reference is still left of the robot.
            assert metrics['street_light']['lidar_confirmed']
            assert metrics['drive_reason']=='tracking'
            assert metrics['road_center_control']['active']
            assert metrics['road_center_control']['reference_offset_m']<0.
            assert not metrics['lidar_obstacles']['stops_enabled']
        else:
            assert cmd['y']<0 and metrics['road_center_control']['active']
        detection.pole[:]=False
        for _ in range(3):metrics=tick(node)
        cmd=json.loads(ros.published[SPORT][-1].parameter)
        assert cmd['x']>0 and cmd['y']<0 and cmd['z']==0.
        assert metrics['road_center_control']['active']

    ros.run(scenario,'--branch-preference','center')


@pytest.mark.parametrize('value',['nan','-1','0','0.16'])
def test_invalid_road_lateral_speed_rejected(value):
    with pytest.raises(SystemExit):debug.parse_args(['task-drive','--road-max-lateral-mps',value])


def test_road_geometry_discards_invalid_points_and_keeps_corner_direction():
    path=road_path(1.)
    points=np.vstack(([[np.nan,2.],[np.inf,0.]],path.points_xy[::-1],path.points_xy[0]))
    out=command(replace(path,points_xy=points))
    assert out.vy>0. and out.yaw_rate==0.
