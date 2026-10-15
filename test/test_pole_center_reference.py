"""A Pole shifts the bounded road reference, without issuing its own motion."""
from dataclasses import replace
import math
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from local_path import SmoothedPath
from street_light_approach import ConfirmedPole,StreetLightApproach,StreetLightConfig
from swin_l_drive_control import DriveConfig,DriveDecision,RoadCenterState,decide_drive


def road(center=0.,half_width=2.,ys=None):
    xs=np.linspace(3.,8.,20);ys=np.full_like(xs,center) if ys is None else np.asarray(ys)
    return SmoothedPath(np.column_stack((xs,ys)),.95,0.,'test',road_center_tracking=True,
                        road_corridor=np.column_stack((xs,ys-half_width,ys+half_width)))


def drive(path,**kwargs):
    return decide_drive(path,camera_age_sec=0.,inference_age_sec=0.,config=DriveConfig(),**kwargs)


POLE=ConfirmedPole(math.atan2(2.,5.),math.hypot(5.,2.))


def ramp(controller,path,pole=POLE,end=25.):
    for tick in range(round(end*10)+1):
        result=controller.shift_reference(path,pole,now=tick/10)
    return result


@pytest.mark.parametrize('rate',[.05,.25,.5])
def test_reference_moves_at_configured_rate_to_a_bounded_parallel_line(rate):
    controller=StreetLightApproach(StreetLightConfig(center_shift_rate_mps=rate));path=road();last=0.
    for tick in range(251):
        result=controller.shift_reference(path,POLE,now=tick/10)
        offset=result.road_center_offset_m
        assert 0<=offset<=1.
        assert offset-last<=rate*.1+1e-8
        assert np.allclose(result.points_xy[:,1]-path.points_xy[:,1],offset)
        assert np.array_equal(result.points_xy[:,0],path.points_xy[:,0])
        assert drive(result).yaw_rate==0.
        last=offset
    assert last==pytest.approx(1.)
    assert controller.metrics['reference_clearance_at_pole_m']==pytest.approx(1.)
    assert np.all(path.points_xy[:,1]==0.)
    assert controller.metrics['control_mode']=='shifted_road_center'


def test_loss_returns_reference_gradually_and_never_retains_a_pole_push():
    controller=StreetLightApproach(StreetLightConfig());path=road()
    ramp(controller,path)
    result=controller.shift_reference(path,None,now=25.1)
    assert result.road_center_offset_m==pytest.approx(.975)
    assert controller.metrics['returning_to_center']
    for tick in range(252,461):result=controller.shift_reference(path,None,now=tick/10)
    assert result is path and controller.center_offset_m==0.
    assert drive(result).vy==0.


def test_target_changes_and_repeated_calls_cannot_jump_or_accumulate_reference():
    controller=StreetLightApproach(StreetLightConfig());path=road()
    previous=ramp(controller,path)
    assert controller.shift_reference(previous,POLE,now=25.).road_center_offset_m==pytest.approx(1.)
    # A delayed control call advances at most 0.2s of reference motion.
    other=replace(POLE,bearing_rad=-POLE.bearing_rad)
    result=controller.shift_reference(path,other,now=40.)
    assert result.road_center_offset_m==pytest.approx(.95)
    values=[]
    for tick in range(401,821):
        result=controller.shift_reference(result,other,now=tick/10)
        values.append(result.road_center_offset_m)
    assert values[-1]==pytest.approx(-1.)
    assert all(b<=a+1e-7 for a,b in zip(values,values[1:]))
    assert np.allclose(result.points_xy[:,1],-1.)
    controller.reset();assert controller.center_offset_m==0. and controller.last_reference_at is None


def test_road_edge_limits_uniform_offset_including_the_narrow_far_end():
    path=road(half_width=1.)
    corridor=path.road_corridor.copy();corridor[-1,1:]=[-.55,.55]
    path=replace(path,road_corridor=corridor)
    controller=StreetLightApproach(StreetLightConfig());result=ramp(controller,path)
    assert result.road_center_offset_m==pytest.approx(.55)
    assert result.points_xy[-1,1]==pytest.approx(.10)
    assert controller.metrics['road_edge_limited']
    assert np.all(result.points_xy[:,1]<=corridor[:,2]-.45+1e-6)
    narrow=road(half_width=.2)
    result=controller.shift_reference(narrow,POLE,now=25.1)
    assert result is narrow and controller.center_offset_m==0.


def test_pole_side_is_relative_to_road_center_not_robot_center():
    controller=StreetLightApproach(StreetLightConfig());path=road(center=2.)
    pole=ConfirmedPole(0.,5.)
    result=ramp(controller,path,pole)
    assert result.road_center_offset_m==pytest.approx(-1.)
    assert np.allclose(result.points_xy[:,1],1.)
    assert drive(result).vy>0.  # Robot still lies to the right of its shifted target.


@pytest.mark.parametrize('path',[road(ys=-.25*np.linspace(3.,8.,20)+1.4),
                                 road(ys=.15*(np.linspace(3.,8.,20)-3.)**2)])
def test_reference_shift_does_not_change_road_turn_mode_or_yaw_demand(path):
    controller=StreetLightApproach(StreetLightConfig());shifted=ramp(controller,path)
    before,after=drive(path),drive(shifted)
    assert before.yaw_rate==pytest.approx(after.yaw_rate)
    assert before.vx==pytest.approx(after.vx)


def test_missing_corridor_or_guard_never_invents_a_biased_path():
    controller=StreetLightApproach(StreetLightConfig());path=road();ramp(controller,path)
    no_corridor=replace(path,road_corridor=None)
    assert controller.shift_reference(no_corridor,POLE,now=8.1) is no_corridor
    assert controller.center_offset_m==0.
    stopped=replace(path,stop_reason='branch_path_lost')
    assert controller.shift_reference(stopped,POLE,now=8.2) is stopped
    assert drive(stopped)==DriveDecision.stop('branch_path_lost')
    assert controller.shift_reference(None,POLE,now=8.3) is None


def test_closed_loop_approaches_shifted_center_then_returns_without_turning():
    controller=StreetLightApproach(StreetLightConfig());state=RoadCenterState()
    state.record_command(DriveDecision.stop('task_idle'),now=0.)
    robot_y=0.;positions=[];offsets=[]
    for tick in range(1001):
        now=tick*.1;path=road(center=-robot_y)
        pole=ConfirmedPole(math.atan2(2.-robot_y,5.),math.hypot(5.,2.-robot_y)) if now<40. else None
        reference=controller.shift_reference(path,pole,now=now)
        command=drive(reference,road_state=state,now=now)
        assert command.yaw_rate==0.
        robot_y+=command.vy*.1;state.record_command(command,now=now)
        positions.append(robot_y);offsets.append(reference.road_center_offset_m)
    tolerance=DriveConfig().road_lateral_deadband_m+.01
    assert positions[390]>1.-tolerance and max(positions)<1.01
    assert 0<=positions[-1]<tolerance
    assert all(p>=0. for p in positions)
    assert max(abs(b-a) for a,b in zip(offsets,offsets[1:]))<=.02500001


@pytest.mark.parametrize('name,value',[('center_max_offset_m',1.1),('center_shift_rate_mps',.500001),
                                       ('center_shift_rate_mps',0),('center_shift_rate_mps',-.1),
                                       ('center_shift_rate_mps',float('nan')),('center_shift_rate_mps',float('inf')),
                                       ('center_bias_gain',1.1),('center_edge_margin_m',float('nan')),
                                       ('target_clearance_m',0),('approach_min_forward_mps',.11),
                                       ('approach_forward_margin_m',-1),('approach_time_margin_sec',0),
                                       ('approach_max_forward_accel',float('nan'))])
def test_invalid_reference_configuration_is_rejected(name,value):
    with pytest.raises(ValueError):replace(StreetLightConfig(),**{name:value}).validate()


def test_pole_clearance_can_exceed_old_offset_cap_and_moves_away_when_too_close():
    controller=StreetLightApproach(StreetLightConfig(target_clearance_m=1.5))
    pole=ConfirmedPole(math.atan2(3.,5.),math.hypot(5.,3.))
    result=ramp(controller,road(half_width=4.),pole,end=35.)
    assert result.road_center_offset_m==pytest.approx(1.5)
    assert controller.metrics['reference_clearance_at_pole_m']==pytest.approx(1.5)
    controller.reset()
    close=ConfirmedPole(math.atan2(.3,5.),math.hypot(5.,.3))
    result=ramp(controller,road(),close,end=35.)
    assert result.road_center_offset_m==pytest.approx(-1.2)


def test_tracked_pole_keeps_its_road_side_when_center_estimate_crosses_it():
    controller=StreetLightApproach(StreetLightConfig());controller.target_id=7
    controller.shift_reference(road(half_width=4.),POLE,now=0.)
    controller.shift_reference(road(center=3.,half_width=4.),POLE,now=.1)
    assert controller.clearance_side==1.
    assert controller.metrics['requested_offset_m']==pytest.approx(-2.)


def test_approach_loss_and_opposite_pole_preserve_ordinary_forward_speed():
    controller=StreetLightApproach(StreetLightConfig());path=road()
    normal=drive(path);directions=set()
    opposite=replace(POLE,bearing_rad=-POLE.bearing_rad)
    for tick in range(241):
        now=tick*.1
        pole=POLE if now<8. else None if now<16. else opposite
        reference=controller.shift_reference(path,pole,now=now)
        command=drive(reference)
        assert command.vx==normal.vx
        assert command.yaw_rate==normal.yaw_rate==0.
        assert abs(command.vy)<=DriveConfig().road_max_lateral_mps
        assert controller.metrics['forward_speed_limited'] is False
        assert controller.metrics['speed_limit_reason']=='disabled'
        if command.vy:directions.add(int(np.sign(command.vy)))
    assert directions=={-1,1}


def test_speed_scaling_preserves_turn_curvature_and_reduces_heading_feedforward():
    cfg=DriveConfig();slow=cfg.scaled_speed(.2);slow.validate()
    path=road(ys=.5*np.linspace(3.,8.,20))
    before=drive(path)
    after=decide_drive(path,camera_age_sec=0.,inference_age_sec=0.,config=slow)
    assert after.vx==pytest.approx(before.vx*.2)
    assert after.yaw_rate==pytest.approx(before.yaw_rate*.2)
    assert after.yaw_rate/after.vx==pytest.approx(before.yaw_rate/before.vx)
    straight=road(ys=math.tan(math.radians(6.))*np.linspace(3.,8.,20))
    fast=drive(straight)
    reduced=decide_drive(straight,camera_age_sec=0.,inference_age_sec=0.,config=slow)
    assert reduced.vy==pytest.approx(fast.vy*.2) and reduced.yaw_rate==0.
    state=RoadCenterState();state.record_command(DriveDecision(.5,0.,.18,'tracking'),now=0.)
    capped=decide_drive(path,camera_age_sec=0.,inference_age_sec=0.,config=slow,road_state=state,now=.1)
    assert abs(capped.yaw_rate)<=slow.max_yaw_rps


@pytest.mark.parametrize('hz,latency',[(1.,1.),(2.,.5)])
@pytest.mark.parametrize('rate',[.05,.25,.5])
def test_low_hz_pole_reference_preserves_cruise_speed_while_moving_laterally(hz,latency,rate):
    from local_path import LocalPathConfig,LocalPathEstimate,LocalPathSmoother
    from street_light_approach import StreetLightObservation
    cfg=replace(DriveConfig(),max_forward_mps=.8)
    controller=StreetLightApproach(StreetLightConfig(center_shift_rate_mps=rate));state=RoadCenterState()
    smoother=LocalPathSmoother(LocalPathConfig(unrestricted_path_mode=True,branch_preference='center'))
    state.record_command(DriveDecision.stop('task_idle'),now=0.)
    x=y=0.;pending=[];iid=0;source=None;measured=None
    for tick in range(1201):
        now=tick*.1
        if tick%round(10/hz)==0:
            p=road(center=-y,half_width=5.)
            e=LocalPathEstimate(p.points_xy,.95,1.,10.,p.points_xy,road_center_tracking=True,road_corridor=p.road_corridor)
            target=ConfirmedPole(math.atan2(3.-y,8.-x),math.hypot(8.-x,3.-y))
            pending.append((now+latency,now,e,target))
        while pending and pending[0][0]<=now+1e-7:
            finish,source,e,measured=pending.pop(0);smoother.update(e,finish);iid+=1
        path=smoother.current(now);age=now-source if source is not None else None
        pole=controller.observe((StreetLightObservation(.3,.4,.5,100,.2),),now=now,
            inference_id=iid,inference_age_sec=age,road_tracking=path is not None,
            locate=lambda _: (measured.bearing_rad,measured.distance_m))
        reference=controller.shift_reference(path,pole,now=now)
        command=decide_drive(reference,camera_age_sec=0.,inference_age_sec=age,config=cfg,road_state=state,now=now)
        assert command.yaw_rate==0.
        if command.reason=='tracking':assert command.vx==cfg.max_forward_mps
        assert abs(command.vy)<=cfg.road_max_lateral_mps
        x+=command.vx*.1;y+=command.vy*.1
        state.record_command(command,now=now)
        if x>=8.:
            assert now<20. and y>0.
            assert y<2.15  # The optional reference must not push beyond its 1m goal.
            break
    else:
        pytest.fail('ideal vehicle never passed the Pole')
