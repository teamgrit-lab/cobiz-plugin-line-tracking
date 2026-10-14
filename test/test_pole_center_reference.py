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


def ramp(controller,path,pole=POLE,end=8.):
    for tick in range(round(end*10)+1):
        result=controller.shift_reference(path,pole,now=tick/10)
    return result


def test_reference_moves_slowly_to_a_bounded_parallel_line():
    controller=StreetLightApproach(StreetLightConfig());path=road();last=0.
    for tick in range(81):
        result=controller.shift_reference(path,POLE,now=tick/10)
        offset=result.road_center_offset_m
        assert 0<=offset<=.30
        assert offset-last<=.00500001
        assert np.allclose(result.points_xy[:,1]-path.points_xy[:,1],offset)
        assert np.array_equal(result.points_xy[:,0],path.points_xy[:,0])
        assert drive(result).yaw_rate==0.
        last=offset
    assert last==pytest.approx(.30)
    assert np.all(path.points_xy[:,1]==0.)
    assert controller.metrics['control_mode']=='shifted_road_center'


def test_loss_returns_reference_gradually_and_never_retains_a_pole_push():
    controller=StreetLightApproach(StreetLightConfig());path=road()
    ramp(controller,path)
    result=controller.shift_reference(path,None,now=8.1)
    assert result.road_center_offset_m==pytest.approx(.295)
    assert controller.metrics['returning_to_center']
    for tick in range(82,151):result=controller.shift_reference(path,None,now=tick/10)
    assert result is path and controller.center_offset_m==0.
    assert drive(result).vy==0.


def test_target_changes_and_repeated_calls_cannot_jump_or_accumulate_reference():
    controller=StreetLightApproach(StreetLightConfig());path=road()
    previous=ramp(controller,path)
    assert controller.shift_reference(previous,POLE,now=8.).road_center_offset_m==pytest.approx(.3)
    # A delayed control call advances at most 0.2s of reference motion.
    other=replace(POLE,bearing_rad=-POLE.bearing_rad)
    result=controller.shift_reference(path,other,now=20.)
    assert result.road_center_offset_m==pytest.approx(.29)
    values=[]
    for tick in range(201,331):
        result=controller.shift_reference(result,other,now=tick/10)
        values.append(result.road_center_offset_m)
    assert values[-1]==pytest.approx(-.3)
    assert all(b<=a+1e-7 for a,b in zip(values,values[1:]))
    assert np.allclose(result.points_xy[:,1],-.3)
    controller.reset();assert controller.center_offset_m==0. and controller.last_reference_at is None


def test_road_edge_limits_uniform_offset_including_the_narrow_far_end():
    path=road(half_width=1.)
    corridor=path.road_corridor.copy();corridor[-1,1:]=[-.55,.55]
    path=replace(path,road_corridor=corridor)
    controller=StreetLightApproach(StreetLightConfig());result=ramp(controller,path)
    assert result.road_center_offset_m==pytest.approx(.30)
    assert result.points_xy[-1,1]==pytest.approx(.10)
    assert controller.metrics['road_edge_limited']
    assert np.all(result.points_xy[:,1]<=corridor[:,2]-.45+1e-6)
    narrow=road(half_width=.2)
    result=controller.shift_reference(narrow,POLE,now=8.1)
    assert result is narrow and controller.center_offset_m==0.


def test_pole_side_is_relative_to_road_center_not_robot_center():
    controller=StreetLightApproach(StreetLightConfig());path=road(center=2.)
    pole=ConfirmedPole(math.atan2(1.,5.),math.hypot(5.,1.))
    result=ramp(controller,path,pole)
    assert result.road_center_offset_m==pytest.approx(-.2)
    assert np.allclose(result.points_xy[:,1],1.8)
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
    for tick in range(401):
        now=tick*.1;path=road(center=-robot_y)
        pole=ConfirmedPole(math.atan2(2.-robot_y,5.),math.hypot(5.,2.-robot_y)) if now<20. else None
        reference=controller.shift_reference(path,pole,now=now)
        command=drive(reference,road_state=state,now=now)
        assert command.yaw_rate==0.
        robot_y+=command.vy*.1;state.record_command(command,now=now)
        positions.append(robot_y);offsets.append(reference.road_center_offset_m)
    assert positions[190]>.24 and max(positions)<.31
    assert 0<=positions[-1]<.06
    assert all(p>=0. for p in positions)
    assert max(abs(b-a) for a,b in zip(offsets,offsets[1:]))<=.00500001


@pytest.mark.parametrize('name,value',[('center_max_offset_m',1.1),('center_shift_rate_mps',.16),
                                       ('center_bias_gain',1.1),('center_edge_margin_m',float('nan'))])
def test_invalid_reference_configuration_is_rejected(name,value):
    with pytest.raises(ValueError):replace(StreetLightConfig(),**{name:value}).validate()
