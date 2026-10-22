"""Motion geometry, measurement faults and actual emitted-command constraints."""
from dataclasses import replace
import math
from types import SimpleNamespace as NS
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))

from motion_tracking import CommandLimiter, MotionBuffer, MotionConfig, follow_path, move_points, resample, se2
from local_path import SmoothedPath
from swin_l_drive_control import DriveConfig, DriveDecision


def odom(t, x=0., y=0., yaw=0., vx=0., vy=0., child='robot_center', frame='odom'):
    s=int(t); stamp=NS(sec=s,nanosec=round((t-s)*1e9))
    q=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2))
    v=lambda a,b,c:NS(x=a,y=b,z=c)
    return NS(header=NS(stamp=stamp,frame_id=frame),child_frame_id=child,
              pose=NS(pose=NS(position=v(x,y,0.),orientation=q)),
              twist=NS(twist=NS(linear=v(vx,vy,0.),angular=v(0.,0.,0.))))


def imu(t, wz=0., frame='dog_imu_link'):
    return NS(header=NS(stamp=odom(t).header.stamp,frame_id=frame),
              angular_velocity=NS(x=0.,y=0.,z=wz), angular_velocity_covariance=[0.]*9)


def buffer(**kwargs):
    return MotionBuffer(MotionConfig(**kwargs), np.eye(4), np.eye(4))


def feed(b, t, **kwargs):
    b.on_imu(imu(t),t,round(t*1e9));b.on_odom(odom(t,**kwargs),t,round(t*1e9))


def path(points=None):
    return SmoothedPath(np.array([[3.,0.],[4.,0.],[5.,0.]]) if points is None else np.asarray(points,float), .95, .1, 'connectome_surface_center')


def test_capture_pose_interpolation_translation_and_rotation_have_correct_signs():
    b=buffer()
    feed(b,100.);feed(b,100.05,x=.04,y=.02,yaw=.03);feed(b,100.10,x=.08,y=.04,yaw=.06)
    moved,reason=b.path(path(),100_025_000_000,0,100.10,100_100_000_000)
    assert reason is None
    expected=np.linalg.inv(se2(.08,.04,.06)) @ se2(.02,.01,.015)
    np.testing.assert_allclose(moved.points_xy[[0,-1]],move_points([[3.,0.],[5.,0.]],expected),atol=1e-7)
    assert np.all(moved.points_xy[:,1]<0)
    assert b.metrics['reference_age_sec']==pytest.approx(.075)
    # The entire permission region has the same rigid transform as its path.
    np.testing.assert_allclose(move_points(moved.points_xy,np.linalg.inv(expected)),resample(path().points_xy,.1),atol=1e-7)


def test_heading_wrap_and_sign_flipped_quaternions_are_not_odometry_jumps():
    b=buffer();feed(b,100.,yaw=math.pi-.01);feed(b,100.05,yaw=-math.pi+.01)
    m=odom(100.10,yaw=-math.pi+.02)
    for k in 'xyzw':setattr(m.pose.pose.orientation,k,-getattr(m.pose.pose.orientation,k))
    b.on_odom(m,100.10,100_100_000_000)
    assert b.faults['odom'] is None and b.epoch==0
    matrix=b.relative(100_025_000_000,100_050_000_000,100.1,100_100_000_000)
    assert math.atan2(matrix[1,0],matrix[0,0])==pytest.approx(-.01)


def test_explicit_lever_arm_and_gyro_axis_transform_are_applied():
    mount=np.eye(4);mount[0,3]=.3 # base <- child; child is 30cm ahead of base.
    gyro=np.eye(4);gyro[:2,:2]=[[-1,0],[0,-1]]
    b=MotionBuffer(MotionConfig(),mount,gyro)
    feed(b,100.)
    np.testing.assert_allclose(b.odom[-1].matrix()[:2,2],[-.3,0])
    b.on_imu(imu(100.05,.5),100.05,100_050_000_000)
    b.on_odom(odom(100.05),100.05,100_050_000_000)
    matrix=b.relative(100_050_000_000,100_070_000_000,100.07,100_070_000_000)
    assert math.atan2(matrix[1,0],matrix[0,0])==pytest.approx(-.01)
    assert b.epoch==0


@pytest.mark.parametrize('fault,expected',[
 ('missing_transform','motion_odom_transform_unavailable'),('nan','motion_odom_invalid'),
 ('jump','motion_odom_jump'),('backwards','motion_odom_discontinuity'),
 ('frame','motion_odom_frame_mismatch'),('gyro','motion_gyro_unavailable')])
def test_bad_measurements_invalidate_old_camera_epoch(fault,expected):
    b=buffer();feed(b,100.)
    if fault=='missing_transform':b.child_transform=None
    if fault=='gyro':
        m=imu(100.05);m.angular_velocity_covariance[0]=-1;b.on_imu(m,100.05,100_050_000_000)
    else:
        m=odom(99.99 if fault=='backwards' else 100.05,x=2. if fault=='jump' else 0.)
        if fault=='nan':m.pose.pose.position.x=float('nan')
        if fault=='frame':m.child_frame_id='unknown'
        b.on_odom(m,100.05,100_050_000_000)
    assert expected in b.faults.values()
    assert b.epoch==1
    # A later valid measurement cannot make the pre-fault camera path valid.
    b.child_transform=np.eye(4);feed(b,100.10)
    _,reason=b.path(path(),100_000_000_000,0,100.1,100_100_000_000)
    assert reason=='motion_reference_epoch_changed'


def test_sensor_receipt_source_age_and_bounded_extrapolation_do_not_extend_path_life():
    b=buffer();feed(b,100.)
    _,r=b.path(path(),100_000_000_000,0,100.25,100_250_000_000)
    assert r=='motion_odom_stale'
    _,r=b.path(path(),100_000_000_000,0,100.06,100_060_000_000)
    assert r=='motion_extrapolation_exceeded'
    for t in np.arange(100.05,101.11,.05):feed(b,float(t))
    _,r=b.path(path(),100_000_000_000,0,101.10,101_100_000_000)
    assert r=='motion_reference_stale'
    # Fresh receipt with old sensor timestamps must still stop.
    b.odom[-1]=replace(b.odom[-1],stamp=100.,arrival=101.1)
    _,r=b.path(path(),101_050_000_000,0,101.1,101_100_000_000)
    assert r=='motion_odom_stale'


def test_resampling_preserves_vertices_and_does_not_cut_semantic_corner():
    vertices=np.array([[3,0],[3.03,.1],[3.2,.1],[3.21,-.1]])
    p=resample(vertices,.1)
    for v in vertices:assert np.min(np.linalg.norm(p-v,axis=1))<1e-10
    assert np.max(np.linalg.norm(np.diff(p,axis=0),axis=1))<=.1+1e-10


def test_short_valid_path_uses_real_endpoint_and_current_heading():
    cfg=DriveConfig(max_forward_mps=.8,max_yaw_rps=.18)
    p=path([[2.,.5],[3.,.5]])
    d=follow_path(p,DriveDecision(.8,0,0,'tracking'),cfg)
    assert d.yaw_rate/d.vx==pytest.approx(1/9.25)
    assert d.vx<=.8 and abs(d.yaw_rate)<=.18
    assert follow_path(p,DriveDecision.stop('obstacle_hit'),cfg).reason=='obstacle_hit'


def test_emitted_command_ramps_and_emergency_stop_are_independent_of_inference():
    limiter=CommandLimiter();target=DriveDecision(.8,0,.16,'tracking')
    last=DriveDecision.stop('start')
    for i in range(20):
        t=100+i*.05;d=limiter.limit(target,t)
        assert 0<=d.vx-last.vx<=.3*.05+1e-9
        assert abs(d.yaw_rate-last.yaw_rate)<=.5*.05+1e-9
        assert d.yaw_rate/d.vx==pytest.approx(.2)
        limiter.record(d,t);last=d
    stopped=limiter.limit(DriveDecision.stop('motion_imu_stale'),101.)
    assert stopped.vx==stopped.yaw_rate==0
    limiter.record(stopped,101.)
    assert limiter.limit(target,101.05).vx<=.015+1e-9
    limiter.record(target,102.)
    assert limiter.limit(target,102.3).reason=='motion_control_gap'


def test_permission_and_scan_are_queried_in_the_same_current_frame():
    from lidar_runtime import LidarRuntime
    from lidar_obstacles import ObstacleConfig,ObstacleScan
    from local_avoidance import AvoidanceConfig
    args=NS(path_frame_id='base_link',lidar_to_base_transform='',base_to_camera_transform='',
            lidar_calibration_profile='livox-a2-front',lidar_frame_id='livox_frame')
    c=ObstacleConfig();runtime=LidarRuntime(args,c,AvoidanceConfig())
    observed=np.ones(c.shape,bool);observed[10:15,15:20]=False
    costs=np.zeros(c.shape);costs[30:35,10:15]=2.
    scan=ObstacleScan(np.array([[2.,0.]]),observed,costs,100_000_000_000,100.,1,np.array([0.,0.,1.]),np.zeros(3))
    shifted=runtime._compensate_scan(scan,se2(-.2,0.,0.))
    np.testing.assert_allclose(shifted.points,[[1.8,0.]])
    assert shifted.stamp_ns==scan.stamp_ns and shifted.arrival==scan.arrival
    assert not shifted.observed[8:13,15:20].any()
    assert np.max(shifted.costs)>=2.


@pytest.mark.parametrize('new_yaw',[0.,-.16])
def test_turn_reversal_or_straightening_obeys_yaw_slew_and_pauses_forward(new_yaw):
    limiter=CommandLimiter();previous=DriveDecision(.5,0,.16,'tracking')
    limiter.record(previous,100.)
    target=DriveDecision(.5,0,new_yaw,'tracking')
    d=limiter.limit(target,100.05)
    assert d.vx==0 and d.yaw_rate>0
    assert abs(d.yaw_rate-previous.yaw_rate)<=.025+1e-8
    for i in range(2,16):
        limiter.record(d,100.+(i-1)*.05)
        previous=d;d=limiter.limit(target,100.+i*.05)
        assert abs(d.yaw_rate-previous.yaw_rate)<=.025+1e-8
        assert d.vx-previous.vx<=.015+1e-8
    assert d.vx>0
    assert d.yaw_rate==0 if new_yaw==0 else d.yaw_rate<0


def test_lidar_and_camera_support_use_inverse_matching_motion_transforms(monkeypatch):
    import lidar_runtime
    from lidar_obstacles import ObstacleConfig,ObstacleScan
    from local_avoidance import AvoidanceConfig
    from sensor_fixtures import camera_info,header
    b=buffer();feed(b,100.);feed(b,100.05,x=.05);feed(b,100.10,x=.1)
    args=NS(path_frame_id='base_link',lidar_to_base_transform='',base_to_camera_transform='',
            lidar_calibration_profile='livox-a2-front',lidar_frame_id='livox_frame')
    c=ObstacleConfig();runtime=lidar_runtime.LidarRuntime(args,c,AvoidanceConfig(),motion=b)
    runtime.scan=ObstacleScan(np.array([[2.,0.]]),np.ones(c.shape,bool),np.zeros(c.shape),
                             100_050_000_000,100.05,1,np.array([0.,0.,1.]),np.zeros(3))
    runtime.fault=None;runtime.infos.append(camera_info(100.))
    runtime.set_reference(np.ones((360,640),np.uint8),header(100.,'camera_optical_frame'),(360,640),100.)
    matrices=[]
    def region(_m,_i,matrix,_s,config,*_a,**_k):
        matrices.append(matrix.copy());return np.ones(config.shape,bool),np.ones(config.shape,bool)
    monkeypatch.setattr(lidar_runtime,'camera_region',region)
    fixed=runtime.camera.copy()
    scan,_=runtime._context(1,100.1,100_100_000_000)
    np.testing.assert_allclose(scan.points,[[1.95,0.]])
    delta=np.eye(4);delta[0,3]=.1
    np.testing.assert_allclose(matrices[-1],fixed@delta,atol=1e-8)
    runtime._context(1,100.1,100_100_000_000)
    np.testing.assert_allclose(matrices[-1],fixed@delta,atol=1e-8) # Never compound transforms.
    np.testing.assert_allclose(runtime.camera,fixed)


def test_sensor_fault_is_visible_when_no_camera_path_exists():
    b=buffer()
    b.path(None,0,0,100.,100_000_000_000)
    assert b.snapshot(100.,100_000_000_000)['sensor_reason']=='motion_waiting_for_odom'
    feed(b,100.);b.on_odom(odom(99.99),100.01,100_010_000_000)
    s=b.snapshot(100.01,100_010_000_000)
    assert s['sensor_reason']=='motion_odom_discontinuity'
    assert s['fault_counts']['motion_odom_discontinuity']==1
    assert s['odom_samples']==0


def test_gap_recovery_uses_measured_endpoints_but_never_interpolates_missing_interval():
    b=buffer();feed(b,100.);feed(b,100.05,x=.01)
    _,reason=b.path(path(),100_000_000_000,0,100.3,100_300_000_000)
    assert reason=='motion_odom_stale'
    feed(b,100.4,x=.08)
    assert b.epoch==0
    moved,reason=b.path(path(),100_000_000_000,0,100.4,100_400_000_000)
    assert reason is None and moved.points_xy[0,0]==pytest.approx(2.92)
    _,reason=b.path(path(),100_200_000_000,0,100.4,100_400_000_000)
    assert reason=='motion_history_gap'
    assert b.snapshot(100.4,100_400_000_000)['gap_counts']=={'odom':1,'imu':1}
