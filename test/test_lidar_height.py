from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from lidar_height import (LidarHeightConfig, LidarInputs, cloud_xyz, fit_ground,
                          a2_front_transforms, a2_livox_front_transforms,
                          fuse_height, parse_transform)
from local_path import LocalPathConfig, extract_metric_centerline, metric_path_supported


CAMERA_FROM_BASE = np.array([[0,-1,0,0], [0,0,-1,0], [1,0,0,0], [0,0,0,1]], float)


def header(stamp=100., frame="hesai_lidar"):
    ns = round(stamp*1e9)
    sec, nano = divmod(ns, 10**9)
    return NS(stamp=NS(sec=sec, nanosec=nano), frame_id=frame)


def cloud(points, stamp=100., bigendian=False, rows=1):
    endian = ">" if bigendian else "<"
    dtype = np.dtype(dict(names=["x", "y", "z"], formats=[endian+"f4"]*3,
                         offsets=[0,4,8], itemsize=26))
    raw = np.zeros(len(points), dtype=dtype)
    for i,k in enumerate(("x", "y", "z")): raw[k] = points[:, i]
    width = len(points)//rows
    # Include padding to exercise row_step instead of flat float reshaping.
    data = b"".join(raw[i*width:(i+1)*width].tobytes()+bytes(8) for i in range(rows))
    return NS(header=header(stamp), width=width, height=rows, point_step=26,
              row_step=width*26+8, is_bigendian=bigendian, is_dense=False, data=data,
              fields=[NS(name=k, offset=i*4, datatype=7, count=1)
                      for i,k in enumerate(("x", "y", "z"))])


def points_scene(raise_side=0., floor=-.5):
    x,y = np.meshgrid(np.arange(.25,3.06,.025), np.arange(-1.5,1.51,.025), indexing="ij")
    z = np.full_like(x, floor)
    z[y > .8] += raise_side
    return np.column_stack((x.ravel(), y.ravel(), z.ravel()))


def camera_info(stamp=100., frame="camera", size=(2,2)):
    return NS(header=header(stamp, frame), width=size[1], height=size[0],
              k=[.2,0,1,0,.2,1,0,0,1], d=[0.]*5, distortion_model="plumb_bob")


def imu(stamp=100., frame="hesai_lidar"):
    return NS(header=header(stamp, frame), linear_acceleration=NS(x=0., y=0., z=9.81))


def configs():
    return (LidarHeightConfig(footprint_radius_m=.1), LocalPathConfig(
        near_distance_m=.6, far_distance_m=3., search_half_width_m=2.,
        bev_height_px=49, bev_width_px=81, branch_preference="center"))


def fusion(raise_side=0., floor=-.5, label=2):
    cfg, path = configs()
    return fuse_height(np.full((360,640), label, np.uint8), cloud(points_scene(raise_side, floor)),
                       camera_info(), (2,2), np.eye(4), CAMERA_FROM_BASE,
                       np.array([0,0,1.]), cfg, path, (0,1,2))


@pytest.mark.parametrize("endian", [False,True])
def test_reads_offsets_padding_and_invalid_points(endian):
    xyz = np.array([[1,2,3], [0,0,0], [np.nan,1,2], [4,5,6]], float)
    np.testing.assert_allclose(cloud_xyz(cloud(xyz, bigendian=endian, rows=2)), [[1,2,3],[4,5,6]])


def test_rejects_truncated_cloud_and_nonrigid_calibration():
    msg=cloud(np.array([[1,2,3.]])); msg.data=msg.data[:-1]
    with pytest.raises(ValueError, match="layout"): cloud_xyz(msg)
    with pytest.raises(ValueError, match="16"): parse_transform("1,2,3")
    with pytest.raises(ValueError, match="rigid"): parse_transform(",".join(map(str, (np.eye(4)*2).ravel())))
    assert parse_transform("") is None


@pytest.mark.parametrize("raised", [.15,-.15])
def test_curbs_and_drops_are_excluded_without_losing_current_ground(raised):
    result=fusion(raised)
    assert result.regions[2][:, abs(result.y_values)<.5].any()
    assert not result.gate[:, result.y_values>1.].any()
    assert not result.regions[1].any()
    np.testing.assert_array_equal(result.regions[0], result.regions[2])
    cfg,path=configs()
    estimate=extract_metric_centerline(result.regions[2], result.x_values, result.y_values,
                                      path, hard_gate=result.gate)
    assert estimate is not None
    assert metric_path_supported(estimate.points_xy, result.regions[2], result.x_values, result.y_values)


def test_sidewalk_becomes_the_reference_when_robot_is_on_it():
    low, high = fusion(floor=-.5), fusion(floor=-.35)
    assert high.metrics["plane_offset_m"] == pytest.approx(.35, abs=.002)
    # The camera-visible returns change with elevation; the geometric height
    # acceptance must stay the same, and observed sidewalk remains usable.
    np.testing.assert_array_equal(low.gate, high.gate)
    assert high.regions[2].any()
    np.testing.assert_allclose(low.project_path(np.array([[1.,0.]])),[[1.,1.1]],atol=.001)


def test_sensor_axis_rotation_is_applied_before_height_and_projection():
    cfg,path=configs()
    matrix=np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]], float)
    sensor_points=points_scene(.15)@matrix[:3,:3]
    result=fuse_height(np.full((360,640),2,np.uint8), cloud(sensor_points), camera_info(),
                       (2,2), matrix, CAMERA_FROM_BASE, np.array([0,1.,0]), cfg,path,(2,))
    np.testing.assert_array_equal(result.regions[2], fusion(.15).regions[2])


def test_a2_front_vertical_mount_preserves_ground_height_and_camera_projection():
    base, camera = a2_front_transforms("hesai_lidar", "base_link", "camera_optical_frame")
    np.testing.assert_allclose(base[:3,:3] @ [0.,1.,0.], [0.,0.,1.])
    np.testing.assert_allclose(base[:3,:3] @ [0.,0.,1.], [1.,0.,0.])
    np.testing.assert_allclose(camera @ base, [
        [-1.,0.,0.,.0336], [0.,-1.,0.,-.02884],
        [0.,0.,1.,-.00043], [0.,0.,0.,1.],
    ], atol=1e-8)
    cfg,path=configs(); points=points_scene(.15)
    # Avoid exact cell/range boundaries when two float32 encodings include
    # different sensor origins; compare the physical scene, not rounding ties.
    points[:,:2]+=.007
    raw=(points-base[:3,3]) @ base[:3,:3]
    actual=fuse_height(np.full((360,640),2,np.uint8),cloud(raw),camera_info(),
                       (2,2),base,camera,np.array([0.,1.,0.]),cfg,path,(2,))
    expected=fuse_height(np.full((360,640),2,np.uint8),cloud(points),camera_info(),
                         (2,2),np.eye(4),camera,np.array([0.,0.,1.]),cfg,path,(2,))
    np.testing.assert_array_equal(actual.gate,expected.gate)
    assert actual.metrics['plane_offset_m']==pytest.approx(.5,abs=.002)
    np.testing.assert_allclose(actual.project_path(np.array([[1.,0.]])),
                               expected.project_path(np.array([[1.,0.]])),atol=1e-6)
    with pytest.raises(ValueError,match="calibration_frame_mismatch"):
        a2_front_transforms("rear_lidar","base_link","camera_optical_frame")


def test_measured_livox_mount_matches_slam_camera_calibration_and_excludes_steps():
    base,camera=a2_livox_front_transforms("livox_frame","base_link","camera_optical_frame")
    # Independent measured camera_T_livox, as recorded in the SLAM/FAST-LIVO configs.
    np.testing.assert_allclose(camera@base,[
        [-.999608476,.001682648,.027929602,.031180709],
        [-.024786027,-.516401421,-.855987865,-.138267329],
        [.012982560,-.856344989,.516240944,.035867595],
        [0,0,0,1]],atol=5e-7)
    assert np.rad2deg(np.arccos(base[2,2]))==pytest.approx(31.13,abs=.05)
    points=points_scene(.15);points[:,:2]+=.007
    raw=(points-base[:3,3])@base[:3,:3]
    cfg,path=configs();up=base[:3,:3].T@np.array([0,0,1.])
    actual=fuse_height(np.full((360,640),2,np.uint8),cloud(raw),camera_info(),
                       (2,2),base,camera,up,cfg,path,(2,))
    expected=fuse_height(np.full((360,640),2,np.uint8),cloud(points),camera_info(),
                         (2,2),np.eye(4),camera,np.array([0,0,1.]),cfg,path,(2,))
    np.testing.assert_array_equal(actual.gate,expected.gate)
    assert actual.metrics["plane_offset_m"]==pytest.approx(.5,abs=.002)
    for frame,body,cam in [("hesai_lidar","base_link","camera_optical_frame"),
                           ("livox_frame","body","camera_optical_frame"),
                           ("livox_frame","base_link","d435i_color_optical_frame")]:
        with pytest.raises(ValueError,match="calibration_frame_mismatch"):
            a2_livox_front_transforms(frame,body,cam)


def test_live_cloud_freshness_does_not_reuse_the_old_inference_scan():
    inputs=LidarInputs(); cfg,_=configs()
    inputs.add("cloud",cloud(np.array([[1,0,-.5]])),1.)
    with pytest.raises(ValueError,match="stale"):
        inputs.check_cloud_freshness(2.,cfg,101_000_000_000)
    inputs.add("cloud",cloud(np.array([[1,0,-.5]]),101.),2.)
    inputs.check_cloud_freshness(2.,cfg,101_000_000_000)
    # Replayed source time remains invalid even if it just arrived.
    inputs.add("cloud",cloud(np.array([[1,0,-.5]]),100.),2.)
    with pytest.raises(ValueError,match="stale"):
        inputs.check_cloud_freshness(2.,cfg,101_000_000_000)


def test_delayed_scan_pairing_credits_only_processing_time_and_requires_a_live_stream():
    inputs=LidarInputs();cfg,_=configs();target=header(100.,"camera")
    inputs.add("cloud",cloud(np.array([[1,0,-.5]])),1.3)
    inputs.add("imu",imu(),1.3)
    inputs.add("info",camera_info(),1.3)
    # The only scan is really stale at completion: processing credit alone
    # cannot make an expired stream usable.
    with pytest.raises(ValueError,match="stale"):
        inputs.snapshot_after_processing(target,1.,1.8,cfg,100_800_000_000)
    inputs.add("cloud",cloud(np.array([[1,0,-.5]]),100.8),1.8)
    sample,_,_=inputs.snapshot_after_processing(target,1.,1.8,cfg,100_800_000_000)
    assert sample.message.header.stamp.sec==100
    assert sample.message.header.stamp.nanosec==0
    with pytest.raises(ValueError,match="stale"):
        inputs.snapshot_after_processing(target,1.,2.2,cfg,101_200_000_000)


def test_incorrect_axis_calibration_cannot_treat_sensor_y_as_body_up():
    cfg,path=configs()
    with pytest.raises(ValueError, match="gravity_axis_invalid"):
        fuse_height(np.full((360,640),2,np.uint8),cloud(points_scene()),camera_info(),
                    (2,2),np.eye(4),CAMERA_FROM_BASE,np.array([0,1.,0]),cfg,path,(2,))


def test_approaching_raised_platform_cannot_redefine_current_ground():
    cfg,path=configs(); previous=fusion()
    with pytest.raises(ValueError, match="ground_reference_jump"):
        fuse_height(np.full((360,640),2,np.uint8), cloud(points_scene(floor=-.35)),
                    camera_info(), (2,2), np.eye(4), CAMERA_FROM_BASE,
                    np.array([0,0,1.]), cfg,path,(2,),
                    reference_plane=(np.asarray(previous.metrics["plane_normal_base"]),
                                     previous.metrics["plane_offset_m"]))


def test_unobserved_cells_and_disconnected_same_height_island_stay_unavailable():
    cfg,path=configs(); points=points_scene()
    points=points[(points[:,0]<1.3)|(points[:,0]>1.8)]
    result=fuse_height(np.full((360,640),2,np.uint8), cloud(points), camera_info(),
                       (2,2),np.eye(4),CAMERA_FROM_BASE,np.array([0,0,1.]),
                       replace(cfg, seed_far_m=1.2),path,(2,))
    assert result.regions[2][result.x_values<1.].any()
    assert not result.regions[2][result.x_values>2.].any()
    assert not result.gate[(result.x_values>1.4)&(result.x_values<1.7)].any()


def test_closing_and_fitting_cannot_cross_a_forbidden_height_row():
    _,path=configs(); xs=np.linspace(.6,3,49); ys=np.linspace(2,-2,81)
    region=np.zeros((49,81),np.uint8); region[:,20:60]=255
    gate=np.ones_like(region,bool); gate[20:22]=False
    assert extract_metric_centerline(region,xs,ys,path,hard_gate=gate) is None


def test_ground_fit_rejects_wall_and_insufficient_support():
    cfg,_=configs()
    with pytest.raises(ValueError, match="ground"):
        fit_ground(np.array([[1,0,-.5]]*20),np.array([0,0,1.]),cfg)
    points=points_scene(); points[:,2]=points[:,0]*2-.5
    with pytest.raises(ValueError, match="ground"):
        fit_ground(points,np.array([0,0,1.]),cfg)


def test_inputs_match_source_time_and_reject_stale_unsynchronized_or_wrong_imu_frame():
    inputs=LidarInputs(); cfg,_=configs(); camera=header(100.1,"camera")
    with pytest.raises(ValueError, match="waiting_for_cloud"): inputs.snapshot(camera,1.,cfg)
    inputs.add("cloud",cloud(np.array([[1,0,-.5]]),100.),1.)
    inputs.add("imu",imu(100.),1.)
    inputs.add("info",camera_info(),1.)
    sample,_,up=inputs.snapshot(camera,1.1,cfg,100_100_000_000)
    assert sample.message.header.stamp.sec==100
    np.testing.assert_allclose(up,[0,0,1])
    with pytest.raises(ValueError, match="stale"): inputs.snapshot(camera,2.,cfg)
    with pytest.raises(ValueError, match="unsynchronized"): inputs.snapshot(header(101.,"camera"),1.1,cfg)
    inputs.add("imu",imu(100.,"other"),1.)
    with pytest.raises(ValueError, match="frame_mismatch"): inputs.snapshot(camera,1.1,cfg)


def test_missing_imu_camera_calibration_and_invalid_gravity_are_unavailable():
    inputs=LidarInputs(); cfg,_=configs(); camera=header(100.,"camera")
    inputs.add("cloud",cloud(np.array([[1,0,-.5]])),1.)
    with pytest.raises(ValueError,match="waiting_for_imu"): inputs.snapshot(camera,1.,cfg)
    inputs.add("imu",imu(),1.)
    with pytest.raises(ValueError,match="waiting_for_camera_info"): inputs.snapshot(camera,1.,cfg)
    inputs.add("info",camera_info(),1.)
    invalid=imu(); invalid.linear_acceleration.z=0.
    inputs.add("imu",invalid,1.)
    inputs.add("imu",invalid,1.)
    with pytest.raises(ValueError,match="gravity_invalid"): inputs.snapshot(camera,1.,cfg)


@pytest.mark.parametrize("unit,magnitude", [("auto", .99), ("auto", 9.81),
                                           ("g", .99), ("mps2", 9.81)])
def test_livox_tilted_gravity_accepts_g_and_standard_ros_acceleration(unit, magnitude):
    inputs=LidarInputs(); cfg=replace(configs()[0], imu_accel_unit=unit)
    up=np.array([.01,.56,.83]); up/=np.linalg.norm(up)
    sample=imu(frame="livox_frame")
    sample.linear_acceleration=NS(**dict(zip(("x","y","z"),up*magnitude)))
    msg=cloud(np.array([[1,0,-.5]])); msg.header.frame_id="livox_frame"
    for kind,message in [("cloud",msg),("imu",sample),("info",camera_info())]:
        inputs.add(kind,message,1.)
    _,_,actual=inputs.snapshot(header(100.,"camera"),1.,cfg)
    np.testing.assert_allclose(actual,up)


@pytest.mark.parametrize("unit,magnitude", [("auto",0.),("auto",3.),("auto",20.),
                                           ("auto",np.nan),("g",9.81),("mps2",1.)])
def test_acceleration_units_do_not_accept_invalid_gravity(unit, magnitude):
    inputs=LidarInputs(); cfg=replace(configs()[0], imu_accel_unit=unit)
    sample=imu();sample.linear_acceleration.z=magnitude
    for kind,message in [("cloud",cloud(np.array([[1,0,-.5]]))),
                         ("imu",sample),("info",camera_info())]:inputs.add(kind,message,1.)
    with pytest.raises(ValueError,match="gravity_invalid"):
        inputs.snapshot(header(100.,"camera"),1.,cfg)
    with pytest.raises(ValueError,match="imu_accel_unit"):
        replace(cfg,imu_accel_unit="invalid").validate()


@pytest.mark.parametrize("change", ["size", "intrinsics", "distortion"])
def test_mismatched_or_unsupported_camera_calibration_is_rejected(change):
    info=camera_info(); cfg,path=configs()
    if change=="size": info.width=4
    if change=="intrinsics": info.k=[0.]*9
    if change=="distortion": info.distortion_model="equidistant"
    with pytest.raises(ValueError,match="lidar_camera_"):
        fuse_height(np.full((360,640),2,np.uint8),cloud(points_scene()),info,
                    (2,2),np.eye(4),CAMERA_FROM_BASE,np.array([0,0,1.]),cfg,path,(2,))
