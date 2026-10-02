"""Drive guards with actual height fusion and mocked ROS/model boundaries."""
import json
import sys
import time
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_apriltag_task_stop_ros import Message, RosHarness, SPORT, ZERO, debug
from test_lidar_height import CAMERA_FROM_BASE, camera_info, cloud, imu, points_scene


@pytest.fixture(autouse=True)
def unitree_fixture_frame(monkeypatch):
    # Existing geometry fixtures explicitly model the optional Unitree sensor.
    monkeypatch.setitem(debug.ENV,"SWIN_L_LIDAR_FRAME_ID","hesai_lidar")


def matrix_argument(matrix):
    return ",".join(map(str, matrix.ravel()))


def test_tilted_livox_g_imu_filters_steps_and_holds_command_on_sensor_loss(monkeypatch):
    ros=RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV,"SWIN_L_LIDAR_HEIGHT_ENABLED","true")
    monkeypatch.setitem(debug.ENV,"SWIN_L_LIDAR_FRAME_ID","livox_frame")
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV,"LINE_TRACKING_STOP_ON_"+check.upper(),"false")
    # Synthetic measured mount: forward tilt, plus nonzero translation. This
    # verifies projection/height logic; it is not a calibration for the robot.
    angle=np.deg2rad(-34.)
    base=np.eye(4);base[:3,:3]=[[np.cos(angle),0,np.sin(angle)],[0,1,0],
                              [-np.sin(angle),0,np.cos(angle)]]
    base[:3,3]=[.2,0,.1]
    def scenario(node):
        assert "/livox/lidar" in ros.subscriptions
        assert "/livox/imu" in ros.subscriptions
        ros.start();node.publish_state()
        def feed(points):
            ros.now+=.05;stamp=ros.clock_ns()/1e9
            previous=ros.metrics()["inference_count"]
            raw=(points-base[:3,3])@base[:3,:3]
            msg=cloud(raw,stamp);msg.header.frame_id="livox_frame"
            up=base[:3,:3].T@np.array([0,0,.99])
            sample=imu(stamp,frame="livox_frame")
            sample.linear_acceleration=NS(**dict(zip(("x","y","z"),up)))
            node.on_camera_info(camera_info(stamp));node.on_lidar_imu(sample);node.on_lidar(msg)
            image=Message();image.header.stamp=ros.stamp();node.on_image(image)
            deadline=time.perf_counter()+5
            while True:
                node.publish_state()
                if ros.metrics()["inference_count"]>previous:return ros.metrics()
                assert time.perf_counter()<deadline,ros.errors
                time.sleep(.001)
        good=feed(points_scene())
        assert good["lidar_height"]["filter_applied"] and good["path_tracked"]
        np.testing.assert_allclose(good["lidar_height"]["up_base"],[0,0,1],atol=1e-6)
        saved=json.loads(ros.published[SPORT][-1].parameter);assert saved["x"]>0
        ros.now+=.6;node.publish_state()
        assert ros.metrics()["lidar_height"]["reason"]=="lidar_stale"
        assert json.loads(ros.published[SPORT][-1].parameter)==saved
        blocked=points_scene();blocked[(blocked[:,0]>.9)&(blocked[:,0]<1.2),2]+=.15
        result=feed(blocked)
        assert result["lidar_height"]["reason"]=="lidar_path_unavailable"
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
    ros.run(scenario,"--lidar-to-base-transform",matrix_argument(base),
            "--base-to-camera-transform",matrix_argument(CAMERA_FROM_BASE),
            "--near-distance-m",".6","--far-distance-m","3",
            "--search-half-width-m","2","--lidar-footprint-radius-m",".1",
            "--bev-height-px","49","--bev-width-px","81","--branch-preference","center")


def feed_height(ros,node,points=None,invalid_cloud=False):
    ros.now += .05
    node.publish_state(); target=ros.metrics()["inference_count"]+1
    stamp=ros.clock_ns()/1e9
    node.on_camera_info(camera_info(stamp))
    node.on_lidar_imu(imu(stamp))
    msg=cloud(points_scene() if points is None else points,stamp)
    if invalid_cloud: msg.data=msg.data[:-1]
    node.on_lidar(msg)
    image=Message(); image.header.stamp=ros.stamp(); node.on_image(image)
    deadline=time.perf_counter()+5
    while True:
        node.publish_state()
        if ros.metrics()["inference_count"]>=target: return ros.metrics()
        assert time.perf_counter()<deadline,ros.errors
        time.sleep(.001)


def test_unitree_sensor_loss_and_ground_fit_failure_hold_motion_but_valid_height_exclusion_stops(monkeypatch):
    ros=RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV,"SWIN_L_LIDAR_HEIGHT_ENABLED","true")
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV,"LINE_TRACKING_STOP_ON_"+check.upper(),"false")
    # A left-side surface produces a turn, exercising both saved speed and yaw.
    mask=np.zeros((360,640),np.uint8); mask[:,:320]=2
    monkeypatch.setattr(debug,"BestSoFarSegmenter",lambda _:NS(
        device=NS(type="cuda"),reset=lambda:None,
        segment=lambda _frame,**kwargs:NS(selected_mask=mask.copy(),inference_seconds=.01)))

    def scenario(node):
        node.drive_config=replace(node.drive_config,heading_gain=2.)
        assert debug.DEFAULT_LIDAR_TOPIC in ros.subscriptions
        assert debug.DEFAULT_IMU_TOPIC in ros.subscriptions
        assert debug._camera_info_topic(debug.DEFAULT_IMAGE_TOPIC) in ros.subscriptions
        ros.start()
        node.publish_state()
        assert ros.metrics()["drive_reason"]=="waiting_for_path"
        assert ros.metrics()["lidar_height"]["reason"]=="lidar_waiting_for_result"
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO

        def feed(points=None, invalid_cloud=False):
            return feed_height(ros,node,points,invalid_cloud)

        good=feed()
        assert good["path_tracked"]
        assert good["lidar_height"]["reason"] is None
        assert json.loads(ros.published[SPORT][-1].parameter)["x"]>0
        assert node.last_valid_forward_mps is not None
        saved_command=json.loads(ros.published[SPORT][-1].parameter)
        saved_speed=node.last_valid_forward_mps
        saved_yaw=node.last_valid_yaw_rate
        assert 0<saved_speed<node.drive_config.max_forward_mps
        assert saved_yaw!=0

        def assert_held(result,height_reason):
            assert result["drive_reason"]=="tracking_path_hold"
            assert result["lidar_height"]["reason"]==height_reason
            assert json.loads(ros.published[SPORT][-1].parameter)==saved_command
            assert node.last_valid_forward_mps==saved_speed
            assert node.last_valid_yaw_rate==saved_yaw
            assert not result["path_tracked"]
            assert not ros.published["/line_tracking/swin_l/local_path"][-1].poses

        ros.now += .6
        node.publish_state()
        assert_held(ros.metrics(),"lidar_stale")
        # A long outage does not invent a timeout or erase the saved command.
        ros.now+=5.; node.publish_state()
        assert_held(ros.metrics(),"lidar_stale")

        assert feed()["path_tracked"]
        raised=points_scene(); raised[:,2]=-.35
        jump=feed(raised)
        assert_held(jump,"lidar_ground_reference_jump")
        assert feed()["path_tracked"]

        bad=feed(invalid_cloud=True)
        assert_held(bad,"lidar_processing_error")

        assert feed()["path_tracked"]
        # Ground-fit failure follows the same command-hold policy as sensor loss.
        no_ground=points_scene(); no_ground[:,2]=.2
        lost=feed(no_ground)
        assert_held(lost,"lidar_ground_unavailable")

        assert feed()["path_tracked"]
        blocked=points_scene()
        blocked[(blocked[:,0]>.9)&(blocked[:,0]<1.2),2]+=.15
        excluded=feed(blocked)
        assert excluded["drive_reason"]=="lidar_path_unavailable"
        assert excluded["lidar_height"]["reason"]=="lidar_path_unavailable"
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
        assert node.last_valid_forward_mps is None
        assert node.last_valid_yaw_rate is None
        assert feed()["path_tracked"]
        node.on_task_event(Message(json.dumps({"type":"TASK_ABORTED","task_id":"tag-stop-1"})))
        ros.now+=1.1; node.publish_state(); ros.start(task_id="height-restart")
        node.publish_state()
        assert ros.metrics()["drive_reason"]=="waiting_for_path"
        assert node.last_valid_forward_mps is None
        assert node.last_valid_yaw_rate is None
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO

    ros.run(scenario,"--lidar-to-base-transform",matrix_argument(np.eye(4)),
            "--base-to-camera-transform",matrix_argument(CAMERA_FROM_BASE),
            "--near-distance-m",".6","--far-distance-m","3",
            "--search-half-width-m","2","--lidar-footprint-radius-m",".1",
            "--bev-height-px","49","--bev-width-px","81","--branch-preference","center")


def test_sensor_loss_uses_explicit_existing_path_unavailable_stop_setting(monkeypatch):
    ros=RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV,"SWIN_L_LIDAR_HEIGHT_ENABLED","true")
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV,"LINE_TRACKING_STOP_ON_"+check.upper(),"false")
    monkeypatch.setitem(debug.ENV,"LINE_TRACKING_STOP_ON_PATH_UNAVAILABLE","true")

    def scenario(node):
        ros.start(); assert feed_height(ros,node)["path_tracked"]
        ros.now+=.6; node.publish_state()
        assert ros.metrics()["lidar_height"]["reason"]=="lidar_stale"
        assert ros.metrics()["drive_reason"]=="path_unavailable"
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO

    ros.run(scenario,"--lidar-to-base-transform",matrix_argument(np.eye(4)),
            "--base-to-camera-transform",matrix_argument(CAMERA_FROM_BASE),
            "--near-distance-m",".6","--far-distance-m","3",
            "--search-half-width-m","2","--lidar-footprint-radius-m",".1",
            "--bev-height-px","49","--bev-width-px","81","--branch-preference","center")


@pytest.mark.parametrize("missing", [False,True])
def test_timestamped_tf_is_required_when_calibration_matrices_are_blank(monkeypatch,missing):
    ros=RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV,"SWIN_L_LIDAR_HEIGHT_ENABLED","true")
    monkeypatch.setitem(debug.ENV,"LINE_TRACKING_STOP_ON_STARTUP_HOLD","false")
    calls=[]
    class Buffer:
        def lookup_transform(self,target,source,stamp):
            calls.append((target,source,stamp.sec,stamp.nanosec))
            if missing: raise RuntimeError("no calibrated TF")
            # A 120 degree rotation around (1,-1,1) produces CAMERA_FROM_BASE.
            q=[0,0,0,1] if target=="base_link" else [.5,-.5,.5,.5]
            return NS(transform=NS(translation=NS(x=0.,y=0.,z=0.),
                                   rotation=NS(**dict(zip(("x","y","z","w"),q)))))
    monkeypatch.setitem(sys.modules,"tf2_ros",NS(Buffer=Buffer,TransformListener=lambda *_:object()))
    monkeypatch.setitem(sys.modules,"rclpy.time",NS(Time=NS(from_msg=lambda stamp:stamp)))

    def scenario(node):
        ros.start(); result=feed_height(ros,node)
        if missing:
            assert result["drive_reason"] in ("tracking", "tracking_slow_turn")
            assert result["lidar_height"]["reason"]=="lidar_transform_unavailable"
            assert result["lidar_height"]["vision_fallback"]
            assert not result["lidar_height"]["filter_applied"]
            assert json.loads(ros.published[SPORT][-1].parameter)["x"]>0
        else:
            assert result["path_tracked"]
            assert calls[0][:2]==("base_link","hesai_lidar")
            assert calls[1][:2]==("camera","base_link")
            assert calls[0][2:]==calls[1][2:]

    ros.run(scenario,"--lidar-to-base-transform","","--base-to-camera-transform","",
            "--near-distance-m",".6","--far-distance-m","3",
            "--search-half-width-m","2","--lidar-footprint-radius-m",".1",
            "--bev-height-px","49","--bev-width-px","81","--branch-preference","center")


def test_slow_gpu_inference_uses_the_scan_validated_before_inference(monkeypatch):
    ros=RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV,"SWIN_L_LIDAR_HEIGHT_ENABLED","true")
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV,"LINE_TRACKING_STOP_ON_"+check.upper(),"false")
    def segment(_frame,**_kwargs):
        # Inference exceeds the source-age budget. New sensor callbacks keep
        # arriving while the model works, without changing its matched scan.
        ros.now+=.8
        ros.node.on_lidar(cloud(points_scene(),ros.clock_ns()/1e9))
        return NS(selected_mask=np.full((360,640),2,np.uint8),inference_seconds=.8)
    monkeypatch.setattr(debug,"BestSoFarSegmenter",lambda _:NS(
        device=NS(type="cuda"),reset=lambda:None,segment=segment))
    def scenario(node):
        ros.start(); result=feed_height(ros,node)
        assert result["path_tracked"]
        assert result["lidar_height"]["reason"] is None
        assert result["lidar_height"]["filter_applied"]
        assert result["lidar_height"]["cloud_stamp_ns"]<ros.clock_ns()
        ros.now+=.3
        node.on_lidar(cloud(points_scene(),ros.clock_ns()/1e9))
        node.publish_state()
        # The live stream is fresh, but a completed result still has a bounded
        # total age; it cannot live forever after a long GPU operation.
        assert ros.metrics()["lidar_height"]["reason"]=="lidar_stale"
        assert ros.metrics()["drive_reason"]=="tracking_path_hold"
        ros.now+=.6;node.publish_state()
        assert ros.metrics()["lidar_height"]["reason"]=="lidar_stale"
        assert ros.metrics()["drive_reason"]=="tracking_path_hold"
    ros.run(scenario,"--lidar-to-base-transform",matrix_argument(np.eye(4)),
            "--base-to-camera-transform",matrix_argument(CAMERA_FROM_BASE),
            "--near-distance-m",".6","--far-distance-m","3",
            "--search-half-width-m","2","--lidar-footprint-radius-m",".1",
            "--bev-height-px","49","--bev-width-px","81","--branch-preference","center")


def test_a2_vertical_profile_does_not_require_a_tf_publisher(monkeypatch):
    ros=RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV,"SWIN_L_LIDAR_HEIGHT_ENABLED","true")
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV,"LINE_TRACKING_STOP_ON_"+check.upper(),"false")
    def scenario(node):
        assert node.tf_buffer is None
        ros.start(); ros.now+=.05; stamp=ros.clock_ns()/1e9
        base,_=debug.a2_front_transforms("hesai_lidar","base_link","camera_optical_frame")
        points=points_scene();raw=(points-base[:3,3])@base[:3,:3]
        node.on_camera_info(camera_info(stamp,frame="camera_optical_frame"))
        sample=imu(stamp);sample.linear_acceleration=NS(x=0.,y=9.81,z=0.)
        node.on_lidar_imu(sample);node.on_lidar(cloud(raw,stamp))
        image=Message();image.header.stamp=ros.stamp();image.header.frame_id="camera_optical_frame"
        node.on_image(image)
        deadline=time.perf_counter()+5
        while True:
            node.publish_state()
            if ros.metrics()["inference_count"]:break
            assert time.perf_counter()<deadline,ros.errors
            time.sleep(.001)
        result=ros.metrics()
        assert result["lidar_height"]["reason"] is None
        assert result["lidar_height"]["calibration_source"]=="unitree-a2-front"
        assert result["lidar_height"]["filter_applied"]
        assert result["path_tracked"]
    ros.run(scenario,"--lidar-calibration-profile","unitree-a2-front",
            "--lidar-to-base-transform","","--base-to-camera-transform","",
            "--near-distance-m",".6","--far-distance-m","3",
            "--search-half-width-m","2","--lidar-footprint-radius-m",".1",
            "--bev-height-px","49","--bev-width-px","81","--branch-preference","center")


def test_scan_delivered_during_gpu_inference_is_paired_without_false_stale(monkeypatch):
    ros=RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV,"SWIN_L_LIDAR_HEIGHT_ENABLED","true")
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV,"LINE_TRACKING_STOP_ON_"+check.upper(),"false")
    def segment(_frame,**_kwargs):
        stamp=ros.clock_ns()/1e9
        ros.now+=.3
        ros.node.on_camera_info(camera_info(stamp))
        ros.node.on_lidar_imu(imu(stamp))
        ros.node.on_lidar(cloud(points_scene(),stamp))
        ros.now+=.5
        ros.node.on_lidar(cloud(points_scene(),ros.clock_ns()/1e9))
        return NS(selected_mask=np.full((360,640),2,np.uint8),inference_seconds=.8)
    monkeypatch.setattr(debug,"BestSoFarSegmenter",lambda _:NS(
        device=NS(type="cuda"),reset=lambda:None,segment=segment))
    def scenario(node):
        ros.start();ros.now+=.05
        image=Message();image.header.stamp=ros.stamp();node.on_image(image)
        deadline=time.perf_counter()+5
        while True:
            node.publish_state()
            if ros.metrics()["inference_count"]:break
            assert time.perf_counter()<deadline,ros.errors
            time.sleep(.001)
        result=ros.metrics()
        assert result["lidar_height"]["reason"] is None
        assert result["lidar_height"]["filter_applied"]
        assert not result["lidar_height"]["vision_fallback"]
        assert result["path_tracked"]
    ros.run(scenario,"--lidar-to-base-transform",matrix_argument(np.eye(4)),
            "--base-to-camera-transform",matrix_argument(CAMERA_FROM_BASE),
            "--near-distance-m",".6","--far-distance-m","3",
            "--search-half-width-m","2","--lidar-footprint-radius-m",".1",
            "--bev-height-px","49","--bev-width-px","81","--branch-preference","center")


def test_startup_sensor_failure_can_use_vision_but_a_valid_height_block_cannot(monkeypatch):
    ros=RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV,"SWIN_L_LIDAR_HEIGHT_ENABLED","true")
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV,"LINE_TRACKING_STOP_ON_"+check.upper(),"false")
    def scenario(node):
        ros.start()
        # No LiDAR inputs on the first camera frame.
        image=Message();image.header.stamp=ros.stamp();node.on_image(image)
        deadline=time.perf_counter()+5
        while True:
            node.publish_state()
            if ros.metrics()["inference_count"]:break
            assert time.perf_counter()<deadline,ros.errors
            time.sleep(.001)
        result=ros.metrics()
        assert result["path_tracked"]
        assert result["lidar_height"]["vision_fallback"]
        assert result["lidar_height"]["reason"]=="lidar_waiting_for_cloud"
        assert json.loads(ros.published[SPORT][-1].parameter)["x"]>0
        blocked=points_scene();blocked[(blocked[:,0]>.9)&(blocked[:,0]<1.2),2]+=.15
        result=feed_height(ros,node,blocked)
        assert result["drive_reason"]=="lidar_path_unavailable"
        assert not result["lidar_height"]["vision_fallback"]
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
        # Later sensor failure must not re-enable startup fallback after a real
        # height result has established the current task's reference.
        result=feed_height(ros,node,invalid_cloud=True)
        assert not result["lidar_height"]["vision_fallback"]
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
    ros.run(scenario,"--lidar-to-base-transform",matrix_argument(np.eye(4)),
            "--base-to-camera-transform",matrix_argument(CAMERA_FROM_BASE),
            "--near-distance-m",".6","--far-distance-m","3",
            "--search-half-width-m","2","--lidar-footprint-radius-m",".1",
            "--bev-height-px","49","--bev-width-px","81","--branch-preference","center")


@pytest.mark.parametrize("option,value", [
    ("--lidar-max-age-sec","nan"), ("--lidar-max-up-m","-1"),
    ("--lidar-to-base-transform","1,2,3"), ("--lidar-min-cell-points","0"),
])
def test_invalid_height_configuration_fails_before_startup(option,value):
    with pytest.raises(SystemExit): debug.parse_args(["ros2",option,value])


@pytest.mark.parametrize("profile,frame,unit", [("tf","hesai_lidar","mps2"),
    ("unitree-a2-front","hesai_lidar","mps2"),("tf","livox_frame","g")])
def test_mcap_fuses_height_and_clears_path_at_a_reference_jump(tmp_path,monkeypatch,profile,frame,unit):
    source=tmp_path/"sensor.mcap"; source.touch()
    frames=[]; paths=[]
    monkeypatch.setattr(debug,"_build_writer",lambda *_:NS(write=frames.append,release=lambda:None))
    monkeypatch.setattr(debug,"BestSoFarSegmenter",lambda _:NS(
        segment=lambda _:NS(selected_mask=np.full((360,640),2,np.uint8),total_seconds=.01),
        metadata=lambda:{}))
    def overlay(frame,mask,estimate,path,*args,**kwargs):
        paths.append(path); return frame
    monkeypatch.setattr(debug,"render_local_path_overlay",overlay)
    def events(_path,topics,start_time_ns=0):
        assert debug.DEFAULT_LIDAR_TOPIC in topics
        for stamp,floor in [(100.,-.5),(100.5,-.35)]:
            image=Message(); image.header.stamp=NS(sec=int(stamp),nanosec=round(stamp%1*1e9))
            points=points_scene(floor=floor);sample=imu(stamp,frame=frame)
            if unit=="g":sample.linear_acceleration.z=.99
            if profile=="unitree-a2-front":
                image.header.frame_id="camera_optical_frame"
                base,_=debug.a2_front_transforms("hesai_lidar","base_link","camera_optical_frame")
                points=(points-base[:3,3])@base[:3,:3]
                sample.linear_acceleration=NS(x=0.,y=9.81,z=0.)
            scan=cloud(points,stamp);scan.header.frame_id=frame
            for topic,msg in [(debug.DEFAULT_IMU_TOPIC,sample),
                              ("/camera/camera_info",camera_info(stamp,frame=image.header.frame_id)),
                              (debug.DEFAULT_LIDAR_TOPIC,scan),
                              ("/camera/image_raw",image)]:
                yield NS(name="sensor_msgs/msg/Image"),NS(topic=topic),NS(log_time=round(stamp*1e9)),msg
    monkeypatch.setattr(debug,"_iter_mcap_events",events)
    matrices=["--lidar-to-base-transform",matrix_argument(np.eye(4)),
              "--base-to-camera-transform",matrix_argument(CAMERA_FROM_BASE)] if profile=="tf" else []
    args=debug.parse_args(["mcap","--input",str(source),"--output",str(tmp_path/"out.mp4"),
                          "--report",str(tmp_path/"out.json"),"--image-topic","/camera/image_raw",
                          "--lidar-height-enabled","--lidar-calibration-profile",profile,*matrices,
                          "--lidar-frame-id",frame,"--lidar-imu-accel-unit",unit,
                          "--near-distance-m",".6","--far-distance-m","3",
                          "--search-half-width-m","2","--lidar-footprint-radius-m",".1",
                          "--bev-height-px","49","--bev-width-px","81","--branch-preference","center"])
    assert debug.run_mcap(args)==0
    assert len(frames)==2 and paths[0] is not None and paths[1] is None
    report=json.loads((tmp_path/"out.json").read_text())
    assert report["lidar_height"]["reason"]=="lidar_ground_reference_jump"


def test_mcap_height_requires_measured_transforms_before_model_initialization(tmp_path,monkeypatch):
    source=tmp_path/"unitree.mcap"; source.touch()
    monkeypatch.setattr(debug,"BestSoFarSegmenter",lambda _:pytest.fail("model must not load without calibration"))
    args=debug.parse_args(["mcap","--input",str(source),"--output",str(tmp_path/"out.mp4"),
                          "--lidar-height-enabled","--lidar-to-base-transform","",
                          "--base-to-camera-transform",""])
    with pytest.raises(ValueError,match="MCAP height fusion needs"):
        debug.run_mcap(args)
