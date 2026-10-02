"""Completed-inference recovery windows with mocked ROS/model boundaries."""
import json
import time
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_apriltag_task_stop_ros import Message, RosHarness, SPORT, ZERO, debug
from test_lidar_height import CAMERA_FROM_BASE, camera_info, cloud, imu, points_scene


@pytest.mark.parametrize("with_lidar", [False, True])
@pytest.mark.parametrize("recover_at", [2, 5, None])
def test_completed_inferences_recover_straight_stop_on_fifth_failure_and_reset(monkeypatch,with_lidar,recover_at):
    ros=RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV,"LINE_TRACKING_PATH_LOSS_RECOVERY_ENABLED","true")
    monkeypatch.setitem(debug.ENV,"SWIN_L_LIDAR_HEIGHT_ENABLED",str(with_lidar))
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV,"LINE_TRACKING_STOP_ON_"+check.upper(),"false")
    good=np.zeros((360,640),np.uint8);good[:,:320]=2
    detection=NS(mask=good.copy())
    monkeypatch.setattr(debug,"BestSoFarSegmenter",lambda _:NS(
        device=NS(type="cuda"),reset=lambda:None,
        segment=lambda _frame,**kwargs:NS(selected_mask=detection.mask.copy(),inference_seconds=.01)))

    def scenario(node):
        node.drive_config=replace(node.drive_config,heading_gain=2.)
        ros.start()
        def feed(*,bad_cloud=False,points=None):
            ros.now+=.05;node.publish_state();previous=ros.metrics()["inference_count"]
            stamp=ros.clock_ns()/1e9
            if with_lidar:
                scan=cloud(points_scene() if points is None else points,stamp)
                scan.header.frame_id="livox_frame"
                if bad_cloud:scan.data=scan.data[:-1]
                sample=imu(stamp,frame="livox_frame");sample.linear_acceleration.z=.99
                node.on_camera_info(camera_info(stamp));node.on_lidar_imu(sample);node.on_lidar(scan)
            image=Message();image.header.stamp=ros.stamp();node.on_image(image)
            deadline=time.perf_counter()+5
            while True:
                node.publish_state()
                if ros.metrics()["inference_count"]>previous:return ros.metrics()
                assert time.perf_counter()<deadline,ros.errors
                time.sleep(.001)
        # Missing paths at task startup must never initiate forward motion.
        detection.mask[:]=0
        assert feed()["drive_reason"]=="waiting_for_path"
        if with_lidar:
            # A simultaneous missing visual path must not erase an independent
            # committed-branch stop reason at the ROS readiness boundary.
            assert node.height_reason(2,ros.now)=="vision_path_unavailable"
            guarded=debug.SmoothedPath(np.empty((0,2)),0.,0.,"test",stop_reason="branch_path_lost")
            with monkeypatch.context() as branch_patch:
                branch_patch.setattr(debug.LocalPathSmoother,"current",lambda *_:guarded)
                assert node.drive_readiness(2,ros.now)[1].reason=="branch_path_lost"
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
        detection.mask=good.copy();assert feed()["path_tracked"]
        saved=json.loads(ros.published[SPORT][-1].parameter)
        assert saved["x"]>0 and abs(saved["z"])>0
        assert ros.metrics()["path_recovery"]["failed_inferences"]==0
        for count in range(1,6):
            detection.mask=good.copy() if count==recover_at else np.zeros_like(good)
            result=feed()
            if count==recover_at:
                assert result["path_tracked"]
                assert result["path_recovery"]["failed_inferences"]==0
                assert json.loads(ros.published[SPORT][-1].parameter)["z"]!=0
                break
            expected="tracking_path_recovery" if count<5 else "path_recovery_exhausted"
            assert result["drive_reason"]==expected
            assert result["path_recovery"]["failed_inferences"]==count
            command=json.loads(ros.published[SPORT][-1].parameter)
            assert command==({"x":saved["x"],"y":0.,"z":0.} if count<5 else ZERO)
            assert not result["path_yaw_held"]
            for _ in range(10):node.publish_state()
            assert ros.metrics()["path_recovery"]["failed_inferences"]==count
            assert ros.metrics()["drive_reason"]==expected
        if recover_at is None:
            if with_lidar:
                # A later sensor fault cannot revive motion after the visual
                # recovery window has already exhausted its budget.
                assert feed(bad_cloud=True)["drive_reason"]=="path_recovery_exhausted"
                assert ros.metrics()["path_recovery"]["failed_inferences"]==5
                assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
            detection.mask=good.copy();assert feed()["path_tracked"]
        # Successful detection starts a new, independent recovery window.
        detection.mask[:]=0
        assert feed()["drive_reason"]=="tracking_path_recovery"
        assert ros.metrics()["path_recovery"]["failed_inferences"]==1
        if with_lidar:
            saved_yaw=node.last_valid_yaw_rate
            assert feed(bad_cloud=True)["drive_reason"]=="tracking_path_hold"
            assert ros.metrics()["path_recovery"]["failed_inferences"]==1
            assert json.loads(ros.published[SPORT][-1].parameter)["z"]==saved_yaw
            detection.mask=good.copy();assert feed()["path_tracked"]
            blocked=points_scene();blocked[(blocked[:,0]>.9)&(blocked[:,0]<1.2),2]+=.15
            assert feed(points=blocked)["drive_reason"]=="lidar_path_unavailable"
            assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
        node.on_task_event(Message(json.dumps({"type":"TASK_ABORTED","task_id":"tag-stop-1"})))
        ros.now+=1.1;node.publish_state();ros.start(task_id="new-window");node.publish_state()
        assert ros.metrics()["path_recovery"]["failed_inferences"]==0
        assert node.last_valid_forward_mps is None
        detection.mask[:]=0
        assert feed()["drive_reason"]=="waiting_for_path"
    matrix=lambda m:",".join(map(str,m.ravel()))
    ros.run(scenario,"--lidar-to-base-transform",matrix(np.eye(4)),
            "--base-to-camera-transform",matrix(CAMERA_FROM_BASE),
            "--near-distance-m",".6","--far-distance-m","3",
            "--search-half-width-m","2","--lidar-footprint-radius-m",".1",
            "--bev-height-px","49","--bev-width-px","81","--branch-preference","center")
