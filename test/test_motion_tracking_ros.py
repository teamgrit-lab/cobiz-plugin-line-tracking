"""The actual task publisher is gated by motion sensors at every command tick."""
import json
import sys
from types import SimpleNamespace as NS
import numpy as np
import pytest

from test_apriltag_task_stop_ros import RosHarness,Message,SPORT,ZERO,debug
from test_lidar_obstacles_ros import complete_inference
from test_motion_tracking import odom,imu
from local_path import LocalPathEstimate

IDENTITY='1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1'


def setup(monkeypatch):
    ros=RosHarness(monkeypatch)
    monkeypatch.setattr(sys.modules['nav_msgs.msg'],'Odometry',Message,raising=False)
    monkeypatch.setitem(debug.ENV,'LINE_TRACKING_MAX_FORWARD_MPS','.80')
    for name in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV,'LINE_TRACKING_STOP_ON_'+name.upper(),'false')
    class Client:
        metrics={'ready':True,'union_surface':1}
        def __init__(self,*_a,**_k):pass
        def plan(self,*_a):
            p=np.column_stack((np.linspace(3.,8.,60),np.zeros(60)))
            return {m:LocalPathEstimate(p,.95,1.,3.,p) for m in (0,1,2)}
    monkeypatch.setattr(debug,'ConnectomeClient',Client)
    return ros


def sensors(node,ros,t,x=0.,y=0.,yaw=0.,wz=0.):
    ros.now=t
    stamp=ros.clock_ns()/1e9
    node.on_motion_imu(imu(stamp,wz));node.on_motion_odom(odom(stamp,x=x,y=y,yaw=yaw))


def args():
    return ('--path-planner','connectome','--motion-enabled',
            '--motion-base-from-child-transform',IDENTITY,'--motion-base-from-imu-transform',IDENTITY)


@pytest.mark.parametrize('mode',[0,1,2])
def test_motion_updates_commands_without_new_inference_and_stale_sensors_stop(monkeypatch,mode):
    ros=setup(monkeypatch);periods=[]
    monkeypatch.setattr(sys.modules['rclpy.node'].Node,'create_timer',lambda self,period,callback:periods.append(period))
    def scenario(node):
        ros.start()
        for t in np.arange(1.50,2.101,.05):sensors(node,ros,float(t))
        result=complete_inference(node,ros)
        count=result['inference_count']
        first=json.loads(ros.published[SPORT][-1].parameter)
        assert 0<first['x']<=.015+1e-8
        assert result['motion_tracking']['ready'] and result['motion_tracking']['control_hz']==20
        assert ros.subscription_types['/dog_odom'] is Message
        assert '/dog_imu_raw' in ros.subscriptions
        assert periods==[.05]
        sensors(node,ros,2.15,y=.03)
        node.publish_state()
        second=json.loads(ros.published[SPORT][-1].parameter)
        assert second['z']<0 and second['x']<=first['x']+.015+1e-8
        assert ros.metrics()['inference_count']==count # No new R50/Flyvis result.
        assert ros.metrics()['motion_tracking']['translation_m']==pytest.approx(.03)
        ros.now=2.4;node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
        assert ros.metrics()['drive_reason']=='motion_odom_stale'
        # The final publication boundary rejects stale measurements too.
        assert node.publish_drive(debug.DriveDecision(.3,0,.1,'tracking')).vx==0
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
    ros.run(scenario,*args(),'--path-mask-class',str(mode))


def test_odom_jump_cannot_reuse_pre_jump_camera_path_after_sensors_recover(monkeypatch):
    ros=setup(monkeypatch)
    def scenario(node):
        ros.start()
        for t in np.arange(1.5,2.101,.05):sensors(node,ros,float(t))
        complete_inference(node,ros)
        sensors(node,ros,2.15,x=2.) # Implausible discontinuity invalidates epoch.
        node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
        sensors(node,ros,2.20,x=2.)
        node.publish_state()
        assert ros.metrics()['drive_reason']=='motion_reference_epoch_changed'
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
        # Only a new image/plan from the new motion epoch may resume.
        complete_inference(node,ros)
        assert json.loads(ros.published[SPORT][-1].parameter)['x']>0
        node.release_task_control('cancel')
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
    ros.run(scenario,*args(),'--path-mask-class','1')


def test_missing_motion_input_is_mandatory_even_with_all_optional_stops_off(monkeypatch):
    ros=setup(monkeypatch)
    def scenario(node):
        ros.start();ros.now=2.1
        result=complete_inference(node,ros)
        assert result['drive_reason']=='motion_waiting_for_odom'
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
    ros.run(scenario,*args())


def test_current_path_stamp_is_not_relabelled_as_old_camera_stamp(monkeypatch):
    ros=setup(monkeypatch)
    def scenario(node):
        ros.start()
        for t in np.arange(1.5,2.101,.05):sensors(node,ros,float(t))
        complete_inference(node,ros)
        sensors(node,ros,2.15,x=.03)
        node.publish_state()
        published=ros.published['/line_tracking/swin_l/local_path'][-1]
        assert debug._stamp_ns(published.header)==ros.clock_ns()
        assert ros.metrics()['motion_tracking']['source_stamp_sec']==pytest.approx(102.1)
    ros.run(scenario,*args())


def test_slow_telemetry_never_throttles_motion_updates_or_sensor_stops(monkeypatch):
    ros=setup(monkeypatch)
    def scenario(node):
        ros.start()
        for t in np.arange(1.5,2.101,.05):sensors(node,ros,float(t))
        # Wait on commands: the normal helper waits on telemetry, while this
        # fixture deliberately holds ROS/monotonic time inside one report slot.
        import time
        from test_lidar_obstacles_ros import image
        image(node,ros);deadline=time.perf_counter()+4
        while True:
            node.publish_state()
            if json.loads(ros.published[SPORT][-1].parameter)['x']>0:break
            assert time.perf_counter()<deadline
            time.sleep(.001)
        metrics_topic='/line_tracking/swin_l/metrics'
        count=len(ros.published[metrics_topic]);moves=len(ros.published[SPORT])
        sensors(node,ros,2.15,y=.03);node.publish_state()
        assert len(ros.published[metrics_topic])==count
        assert len(ros.published[SPORT])==moves+1
        assert json.loads(ros.published[SPORT][-1].parameter)['z']<0
        ros.now=2.4;node.publish_state()
        assert len(ros.published[metrics_topic])==count
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
        node.release_task_control('cancel')
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
    ros.run(scenario,*args(),'--telemetry-hz','1')


def test_process_results_keep_parent_motion_and_cancellation_gates(monkeypatch):
    ros=setup(monkeypatch);closed=[]
    monkeypatch.setattr(debug,'_connectome_client_from_args',lambda *_:NS(
        url='http://127.0.0.1:8091/v1/plan',config=debug.PlannerConfig(),timeout=.8,
        metrics={'ready':False}))
    class Process:
        device=NS(type='cuda');cuda_memory={}
        def __init__(self,*_):pass
        def reset(self):pass
        def close(self):closed.append(True)
        def infer(self,rgb,stamp,generation,sequence):
            p=np.array([[3.,0.],[8.,0.]])
            estimate=LocalPathEstimate(p,.9,1.,3.,p)
            return {'result':NS(selected_mask=np.ones((360,640),np.uint8),inference_seconds=.01),
                'estimates':{m:estimate for m in (0,1,2)},'planner':{'ready':True,'union_surface':1},
                'segmentation_finished_at':debug.time.monotonic()}
    monkeypatch.setattr(debug,'PerceptionProcess',Process)
    def scenario(node):
        ros.start()
        for t in np.arange(1.5,2.101,.05):sensors(node,ros,float(t))
        result=complete_inference(node,ros)
        assert result['pipeline_timing']['inference_process']
        assert json.loads(ros.published[SPORT][-1].parameter)['x']>0
        ros.now=2.4;node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
        node.release_task_control('cancel')
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
    ros.run(scenario,*args(),'--inference-process')
    assert closed
