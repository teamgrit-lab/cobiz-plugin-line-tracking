"""Camera worker -> LiDAR association -> opposite sidestep -> Sport conversion."""
import json
from dataclasses import asdict
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_apriltag_task_stop_ros import SPORT, ZERO, debug
from test_lidar_obstacles_ros import enabled_ros, sensors, complete_inference
from swin_l_drive_control import DriveDecision
import road_objects


@pytest.mark.parametrize('y,sport_sign',[(.5,1),(-.5,-1)])
def test_front_model_sidestep_survives_timer_limit_and_sport_sign_conversion(monkeypatch,y,sport_sign):
    ros=enabled_ros(monkeypatch)
    monkeypatch.setitem(debug.ENV,'LIDAR_OBSTACLE_HALF_WIDTH_M','4.0')
    monkeypatch.setitem(debug.ENV,'LIDAR_OBSTACLE_REAR_M','3.0')
    monkeypatch.setattr(road_objects,'RoadObjectDetector',lambda *args:NS(detect=lambda rgb:[
        {'label':'person','class_id':1,'score':.95,'box':[0,0,640,350]}]))
    def segment(_frame,**kwargs):
        return NS(selected_mask=np.ones((270,480),np.uint8),inference_seconds=.01,
                  front_command=asdict(DriveDecision(.12,0.,0.,'tracking')),front_metrics={})
    monkeypatch.setattr(debug,'_segmenter_from_args',lambda _:NS(device=NS(type='cuda'),reset=lambda:None,segment=segment))
    def scenario(node):
        ros.start();ros.now=2.1
        sensors(node,ros,[[3,y],[3.05,y+.05],[3.1,y-.05]])
        result=complete_inference(node,ros)
        command=json.loads(ros.published[SPORT][-1].parameter)
        assert result['drive_reason']=='lidar_avoiding'
        assert command['y']*sport_sign>0
        assert command['x']<=.12  # Legacy decide_drive must not replace learned .12.
        assert node.obstacles.metrics['road_objects'][0]['label']=='person'
        for _ in range(50):
            ros.now+=.05
            sensors(node,ros,[[3,y],[3.05,y+.05],[3.1,y-.05]])
            complete_inference(node,ros)
        command=json.loads(ros.published[SPORT][-1].parameter)
        assert .3<=abs(command['y'])<=.8
        assert command['y']*sport_sign>0
        # No fresh image result: a new LiDAR scan must not keep fast avoidance alive.
        ros.now+=.9;sensors(node,ros,[[3,y],[3.05,y+.05]])
        node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
        assert 'road_object_' in ros.metrics()['drive_reason']
        node.release_task_control('cancel')
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
    ros.run(scenario,'--path-mask-class','1','--path-planner','local','--road-objects-enabled',
            '--road-object-sha256','0'*64,'--front-connectome-url','http://127.0.0.1:8092/v1/front-road',
            '--front-connectome-sha256','0'*64)
