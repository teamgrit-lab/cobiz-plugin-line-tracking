import sys
from pathlib import Path
from dataclasses import replace

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from road_objects import RoadObjectConfig, associate_objects, exclusion_mask, box_visible_fraction
from road_object_avoidance import RoadObjectAvoidanceController
from local_avoidance import AvoidanceConfig
from lidar_obstacles import ObstacleConfig
from lidar_common import calibrated_transforms
from sensor_fixtures import camera_info
from test_local_avoidance import scene, path, NOMINAL

C = ObstacleConfig(horizontal_range_m=6., fov_deg=360., half_width_m=4., rear_m=3.)


def observed(points=()):
    return replace(scene(points), observed=np.ones(C.shape, bool), costs=np.zeros(C.shape))


@pytest.mark.parametrize('label', ['person', 'car', 'truck'])
@pytest.mark.parametrize('y,sign', [(0.5, -1), (-0.5, 1)])
def test_objects_choose_opposite_side_and_reach_requested_target_band(label, y, sign):
    ctrl = RoadObjectAvoidanceController(AvoidanceConfig(), C, RoadObjectConfig())
    ctrl.set_objects([{'label': label, 'x': 3., 'y': y}], 100)
    commands = []
    for i in range(60):
        now = i*.05
        scan = replace(observed([[3., y]]), stamp_ns=100_000_000_000+round(now*1e9), arrival=now)
        commands.append(ctrl.apply(NOMINAL, path(), scan, np.ones(C.shape, bool), now=now, clock_ns=scan.stamp_ns))
    assert all(c.vy*sign > 0 for c in commands)
    assert .3 <= abs(commands[-1].vy) <= .8
    assert .3 <= abs(ctrl.metrics['target_lateral_mps']) <= .8
    assert all(abs(b.vy-a.vy) <= .3*.05+1e-8 for a,b in zip(commands,commands[1:]))


def test_opposite_side_blocked_stops_instead_of_switching_towards_person():
    ctrl = RoadObjectAvoidanceController(AvoidanceConfig(), C, RoadObjectConfig())
    ctrl.set_objects([{'label': 'person', 'x': 3., 'y': .5}], 100)
    region = C.centers()[..., 1] >= -.45
    command = ctrl.apply(NOMINAL, path(), observed([[3., .5]]), region, now=0, clock_ns=100_000_000_000)
    assert command.vx == command.vy == command.yaw_rate == 0


def test_unobserved_side_and_stale_cloud_cannot_authorize_fast_motion():
    for scan in [replace(observed(), observed=np.zeros(C.shape, bool)), replace(observed(), arrival=-1)]:
        ctrl = RoadObjectAvoidanceController(AvoidanceConfig(), C, RoadObjectConfig())
        ctrl.set_objects([{'label': 'car', 'x': 3., 'y': -.5}], 100)
        command = ctrl.apply(NOMINAL, path(), scan, np.ones(C.shape, bool), now=0, clock_ns=100_000_000_000)
        assert command.vx == command.vy == 0


def association_scene(y=.4):
    xyz = np.array([[3., y, 0.], [3.05, y+.05, .05], [3.1, y-.05, -.05]])
    info = camera_info()
    _, camera = calibrated_transforms('livox-a2-front','livox_frame','base_link','camera_optical_frame')
    image = xyz @ camera[:3,:3].T + camera[:3,3]
    uv = cv2.projectPoints(image, np.zeros(3), np.zeros(3), np.asarray(info.k).reshape(3,3), np.asarray(info.d))[0].reshape(-1,2)
    box = [*np.min(uv,axis=0)-10, *np.max(uv,axis=0)+10]
    detection = {'label':'person', 'class_id':1, 'score':.9, 'box':box}
    return detection, replace(observed(xyz[:,:2]), obstacle_xyz=xyz), info, camera


def test_image_left_matches_body_left_using_calibrated_lidar_not_image_center():
    detection, scan, info, camera = association_scene()
    result = associate_objects([detection],scan,info,camera,np.ones((270,480),np.uint8),RoadObjectConfig())
    assert len(result)==1 and result[0]['y'] > 0 and result[0]['x']==pytest.approx(3.05)
    assert associate_objects([detection],scan,info,camera,np.full((270,480),2,np.uint8),RoadObjectConfig()) == []
    with pytest.raises(ValueError,match='depth_unavailable'):
        associate_objects([detection],replace(scan,obstacle_xyz=None),info,camera,np.ones((270,480),np.uint8),RoadObjectConfig())


@pytest.mark.parametrize('kw',[{'min_lateral_mps':.2},{'max_lateral_mps':.9},{'min_lateral_mps':.7,'max_lateral_mps':.6},{'max_range_m':float('nan')}])
def test_invalid_object_settings_rejected(kw):
    with pytest.raises(ValueError):
        replace(RoadObjectConfig(),**kw).validate()


def test_near_person_not_swallowed_by_dense_distant_background():
    detection, scan, info, camera = association_scene()
    xyz=np.vstack([scan.obstacle_xyz, np.tile(scan.obstacle_xyz*3,(100,1))])
    result=associate_objects([detection],replace(scan,obstacle_xyz=xyz),info,camera,np.ones((270,480),np.uint8),RoadObjectConfig())
    assert len(result)==1 and result[0]['x']==pytest.approx(3.05)
    # Measured objects beyond the configured trigger distance do not cause a sidestep.
    far=associate_objects([detection],replace(scan,obstacle_xyz=xyz[3:]),info,camera,np.ones((270,480),np.uint8),RoadObjectConfig())
    assert far==[]


def test_existing_handle_exclusion_does_not_hide_clear_left_objects():
    from local_path import DEFAULT_ROAD_ROI_EXCLUSION_POLYGON
    excluded=exclusion_mask((360,640),DEFAULT_ROAD_ROI_EXCLUSION_POLYGON)
    assert box_visible_fraction([480,10,630,290],excluded)<.5
    assert box_visible_fraction([100,100,200,350],excluded)==1


@pytest.mark.parametrize('side',[-1,1])
def test_moving_car_rectangle_is_not_entered_during_opposite_sidestep(side):
    ctrl=RoadObjectAvoidanceController(AvoidanceConfig(),C,RoadObjectConfig())
    robot=np.zeros(2);minimum=np.inf;peak=0.
    for tick in range(400):
        now=tick*.05
        center=np.array([8.-.35*now,side*1.2])-robot
        points=np.vstack([np.column_stack((np.linspace(-2.15,2.15,32),np.full(32,y))) for y in [-.9,.9]])+center
        points=np.vstack((points, np.column_stack((np.full(24,-2.15),np.linspace(-.9,.9,24)))+center))
        scan=replace(observed(points),arrival=now,stamp_ns=100_000_000_000+round(now*1e9))
        ctrl.set_objects([{'label':'car','x':float(center[0]),'y':float(center[1])}] if 0<center[0]<=6 else [],tick)
        region=np.abs(C.centers()[...,1]+robot[1])<=5.5
        command=ctrl.apply(NOMINAL,path(-robot[1]),scan,region,now=now,clock_ns=scan.stamp_ns)
        robot+=np.array([command.vx,command.vy])*.05
        dx=max(abs(center[0])-(2.15+C.front_m),0)
        dy=max(abs(center[1])-(.9+C.robot_half_width_m),0)
        minimum=min(minimum,np.hypot(dx,dy));peak=max(peak,abs(command.vy))
    assert minimum>C.margin_m
    assert .3<=peak<=.8
    assert abs(robot[1])<5.5-C.robot_half_width_m
