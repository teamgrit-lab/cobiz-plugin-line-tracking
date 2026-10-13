"""Calibrated 3-D corroboration is independent of obstacle and IMU checks."""

from types import SimpleNamespace as NS

import numpy as np
import pytest

from lidar_obstacles import ObstacleConfig
from lidar_runtime import LidarRuntime
from local_avoidance import AvoidanceConfig
from sensor_fixtures import camera_info, cloud, header
from street_light_approach import StreetLightObservation
from swin_l_drive_control import DriveDecision


OBSERVATION = StreetLightObservation(.30, .40, .30, 100, .20)


def runtime():
    args = NS(path_frame_id="base_link", lidar_frame_id="livox_frame",
              lidar_to_base_transform="", base_to_camera_transform="",
              lidar_calibration_profile="livox-a2-front",
              street_light_lidar_max_range_m=15., street_light_lidar_min_points=3,
              street_light_max_age_sec=1.)
    sensor = LidarRuntime(args, ObstacleConfig(), AvoidanceConfig(), confirmation_only=True)
    info = camera_info(100.)
    info.d = []
    sensor.on_info(info)
    sensor.set_reference(np.ones((360, 640), np.uint8), header(100., "camera_optical_frame"),
                         (360, 640), 100.)
    return sensor


def points_in_box(sensor, *, u=.35, v=.25, count=3, depth=5.):
    info = sensor.infos[-1]
    k = np.asarray(info.k).reshape(3, 3)
    pixels = np.column_stack((u * info.width + np.arange(count) - count // 2,
                              np.full(count, v * info.height), np.ones(count)))
    camera = (pixels @ np.linalg.inv(k).T) * depth
    base = (camera - sensor.camera[:3, 3]) @ sensor.camera[:3, :3]
    return (base - sensor.base[:3, 3]) @ sensor.base[:3, :3]


def load(sensor, points, stamp=100.):
    sensor.on_cloud(cloud(points, stamp), 100., 100_000_000_000)


def test_actual_3d_returns_confirm_above_height_slice_and_beyond_obstacle_radius_without_imu():
    sensor = runtime()
    load(sensor, points_in_box(sensor))
    assert not sensor.imus
    assert np.all(sensor.scan.points[:, 2] > sensor.config.max_height_m)
    bearing, distance = sensor.locate_street_light(OBSERVATION, 100., 100_000_000_000)
    assert bearing > 0
    assert distance > sensor.config.horizontal_range_m
    assert sensor.metrics["lidar_confirmed"] is True
    assert sensor.metrics["lidar_match_count"] == 3
    assert sensor.metrics["stops_enabled"] is False


@pytest.mark.parametrize("count", [0, 1, 2])
def test_sparse_returns_do_not_confirm_an_object(count):
    sensor = runtime()
    load(sensor, points_in_box(sensor, count=count))
    assert sensor.locate_street_light(OBSERVATION, 100., 100_000_000_000)[1] is None
    assert sensor.metrics["lidar_confirmed"] is False


@pytest.mark.parametrize("u,v,depth", [(.35, .7, 5.), (.35, .1, 5.), (.7, .25, 5.), (.35, .25, -5.), (.35, .25, 20.)])
def test_different_vertical_location_side_behind_camera_or_outside_range_does_not_confirm(u, v, depth):
    sensor = runtime()
    load(sensor, points_in_box(sensor, u=u, v=v, depth=depth))
    assert sensor.locate_street_light(OBSERVATION, 100., 100_000_000_000)[1] is None


@pytest.mark.parametrize("fault", ["missing", "stale", "old_stamp", "malformed", "wrong_frame", "no_info", "camera_frame", "unsynchronized"])
def test_unavailable_confirmation_never_changes_a_drive_command(fault):
    sensor = runtime()
    load(sensor, points_in_box(sensor))
    now, clock = 100., 100_000_000_000
    if fault == "missing":
        sensor.reset_reference()
    elif fault == "stale":
        now, clock = 100.6, 100_600_000_000
    elif fault == "old_stamp":
        load(sensor, points_in_box(sensor), stamp=99.)
    elif fault == "malformed":
        packet = cloud([[1., 0., 0.]])
        packet.data = b""
        sensor.on_cloud(packet, now, clock)
    elif fault == "wrong_frame":
        sensor.on_cloud(cloud([[1., 0., 0.]], frame="wrong"), now, clock)
    elif fault == "no_info":
        sensor.infos.clear()
    elif fault == "camera_frame":
        sensor.locate_street_light(OBSERVATION, now, clock)
        sensor.args.lidar_calibration_profile = "tf"
        info = camera_info(100.)
        info.header.frame_id = "other_camera"
        sensor.on_info(info)
        sensor.set_reference(np.ones((360, 640), np.uint8), header(100., "other_camera"),
                             (360, 640), 100.)
    else:
        now, clock = 100.7, 100_700_000_000
        sensor.on_cloud(cloud(points_in_box(sensor), stamp=now), now, clock)
    with pytest.raises(ValueError):
        sensor.locate_street_light(OBSERVATION, now, clock)
    for command in (DriveDecision(.5, 0., .1, "tracking"),
                    DriveDecision(.5, .15, 0., "street_light_approaching"),
                    DriveDecision.stop("task_aborted_by_server")):
        assert sensor.apply(command, None, 1, now, clock) is command
        assert sensor.guard(command, 1, now, clock) is command


def test_reset_discards_previous_task_confirmation():
    sensor = runtime()
    load(sensor, points_in_box(sensor))
    assert sensor.locate_street_light(OBSERVATION, 100., 100_000_000_000)[1] is not None
    sensor.reset_reference()
    assert sensor.scan is None and sensor.reference is None
