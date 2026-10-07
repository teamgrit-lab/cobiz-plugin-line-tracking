import sys
from pathlib import Path
from dataclasses import replace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from lidar_obstacles import ObstacleConfig, ObstacleScan
from local_avoidance import AvoidanceConfig, AvoidanceController
from local_path import SmoothedPath
from swin_l_drive_control import DriveDecision


C = ObstacleConfig()
NOMINAL = DriveDecision(0.2, 0.0, 0.0, "tracking")


def scene(points=()):
    p = np.asarray(points, float).reshape(-1, 2)
    return ObstacleScan(
        p,
        np.ones(C.shape, bool),
        np.zeros(C.shape),
        100_000_000_000,
        0.0,
        len(p),
        np.array([0.0, 0.0, 1.0]),
        np.zeros(3),
    )


def path(y=0):
    return SmoothedPath(np.array([[0.1, y], [3.0, y], [6.0, y]]), 1.0, 0.0, "test")


@pytest.mark.parametrize("preference,sign", [("right", -1), ("left", 1)])
def test_head_on_symmetry_is_broken_by_configured_side(preference, sign):
    ctrl = AvoidanceController(AvoidanceConfig(preference=preference), C)
    command = ctrl.apply(
        NOMINAL,
        path(),
        scene([[2.0, 0.0]]),
        np.ones(C.shape, bool),
        now=0.0,
        clock_ns=100_000_000_000,
    )
    assert command.vx > 0 and command.vy * sign > 0
    assert abs(command.vy) <= ctrl.config.max_accel_mps2 / ctrl.config.control_hz + 1e-8


def test_side_preference_yields_to_observed_corridor_and_locks_safe_choice():
    ctrl = AvoidanceController(AvoidanceConfig(preference="right"), C)
    region = C.centers()[..., 1] >= -0.45
    command = ctrl.apply(
        NOMINAL, path(), scene([[2.0, 0.0]]), region, now=0.0, clock_ns=100_000_000_000
    )
    assert command.vy > 0
    assert ctrl.side == 1


@pytest.mark.parametrize("fault", ["stale", "empty_region", "unknown", "collision"])
def test_unsafe_conditions_cannot_emit_a_positive_command(fault):
    ctrl = AvoidanceController(AvoidanceConfig(), C)
    scan = scene([[0.7, 0.0]]) if fault == "collision" else scene()
    region = (
        np.zeros(C.shape, bool) if fault == "empty_region" else np.ones(C.shape, bool)
    )
    if fault == "unknown":
        scan = replace(scan, observed=np.zeros(C.shape, bool))
    now = 0.4 if fault == "stale" else 0.0
    result = ctrl.apply(
        NOMINAL,
        path(),
        scan,
        region,
        now=now,
        clock_ns=100_000_000_000 + round(now * 1e9),
    )
    assert result.vx == result.vy == result.yaw_rate == 0


def test_lateral_return_tracks_original_path_after_obstacle_clears():
    ctrl = AvoidanceController(AvoidanceConfig(), C)
    result = ctrl.apply(
        NOMINAL,
        path(0.2),
        scene(),
        np.ones(C.shape, bool),
        now=0.0,
        clock_ns=100_000_000_000,
    )
    assert result.vy > 0 and ctrl.metrics["state"] == "returning"


@pytest.mark.parametrize("object_speed", [0.5, 1.0])
def test_oncoming_object_is_passed_with_clearance_then_original_path_is_rejoined(
    object_speed,
):
    ctrl = AvoidanceController(AvoidanceConfig(), C)
    robot = np.zeros(2)
    peak = 0.0
    minimum_clearance = np.inf
    scan = None
    for tick in range(360):
        now = tick * 0.05
        point = np.array([6 - object_speed * now, 0.0]) - robot
        if tick % 2 == 0:
            scan = scene([point]) if point[0] >= -1 else scene()
            scan = replace(
                scan, arrival=now, stamp_ns=100_000_000_000 + round(now * 1e9)
            )
        command = ctrl.apply(
            NOMINAL,
            path(-robot[1]),
            scan,
            np.ones(C.shape, bool),
            now=now,
            clock_ns=100_000_000_000 + round(now * 1e9),
        )
        robot += np.array([command.vx, command.vy]) * 0.05
        clearance = np.hypot(
            max(point[0] - C.front_m, -C.back_m - point[0], 0),
            max(abs(point[1]) - C.robot_half_width_m, 0),
        )
        minimum_clearance = min(minimum_clearance, clearance)
        peak = max(peak, abs(robot[1]))
    assert minimum_clearance > C.margin_m + C.resolution_m / 2
    assert 0.4 < peak < 1.2
    assert robot[0] > 2 and abs(robot[1]) < 0.15


def test_swept_footprint_blocks_turning_corner_even_if_centerline_is_clear():
    ctrl = AvoidanceController(AvoidanceConfig(), C)
    scan = scene([[0.6, 0.5]])
    ok, _, reason = ctrl.trajectory_check(0, 0, 0.18, scan, np.ones(C.shape, bool))
    assert not ok and reason == "obstacle_collision"


def test_default_front_sector_allows_straight_motion_but_not_unobserved_rear_sweep():
    from lidar_obstacles import ObstacleDetector
    from sensor_fixtures import cloud, imu

    angles = np.linspace(-np.pi, np.pi, 1440, endpoint=False)
    returns = np.column_stack((10 * np.cos(angles), 10 * np.sin(angles), np.zeros(len(angles))))
    scan = ObstacleDetector(C).update(cloud(returns), imu(), np.eye(4), 0.0)
    ctrl = AvoidanceController(AvoidanceConfig(), C)
    region = np.ones(C.shape, bool)
    assert ctrl.trajectory_check(.2, 0, 0, scan, region)[0]
    ok, _, reason = ctrl.trajectory_check(.2, .15, 0, scan, region)
    assert not ok and reason == "avoidance_space_unobserved"
    command = ctrl.apply(NOMINAL, path(), scan, region, now=0.0, clock_ns=100_000_000_000)
    assert command.vx > 0 and command.vy == 0


def test_branch_stop_and_path_loss_cannot_be_bypassed_by_repulsion():
    ctrl = AvoidanceController(AvoidanceConfig(), C)
    for reference, nominal in [
        (None, DriveDecision(0.2, 0, 0, "tracking_path_recovery")),
        (
            replace(path(), stop_reason="branch_path_lost"),
            DriveDecision.stop("branch_path_lost"),
        ),
    ]:
        result = ctrl.apply(
            nominal,
            reference,
            scene(),
            np.ones(C.shape, bool),
            now=0,
            clock_ns=100_000_000_000,
        )
        assert result.vx == result.vy == 0


@pytest.mark.parametrize(
    "values",
    [{"preference": "center"}, {"control_hz": 0.0}, {"max_lateral_mps": float("inf")}],
)
def test_invalid_avoidance_settings_rejected(values):
    with pytest.raises(ValueError):
        replace(AvoidanceConfig(), **values).validate()
