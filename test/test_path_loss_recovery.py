"""Behavioral simulation of the stop/wait/scan controller without ROS."""

import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from swin_l_drive_control import DriveConfig, DriveDecision, PathLossRecovery
from local_path import SmoothedPath


TRACK = DriveDecision(.3, 0., .1, "tracking")
LOST = DriveDecision.stop("path_recovery_waiting")


class Simulation:
    def __init__(self, config=None):
        self.recovery = PathLossRecovery(config or DriveConfig(), output_hz=10.)
        self.now = 0.
        self.frame = 0
        self.command = TRACK
        self.angle = 0.
        self.recovery.record_command(TRACK, now=self.now)

    def tick(self, normal=LOST, *, new_frame=True, fresh=True, source_at=None,
             confidence=.9, override=None, dt=.1, path=None):
        self.now += dt
        self.angle += self.command.yaw_rate * dt
        self.frame += int(new_frame)
        result = self.recovery.decide(
            normal, now=self.now, inference_id=self.frame,
            inference_source_at=self.now if source_at is None else source_at,
            sensors_ready=fresh, sensor_reason="inference_stale",
            path_confidence=confidence, path=path,
        )
        self.command = result if override is None else override
        self.recovery.record_command(self.command, now=self.now)
        return result


def curved_path(heading_deg, *, offset=0.):
    x = np.linspace(1., 6., 20)
    # The robot can be on either side of a path that bends the other way.
    y = offset + np.tan(np.radians(heading_deg))*np.maximum(x-3., 0.)
    return SmoothedPath(np.column_stack((x, y)), .9, 0., "test")


def begin_scan(sim):
    for _ in range(20):
        command = sim.tick()
        if sim.recovery.phase.startswith("scan_"):
            return command
    pytest.fail("search did not start")


@pytest.mark.parametrize("direction,sign,fallback", [("left", 1, "right"), ("right", -1, "left")])
def test_last_path_bend_overrides_fallback_and_completes_both_sides(direction, sign, fallback):
    sim = Simulation(DriveConfig(path_recovery_first_direction=fallback))
    sim.tick(TRACK, path=curved_path(sign*25, offset=-sign*3))
    first = begin_scan(sim)
    assert first.reason == "path_recovery_scan_" + direction
    assert first.yaw_rate*sign > 0
    assert sim.recovery.direction_source == "last_path"
    scan_origin = sim.angle
    angles = []
    for _ in range(420):
        command = sim.tick()
        angles.append(sim.angle-scan_origin)
        assert command.vx == command.vy == 0
        if sim.recovery.phase == "exhausted":
            break
    else:
        pytest.fail("search did not finish")
    assert max(angles) == pytest.approx(math.pi/2, abs=1e-6)
    assert min(angles) == pytest.approx(-math.pi/2, abs=1e-6)
    assert angles[-1] == pytest.approx(0., abs=1e-6)
    assert (angles.index(max(angles)) < angles.index(min(angles))) == (direction == "left")
    assert sim.recovery.first_direction == direction


@pytest.mark.parametrize("heading", [0., 2., -2.])
def test_straight_or_small_bend_uses_configured_fallback_even_when_path_is_offset(heading):
    sim = Simulation(DriveConfig(path_recovery_first_direction="right"))
    sim.tick(TRACK, path=curved_path(heading, offset=2.))
    assert begin_scan(sim).reason == "path_recovery_scan_right"
    assert sim.recovery.direction_source == "configured"


def test_latest_straight_path_clears_previous_bend():
    sim = Simulation(DriveConfig(path_recovery_first_direction="left"))
    sim.tick(TRACK, path=curved_path(-25))
    sim.tick(TRACK, path=curved_path(0))
    assert begin_scan(sim).reason == "path_recovery_scan_left"
    assert sim.recovery.direction_source == "configured"


@pytest.mark.parametrize("issue", ["confidence", "stale_sensor", "stale_capture", "future_capture",
                                   "guard", "held_command", "same_inference", "path_age",
                                   "same_capture", "older_capture", "older_inference"])
def test_unreliable_or_repeated_paths_cannot_replace_last_bend(issue):
    sim = Simulation(DriveConfig(path_recovery_first_direction="left"))
    sim.tick(TRACK, path=curved_path(-25))
    arguments = {"path": curved_path(25)}
    normal = TRACK
    if issue == "confidence":
        arguments["path"] = replace(arguments["path"], confidence=.48)
    elif issue == "stale_sensor":
        arguments["fresh"] = False
    elif issue == "stale_capture":
        arguments["source_at"] = -10.
    elif issue == "future_capture":
        arguments["source_at"] = 10.
    elif issue == "guard":
        normal = DriveDecision.stop("branch_path_lost")
    elif issue == "held_command":
        normal = replace(TRACK, reason="tracking_path_hold")
    elif issue == "same_inference":
        arguments["new_frame"] = False
    elif issue == "path_age":
        arguments["path"] = replace(arguments["path"], age_sec=10.)
    elif issue == "same_capture":
        arguments["source_at"] = sim.now
    elif issue == "older_capture":
        arguments["source_at"] = sim.now-.01
    elif issue == "older_inference":
        sim.frame -= 2
    sim.tick(normal, **arguments)
    assert begin_scan(sim).reason == "path_recovery_scan_right"
    assert sim.recovery.direction_source == "last_path"


def test_old_path_hint_expires_before_new_search():
    sim = Simulation()
    sim.tick(TRACK, path=curved_path(-25))
    sim.tick(dt=5.1)
    assert begin_scan(sim).reason == "path_recovery_scan_left"
    assert sim.recovery.direction_source == "configured"


@pytest.mark.parametrize("points", [[], [[1., 0.]], [[1., 0.], [1., -2.]],
                                    [[1., 0.], [1.1, -2.]], [[-2., 0.], [-1., -2.]],
                                    [[1., float("nan")], [2., float("inf")]]])
def test_insufficient_geometry_uses_fallback(points):
    sim = Simulation()
    path = replace(curved_path(-25), points_xy=np.asarray(points))
    sim.tick(TRACK, path=path)
    assert begin_scan(sim).reason == "path_recovery_scan_left"


def test_confirmed_path_selects_next_attempt_and_task_reset_clears_hint():
    sim = Simulation()
    sim.tick(TRACK, path=curved_path(-25))
    assert begin_scan(sim).reason == "path_recovery_scan_right"
    assert sim.tick(TRACK, path=curved_path(25)).reason == "path_recovery_confirming"
    assert sim.recovery.first_direction == "right"
    assert sim.tick(TRACK, path=curved_path(25)).reason == "tracking"
    assert begin_scan(sim).reason == "path_recovery_scan_left"
    assert sim.recovery.direction_source == "last_path"
    sim.recovery.reset()
    assert sim.recovery.last_path_heading_rad is None
    assert sim.recovery.last_path_source_at is None
    assert sim.recovery.first_direction == "left"
    assert sim.tick().yaw_rate == 0
    sim.tick(TRACK)
    assert begin_scan(sim).reason == "path_recovery_scan_left"
    assert sim.recovery.direction_source == "configured"


@pytest.mark.parametrize("first_direction", ["left", "right"])
def test_wait_then_scan_both_sides_and_return_without_forward_motion(first_direction):
    sim = Simulation(DriveConfig(path_recovery_first_direction=first_direction))
    sim.command = DriveDecision.stop("lost")
    commands = []
    angles = []
    for _ in range(450):
        decision = sim.tick()
        commands.append(decision)
        angles.append(sim.angle)
        assert decision.vx == decision.vy == 0
        assert abs(decision.yaw_rate) <= .18
        if sim.recovery.phase == "exhausted":
            break
    else:
        pytest.fail("search did not finish")
    assert all(command.reason == "path_recovery_waiting" for command in commands[:10])
    assert commands[10].reason == "path_recovery_scan_" + first_direction
    assert max(angles) == pytest.approx(math.pi/2, abs=1e-6)
    assert min(angles) == pytest.approx(-math.pi/2, abs=1e-6)
    assert angles[-1] == pytest.approx(0., abs=1e-6)
    left_index = angles.index(max(angles))
    right_index = angles.index(min(angles))
    assert (left_index < right_index) == (first_direction == "left")
    for _ in range(100):
        assert sim.tick().reason == "path_recovery_exhausted"


@pytest.mark.parametrize("first_direction", ["left", "right"])
def test_endpoint_waits_for_image_captured_after_turn_before_reversing(first_direction):
    sim = Simulation(DriveConfig(path_recovery_first_direction=first_direction))
    for _ in range(120):
        sim.tick()
        if sim.recovery.phase == "hold_" + first_direction:
            break
    sign = 1 if first_direction == "left" else -1
    assert sim.recovery.angle_rad == pytest.approx(sign * math.pi/2)
    old_source = sim.now - .1
    assert sim.tick(source_at=old_source).reason == "path_recovery_hold_" + first_direction
    second = "right" if first_direction == "left" else "left"
    assert sim.tick().reason == "path_recovery_scan_" + second


def test_resume_requires_distinct_confident_results_captured_after_stopping():
    sim = Simulation()
    sim.tick()
    assert sim.tick(TRACK).reason == "path_recovery_confirming"
    confirmation_started = sim.now
    for _ in range(10):
        assert sim.tick(TRACK, new_frame=False).vx == 0
    # An in-flight result captured before the confirmation stop cannot resume.
    assert sim.tick(TRACK, source_at=confirmation_started-.01).vx == 0
    assert sim.tick(TRACK, confidence=.48).vx == 0
    assert sim.recovery.confirmed_frames == 0
    assert sim.tick(TRACK).reason == "path_recovery_confirming"
    assert sim.tick(TRACK).reason == "tracking"
    assert not sim.recovery.active
    # A subsequent loss receives its own complete waiting window.
    assert sim.tick().reason == "path_recovery_waiting"


@pytest.mark.parametrize("guard", ["branch_path_lost", "lidar_path_unavailable",
                                   "path_unavailable", "camera_conversion_error"])
def test_guards_and_sensor_failures_stop_and_do_not_advance_scan(guard):
    sim = Simulation()
    for _ in range(30):
        sim.tick()
    assert sim.command.reason == "path_recovery_scan_left"
    assert sim.tick(DriveDecision.stop(guard)).reason == guard
    paused_angle = sim.recovery.angle_rad
    for _ in range(10):
        assert sim.tick(fresh=False).reason == "inference_stale"
    assert sim.recovery.angle_rad == paused_angle
    assert sim.tick().reason == "path_recovery_scan_left"


def test_apriltag_override_does_not_consume_rotation_budget():
    sim = Simulation()
    for _ in range(30):
        sim.tick()
    zero = DriveDecision.stop("apriltag_verifying")
    sim.tick(override=zero)
    paused_angle = sim.recovery.angle_rad
    for _ in range(10):
        sim.tick(override=zero)
    assert sim.recovery.angle_rad == paused_angle
    assert sim.tick().reason == "path_recovery_scan_left"


def test_control_stall_ends_attempt_and_cannot_restart_without_confirmed_path():
    sim = Simulation()
    for _ in range(30):
        sim.tick()
    assert sim.tick(dt=1.).reason == "path_recovery_exhausted"
    assert sim.tick().reason == "path_recovery_exhausted"
    assert sim.tick(TRACK).reason == "path_recovery_confirming"
    assert sim.tick(TRACK).reason == "tracking"


def test_skipped_inference_cannot_establish_consecutive_path_confirmation():
    sim = Simulation()
    sim.tick()
    assert sim.tick(TRACK).reason == "path_recovery_confirming"
    # The control loop did not observe the intervening inference result.
    sim.frame += 1
    assert sim.tick(TRACK).reason == "path_recovery_confirming"
    assert sim.recovery.confirmed_frames == 1
    assert sim.tick(TRACK).reason == "tracking"


def test_repeated_capture_timestamp_cannot_complete_confirmation():
    sim = Simulation(DriveConfig(path_recovery_confirm_frames=3))
    sim.tick()
    sim.tick(TRACK)
    assert sim.tick(TRACK).reason == "path_recovery_confirming"
    captured_at = sim.now
    assert sim.tick(TRACK, source_at=captured_at).reason == "path_recovery_confirming"
    assert sim.recovery.confirmed_frames == 2
    assert sim.tick(TRACK).reason == "tracking"


def test_initial_missing_path_and_task_reset_never_start_scanning():
    sim = Simulation()
    sim.recovery.reset()
    for _ in range(50):
        assert sim.tick(DriveDecision.stop("waiting_for_path")).yaw_rate == 0
    assert not sim.recovery.active


@pytest.mark.parametrize("changes", [
    {"path_recovery_wait_sec": 0}, {"path_recovery_wait_sec": float("nan")},
    {"path_recovery_yaw_rps": .19}, {"path_recovery_yaw_rps": float("inf")},
    {"path_recovery_confirm_frames": 1}, {"path_recovery_confirm_frames": True},
    {"path_recovery_first_direction": "up"}, {"path_recovery_first_direction": ""},
    {"path_recovery_first_direction": None}, {"path_recovery_first_direction": True},
])
def test_invalid_search_settings_are_rejected(changes):
    with pytest.raises(ValueError):
        DriveConfig(**changes).validate()
