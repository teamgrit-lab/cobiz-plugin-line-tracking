"""Behavioral simulation of the stop/wait/scan controller without ROS."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from swin_l_drive_control import DriveConfig, DriveDecision, PathLossRecovery


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
             confidence=.9, override=None, dt=.1):
        self.now += dt
        self.angle += self.command.yaw_rate * dt
        self.frame += int(new_frame)
        result = self.recovery.decide(
            normal, now=self.now, inference_id=self.frame,
            inference_source_at=self.now if source_at is None else source_at,
            sensors_ready=fresh, sensor_reason="inference_stale",
            path_confidence=confidence,
        )
        self.command = result if override is None else override
        self.recovery.record_command(self.command, now=self.now)
        return result


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
