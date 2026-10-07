"""Live-node stop/wait/scan recovery with mocked ROS and inference boundaries."""

import json
import time
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_apriltag_task_stop_ros import Message, RosHarness, SPORT, ZERO, debug


def run_recovery(monkeypatch, scenario, *, extra_args=()):
    ros = RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_PATH_LOSS_RECOVERY_ENABLED", "true")
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_AVOIDANCE_ENABLED", "false")
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_" + check.upper(), "false")
    # Exercise the short task safety timeout while running the longer search.
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_UNSAFE_TIMEOUT", "true")
    good = np.zeros((360, 640), np.uint8)
    good[:, :320] = 2
    detection = NS(mask=good.copy())
    monkeypatch.setattr(debug, "BestSoFarSegmenter", lambda _: NS(
        device=NS(type="cuda"), reset=lambda: None,
        segment=lambda _frame, **kwargs: NS(
            selected_mask=detection.mask.copy(), inference_seconds=.01)))

    def exercise(node):
        node.drive_config = replace(node.drive_config, heading_gain=2.)
        ros.start(duration=200)

        def feed(*, visible=False, dt=.1):
            detection.mask = good.copy() if visible else np.zeros_like(good)
            ros.now += dt
            previous = ros.metrics()["inference_count"] if ros.published.get(
                debug.DEFAULT_METRICS_TOPIC) else 0
            image = Message()
            image.header.stamp = ros.stamp()
            node.on_image(image)
            deadline = time.perf_counter()+5
            while True:
                node.publish_state()
                if ros.metrics()["inference_count"] > previous:
                    return ros.metrics()
                assert time.perf_counter() < deadline, ros.errors
                time.sleep(.001)

        scenario(ros, node, feed)

    ros.run(exercise, "--near-distance-m", ".6", "--far-distance-m", "3",
            "--search-half-width-m", "2",
            "--bev-height-px", "49", "--bev-width-px", "81",
            "--branch-preference", "center", *extra_args)


@pytest.mark.parametrize("recover_phase", ["waiting", "scan_left", "scan_right"])
@pytest.mark.parametrize("first_direction", ["left", "right"])
def test_loss_retains_motion_then_stops_waits_scans_and_confirms_path(
        monkeypatch, recover_phase, first_direction):
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_PATH_RECOVERY_FIRST_DIRECTION", first_direction)
    def scenario(ros, node, feed):
        assert feed()["drive_reason"] == "waiting_for_path"
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert feed(visible=True)["path_tracked"]
        saved = json.loads(ros.published[SPORT][-1].parameter)
        assert saved["x"] > 0

        for count in (1, 2):
            result = feed()
            assert result["drive_reason"] == "tracking_path_recovery"
            assert result["path_recovery"]["failed_inferences"] == count
            assert not result["path_recovery"]["active"]
            assert json.loads(ros.published[SPORT][-1].parameter) == {
                "x": saved["x"], "y": 0., "z": 0.,
            }

        result = feed()
        assert result["drive_reason"] == "path_recovery_waiting"
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert result["path_recovery"]["failed_inferences"] == 3
        assert result["path_recovery"]["first_direction"] == first_direction
        # Output ticks cannot consume inference confirmations or increase losses.
        for _ in range(10):
            node.publish_state()
        assert ros.metrics()["path_recovery"]["failed_inferences"] == 3
        for _ in range(350):
            if result["path_recovery"]["phase"] == recover_phase:
                break
            result = feed()
            command = json.loads(ros.published[SPORT][-1].parameter)
            assert command["x"] == command["y"] == 0
            assert abs(command["z"]) <= .18
            assert node.tasks.active is not None
        else:
            pytest.fail("search never reached " + recover_phase)
        if recover_phase == "scan_left":
            assert command["z"] > 0
        elif recover_phase == "scan_right":
            assert command["z"] < 0
        assert feed(visible=True)["drive_reason"] == "path_recovery_confirming"
        for _ in range(10):
            node.publish_state()
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert feed(visible=True)["path_tracked"]
        assert not ros.metrics()["path_recovery"]["active"]
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] > 0

        assert feed()["drive_reason"] == "tracking_path_recovery"
        assert feed()["drive_reason"] == "tracking_path_recovery"
        assert feed()["drive_reason"] == "path_recovery_waiting"
        node.on_task_event(Message(json.dumps({"type": "TASK_ABORTED", "task_id": "tag-stop-1"})))
        ros.now += 1.1
        node.publish_state()
        ros.start(task_id="new-window")
        assert feed()["drive_reason"] == "waiting_for_path"
        assert not ros.metrics()["path_recovery"]["active"]
        assert node.last_valid_forward_mps is None

    run_recovery(monkeypatch, scenario)


@pytest.mark.parametrize("first_direction", ["left", "right"])
def test_unsuccessful_search_returns_to_center_and_remains_stopped(monkeypatch, first_direction):
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_PATH_RECOVERY_FIRST_DIRECTION", first_direction)
    def scenario(ros, node, feed):
        feed(visible=True)
        feed()
        feed()
        result = feed()
        angles = []
        for _ in range(420):
            angles.append(result["path_recovery"]["estimated_angle_deg"])
            if result["path_recovery"]["exhausted"]:
                break
            result = feed()
            assert json.loads(ros.published[SPORT][-1].parameter)["x"] == 0
        else:
            pytest.fail("search did not end")
        assert max(angles) == pytest.approx(90., abs=.01)
        assert min(angles) == pytest.approx(-90., abs=.01)
        assert angles[-1] == pytest.approx(0., abs=.01)
        assert (angles.index(max(angles)) < angles.index(min(angles))) == (first_direction == "left")
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert feed()["drive_reason"] == "path_recovery_exhausted"
        # Finding a path after exhaustion can still resume while the task is active.
        assert feed(visible=True)["drive_reason"] == "path_recovery_confirming"
        assert feed(visible=True)["path_tracked"]

    run_recovery(monkeypatch, scenario)


@pytest.mark.parametrize("recover_at", [2, 3])
def test_path_found_before_loss_stop_resumes_tracking_without_wait_or_scan(
        monkeypatch, recover_at):
    def scenario(ros, node, feed):
        feed(visible=True)
        for _ in range(recover_at-1):
            assert feed()["drive_reason"] == "tracking_path_recovery"
        result = feed(visible=True)
        assert result["drive_reason"] in ("tracking", "tracking_slow_turn")
        assert result["path_tracked"]
        assert result["path_recovery"]["failed_inferences"] == 0
        assert not result["path_recovery"]["active"]
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] > 0

    run_recovery(monkeypatch, scenario)


def test_rotation_wait_is_one_second_from_actual_stop_not_initial_loss(monkeypatch):
    def scenario(ros, node, feed):
        feed(visible=True)
        assert feed()["drive_reason"] == "tracking_path_recovery"
        # More than a second passes before the third failed inference. The scan
        # must still wait a full second after the resulting zero command.
        assert feed(dt=.7)["drive_reason"] == "tracking_path_recovery"
        result = feed(dt=.7)
        assert result["drive_reason"] == "path_recovery_waiting"
        stopped_at = ros.now
        for _ in range(9):
            assert feed()["drive_reason"] == "path_recovery_waiting"
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert feed()["drive_reason"] == "path_recovery_scan_left"
        assert ros.now-stopped_at == pytest.approx(1.)
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] == 0

    run_recovery(monkeypatch, scenario)


def test_failed_confirmation_during_search_does_not_restore_initial_forward_recovery(monkeypatch):
    def scenario(ros, node, feed):
        feed(visible=True)
        feed()
        feed()
        assert feed()["drive_reason"] == "path_recovery_waiting"
        assert feed(visible=True)["drive_reason"] == "path_recovery_confirming"
        # This missing path starts a fresh inference failure streak, but the
        # ongoing search must retain zero forward speed until confirmation.
        result = feed()
        assert result["path_recovery"]["failed_inferences"] == 1
        assert result["drive_reason"] == "path_recovery_waiting"
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    run_recovery(monkeypatch, scenario)


def test_search_stops_on_stale_inference_even_when_optional_stops_are_disabled(monkeypatch):
    def scenario(ros, node, feed):
        feed(visible=True)
        for _ in range(25):
            feed()
        # Continue publishing regularly to isolate source staleness from a timer stall.
        for _ in range(60):
            ros.now += .1
            node.publish_state()
        assert ros.metrics()["drive_reason"] in ("camera_stale", "inference_stale")
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert feed()["drive_reason"] == "path_recovery_scan_left"

    run_recovery(monkeypatch, scenario)
