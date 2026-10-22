import json
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from test_apriltag_task_stop_ros import Message, RosHarness, SPORT, debug


def send(ros, command="START", *, mask=2, segment="1", session="owner"):
    ros.subscriptions["/line_tracking/segment_command"](Message(json.dumps({
        "protocol": 1, "session_id": session, "segment_id": segment,
        "command": command, "selected_mask": mask, "lease_sec": 1.,
        "stamp_sec": ros.clock_ns() / 1e9,
    })))


def status(ros):
    return json.loads(ros.published["/line_tracking/segment_status"][-1].data)


def velocity(ros):
    return json.loads(ros.published["/line_tracking/command"][-1].data)


def run(ros, scenario):
    ros.rclpy.spin = scenario
    assert debug.run_ros2(debug.parse_args([
        "navigation-worker", "--inference-hz", "1000"])) == 0
    assert not ros.errors


def legacy(ros, kind="TASK_REGISTERED", task="legacy", mask=2):
    ros.subscriptions["/task_event"](Message(json.dumps({
        "type": kind, "action_name": "LINE_TRACKING", "task_id": task,
        "payload": {"selected_mask": mask, "duration_sec": 30},
    })))


def grant_legacy(ros, state="GRANTED"):
    request = json.loads(ros.published["/line_tracking/task_request"][-1].data)
    response = {k: request[k] for k in ("protocol", "session_id", "task_id")}
    response.update(state=state, stamp_sec=ros.clock_ns()/1e9, lease_sec=1.)
    ros.subscriptions["/line_tracking/task_control"](Message(json.dumps(response)))
    return response


@pytest.mark.parametrize("mask", [0, 1, 2])
def test_shared_worker_runs_legacy_tasks_then_navigation_with_one_model(monkeypatch, mask):
    ros = RosHarness(monkeypatch)
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_" + check.upper(), "false")
    loads = []
    def load(_config):
        loads.append(True)
        return SimpleNamespace(device=SimpleNamespace(type="cuda"), reset=lambda: None,
            segment=lambda _frame, **kwargs: SimpleNamespace(selected_mask=np.full(
                (360, 640), kwargs["path_mask_class"] or 2, np.uint8), inference_seconds=.01))
    monkeypatch.setattr(debug, "BestSoFarSegmenter", load)

    def scenario(node):
        legacy(ros, mask=mask)
        assert SPORT not in ros.published and not ros.published["/task_state"]
        node.publish_state()
        assert not ros.metrics()["inference_enabled"]
        grant_legacy(ros)
        assert ros.task_state()["type"] == "TASK_STARTED"
        assert node.tasks.active.selected_mask == mask
        ros.camera_ready()
        node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] > 0
        assert not node.drive_config.stop_on_path_unavailable  # legacy settings restored
        send(ros, session="conflict")
        assert status(ros)["state"] == "REJECTED"
        assert node.tasks.active.key == "legacy"
        legacy(ros, "TASK_ABORTED")
        assert ros.task_state()["type"] == "TASK_ABORTED"
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] == 0
        assert node.standalone.context is None
        ros.now = 1.1
        node.publish_state()
        sport_count = len(ros.published[SPORT])
        send(ros, session="next-navigation")
        ros.camera_ready()
        node.publish_state()
        assert velocity(ros)["vx"] > 0
        assert node.drive_config.stop_on_path_unavailable
        assert len(ros.published[SPORT]) == sport_count
        assert len(loads) == 1
    run(ros, scenario)


@pytest.mark.parametrize("failure", ["timeout", "denied", "lost_lease"])
def test_standalone_requires_live_control_and_stops_on_loss(monkeypatch, failure):
    ros = RosHarness(monkeypatch)
    def scenario(node):
        legacy(ros)
        if failure == "denied":
            grant_legacy(ros, "REJECTED")
        elif failure == "timeout":
            ros.now = 5.1
            node.publish_state()
        else:
            grant_legacy(ros)
            ros.now = 1.01
            node.publish_state()
            assert json.loads(ros.published[SPORT][-1].parameter)["x"] == 0
        assert ros.task_state()["type"] == ("TASK_ABORTED" if failure == "lost_lease" else "TASK_REJECTED")
        assert node.tasks.active is None and node.standalone.context is None
        node.publish_state()
        assert not ros.metrics()["inference_enabled"]
        if failure != "lost_lease":
            assert SPORT not in ros.published
    run(ros, scenario)


def test_legacy_task_cannot_replace_an_active_navigation_segment(monkeypatch):
    ros = RosHarness(monkeypatch)
    def scenario(node):
        send(ros)
        identity = dict(node.navigation.active)
        legacy(ros)
        assert ros.task_state()["type"] == "TASK_REJECTED"
        assert node.navigation.active == identity
        assert node.standalone.context is None
        assert SPORT not in ros.published
        send(ros, "STOP")
    run(ros, scenario)


@pytest.mark.parametrize("terminal", ["duration", "tag"])
def test_shared_mode_retains_legacy_completion_and_releases_control(monkeypatch, terminal):
    ros = RosHarness(monkeypatch)
    def scenario(node):
        legacy(ros)
        grant_legacy(ros)
        if terminal == "tag":
            node.complete_apriltag_task(7)
        else:
            ros.now = .3
            grant_legacy(ros)  # keep ownership alive for this lifecycle boundary
            node.tasks.tracking_seen = True
            node.tasks.active = debug.replace(node.tasks.active, started_at=-30.)
            monkeypatch.setattr(node, "drive_readiness", lambda *_args:
                (None, debug.DriveDecision(vx=.2, vy=0., yaw_rate=0., reason="tracking")))
            node.publish_state()
        assert ros.task_state()["type"] == "TASK_COMPLETED"
        assert node.standalone.context is None
        assert json.loads(ros.published["/line_tracking/task_request"][-1].data)["command"] == "RELEASE"
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] == 0
    run(ros, scenario)


@pytest.mark.parametrize("mask", [1, 2])
def test_worker_reuses_real_inference_control_without_task_or_sport_ownership(monkeypatch, mask):
    ros = RosHarness(monkeypatch)
    counts = {"load": 0, "reset": 0}

    def load(_config):
        counts["load"] += 1
        def reset():
            counts["reset"] += 1
        return SimpleNamespace(device=SimpleNamespace(type="cuda"), reset=reset,
            segment=lambda _frame, **kwargs: SimpleNamespace(
                selected_mask=np.full((360, 640), kwargs["path_mask_class"], np.uint8), inference_seconds=.01))
    monkeypatch.setattr(debug, "BestSoFarSegmenter", load)

    def scenario(node):
        assert "/task_event" in ros.subscriptions
        assert not ros.published["/task_state"]
        assert SPORT not in ros.published
        node.on_image(Message())
        send(ros, mask=mask)
        assert status(ros)["state"] == "ACCEPTED"
        assert node.tasks.active.selected_mask == mask
        assert velocity(ros)["vx"] == 0
        ros.camera_ready()
        node.publish_state()
        assert status(ros)["state"] == "READY"
        assert velocity(ros)["vx"] > 0
        send(ros, "STOP", mask=mask)
        assert status(ros)["state"] == "STOPPED"
        assert velocity(ros)["vx"] == 0
        assert node.tasks.active is None
        assert not ros.node.navigation.active
        send(ros, mask=mask)  # delayed START cannot revive the completed segment
        assert status(ros)["state"] == "STOPPED"
        send(ros, mask=3-mask, segment="2")
        assert node.tasks.active.selected_mask == 3-mask
        assert velocity(ros)["vx"] == 0
        assert counts["load"] == 1
        assert SPORT not in ros.published and not ros.published["/task_state"]
    run(ros, scenario)


def test_worker_lease_expiry_stops_inference_and_does_not_run_recovery(monkeypatch):
    ros = RosHarness(monkeypatch)
    def scenario(node):
        send(ros)
        ros.camera_ready()
        node.publish_state()
        assert velocity(ros)["vx"] > 0
        ros.now = 1.01
        node.publish_state()
        assert node.tasks.active is None
        assert status(ros)["state"] == "ERROR"
        assert status(ros)["reason"] == "navigation_lease_expired"
        assert velocity(ros)["vx"] == velocity(ros)["yaw_rate"] == 0
        assert not node.drive_config.path_loss_recovery_enabled
        assert node.drive_config.stop_on_path_unavailable
    run(ros, scenario)


def test_stopped_inflight_inference_cannot_seed_next_surface(monkeypatch):
    ros = RosHarness(monkeypatch)
    entered, release = threading.Event(), threading.Event()
    def segment(_frame, **_kwargs):
        entered.set()
        assert release.wait(3)
        return SimpleNamespace(selected_mask=np.full((360, 640), 2, np.uint8), inference_seconds=.01)
    monkeypatch.setattr(debug, "BestSoFarSegmenter", lambda _config:
        SimpleNamespace(device=SimpleNamespace(type="cuda"), reset=lambda: None, segment=segment))
    def scenario(node):
        try:
            send(ros)
            frame = Message()
            frame.header.stamp = ros.stamp()
            node.on_image(frame)
            assert entered.wait(3)
            send(ros, "STOP")
            send(ros, mask=1, segment="2")
            release.set()
            deadline = time.perf_counter() + .1
            while time.perf_counter() < deadline:
                node.publish_state()
                time.sleep(.001)
            assert node.tasks.active.selected_mask == 1
            assert velocity(ros)["vx"] == 0
            assert ros.metrics()["inference_count"] == 0
        finally:
            release.set()
    run(ros, scenario)
