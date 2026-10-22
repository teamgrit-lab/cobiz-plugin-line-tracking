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
        assert "/task_event" not in ros.subscriptions
        assert "/task_state" not in ros.published
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
        assert SPORT not in ros.published and "/task_state" not in ros.published
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
