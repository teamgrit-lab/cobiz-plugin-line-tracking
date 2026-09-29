"""Task-gated inference using the real worker and ROS callback lifecycle."""

import json
import signal
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from test_apriltag_task_stop_ros import Message, RosHarness, SPORT, ZERO, debug


@pytest.fixture
def ros(monkeypatch):
    harness = RosHarness(monkeypatch)
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_" + check.upper(), "false")
    return harness


def image(ros, value=0):
    message = Message(bytes([value]) * 12)
    message.header.stamp = ros.stamp()
    ros.node.on_image(message)


def wait_for_count(ros, count):
    deadline = time.perf_counter() + 3.0
    while True:
        ros.node.publish_state()
        if ros.metrics()["inference_count"] == count:
            return
        assert time.perf_counter() < deadline, ros.errors
        time.sleep(0.001)


@pytest.mark.parametrize("terminal", ["server", "timeout", "apriltag"])
def test_idle_rejection_completion_and_restart_keep_one_loaded_model(ros, monkeypatch, terminal):
    counts = {"loads": 0, "resets": 0, "frames": 0, "decodes": 0}
    original_decode = debug.camera_image_rgb

    def decode(*args):
        counts["decodes"] += 1
        return original_decode(*args)

    def reset():
        counts["resets"] += 1

    def segment(_frame, **_kwargs):
        counts["frames"] += 1
        return SimpleNamespace(selected_mask=np.full((360, 640), 2, np.uint8), inference_seconds=0.01)

    def load(_config):
        counts["loads"] += 1
        return SimpleNamespace(device=SimpleNamespace(type="cuda"), reset=reset, segment=segment)

    monkeypatch.setattr(debug, "BestSoFarSegmenter", load)
    monkeypatch.setattr(debug, "camera_image_rgb", decode)
    if terminal == "timeout":
        monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_TASK_TIMEOUT", "true")
    if terminal == "apriltag":
        monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_APRILTAG", "true")

    def scenario(node):
        for _ in range(10):
            image(ros)
        node.on_image(Message(bytes(1)))  # Idle frames are not even validated.
        node.on_task_event(Message(json.dumps({
            "type": "TASK_REGISTERED", "action_name": "LINE_TRACKING",
            "task_id": "invalid", "payload": {"selected_mask": 9},
        })))
        assert ros.task_state()["type"] == "TASK_REJECTED"
        node.publish_state()
        assert ros.metrics()["inference_enabled"] is False
        assert ros.metrics()["inference_count"] == 0
        assert SPORT not in ros.published

        ros.start(duration=3)
        node.publish_state()
        assert ros.metrics()["inference_enabled"] is True
        assert ros.metrics()["path_tracked"] is False
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert counts == {"loads": 1, "resets": 0, "frames": 0, "decodes": 0}
        image(ros)
        wait_for_count(ros, 1)
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] > 0

        if terminal == "server":
            node.on_task_event(Message(json.dumps({"type": "TASK_ABORTED", "task_id": "tag-stop-1"})))
        elif terminal == "timeout":
            ros.now = 3.1
            node.publish_state()
        else:
            node.complete_apriltag_task(7)
        assert node.tasks.active is None
        for _ in range(10):
            image(ros)
        node.publish_state()
        assert ros.metrics()["inference_enabled"] is False
        assert ros.metrics()["path_tracked"] is False
        assert ros.metrics()["inference_count"] == 1
        assert counts == {"loads": 1, "resets": 1, "frames": 1, "decodes": 1}

        ros.now += 1.1
        node.publish_state()  # Finish the command-publisher release window.
        ros.start(task_id="second")
        node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert ros.metrics()["path_tracked"] is False
        assert ros.metrics()["performance"]["sample_count"] == 0
        ros.now += 0.1
        image(ros)
        wait_for_count(ros, 2)
        assert ros.metrics()["drive_reason"] == "tracking"
        assert counts == {"loads": 1, "resets": 2, "frames": 2, "decodes": 2}

    ros.run(scenario)


def test_cancelled_inflight_result_cannot_seed_the_next_task(ros, monkeypatch):
    entered = [threading.Event(), threading.Event()]
    release = [threading.Event(), threading.Event()]
    calls, resets = [], []
    in_forward = False

    def reset():
        assert not in_forward  # Never reset the model concurrently with forward.
        resets.append(len(calls))

    def segment(frame, **_kwargs):
        nonlocal in_forward
        index = len(calls)
        calls.append(int(frame[0, 0, 0]))
        in_forward = True
        entered[index].set()
        assert release[index].wait(timeout=3.0)
        in_forward = False
        return SimpleNamespace(selected_mask=np.full((360, 640), 2, np.uint8), inference_seconds=0.01)

    monkeypatch.setattr(debug, "BestSoFarSegmenter", lambda _config: SimpleNamespace(
        device=SimpleNamespace(type="cuda"), reset=reset, segment=segment,
    ))

    def scenario(node):
        try:
            ros.start()
            image(ros, 1)
            assert entered[0].wait(timeout=3.0)
            node.on_task_event(Message(json.dumps({"type": "TASK_ABORTED", "task_id": "tag-stop-1"})))
            image(ros, 99)  # Must never be decoded in the next task.
            ros.now = 1.1
            node.publish_state()
            ros.start(task_id="second")
            image(ros, 2)
            release[0].set()
            assert entered[1].wait(timeout=3.0)
            node.publish_state()
            assert calls == [1, 2]
            assert resets == [0, 1]
            assert ros.metrics()["inference_count"] == 0
            assert ros.metrics()["path_tracked"] is False
            assert ros.metrics()["path_unavailable_inferences"] == 0
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
            release[1].set()
            wait_for_count(ros, 1)
            assert ros.metrics()["drive_reason"] == "tracking"
        finally:
            for event in release:
                event.set()

    ros.run(scenario)


def test_ros2_debug_mode_still_infers_without_a_task(ros):
    def spin(node):
        assert node.tasks is None
        image(ros)
        wait_for_count(ros, 1)
        assert ros.metrics()["inference_enabled"] is True
        assert ros.metrics()["path_tracked"] is True
        assert SPORT not in ros.published

    ros.rclpy.spin = spin
    assert debug.run_ros2(debug.parse_args(["ros2"])) == 0
    assert not ros.errors


def test_sigterm_during_path_read_pauses_without_deadlock_or_resumed_motion(ros, monkeypatch):
    original = debug.LocalPathSmoother._current_unlocked

    def scenario(node):
        ros.start()
        image(ros)
        wait_for_count(ros, 1)
        stop_start = len(ros.events)
        interrupted = False

        def interrupt_path_read(smoother, timestamp):
            nonlocal interrupted
            if threading.current_thread() is threading.main_thread() and not interrupted:
                interrupted = True
                # current() holds both the path lock and the node state lock.
                signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
            return original(smoother, timestamp)

        monkeypatch.setattr(debug.LocalPathSmoother, "_current_unlocked", interrupt_path_read)
        node.publish_state()
        assert interrupted
        assert node.tasks.active is None
        assert ros.metrics()["inference_enabled"] is False
        ros.hard_stop_before_task_state(stop_start, "TASK_ABORTED", "sigterm")
        for topic, message in ros.events[stop_start:]:
            if topic == SPORT and message.header.identity.api_id == 1008:
                assert json.loads(message.parameter) == ZERO

    ros.run(scenario)
