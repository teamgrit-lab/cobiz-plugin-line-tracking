"""Standalone tasks stop the retained Navigation localization session."""

import json

import pytest

from test_apriltag_task_stop_ros import Message, RosHarness, SPORT, debug
from test_navigation_worker_ros import grant_legacy, legacy, run, send


STATUS = "/grit_slam/localization/status"
COMMAND = "/grit_slam/localization/command"


def localization(ros, state="READY", task_id="navigation-1", map_uuid="map-1"):
    ros.subscriptions[STATUS](Message(json.dumps({
        "state": state, "task_id": task_id, "map_uuid": map_uuid,
    })))


def commands(ros):
    return [json.loads(message.data) for message in ros.published[COMMAND]]


def stop(task_id="navigation-1", map_uuid="map-1"):
    return dict(command="STOP", task_id=task_id, map_uuid=map_uuid, grace_sec=0)


@pytest.mark.parametrize("state", ["STARTING", "READY"])
def test_task_stops_retained_session_before_sport_output_and_retries(monkeypatch, state):
    ros = RosHarness(monkeypatch)

    def scenario(node):
        qos = ros.subscription_qos[STATUS]
        assert qos.reliability is ros.reliable
        assert qos.durability is ros.transient_local
        assert qos.history is ros.keep_last and qos.depth == 1
        localization(ros, state)
        assert commands(ros) == []  # Loading the model must not stop SLAM.
        ros.start()
        assert commands(ros) == [stop()]
        topics = [topic for topic, _ in ros.events]
        assert topics.index(COMMAND) < topics.index(SPORT) < topics.index("/task_state")
        ros.now = .5
        node.publish_state()
        assert commands(ros) == [stop()]
        ros.now = 1.
        node.publish_state()
        assert commands(ros) == [stop(), stop()]
        localization(ros, "STOPPED")
        ros.now = 1.5
        node.publish_state()
        assert commands(ros) == [stop(), stop()]

    ros.run(scenario)


def test_late_slam_status_and_restarts_are_stopped_only_during_task(monkeypatch):
    ros = RosHarness(monkeypatch)
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_" + check.upper(), "false")

    def scenario(node):
        ros.start()  # A deployment without SLAM still accepts tasks.
        assert commands(ros) == []
        localization(ros, "STARTING")
        assert commands(ros) == [stop()]
        localization(ros, "STOPPED")
        localization(ros, task_id="navigation-2", map_uuid="map-2")
        assert commands(ros) == [stop(), stop("navigation-2", "map-2")]
        node.on_task_event(Message(json.dumps({
            "type": "TASK_ABORTED", "task_id": "tag-stop-1",
        })))
        localization(ros, task_id="navigation-3", map_uuid="map-3")
        ros.now = 1.1
        node.publish_state()
        assert commands(ros) == [stop(), stop("navigation-2", "map-2")]
        ros.start(task_id="next-task")
        assert commands(ros)[-1] == stop("navigation-3", "map-3")
        assert all(command["command"] == "STOP" for command in commands(ros))

    ros.run(scenario)


@pytest.mark.parametrize("raw", [
    "invalid-json", "null", "[]", "42", "{}",
    '{"state":"READY","map_uuid":"map-1"}',
    '{"state":"READY","task_id":"navigation-1"}',
    '{"state":"READY","task_id":true,"map_uuid":"map-1"}',
    '{"state":"READY","task_id":"navigation-1","map_uuid":[]}',
])
def test_malformed_status_cannot_replace_known_session(monkeypatch, raw):
    ros = RosHarness(monkeypatch)

    def scenario(node):
        localization(ros)
        ros.subscriptions[STATUS](Message(raw))
        ros.start()
        assert commands(ros) == [stop()]

    ros.run(scenario)


@pytest.mark.parametrize("state", ["STOPPED", "ERROR"])
def test_inactive_localization_and_rejected_tasks_do_not_send_stop(monkeypatch, state):
    ros = RosHarness(monkeypatch)

    def scenario(node):
        localization(ros)
        node.on_task_event(Message(json.dumps({
            "type": "TASK_REGISTERED", "action_name": "LINE_TRACKING",
            "task_id": "invalid-task", "payload": {"selected_mask": 9},
        })))
        assert ros.task_state()["type"] == "TASK_REJECTED"
        assert commands(ros) == []
        localization(ros, state)
        ros.start()
        assert commands(ros) == []

    ros.run(scenario)


@pytest.mark.parametrize("granted", [False, True])
def test_shared_worker_stops_slam_only_after_standalone_grant(monkeypatch, granted):
    ros = RosHarness(monkeypatch)

    def scenario(node):
        localization(ros)
        send(ros)  # Navigation-owned segments require SLAM guidance.
        localization(ros)
        node.publish_state()
        assert commands(ros) == []
        send(ros, "STOP")
        legacy(ros)
        localization(ros)
        node.publish_state()
        assert commands(ros) == []
        grant_legacy(ros, "GRANTED" if granted else "REJECTED")
        assert commands(ros) == ([stop()] if granted else [])
        if granted:
            topics = [topic for topic, _ in ros.events]
            assert topics.index(COMMAND) < topics.index(SPORT)
            legacy(ros, "TASK_ABORTED")
            localization(ros, task_id="next-navigation")
            assert commands(ros) == [stop()]

    run(ros, scenario)


def test_slam_publication_failure_aborts_motion(monkeypatch):
    ros = RosHarness(monkeypatch)

    def scenario(node):
        ros.start()

        def fail(_message):
            raise RuntimeError("SLAM command transport failed")

        monkeypatch.setattr(node.slam_command_publisher, "publish", fail)
        localization(ros)
        assert node.tasks.active is None
        assert ros.task_state()["reason"] == "slam_stop_publish_error"
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] == 0
        assert any("SLAM STOP publication failed" in error for error in ros.errors)

    ros.run(scenario, allow_errors=True)


def test_debug_mode_has_no_slam_control(monkeypatch):
    ros = RosHarness(monkeypatch)

    def scenario(node):
        node.publish_state()
        assert STATUS not in ros.subscriptions
        assert COMMAND not in ros.published

    ros.rclpy.spin = scenario
    assert debug.run_ros2(debug.parse_args(["ros2"])) == 0
    assert not ros.errors
