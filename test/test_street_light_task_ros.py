"""End-to-end selected_mask=3 task/Sport behavior with generated ROS packets."""

import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_apriltag_task_stop_ros import Message, SPORT, ZERO, RosHarness, debug
from test_lidar_obstacles_ros import complete_inference, enabled_ros, sensors
from sensor_fixtures import camera_info


def start(ros, task_id="light-3"):
    ros.node.on_task_event(Message(json.dumps({
        "type": "TASK_REGISTERED", "action_name": "LINE_TRACKING", "task_id": task_id,
        "payload": {"selected_mask": 3, "duration_sec": 30},
    })))


def model(monkeypatch, detection, *, label=5):
    calls = []

    def segment(_frame, **kwargs):
        calls.append(kwargs)
        return NS(selected_mask=detection.road.copy(), street_light_mask=detection.light.copy(),
                  inference_seconds=0.01)

    monkeypatch.setattr(debug, "BestSoFarSegmenter", lambda _: NS(
        device=NS(type="cuda"), reset=lambda: None, street_light_id=label, segment=segment))
    return calls


def scene():
    light = np.zeros((360, 640), bool)
    light[70:90, 225:245] = True
    return NS(road=np.ones((360, 640), np.uint8), light=light)


def tick(ros, node):
    ros.now += 0.1
    sensors(node, ros)
    # Deterministic pinhole camera, still using the measured camera/base pose.
    info = camera_info(ros.clock_ns() / 1e9)
    info.d = []
    node.on_camera_info(info)
    return complete_inference(node, ros)


def test_task_cruises_approaches_laterally_resumes_and_does_not_complete(monkeypatch):
    ros = enabled_ros(monkeypatch)
    detection = scene()
    calls = model(monkeypatch, detection)

    def scenario(node):
        for payload in (None, [], 3, "invalid"):
            node.on_task_event(Message(json.dumps(payload)))
        assert node.command_publisher is None
        start(ros)
        assert ros.task_state()["type"] == "TASK_STARTED"
        assert node.tasks.active.selected_mask == 3
        ros.now = 2.1
        detection.light[:] = False
        metrics = tick(ros, node)
        assert metrics["selected_mask"] == 3
        assert metrics["path_mask_class"] == 1
        assert metrics["path_surface"] == "ROAD"
        command = json.loads(ros.published[SPORT][-1].parameter)
        assert command["x"] > 0 and command["y"] == command["z"] == 0
        detection.light[70:90, 225:245] = True
        for _ in range(3):
            metrics = tick(ros, node)
        assert metrics["drive_reason"] == "street_light_approaching"
        assert metrics["street_light"]["range_m"] is None
        ros.now += 0.05  # The next control tick can ramp from cruise to lateral.
        node.publish_state()
        command = json.loads(ros.published[SPORT][-1].parameter)
        assert command["y"] < 0  # Visual left -> internal +vy -> Sport -y.
        assert command["z"] == 0
        assert calls[-1]["detect_street_lights"] is True
        # A current LiDAR return in the lamp's image columns supplies proximity.
        ros.now += 0.1
        sensors(node, ros, [[0.85, 0.20]])
        info = camera_info(ros.clock_ns() / 1e9)
        info.d = []
        node.on_camera_info(info)
        node.publish_state()
        metrics = ros.metrics()
        assert metrics["drive_reason"] == "street_light_reached"
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert metrics["street_light"]["visits"] == 1
        assert node.tasks.active is not None
        for _ in range(4):
            metrics = tick(ros, node)
            assert metrics["drive_reason"] == "street_light_cruising"
            assert metrics["street_light"]["state"] == "cruising"
            assert metrics["street_light"]["visited_ids"] == [1]
            assert metrics["street_light"]["pending_ids"] == []
            command = json.loads(ros.published[SPORT][-1].parameter)
            assert command["x"] > 0 and command["y"] == command["z"] == 0
        assert not any(json.loads(m.data)["type"] == "TASK_COMPLETED"
                       for m in ros.published["/task_state"])
        # Cancellation suspends inference and resets scenario history.
        node.on_task_event(Message(json.dumps({"type": "TASK_ABORTED", "task_id": "light-3"})))
        assert node.street_lights.phase == "cruising"
        assert node.street_lights.visits == 0
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(scenario, "--branch-preference", "center")


def test_task_visits_both_continuously_visible_lights_and_excludes_completed_ids(monkeypatch):
    ros = enabled_ros(monkeypatch)
    detection = scene()
    detection.light[70:90, 395:415] = True
    model(monkeypatch, detection)

    def scenario(node):
        start(ros)
        ros.now = 2.1
        for _ in range(3):
            metrics = tick(ros, node)
        assert metrics["street_light"]["target_id"] == 1
        assert metrics["street_light"]["pending_ids"] == [2]
        for target_id, point, sport_sign in ((1, [0.85, 0.20], -1),
                                             (2, [0.85, -0.20], 1)):
            ros.now += 0.05
            node.publish_state()
            command = json.loads(ros.published[SPORT][-1].parameter)
            assert command["y"] * sport_sign > 0
            assert command["z"] == 0
            ros.now += 0.1
            sensors(node, ros, [point])
            info = camera_info(ros.clock_ns() / 1e9)
            info.d = []
            node.on_camera_info(info)
            node.publish_state()
            metrics = ros.metrics()
            assert metrics["drive_reason"] == "street_light_reached"
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
            assert metrics["street_light"]["visited_ids"] == list(range(1, target_id + 1))
            # All detections stay present. No global disappearance is needed.
            metrics = tick(ros, node)
            if target_id == 1:
                assert metrics["drive_reason"] == "street_light_approaching"
                assert metrics["street_light"]["target_id"] == 2
                assert metrics["street_light"]["visible_ids"] == [1, 2]
        for _ in range(4):
            metrics = tick(ros, node)
            assert metrics["drive_reason"] == "street_light_cruising"
            assert metrics["street_light"]["visits"] == 2
            assert metrics["street_light"]["pending_ids"] == []
        # Occlusion does not reset the visit ledger during the same task.
        detection.light[:] = False
        for _ in range(4):
            tick(ros, node)
        detection.light[70:90, 225:245] = True
        detection.light[70:90, 395:415] = True
        for _ in range(4):
            metrics = tick(ros, node)
            assert metrics["drive_reason"] == "street_light_cruising"
            assert metrics["street_light"]["visited_ids"] == [1, 2]
        assert node.tasks.active is not None
        node.on_task_event(Message(json.dumps({"type": "TASK_ABORTED", "task_id": "light-3"})))
        assert node.street_lights.metrics["visited_ids"] == []
        assert node.street_lights.metrics["visible_ids"] == []

    ros.run(scenario, "--branch-preference", "center")


@pytest.mark.parametrize("fault", ["cloud_stale", "road_lost", "inference_stale", "collision", "target_lost"])
def test_scenario_cannot_bypass_sensors_road_or_collision_gates(monkeypatch, fault):
    ros = enabled_ros(monkeypatch)
    detection = scene()
    model(monkeypatch, detection)

    def scenario(node):
        start(ros)
        ros.now = 2.1
        for _ in range(3):
            metrics = tick(ros, node)
        assert metrics["drive_reason"] == "street_light_approaching"
        if fault == "cloud_stale":
            ros.now += 0.31
            node.publish_state()
        elif fault == "road_lost":
            detection.road[:] = 2  # A sidewalk must not satisfy scenario 3.
            tick(ros, node)
        elif fault == "inference_stale":
            ros.now += 1.1
            sensors(node, ros)
            node.publish_state()
        elif fault == "target_lost":
            detection.light[:] = False
            tick(ros, node)
            assert ros.metrics()["drive_reason"] == "street_light_target_lost"
        else:
            ros.now += 0.1
            sensors(node, ros, [[0.6, 0.0]])
            node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert node.tasks.active is not None

    ros.run(scenario, "--branch-preference", "center")


@pytest.mark.parametrize("fault,reason", [
    ("disabled", "street_light_requires_avoidance"),
    ("label", "street_light_label_unavailable"),
    ("distance", "street_light_stop_distance_out_of_range"),
])
def test_unusable_scenario_is_rejected_before_creating_control(monkeypatch, fault, reason):
    ros = RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_AVOIDANCE_ENABLED", "false" if fault == "disabled" else "true")
    if fault == "distance":
        monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STREET_LIGHT_STOP_DISTANCE_M", "0.5")
    model(monkeypatch, scene(), label=None if fault == "label" else 5)

    def scenario(node):
        start(ros)
        assert ros.task_state()["type"] == "TASK_REJECTED"
        assert ros.task_state()["reason"] == reason
        assert node.tasks.active is None
        assert node.command_publisher is None
        assert SPORT not in ros.published

    ros.run(scenario)
