"""End-to-end selected_mask=3 task/Sport behavior with generated ROS packets."""

import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_apriltag_task_stop_ros import Message, SPORT, ZERO, RosHarness, debug
from test_lidar_obstacles_ros import complete_inference, enabled_ros, sensors
from sensor_fixtures import camera_info, cloud


def start(ros, task_id="light-3", *, duration=30):
    ros.node.on_task_event(Message(json.dumps({
        "type": "TASK_REGISTERED", "action_name": "LINE_TRACKING", "task_id": task_id,
        "payload": {"selected_mask": 3, "duration_sec": duration},
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


@pytest.mark.parametrize("configured_enabled", [False, True])
def test_task_cruises_approaches_laterally_resumes_and_does_not_complete(monkeypatch, configured_enabled):
    ros = enabled_ros(monkeypatch)
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_AVOIDANCE_ENABLED", str(configured_enabled).lower())
    detection = scene()
    calls = model(monkeypatch, detection)

    def scenario(node):
        for payload in (None, [], 3, "invalid"):
            node.on_task_event(Message(json.dumps(payload)))
        assert node.command_publisher is None
        start(ros)
        assert ros.task_state()["type"] == "TASK_STARTED"
        assert node.obstacles is not None
        assert node.tasks.active.selected_mask == 3
        ros.now = 2.1
        detection.light[:] = False
        metrics = tick(ros, node)
        assert metrics["selected_mask"] == 3
        assert metrics["lidar_obstacles"]["enabled"] is True
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
        assert metrics["street_light"]["transition"] == "reached"
        assert metrics["street_light"]["bias_active"] is False
        assert metrics["street_light"]["visits"] == 1
        assert node.tasks.active is not None
        for _ in range(4):
            metrics = tick(ros, node)
            assert metrics["drive_reason"] == "lidar_tracking"
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
        assert (node.obstacles is not None) == configured_enabled

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
            assert metrics["street_light"]["transition"] == "reached"
            assert metrics["street_light"]["bias_active"] is False
            assert metrics["street_light"]["visited_ids"] == list(range(1, target_id + 1))
            # All detections stay present. No global disappearance is needed.
            metrics = tick(ros, node)
            if target_id == 1:
                assert metrics["drive_reason"] == "street_light_approaching"
                assert metrics["street_light"]["target_id"] == 2
                assert metrics["street_light"]["visible_ids"] == [1, 2]
        for _ in range(4):
            metrics = tick(ros, node)
            assert metrics["drive_reason"] == "lidar_tracking"
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
            assert metrics["drive_reason"] == "lidar_tracking"
            assert metrics["street_light"]["visited_ids"] == [1, 2]
        assert node.tasks.active is not None
        node.on_task_event(Message(json.dumps({"type": "TASK_ABORTED", "task_id": "light-3"})))
        assert node.street_lights.metrics["visited_ids"] == []
        assert node.street_lights.metrics["visible_ids"] == []

    ros.run(scenario, "--branch-preference", "center")


@pytest.mark.parametrize("selected_mask", [0, 1, 2])
@pytest.mark.parametrize("terminal", ["server", "timeout", "apriltag"])
def test_automatic_lidar_is_released_before_camera_only_tasks(monkeypatch, selected_mask, terminal):
    ros = enabled_ros(monkeypatch)
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_AVOIDANCE_ENABLED", "false")
    if terminal == "timeout":
        monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_TASK_TIMEOUT", "true")
    if terminal == "apriltag":
        monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_APRILTAG", "true")
    detection = scene()
    detection.light[:] = False
    model(monkeypatch, detection)
    sensor_topics = ("/livox/lidar", "/livox/imu", "/a2/front_camera/res_360p/camera_info")

    def scenario(node):
        assert node.obstacles is None
        assert not set(sensor_topics) & ros.subscriptions.keys()
        assert node.timer.period == pytest.approx(0.1)
        start(ros, duration=3)
        assert ros.task_state()["type"] == "TASK_STARTED"
        first_runtime, lidar_timer = node.obstacles, node.timer
        assert first_runtime is not None
        assert all(topic in ros.subscriptions for topic in sensor_topics)
        late_callbacks = [ros.subscriptions[topic] for topic in sensor_topics]
        assert lidar_timer.period == pytest.approx(0.05)
        ros.now = 2.1
        ros.detect()  # Keep the AprilTag heartbeat live for that completion case.
        metrics = complete_inference(node, ros)  # Camera alone cannot start scenario 3.
        assert metrics["lidar_obstacles"]["enabled"] is True
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        metrics = tick(ros, node)
        assert metrics["drive_reason"] == "lidar_tracking"
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] > 0

        if terminal == "server":
            node.on_task_event(Message(json.dumps({"type": "TASK_ABORTED", "task_id": "light-3"})))
        elif terminal == "timeout":
            ros.now = 3.01
            sensors(node, ros)
            node.publish_state()
            assert ros.task_state()["type"] == "TASK_COMPLETED"
        else:
            node.complete_apriltag_task(7)
            assert ros.task_state()["type"] == "TASK_COMPLETED"
        assert node.tasks.active is None
        assert node.obstacles is None
        assert not set(sensor_topics) & ros.subscriptions.keys()
        assert lidar_timer.destroyed
        assert node.timer.period == pytest.approx(0.1)
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        for callback in late_callbacks:
            callback(Message())  # A queued sensor callback after unsubscribe is harmless.
        node.publish_state()
        assert ros.metrics()["lidar_obstacles"]["enabled"] is False

        ros.now += 1.1
        node.publish_state()  # Finish the Sport publisher release window.
        detection.road[:] = 2 if selected_mask == 2 else 1
        node.on_task_event(Message(json.dumps({
            "type": "TASK_REGISTERED", "action_name": "LINE_TRACKING", "task_id": "camera-only",
            "payload": {"selected_mask": selected_mask},
        })))
        assert ros.task_state()["type"] == "TASK_STARTED"
        ros.detect(frame=2)
        metrics = complete_inference(node, ros)
        assert metrics["selected_mask"] == selected_mask
        assert metrics["lidar_obstacles"]["enabled"] is False
        assert metrics["drive_reason"] == "tracking"
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] > 0
        assert node.obstacles is None
        start(ros, task_id="busy-request")
        assert ros.task_state()["reason"] == "another_line_tracking_task_active"
        assert node.obstacles is None  # A rejected 3 request must not enable LiDAR.

        node.on_task_event(Message(json.dumps({"type": "TASK_ABORTED", "task_id": "camera-only"})))
        ros.now += 1.1
        node.publish_state()
        start(ros, task_id="light-3-again")
        assert ros.task_state()["type"] == "TASK_STARTED"
        assert node.obstacles is not first_runtime
        assert node.obstacles.scan is None  # Previous sensor data must not survive reactivation.
        assert node.timer.period == pytest.approx(0.05)
        assert all(topic in ros.subscriptions for topic in sensor_topics)

    ros.run(scenario, "--branch-preference", "center")


@pytest.mark.parametrize("profile", ["livox-a2-front", "tf"])
def test_default_scenario_three_automatically_enables_lidar_despite_cli_disable(monkeypatch, profile):
    import sys

    ros = RosHarness(monkeypatch)
    model(monkeypatch, scene())
    listeners = []

    class Listener:
        def __init__(self, _buffer, _node):
            self.unregistered = False
            listeners.append(self)

        def unregister(self):
            self.unregistered = True

    monkeypatch.setitem(sys.modules, "tf2_ros", NS(Buffer=object, TransformListener=Listener))

    def scenario(node):
        assert node.obstacles is None
        assert not listeners
        ros.start()  # No selected_mask in the payload: use SWIN_L_PATH_MASK_CLASS / CLI default.
        assert node.tasks.active.selected_mask == 3
        assert node.obstacles is not None
        assert "/livox/lidar" in ros.subscriptions
        node.publish_state()
        assert ros.metrics()["lidar_obstacles"]["enabled"] is True
        assert bool(listeners) == (profile == "tf")
        node.on_task_event(Message(json.dumps({"type": "TASK_ABORTED", "task_id": "tag-stop-1"})))
        assert node.obstacles is None
        assert node.tf_listener is node.tf_buffer is None
        assert all(listener.unregistered for listener in listeners)

    ros.run(scenario, "--path-mask-class", "3", "--no-avoidance-enabled",
            "--lidar-calibration-profile", profile)


@pytest.mark.parametrize("approaching", [False, True])
def test_ten_hz_lidar_with_twenty_hz_control_tolerates_short_gaps_stops_and_resumes(monkeypatch, approaching):
    ros = enabled_ros(monkeypatch)
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_AVOIDANCE_ENABLED", "false")
    detection = scene()
    if not approaching:
        detection.light[:] = False
    model(monkeypatch, detection)
    expected_reason = "street_light_approaching" if approaching else "lidar_tracking"

    def assert_moving(metrics):
        assert metrics["drive_reason"] == expected_reason
        command = json.loads(ros.published[SPORT][-1].parameter)
        assert command["x"] > 0 and command["z"] == 0
        if approaching:
            assert command["y"] < 0

    def scenario(node):
        start(ros)
        ros.now = 2.1
        for _ in range(3):
            metrics = tick(ros, node)
        assert metrics["drive_reason"] == expected_reason
        assert node.obstacles.config.max_age_sec == .5
        assert node.timer.period == pytest.approx(.05)

        # The 20 Hz control tick between 10 Hz clouds reuses a valid scan.
        control_started = ros.now
        for step in range(1, 11):
            ros.now = control_started + step * .05
            if step % 2 == 0:
                sensors(node, ros)
                metrics = complete_inference(node, ros)
            else:
                node.publish_state()
                metrics = ros.metrics()
            assert_moving(metrics)

        last_scan = node.obstacles.scan
        # Camera updates continue while several LiDAR publications are missed.
        for age in (.15, .25, .35, .45):
            ros.now = last_scan.arrival + age
            assert_moving(complete_inference(node, ros))
        ros.now = last_scan.arrival + .55
        metrics = complete_inference(node, ros)
        assert metrics["drive_reason"] == "lidar_stale"
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert node.tasks.active is not None

        # Receiving an old stamped packet cannot refresh the scan's lifetime.
        node.on_lidar(cloud([[1, 0, 0]], last_scan.stamp_ns / 1e9))
        node.publish_state()
        assert ros.metrics()["drive_reason"] == "lidar_stale"
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

        ros.now += .1
        sensors(node, ros)
        assert_moving(complete_inference(node, ros))
        assert node.obstacles.scan.stamp_ns > last_scan.stamp_ns

    ros.run(scenario, "--branch-preference", "center")


@pytest.mark.parametrize("fault", ["cloud_stale", "road_lost", "collision"])
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
            ros.now += node.obstacles.config.max_age_sec + .01
            node.publish_state()
            assert ros.metrics()["drive_reason"] == "lidar_stale"
        elif fault == "road_lost":
            detection.road[:] = 2  # A sidewalk must not satisfy scenario 3.
            tick(ros, node)
        else:
            ros.now += 0.1
            sensors(node, ros, [[0.6, 0.0]])
            node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert node.tasks.active is not None

    ros.run(scenario, "--branch-preference", "center")


@pytest.mark.parametrize("fault", ["inference_delayed", "target_lost", "geometry"])
def test_light_bias_failure_resumes_road_without_stopping_or_retaining_lateral(monkeypatch, fault):
    ros = enabled_ros(monkeypatch)
    detection = scene()
    model(monkeypatch, detection)

    def scenario(node):
        start(ros)
        ros.now = 2.1
        for _ in range(3):
            metrics = tick(ros, node)
        assert metrics["drive_reason"] == "street_light_approaching"
        if fault == "inference_delayed":
            ros.now += 1.1
            sensors(node, ros)
            node.publish_state()
            metrics = ros.metrics()
            assert metrics["street_light"]["bias_reason"] == "observations_stale"
        elif fault == "target_lost":
            detection.light[:] = False
            metrics = tick(ros, node)
            assert metrics["street_light"]["transition"] == "target_lost"
        else:
            def invalid(_observation, _now, _clock):
                raise ValueError("street_light_geometry_invalid")
            monkeypatch.setattr(node.obstacles, "locate_street_light", invalid)
            metrics = tick(ros, node)
        assert metrics["drive_reason"] in ("lidar_tracking", "lidar_returning")
        assert metrics["street_light"]["bias_active"] is False
        command = json.loads(ros.published[SPORT][-1].parameter)
        assert command["x"] > 0 and command["z"] == 0
        assert node.obstacles.controller.target_velocity[1] == 0
        # Deceleration still obeys the shared lateral acceleration limit.
        for _ in range(3):
            ros.now += .05
            sensors(node, ros)
            node.publish_state()
        command = json.loads(ros.published[SPORT][-1].parameter)
        assert command["x"] > 0 and command["y"] == command["z"] == 0
        assert node.tasks.active is not None

    ros.run(scenario, "--branch-preference", "center")


def test_blocked_light_bias_keeps_safe_road_tracking_without_repeated_speed_resets(monkeypatch):
    ros = enabled_ros(monkeypatch)
    model(monkeypatch, scene())

    def scenario(node):
        start(ros)
        original_context = node.obstacles._context

        def corridor(mask_class, now, clock_ns):
            scan, _ = original_context(mask_class, now, clock_ns)
            config = node.obstacles.config
            region = np.abs(config.centers()[..., 1]) <= config.robot_half_width_m + config.margin_m
            return scan, region

        monkeypatch.setattr(node.obstacles, "_context", corridor)
        ros.now = 2.1
        previous_speed = 0.0
        for _ in range(10):
            metrics = tick(ros, node)
            command = json.loads(ros.published[SPORT][-1].parameter)
            assert command["x"] >= previous_speed
            assert command["y"] == command["z"] == 0
            previous_speed = command["x"]
        assert metrics["lidar_obstacles"]["street_light_bias_blocked_reason"] == "avoidance_space_unobserved"
        assert metrics["drive_reason"] == "lidar_tracking"
        assert metrics["street_light"]["bias_active"] is False
        assert previous_speed == pytest.approx(node.obstacles.avoidance.max_forward_mps)

    ros.run(scenario, "--branch-preference", "center")


def test_general_inference_stop_still_overrides_optional_light_bias(monkeypatch):
    ros = enabled_ros(monkeypatch)
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_INFERENCE_STALE", "true")
    model(monkeypatch, scene())

    def scenario(node):
        start(ros)
        ros.now = 2.1
        for _ in range(3):
            tick(ros, node)
        ros.now += node.drive_config.max_inference_age_sec + .1
        sensors(node, ros)
        node.publish_state()
        assert ros.metrics()["drive_reason"] == "inference_stale"
        assert ros.metrics()["street_light"]["bias_active"] is False
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(scenario, "--branch-preference", "center")


def test_delayed_light_evidence_does_not_abort_a_normally_tracking_task_at_its_deadline(monkeypatch):
    ros = enabled_ros(monkeypatch)
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_TASK_TIMEOUT", "true")
    model(monkeypatch, scene())

    def scenario(node):
        start(ros, duration=4)
        ros.now = 2.8
        for _ in range(3):
            metrics = tick(ros, node)
        assert metrics["drive_reason"] == "street_light_approaching"
        ros.now += 1.1
        sensors(node, ros)
        node.publish_state()
        assert ros.task_state()["type"] == "TASK_COMPLETED"
        assert node.tasks.active is None
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(scenario, "--branch-preference", "center")


@pytest.mark.parametrize("fault,reason", [
    ("label", "street_light_label_unavailable"),
    ("distance", "street_light_stop_distance_out_of_range"),
])
def test_unusable_scenario_is_rejected_before_creating_control(monkeypatch, fault, reason):
    ros = RosHarness(monkeypatch)
    if fault == "distance":
        monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STREET_LIGHT_STOP_DISTANCE_M", "0.5")
    model(monkeypatch, scene(), label=None if fault == "label" else 5)

    def scenario(node):
        start(ros)
        assert ros.task_state()["type"] == "TASK_REJECTED"
        assert ros.task_state()["reason"] == reason
        assert node.tasks.active is None
        assert node.command_publisher is None
        assert node.obstacles is None
        assert "/livox/lidar" not in ros.subscriptions
        assert SPORT not in ros.published

    ros.run(scenario)
