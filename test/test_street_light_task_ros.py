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


def confirmed_points(node, ros, *, centers=(235, 405), distance=5., count=3):
    info = camera_info(ros.clock_ns() / 1e9)
    info.d = []
    node.on_camera_info(info)
    k = np.asarray(info.k).reshape(3, 3)
    camera = node.obstacles.camera
    pixels = np.array([[u + offset, 80., 1.] for u in centers
                       for offset in range(-(count // 2), count - count // 2)])
    depth = max(.1, distance - .3381)
    camera_points = (pixels @ np.linalg.inv(k).T) * depth
    base_points = (camera_points - camera[:3, 3]) @ camera[:3, :3]
    base = node.obstacles.base
    raw = (base_points - base[:3, 3]) @ base[:3, :3]
    node.on_lidar(cloud(raw, ros.clock_ns() / 1e9))


def tick(ros, node, *, confirm=True):
    ros.now += 0.1
    if confirm:
        confirmed_points(node, ros)
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
        assert metrics["street_light"]["range_m"] > 1.5
        assert metrics["street_light"]["lidar_confirmed"] is True
        assert metrics["lidar_obstacles"]["stops_enabled"] is False
        ros.now += 0.05  # The next control tick can ramp from cruise to lateral.
        node.publish_state()
        command = json.loads(ros.published[SPORT][-1].parameter)
        assert command["y"] < 0  # Visual left -> internal +vy -> Sport -y.
        assert command["z"] == 0
        assert calls[-1]["detect_street_lights"] is True
        # A current LiDAR return in the lamp's image columns supplies proximity.
        ros.now += 0.1
        confirmed_points(node, ros, centers=(235,), distance=.85)
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
            assert metrics["drive_reason"] == "tracking"
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
            confirmed_points(node, ros, centers=(235 if target_id == 1 else 405,), distance=.85)
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
            assert metrics["drive_reason"] == "tracking"
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
            assert metrics["drive_reason"] == "tracking"
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
    sensor_topics = ("/livox/lidar", "/a2/front_camera/res_360p/camera_info")

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
        assert lidar_timer.period == pytest.approx(0.1)
        ros.now = 2.1
        ros.detect()  # Keep the AprilTag heartbeat live for that completion case.
        metrics = complete_inference(node, ros)  # Camera alone can track the road, without bias.
        assert metrics["lidar_obstacles"]["enabled"] is True
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] > 0
        metrics = tick(ros, node)
        assert metrics["drive_reason"] == "tracking"
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
        assert node.timer.period == pytest.approx(0.1)
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


@pytest.mark.parametrize("configured_enabled", [False, True])
@pytest.mark.parametrize("fault", ["missing", "stale", "old_stamp", "malformed", "collision", "no_info", "unmatched"])
def test_lidar_confirmation_failure_never_stops_road_tracking(monkeypatch, configured_enabled, fault):
    ros = enabled_ros(monkeypatch)
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_AVOIDANCE_ENABLED", str(configured_enabled).lower())
    detection = scene()
    model(monkeypatch, detection)

    def scenario(node):
        start(ros)
        ros.now = 2.1
        assert node.obstacles.confirmation_only
        assert "/livox/imu" not in ros.subscriptions
        if fault == "missing":
            for _ in range(3):
                metrics = tick(ros, node, confirm=False)
        else:
            for _ in range(3):
                metrics = tick(ros, node)
            assert metrics["drive_reason"] == "street_light_approaching"
            if fault == "stale":
                ros.now += .6
            elif fault == "old_stamp":
                node.on_lidar(cloud([[1., 0., 0.]], ros.clock_ns() / 1e9 - 1.))
            elif fault == "malformed":
                packet = cloud([[1., 0., 0.]], ros.clock_ns() / 1e9)
                packet.data = b""
                node.on_lidar(packet)
            elif fault == "collision":
                sensors(node, ros, [[.6, 0.]])
            elif fault == "no_info":
                node.obstacles.infos.clear()
            else:
                confirmed_points(node, ros, centers=(500,))
            ros.now += .05
            metrics = complete_inference(node, ros)
        assert metrics["drive_reason"] == "tracking"
        assert metrics["street_light"]["bias_active"] is False
        assert metrics["lidar_obstacles"]["stops_enabled"] is False
        command = json.loads(ros.published[SPORT][-1].parameter)
        assert command["x"] == pytest.approx(node.drive_config.max_forward_mps)
        assert command["y"] == command["z"] == 0
        assert node.tasks.active is not None
        # Confirmation can recover on the next fresh matching cloud.
        metrics = tick(ros, node)
        assert metrics["drive_reason"] == "street_light_approaching"
        assert metrics["street_light"]["lidar_confirmed"] is True
        node.on_task_event(Message(json.dumps({"type": "TASK_ABORTED", "task_id": "light-3"})))
        assert (node.obstacles is not None) == configured_enabled
        if configured_enabled:
            assert not node.obstacles.confirmation_only

    ros.run(scenario, "--branch-preference", "center")


def test_missing_road_still_uses_normal_camera_path_recovery(monkeypatch):
    ros = enabled_ros(monkeypatch)
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_PATH_LOSS_RECOVERY_ENABLED", "true")
    detection = scene()
    model(monkeypatch, detection)

    def scenario(node):
        start(ros)
        ros.now = 2.1
        for _ in range(3):
            tick(ros, node)
        detection.road[:] = 2
        for _ in range(3):
            metrics = tick(ros, node)
        assert metrics["drive_reason"] == "path_recovery_waiting"
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert metrics["street_light"]["bias_active"] is False

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
        assert metrics["drive_reason"] == "tracking"
        assert metrics["street_light"]["bias_active"] is False
        command = json.loads(ros.published[SPORT][-1].parameter)
        assert command["x"] > 0 and command["z"] == 0
        assert command["y"] == 0
        # Ordinary road tracking continues after bias is disabled.
        for _ in range(3):
            ros.now += .05
            sensors(node, ros)
            node.publish_state()
        command = json.loads(ros.published[SPORT][-1].parameter)
        assert command["x"] > 0 and command["y"] == command["z"] == 0
        assert node.tasks.active is not None

    ros.run(scenario, "--branch-preference", "center")


def test_confirmation_does_not_run_obstacle_checks_or_cap_road_speed(monkeypatch):
    ros = enabled_ros(monkeypatch)
    model(monkeypatch, scene())

    def scenario(node):
        start(ros)
        def unused(*_args, **_kwargs):
            pytest.fail("scenario 3 must not run LiDAR obstacle or region gates")
        monkeypatch.setattr(node.obstacles, "_context", unused)
        monkeypatch.setattr(node.obstacles.detector, "update", unused)
        monkeypatch.setattr(node.obstacles.controller, "trajectory_check", unused)
        ros.now = 2.1
        for _ in range(3):
            metrics = tick(ros, node)
        assert metrics["drive_reason"] == "street_light_approaching"
        command = json.loads(ros.published[SPORT][-1].parameter)
        assert command["x"] == pytest.approx(node.drive_config.max_forward_mps)
        assert command["y"] < 0 and command["z"] == 0
        assert metrics["street_light"]["bias_active"] is True

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


def test_confirmation_mode_restores_explicit_lidar_stops_for_next_ordinary_task(monkeypatch):
    ros = enabled_ros(monkeypatch)
    model(monkeypatch, scene())

    def scenario(node):
        original_runtime = node.obstacles
        assert not original_runtime.confirmation_only
        start(ros)
        assert node.obstacles.confirmation_only
        ros.now = 2.1
        assert tick(ros, node, confirm=False)["drive_reason"] == "tracking"
        node.on_task_event(Message(json.dumps({"type": "TASK_ABORTED", "task_id": "light-3"})))
        assert node.obstacles is not original_runtime
        assert not node.obstacles.confirmation_only
        assert "/livox/imu" in ros.subscriptions
        ros.now += 1.1
        node.publish_state()
        node.on_task_event(Message(json.dumps({
            "type": "TASK_REGISTERED", "action_name": "LINE_TRACKING", "task_id": "ordinary-road",
            "payload": {"selected_mask": 1},
        })))
        ros.now += .1
        sensors(node, ros)
        assert complete_inference(node, ros)["drive_reason"] == "lidar_tracking"
        ros.now += .6
        assert complete_inference(node, ros)["drive_reason"] == "lidar_stale"
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


def test_missing_street_light_model_label_rejects_before_creating_control(monkeypatch):
    ros = RosHarness(monkeypatch)
    model(monkeypatch, scene(), label=None)

    def scenario(node):
        start(ros)
        assert ros.task_state()["type"] == "TASK_REJECTED"
        assert ros.task_state()["reason"] == "street_light_label_unavailable"
        assert node.tasks.active is None
        assert node.command_publisher is None
        assert node.obstacles is None
        assert "/livox/lidar" not in ros.subscriptions
        assert SPORT not in ros.published

    ros.run(scenario)


def test_bias_release_distance_is_independent_of_obstacle_footprint_and_radius(monkeypatch):
    ros = enabled_ros(monkeypatch)
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STREET_LIGHT_STOP_DISTANCE_M", "0.5")
    model(monkeypatch, scene())

    def scenario(node):
        start(ros)
        assert ros.task_state()["type"] == "TASK_STARTED"
        ros.now = 2.1
        metrics = tick(ros, node, confirm=False)
        assert metrics["drive_reason"] == "tracking"
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] > 0

    ros.run(scenario, "--branch-preference", "center")
