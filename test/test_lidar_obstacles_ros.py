"""Live 20 Hz output gates fed independently of the segmentation worker."""

import json
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from sensor_fixtures import camera_info, cloud, header, imu
from test_apriltag_task_stop_ros import Message, RosHarness, SPORT, ZERO, debug


def sensors(node, ros, points=()):
    stamp = ros.clock_ns() / 1e9
    base = node.obstacles.base
    angles = np.linspace(-np.pi, np.pi, 1440, endpoint=False)
    ends = base[:3, 3] + np.column_stack(
        (10 * np.cos(angles), 10 * np.sin(angles), np.zeros(len(angles)))
    )
    extra = np.asarray(points, float).reshape(-1, 2)
    if len(extra):
        ends = np.vstack(
            (ends, np.column_stack((extra, np.full(len(extra), base[2, 3]))))
        )
    raw = (ends - base[:3, 3]) @ base[:3, :3]
    node.on_camera_info(camera_info(stamp))
    node.on_lidar_imu(imu(stamp, up=base[:3, :3].T @ [0, 0, 1]))
    node.on_lidar(cloud(raw, stamp))


def image(node, ros):
    message = Message(bytes(360 * 640 * 3))
    message.width, message.height, message.step = 640, 360, 640 * 3
    message.header = header(ros.clock_ns() / 1e9, "camera_optical_frame")
    node.on_image(message)


def complete_inference(node, ros):
    node.publish_state()
    previous = ros.metrics()["inference_count"]
    image(node, ros)
    deadline = time.perf_counter() + 4
    while True:
        node.publish_state()
        if ros.metrics()["inference_count"] > previous:
            return ros.metrics()
        assert time.perf_counter() < deadline, ros.errors
        time.sleep(0.001)


def enabled_ros(monkeypatch, *, full_surround=True):
    ros = RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_AVOIDANCE_ENABLED", "true")
    if full_surround:
        # These existing motion scenarios exercise side choice with observed
        # rear/side space. The restricted default sector is covered separately.
        monkeypatch.setitem(debug.ENV, "LIDAR_OBSTACLE_HORIZONTAL_RANGE_M", "6.0")
        monkeypatch.setitem(debug.ENV, "LIDAR_OBSTACLE_FOV_DEG", "360.0")
    # Avoidance input/collision gates must remain active with all legacy gates off.
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(
            debug.ENV, "LINE_TRACKING_STOP_ON_" + check.upper(), "false"
        )
    return ros


def test_default_sector_reaches_live_detection_and_keeps_rear_space_unknown(monkeypatch):
    ros = enabled_ros(monkeypatch, full_surround=False)

    def scenario(node):
        ros.start()
        ros.now = 2.1
        sensors(node, ros, [[1.4, 0], [1.6, 0], [-.8, .6], [1.2, 1.2], [-.1, 1.3], [-.2, 1.3]])
        scan, config = node.obstacles.scan, node.obstacles.config
        np.testing.assert_allclose(scan.points, [[-.1, 1.3], [1.4, 0]], atol=1e-6)
        assert config.height_reference == "base_link"
        assert config.min_height_m == -.2 and config.max_height_m == .2
        assert node.obstacles.metrics["horizontal_range_m"] == 1.5
        assert node.obstacles.metrics["fov_deg"] == 190
        rear = config.indices([[-.4, .6]])[0]
        assert not scan.observed[tuple(rear)]
        assert not any("odom" in topic for topic in ros.subscriptions)

    ros.run(scenario, "--branch-preference", "center")


@pytest.mark.parametrize("preference,sport_sign", [("right", 1), ("left", -1)])
def test_lateral_sport_command_uses_preference_without_odometry(
    monkeypatch, preference, sport_sign
):
    ros = enabled_ros(monkeypatch)

    def scenario(node):
        ros.start()
        ros.now = 2.1
        sensors(node, ros, [[2, 0]])
        result = complete_inference(node, ros)
        command = json.loads(ros.published[SPORT][-1].parameter)
        assert result["drive_reason"] == "lidar_avoiding"
        assert command["x"] > 0 and command["y"] * sport_sign > 0
        assert result["lidar_obstacles"]["nearest_obstacle_m"] == pytest.approx(
            2, abs=0.06
        )
        assert result["lidar_obstacles"]["expected_hz"] == 10
        assert result["lidar_obstacles"]["control_hz"] == 20
        node.publish_drive(debug.DriveDecision.stop("apriltag_verifying"))
        assert (
            node.obstacles.controller.last_vx == node.obstacles.controller.last_vy == 0
        )
        node.obstacles.reset_reference()
        guarded = node.publish_drive(
            debug.DriveDecision(0.2, 0, 0, "tracking_path_recovery")
        )
        assert guarded.vx == guarded.vy == 0
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(
        scenario, "--avoidance-preference", preference, "--branch-preference", "center"
    )


def test_new_cloud_stops_the_robot_while_model_inference_is_blocked(monkeypatch):
    ros = enabled_ros(monkeypatch)
    started, release = threading.Event(), threading.Event()
    calls = 0

    def segment(_frame, **_kwargs):
        nonlocal calls
        calls += 1
        if calls == 2:
            started.set()
            assert release.wait(4), "test did not release inference"
        return NS(
            selected_mask=np.full((360, 640), 2, np.uint8), inference_seconds=0.01
        )

    monkeypatch.setattr(
        debug,
        "BestSoFarSegmenter",
        lambda _: NS(device=NS(type="cuda"), reset=lambda: None, segment=segment),
    )

    def scenario(node):
        try:
            ros.start()
            ros.now = 2.1
            sensors(node, ros)
            assert complete_inference(node, ros)["drive_reason"] == "lidar_tracking"
            ros.now += 0.1
            image(node, ros)
            assert started.wait(2)
            previous = ros.metrics()["inference_count"]
            sensors(node, ros, [[0.7, 0]])
            node.publish_state()
            assert ros.metrics()["inference_count"] == previous
            assert ros.metrics()["drive_reason"].startswith("obstacle_")
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
            assert node.obstacles.metrics["obstacle_present"]
        finally:
            release.set()

    ros.run(scenario, "--branch-preference", "center")


@pytest.mark.parametrize(
    "fault", ["cloud_stale", "path_stale", "malformed_cloud", "path_lost"]
)
def test_sensor_and_path_faults_override_retained_commands(monkeypatch, fault):
    ros = enabled_ros(monkeypatch)
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_PATH_LOSS_RECOVERY_ENABLED", "true")
    detection = NS(mask=np.full((360, 640), 2, np.uint8))
    monkeypatch.setattr(
        debug,
        "BestSoFarSegmenter",
        lambda _: NS(
            device=NS(type="cuda"),
            reset=lambda: None,
            segment=lambda _frame, **kwargs: NS(
                selected_mask=detection.mask.copy(), inference_seconds=0.01
            ),
        ),
    )

    def scenario(node):
        ros.start()
        ros.now = 2.1
        sensors(node, ros)
        assert complete_inference(node, ros)["drive_reason"] == "lidar_tracking"
        if fault == "cloud_stale":
            ros.now += 0.31
        elif fault == "path_stale":
            ros.now += 2.1
            sensors(node, ros)
        elif fault == "malformed_cloud":
            packet = cloud([[2, 0, 0]], ros.clock_ns() / 1e9 + 0.01)
            packet.data = b"bad"
            node.on_lidar(packet)
        else:
            ros.now += 0.1
            sensors(node, ros)
            detection.mask[:] = 0
            complete_inference(node, ros)
        node.publish_state()
        command = json.loads(ros.published[SPORT][-1].parameter)
        assert command == ZERO
        assert ros.metrics()["drive_reason"] != "tracking_path_recovery"
        if fault == "path_stale":
            assert ros.metrics()["drive_reason"] == "avoidance_path_stale"

    ros.run(scenario, "--branch-preference", "center")


def test_enabled_avoidance_stops_on_path_loss_without_starting_scan_recovery(monkeypatch):
    ros = enabled_ros(monkeypatch)
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_PATH_LOSS_RECOVERY_ENABLED", "true")
    detection = NS(mask=np.full((360, 640), 2, np.uint8))
    monkeypatch.setattr(debug, "BestSoFarSegmenter", lambda _: NS(
        device=NS(type="cuda"), reset=lambda: None,
        segment=lambda _frame, **kwargs: NS(
            selected_mask=detection.mask.copy(), inference_seconds=.01),
    ))

    def scenario(node):
        ros.start()
        ros.now = 2.1
        sensors(node, ros)
        assert complete_inference(node, ros)["drive_reason"] == "lidar_tracking"
        detection.mask[:] = 0
        # Fresh LiDAR continues for longer than the camera-only recovery wait.
        # Neither losing the Path nor passing the wait can trigger blind rotation.
        for _ in range(15):
            ros.now += .1
            sensors(node, ros)
            result = complete_inference(node, ros)
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
            assert not result["path_recovery"]["active"]
            assert result["path_recovery"]["phase"] == "idle"
        for reason in ("path_recovery_scan_left", "path_recovery_scan_right", "path_recovery_return"):
            result = node.publish_drive(debug.DriveDecision(0, 0, .18, reason))
            assert result == debug.DriveDecision.stop("avoidance_path_unavailable")
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(scenario, "--branch-preference", "center")


def test_latest_camera_classification_constrains_visible_space_without_history(
    monkeypatch,
):
    ros = enabled_ros(monkeypatch)

    def scenario(node):
        ros.start()
        ros.now = 2.1
        sensors(node, ros)
        assert complete_inference(node, ros)["drive_reason"] == "lidar_tracking"
        _, region = node.obstacles._context(2, ros.now, ros.clock_ns())
        front, side = node.obstacles.config.indices([[2, 0], [-0.5, 0.5]])
        assert region[tuple(front)] and region[tuple(side)]
        # An updated classification replaces the old one; there is no ego-motion
        # history to keep a previously allowed visible cell alive.
        node.obstacles.set_reference(
            np.zeros((360, 640), np.uint8),
            header(ros.clock_ns() / 1e9, "camera_optical_frame"),
            (360, 640),
            ros.now,
        )
        _, current = node.obstacles._context(2, ros.now, ros.clock_ns())
        assert not current[tuple(front)]
        assert current[tuple(side)]  # Blind camera sides are checked by LiDAR.
        node.obstacles.scan.observed[:] = False
        node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(scenario, "--branch-preference", "center")


def test_no_odom_subscription_and_20hz_commands_between_10hz_scans(monkeypatch):
    ros = enabled_ros(monkeypatch)

    def scenario(node):
        ros.start()
        ros.now = 2.1
        sensors(node, ros, [[2, 0]])
        result = complete_inference(node, ros)
        assert result["drive_reason"] == "lidar_avoiding"
        assert result["lidar_obstacles"]["mode"] == "base_link_local"
        assert not any("odom" in topic for topic in ros.subscriptions)
        first = json.loads(ros.published[SPORT][-1].parameter)
        updates = node.obstacles.controller.target_updates
        ros.now += 0.05
        node.publish_state()
        second = json.loads(ros.published[SPORT][-1].parameter)
        assert second["x"] > first["x"]
        assert second["y"] > first["y"] > 0
        assert max(abs(second[k] - first[k]) for k in ("x", "y")) <= 0.015001
        assert node.obstacles.controller.target_updates == updates
        ros.now += 0.05
        sensors(node, ros, [[1.9, 0]])
        node.publish_state()
        assert node.obstacles.controller.target_updates == updates + 1
        assert ros.metrics()["drive_reason"] == "lidar_avoiding"

    ros.run(scenario, "--branch-preference", "center")


def test_blocked_scan_is_not_replanned_until_new_lidar_arrives(monkeypatch):
    ros = enabled_ros(monkeypatch)

    def scenario(node):
        ros.start()
        ros.now = 2.1
        sensors(node, ros, [[0.7, 0]])
        assert complete_inference(node, ros)["drive_reason"].startswith("obstacle_")
        count = node.obstacles.controller.target_updates
        ros.now += 0.05
        node.publish_state()
        assert node.obstacles.controller.target_updates == count
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        ros.now += 0.05
        sensors(node, ros)
        node.publish_state()
        assert node.obstacles.controller.target_updates == count + 1
        assert ros.metrics()["drive_reason"] == "lidar_tracking"

    ros.run(scenario, "--branch-preference", "center")
