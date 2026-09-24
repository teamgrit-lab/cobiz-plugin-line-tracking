"""Keep the inspection-only ROS mode limited to lightweight output topics."""

from pathlib import Path
import json
import sys
import threading
from types import ModuleType, SimpleNamespace

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import swin_l_local_path_debug as debug  # noqa: E402


def test_performance_summary_reports_latency_percentiles_and_completion_rate():
    summary = debug.summarize_performance(
        [0.10, 0.20, 0.30, 0.40],
        [0.20, 0.25, 0.30, 0.35],
        [1.0, 1.25, 1.50, 1.75],
    )

    assert summary["sample_count"] == 4
    assert summary["inference_mean_ms"] == 250.0
    assert summary["inference_p95_ms"] == 385.0
    assert summary["inference_p99_ms"] == 397.0
    assert summary["processing_mean_ms"] == 275.0
    assert summary["processing_capacity_fps"] == 1.0 / 0.275
    assert summary["completion_fps"] == 4.0
    assert summary["completion_gap_max_ms"] == 250.0
    assert summary["processing_p99_ms"] == pytest.approx(348.5)


@pytest.mark.parametrize("value", ["nan", "inf", "0", "-1"])
def test_live_inference_rejects_invalid_rate(value):
    with pytest.raises(SystemExit):
        debug.parse_args(["ros2", "--inference-hz", value])


def test_task_drive_defaults(monkeypatch):
    for name in tuple(debug.ENV):
        if name.startswith("SWIN_L_APRILTAG_"):
            monkeypatch.delitem(debug.ENV, name)
    monkeypatch.delitem(debug.ENV, "LINE_TRACKING_DEFAULT_DURATION_SEC", raising=False)
    monkeypatch.delitem(debug.ENV, "LINE_TRACKING_MAX_DURATION_SEC", raising=False)
    args = debug.parse_args(["task-drive"])
    assert args.apriltag_detections_topic == "/detections"
    assert args.apriltag_max_age_sec == 1.0
    assert args.apriltag_confirm_window_sec == 1.0
    assert args.apriltag_confirm_min_hits == 3
    assert args.default_task_duration_sec == 500.0
    assert args.max_task_duration_sec == 1000.0
    assert not hasattr(args, "drive_enabled")
    assert not hasattr(args, "calibration_confirmed")
    assert not hasattr(args, "safety_topic")
    assert not hasattr(args, "clearance_topic")


def test_task_drive_apriltag_environment_and_cli(monkeypatch):
    monkeypatch.setitem(debug.ENV, "SWIN_L_APRILTAG_DETECTIONS_TOPIC", "/tags")
    monkeypatch.setitem(debug.ENV, "SWIN_L_APRILTAG_MAX_AGE_SEC", "0.8")
    monkeypatch.setitem(debug.ENV, "SWIN_L_APRILTAG_CONFIRM_WINDOW_SEC", "1.5")
    monkeypatch.setitem(debug.ENV, "SWIN_L_APRILTAG_CONFIRM_MIN_HITS", "4")
    args = debug.parse_args(["task-drive"])
    assert (
        args.apriltag_detections_topic,
        args.apriltag_max_age_sec,
        args.apriltag_confirm_window_sec,
        args.apriltag_confirm_min_hits,
    ) == ("/tags", 0.8, 1.5, 4)
    args = debug.parse_args(["task-drive", "--apriltag-confirm-min-hits", "5"])
    assert args.apriltag_confirm_min_hits == 5


@pytest.mark.parametrize("check", ["low_confidence", "lateral_target", "apriltag"])
def test_stop_switch_environment_and_cli_override(monkeypatch, check):
    env_name = "LINE_TRACKING_STOP_ON_" + check.upper()
    monkeypatch.setitem(debug.ENV, env_name, "false")
    args = debug.parse_args(["task-drive"])
    assert getattr(args, "stop_on_" + check) is False

    args = debug.parse_args(["task-drive", "--stop-on-" + check.replace("_", "-")])
    assert getattr(args, "stop_on_" + check) is True

    monkeypatch.setitem(debug.ENV, env_name, "invalid")
    with pytest.raises(ValueError, match="must be a boolean"):
        debug.parse_args(["task-drive"])


@pytest.mark.parametrize("unrestricted", [True, False])
def test_debug_mode_limits_inference_and_only_publishes_path_metrics(
    monkeypatch,
    unrestricted,
):
    published_topics = []
    subscribed_topics = []
    published_messages = {}

    class FakeNode:
        def __init__(self, _name):
            pass

        def create_subscription(self, _type, topic, _callback, _qos):
            subscribed_topics.append(topic)
            return object()

        def create_publisher(self, _type, topic, _qos):
            published_topics.append(topic)
            messages = published_messages.setdefault(topic, [])
            return SimpleNamespace(publish=messages.append)

        def create_timer(self, *_args):
            return object()

        def get_logger(self):
            return SimpleNamespace(
                info=lambda _message: None, warning=lambda _message: None
            )

        def destroy_node(self):
            pass

    rclpy = ModuleType("rclpy")
    rclpy.init = lambda **_kwargs: None
    deadlines = []
    worker_finished = threading.Event()

    class Queue:
        overwritten = 0

        def get_latest_at(self, deadline):
            deadlines.append(deadline)
            if len(deadlines) == 1:
                return debug.FramePacket(debug.np.zeros((12, 12, 3)), 1)
            worker_finished.set()
            return None

        def close(self):
            pass

    def spin(node):
        assert worker_finished.wait(timeout=5)
        node._publish_state()

    monkeypatch.setattr(debug, "LatestFrameQueue", Queue)
    rclpy.ok = lambda: True
    rclpy.spin = spin
    rclpy.shutdown = lambda: None
    node_module = ModuleType("rclpy.node")
    node_module.Node = FakeNode
    qos_module = ModuleType("rclpy.qos")
    qos_module.HistoryPolicy = SimpleNamespace(KEEP_LAST=object())
    qos_module.ReliabilityPolicy = SimpleNamespace(
        RELIABLE=object(), BEST_EFFORT=object()
    )
    qos_module.QoSProfile = lambda **_kwargs: object()
    cv_bridge = ModuleType("cv_bridge")
    cv_bridge.CvBridge = object
    sensor_msgs = ModuleType("sensor_msgs.msg")
    sensor_msgs.Image = type("Image", (), {})
    std_msgs = ModuleType("std_msgs.msg")
    std_msgs.String = lambda **kwargs: SimpleNamespace(**kwargs)
    nav_msgs = ModuleType("nav_msgs.msg")
    nav_msgs.Path = type("Path", (), {})
    for name, module in (
        ("rclpy", rclpy),
        ("rclpy.node", node_module),
        ("rclpy.qos", qos_module),
        ("cv_bridge", cv_bridge),
        ("sensor_msgs.msg", sensor_msgs),
        ("std_msgs.msg", std_msgs),
        ("nav_msgs.msg", nav_msgs),
    ):
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(
        debug,
        "BestSoFarSegmenter",
        lambda _config: SimpleNamespace(
            device=debug.torch.device("cpu"),
            segment=lambda _frame: SimpleNamespace(
                selected_mask=debug.np.zeros((12, 12), dtype=debug.np.uint8),
                inference_seconds=0.01,
            ),
        ),
    )

    args = debug.parse_args(
        [
            "ros2",
            "--inference-hz",
            "1.25",
            "--unrestricted-path-mode"
            if unrestricted
            else "--no-unrestricted-path-mode",
        ]
    )
    assert not hasattr(args, "overlay_topic")
    assert not hasattr(args, "clearance_topic")
    assert debug.run_ros2(args) == 0
    assert len(deadlines) == 2
    assert deadlines[1] - deadlines[0] >= 0.8
    assert published_topics == [
        args.local_path_topic,
        args.metrics_topic,
    ]

    task_args = debug.parse_args(["task-drive"])
    assert not hasattr(task_args, "overlay_topic")
    assert not hasattr(task_args, "safety_topic")
    assert not hasattr(task_args, "clearance_topic")
    assert subscribed_topics == [args.image_topic]
    metrics = json.loads(published_messages[args.metrics_topic][-1].data)
    assert metrics["path_tracked"] is False
    assert "lidar" not in metrics
    assert "lidar_topic" not in metrics
