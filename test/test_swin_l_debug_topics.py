"""Keep the inspection-only ROS mode limited to lightweight output topics."""

from pathlib import Path
import json
import sys
from types import ModuleType, SimpleNamespace


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


def test_debug_mode_creates_only_camera_subscription_and_path_metrics_publishers(
    monkeypatch,
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
    rclpy.ok = lambda: False
    rclpy.spin = lambda node: node._publish_state()
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
        lambda _config: SimpleNamespace(device=debug.torch.device("cpu")),
    )

    args = debug.parse_args(["ros2"])
    assert not hasattr(args, "overlay_topic")
    assert not hasattr(args, "clearance_topic")
    assert debug.run_ros2(args) == 0
    assert published_topics == [
        args.local_path_topic,
        args.metrics_topic,
    ]

    task_args = debug.parse_args(["task-drive"])
    assert task_args.overlay_topic
    assert not hasattr(task_args, "safety_topic")
    assert not hasattr(task_args, "clearance_topic")
    assert subscribed_topics == [args.image_topic]
    metrics = json.loads(published_messages[args.metrics_topic][-1].data)
    assert metrics["path_tracked"] is False
    assert "lidar" not in metrics
    assert "lidar_topic" not in metrics
