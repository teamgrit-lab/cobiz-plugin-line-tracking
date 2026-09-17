"""Keep the inspection-only ROS mode limited to lightweight output topics."""

from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import swin_l_local_path_debug as debug  # noqa: E402


def test_debug_mode_creates_only_path_metrics_and_safety_publishers(monkeypatch):
    published_topics = []

    class FakeNode:
        def __init__(self, _name):
            pass

        def create_subscription(self, *_args):
            return object()

        def create_publisher(self, _type, topic, _qos):
            published_topics.append(topic)
            return SimpleNamespace(publish=lambda _message: None)

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
    rclpy.spin = lambda _node: None
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
    sensor_msgs.Joy = type("Joy", (), {})
    sensor_msgs.PointCloud2 = type("PointCloud2", (), {})
    std_msgs = ModuleType("std_msgs.msg")
    std_msgs.Bool = type("Bool", (), {})
    std_msgs.Float32 = type("Float32", (), {})
    std_msgs.String = type("String", (), {})
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
    monkeypatch.setattr(debug, "BestSoFarSegmenter", lambda _config: object())

    args = debug.parse_args(["ros2"])
    assert not hasattr(args, "overlay_topic")
    assert not hasattr(args, "clearance_topic")
    assert debug.run_ros2(args) == 0
    assert published_topics == [
        args.local_path_topic,
        args.safety_stop_topic,
        args.metrics_topic,
    ]

    task_args = debug.parse_args(["task-drive"])
    assert task_args.overlay_topic
    assert task_args.clearance_topic
