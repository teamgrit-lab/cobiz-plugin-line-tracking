"""Check that Cobiz's requested surface reaches live task control."""

from dataclasses import replace
import json
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import swin_l_local_path_debug as debug  # noqa: E402
from local_path import LidarSafetyResult  # noqa: E402
from swin_l_drive_control import DriveDecision  # noqa: E402


def test_task_payload_selects_road_for_preflight_and_control(monkeypatch):
    published: dict[str, list] = {}
    publisher_qos = {}

    reliable = object()

    class FakeQoS:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class Message:
        def __init__(self, data=None):
            self.data = data
            self.header = SimpleNamespace()

    class Request:
        def __init__(self):
            self.header = SimpleNamespace(identity=SimpleNamespace(api_id=0))
            self.parameter = ""
            self.binary = []

    class FakeNode:
        def __init__(self, _name):
            pass

        def has_parameter(self, _name):
            return False

        def create_subscription(self, *_args):
            return object()

        def create_publisher(self, _type, topic, _qos):
            messages = published.setdefault(topic, [])
            publisher_qos[topic] = _qos
            return SimpleNamespace(publish=messages.append)

        def create_timer(self, *_args):
            return object()

        def get_logger(self):
            return SimpleNamespace(info=lambda _: None, warning=lambda _: None)

        def get_clock(self):
            return SimpleNamespace(
                now=lambda: SimpleNamespace(to_msg=lambda: SimpleNamespace())
            )

        def destroy_publisher(self, _publisher):
            pass

        def destroy_node(self):
            pass

    rclpy = ModuleType("rclpy")
    rclpy.init = lambda **_kwargs: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda: None

    def spin(node):
        safety = LidarSafetyResult(False, True, False, 0, None, 0.0, "clear")
        checked_classes = []

        def readiness(mask_class, _now):
            checked_classes.append(mask_class)
            decision = (
                DriveDecision(0.1, 0.0, 0.0, "tracking")
                if mask_class == 1
                else DriveDecision.stop("path_unavailable")
            )
            return None, safety, decision

        node.drive_readiness = readiness
        node.on_task_event(
            Message(
                json.dumps(
                    {
                        "type": "TASK_REGISTERED",
                        "action_name": "LINE_TRACKING",
                        "task_id": "road-1",
                        "payload": {"selected_mask": 1, "duration_sec": 30},
                    }
                )
            )
        )
        assert json.loads(published["/task_state"][-1].data)["type"] == "TASK_STARTED"
        assert node.tasks.active.selected_mask == 1
        node.tasks.active = replace(node.tasks.active, started_at=time.monotonic() - 3)
        node.publish_state()
        assert (
            json.loads(published["/line_tracking/swin_l/metrics"][-1].data)[
                "path_mask_class"
            ]
            == 1
        )
        move = published["/api/sport/request"][-1]
        assert move.header.identity.api_id == 1008
        assert json.loads(move.parameter) == {"x": 0.1, "y": 0.0, "z": 0.0}
        assert publisher_qos["/api/sport/request"].reliability is reliable
        assert "/a2_control" not in published
        node.on_task_event(
            Message(json.dumps({"type": "TASK_ABORTED", "task_id": "road-1"}))
        )
        node.publish_state()
        assert (
            json.loads(published["/line_tracking/swin_l/metrics"][-1].data)[
                "path_mask_class"
            ]
            == 2
        )
        assert json.loads(published["/api/sport/request"][-1].parameter) == {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
        }
        assert checked_classes == [1, 1, 2]

    rclpy.spin = spin
    modules = {
        "rclpy": rclpy,
        "rclpy.node": SimpleNamespace(Node=FakeNode),
        "rclpy.qos": SimpleNamespace(
            HistoryPolicy=SimpleNamespace(KEEP_LAST=object()),
            ReliabilityPolicy=SimpleNamespace(RELIABLE=reliable, BEST_EFFORT=object()),
            QoSProfile=FakeQoS,
        ),
        "cv_bridge": SimpleNamespace(CvBridge=object),
        "sensor_msgs.msg": SimpleNamespace(Image=Message, PointCloud2=Message),
        "std_msgs.msg": SimpleNamespace(Bool=Message, Float32=Message, String=Message),
        "nav_msgs.msg": SimpleNamespace(Path=Message),
        "unitree_api.msg": SimpleNamespace(Request=Request),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(
        debug,
        "BestSoFarSegmenter",
        lambda _config: SimpleNamespace(device=SimpleNamespace(type="cuda")),
    )
    monkeypatch.setitem(debug.ENV, "SWIN_L_DRIVE_ENABLED", "true")
    monkeypatch.setitem(debug.ENV, "SWIN_L_CALIBRATION_CONFIRMED", "true")
    monkeypatch.setitem(debug.ENV, "SWIN_L_PATH_MASK_CLASS", "2")

    assert debug.run_ros2(debug.parse_args(["task-drive"])) == 0
