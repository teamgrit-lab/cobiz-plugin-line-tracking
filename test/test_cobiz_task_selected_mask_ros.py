"""Check that Cobiz's requested surface reaches live task control."""

from dataclasses import replace
import json
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace

import pytest
import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import swin_l_local_path_debug as debug  # noqa: E402
from swin_l_drive_control import DriveDecision  # noqa: E402


@pytest.mark.parametrize(
    ("preexisting_control_publishers", "expected_state"),
    [(0, "TASK_STARTED"), (1, "TASK_REJECTED")],
)
def test_task_control_rejects_external_publisher_before_readiness(
    monkeypatch, preexisting_control_publishers, expected_state
):
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

        def count_publishers(self, topic):
            assert topic == "/api/sport/request"
            return preexisting_control_publishers

        def create_timer(self, *_args):
            return object()

        def get_logger(self):
            return SimpleNamespace(info=lambda _: None, warning=lambda _: None)

        def get_clock(self):
            return SimpleNamespace(
                now=lambda: SimpleNamespace(
                    nanoseconds=100_000_000_000, to_msg=lambda: SimpleNamespace()
                )
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
        if not preexisting_control_publishers:
            _, decision = node.drive_readiness(1, time.monotonic())
            assert decision.reason == "apriltag_detections_stale"
            assert (decision.vx, decision.vy, decision.yaw_rate) == (0.0, 0.0, 0.0)
            camera = Message()
            camera.header.stamp = SimpleNamespace(sec=100, nanosec=0)
            node.on_image(camera)
            deadline = time.monotonic() + 2.0
            while not published["/line_tracking/swin_l/overlay"]:
                node._publish_state()
                assert time.monotonic() < deadline, (
                    "inference did not publish an overlay"
                )
                time.sleep(0.001)
            overlay = published["/line_tracking/swin_l/overlay"][-1]
            assert overlay.frame.shape == (360, 640, 3)
        checked_classes = []

        def readiness(mask_class, _now):
            checked_classes.append(mask_class)
            decision = (
                DriveDecision(0.1, 0.0, 0.0, "tracking")
                if mask_class == 1
                else DriveDecision.stop("path_unavailable")
            )
            return None, decision

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
        state = json.loads(published["/task_state"][-1].data)
        assert state["type"] == expected_state
        if preexisting_control_publishers:
            assert state["reason"] == "multiple_control_publishers"
            assert node.tasks.active is None
            assert node.command_publisher is None
            assert checked_classes == []
            return
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
        assert checked_classes == [1, 2]

    rclpy.spin = spin
    modules = {
        "rclpy": rclpy,
        "rclpy.node": SimpleNamespace(Node=FakeNode),
        "rclpy.qos": SimpleNamespace(
            HistoryPolicy=SimpleNamespace(KEEP_LAST=object()),
            ReliabilityPolicy=SimpleNamespace(RELIABLE=reliable, BEST_EFFORT=object()),
            DurabilityPolicy=SimpleNamespace(VOLATILE=object()),
            QoSProfile=FakeQoS,
        ),
        "cv_bridge": SimpleNamespace(
            CvBridge=lambda: SimpleNamespace(
                imgmsg_to_cv2=lambda *_args, **_kwargs: np.zeros(
                    (360, 640, 3), np.uint8
                ),
                cv2_to_imgmsg=lambda frame, **_kwargs: SimpleNamespace(frame=frame),
            )
        ),
        "sensor_msgs.msg": SimpleNamespace(Image=Message),
        "std_msgs.msg": SimpleNamespace(String=Message),
        "nav_msgs.msg": SimpleNamespace(Path=Message),
        "unitree_api.msg": SimpleNamespace(Request=Request),
        "apriltag_msgs.msg": SimpleNamespace(AprilTagDetectionArray=Message),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(
        debug,
        "BestSoFarSegmenter",
        lambda _config: SimpleNamespace(
            device=SimpleNamespace(type="cuda"),
            segment=lambda _frame: SimpleNamespace(
                selected_mask=np.zeros((360, 640), np.uint8)
            ),
        ),
    )
    monkeypatch.setattr(debug, "_path_message", lambda *_args: Message())
    monkeypatch.setitem(debug.ENV, "SWIN_L_PATH_MASK_CLASS", "2")

    assert debug.run_ros2(debug.parse_args(["task-drive"])) == 0
