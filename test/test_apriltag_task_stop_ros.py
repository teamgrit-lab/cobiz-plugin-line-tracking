"""Exercise the live node with generated ROS and model boundaries replaced."""

import json
import signal
import sys
import time
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import swin_l_local_path_debug as debug

SPORT = "/api/sport/request"
METRICS = "/line_tracking/swin_l/metrics"
ZERO = {"x": 0.0, "y": 0.0, "z": 0.0}


class Message:
    def __init__(self, data=None):
        self.data = data
        self.header = SimpleNamespace(
            stamp=SimpleNamespace(sec=0, nanosec=0), frame_id="camera"
        )
        self.poses = []


class AprilTagDetectionArray(Message):
    pass


class Request:
    def __init__(self):
        self.header = SimpleNamespace(identity=SimpleNamespace(api_id=0))
        self.parameter = ""
        self.binary = []


class PoseStamped(Message):
    def __init__(self):
        super().__init__()
        self.pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
        )


class FakeQoS:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class RosHarness:
    def __init__(self, monkeypatch):
        self.now = 0.0
        self.external_publishers = 0
        self.published = {}
        self.events = []
        self.subscriptions = {}
        self.subscription_types = {}
        self.subscription_qos = {}
        self.errors = []
        self.running = True
        self.reliable, self.volatile, self.keep_last = object(), object(), object()
        harness = self

        class FakeNode:
            def __init__(self, _name):
                harness.node = self

            def has_parameter(self, _name):
                return False

            def create_subscription(self, message_type, topic, callback, qos):
                harness.subscriptions[topic] = callback
                harness.subscription_types[topic] = message_type
                harness.subscription_qos[topic] = qos
                return object()

            def create_publisher(self, _message_type, topic, _qos):
                messages = harness.published.setdefault(topic, [])

                def publish(message):
                    messages.append(message)
                    harness.events.append((topic, message))

                return SimpleNamespace(publish=publish)

            def count_publishers(self, topic):
                assert topic == SPORT
                return harness.external_publishers + int(
                    self.command_publisher is not None
                )

            def create_timer(self, *_args):
                return object()

            def get_logger(self):
                return SimpleNamespace(
                    info=lambda _: None,
                    warning=lambda _: None,
                    error=harness.errors.append,
                )

            def get_clock(self):
                return SimpleNamespace(
                    now=lambda: SimpleNamespace(
                        nanoseconds=harness.clock_ns(), to_msg=harness.stamp
                    )
                )

            def destroy_publisher(self, _publisher):
                harness.events.append(("publisher_destroyed", None))

            def destroy_node(self):
                pass

        self.rclpy = ModuleType("rclpy")
        self.rclpy.init = lambda **_kwargs: None
        self.rclpy.ok = lambda: self.running
        self.rclpy.shutdown = lambda: setattr(self, "running", False)
        modules = {
            "rclpy": self.rclpy,
            "rclpy.node": SimpleNamespace(Node=FakeNode),
            "rclpy.qos": SimpleNamespace(
                HistoryPolicy=SimpleNamespace(KEEP_LAST=self.keep_last),
                ReliabilityPolicy=SimpleNamespace(
                    RELIABLE=self.reliable, BEST_EFFORT=object()
                ),
                DurabilityPolicy=SimpleNamespace(VOLATILE=self.volatile),
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
            "geometry_msgs.msg": SimpleNamespace(PoseStamped=PoseStamped),
            "unitree_api.msg": SimpleNamespace(Request=Request),
            "apriltag_msgs.msg": SimpleNamespace(
                AprilTagDetectionArray=AprilTagDetectionArray
            ),
        }
        for name, module in modules.items():
            monkeypatch.setitem(sys.modules, name, module)
        monkeypatch.setattr(debug.time, "monotonic", lambda: self.now)
        monkeypatch.setattr(
            debug,
            "BestSoFarSegmenter",
            lambda _config: SimpleNamespace(
                device=SimpleNamespace(type="cuda"),
                segment=lambda _frame: SimpleNamespace(
                    selected_mask=np.full((360, 640), 2, np.uint8)
                ),
            ),
        )

    def clock_ns(self):
        return 100_000_000_000 + round(self.now * 1_000_000_000)

    def stamp(self):
        seconds, nanoseconds = divmod(self.clock_ns(), 1_000_000_000)
        return SimpleNamespace(sec=seconds, nanosec=nanoseconds)

    def run(self, scenario, *args):
        def spin(node):
            scenario(node)
            assert not self.errors

        self.rclpy.spin = spin
        assert (
            debug.run_ros2(
                debug.parse_args(["task-drive", "--inference-hz", "1000", *args])
            )
            == 0
        )

    def start(self, task_id="tag-stop-1", duration=30):
        self.subscriptions["/task_event"](
            Message(
                json.dumps(
                    {
                        "type": "TASK_REGISTERED",
                        "action_name": "LINE_TRACKING",
                        "task_id": task_id,
                        "payload": {"duration_sec": duration},
                    }
                )
            )
        )
        assert self.task_state()["type"] == "TASK_STARTED"

    def detect(self, tag_id=None, frame=1):
        assert "/detections" in self.subscriptions
        message = AprilTagDetectionArray()
        message.header.stamp.nanosec = frame
        message.detections = [] if tag_id is None else [SimpleNamespace(id=tag_id)]
        self.subscriptions["/detections"](message)

    def task_state(self):
        return json.loads(self.published["/task_state"][-1].data)

    def metrics(self):
        return json.loads(self.published[METRICS][-1].data)

    def hard_stop_before_task_state(self, event_start, state_type, reason):
        events = [
            (topic, message)
            for topic, message in self.events[event_start:]
            if topic in (SPORT, "/task_state")
        ]
        assert [topic for topic, _ in events[:3]] == [SPORT, SPORT, "/task_state"]
        assert [message.header.identity.api_id for _, message in events[:2]] == [
            1003,
            1008,
        ]
        assert json.loads(events[1][1].parameter) == ZERO
        assert json.loads(events[2][1].data)["type"] == state_type
        assert json.loads(events[2][1].data)["reason"] == reason

    def camera_ready(self):
        message = Message()
        message.header.stamp = self.stamp()
        self.subscriptions[debug.DEFAULT_IMAGE_TOPIC](message)
        deadline = time.perf_counter() + 3.0
        while True:
            path, decision = self.node.drive_readiness(2, self.now)
            if decision.reason == "tracking":
                assert path is not None
                return
            assert time.perf_counter() < deadline, decision.reason
            time.sleep(0.001)

    def establish_tracking(self, duration=30):
        self.start(duration=duration)
        self.now = 2.1
        self.detect()
        self.camera_ready()
        self.node.publish_state()
        assert self.metrics()["drive_reason"] == "tracking"
        assert json.loads(self.published[SPORT][-1].parameter)["x"] == 0.1


@pytest.fixture
def ros(monkeypatch):
    return RosHarness(monkeypatch)


@pytest.mark.parametrize("confirm_in_callback", [False, True])
def test_candidate_hard_stops_immediately_and_confirms_after_full_window(
    ros, confirm_in_callback
):
    def scenario(node):
        ros.start()
        qos = ros.subscription_qos.get("/detections")
        assert qos is not None
        assert ros.subscription_types["/detections"] is AprilTagDetectionArray
        assert (qos.depth, qos.reliability, qos.durability, qos.history) == (
            1,
            ros.reliable,
            ros.volatile,
            ros.keep_last,
        )
        candidate_start = len(ros.published[SPORT])
        for frame, stamp in enumerate((0.0, 0.1, 0.2), start=1):
            ros.now = stamp
            ros.detect(tag_id=7, frame=frame)
            if frame == 1:
                assert [
                    message.header.identity.api_id
                    for message in ros.published[SPORT][candidate_start:]
                ] == [1003, 1008]
            node.publish_state()
            assert ros.task_state()["type"] == "TASK_STARTED"
            assert ros.metrics()["drive_reason"] == "apriltag_verifying"
        ros.now = 0.999
        node.publish_state()
        assert ros.task_state()["type"] == "TASK_STARTED"
        assert all(
            json.loads(message.parameter) == ZERO
            for message in ros.published[SPORT][candidate_start:]
            if message.header.identity.api_id == 1008
        )
        ros.now = 1.0
        completion_start = len(ros.events)
        if confirm_in_callback:
            ros.detect(frame=4)
        else:
            node.publish_state()
        ros.hard_stop_before_task_state(
            completion_start, "TASK_COMPLETED", "apriltag_confirmed:7"
        )
        assert node.tasks.active is None
        assert node.command_publisher is not None
        ros.now = 1.9
        ros.detect(frame=5)
        node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert node.command_publisher is not None
        ros.now = 2.0
        node.publish_state()
        assert node.command_publisher is None
        assert ros.task_state()["type"] == "TASK_COMPLETED"
        assert len(ros.published["/task_state"]) == 2

    ros.run(scenario)


def test_verification_defers_startup_failure_until_its_full_window(ros):
    def scenario(node):
        ros.start()
        for frame, stamp in enumerate((1.9, 2.0, 2.1), start=1):
            ros.now = stamp
            ros.detect(tag_id=7, frame=frame)
            node.publish_state()
            assert ros.task_state()["type"] == "TASK_STARTED"
        ros.now = 2.9
        node.publish_state()
        assert ros.task_state()["type"] == "TASK_COMPLETED"
        assert ros.task_state()["reason"] == "apriltag_confirmed:7"

    ros.run(scenario)


def test_empty_heartbeat_permits_real_path_tracking(ros):
    ros.run(lambda _node: ros.establish_tracking())


def test_false_positive_resumes_only_after_camera_and_path_recover(ros):
    def scenario(node):
        ros.establish_tracking()
        ros.now = 2.2
        ros.detect(tag_id=7)
        candidate_start = len(ros.published[SPORT])
        ros.now = 2.9
        ros.detect(frame=2)
        node.publish_state()
        assert ros.metrics()["drive_reason"] == "apriltag_verifying"
        ros.now = 3.2
        ros.detect(frame=3)
        node.publish_state()
        assert ros.metrics()["drive_reason"] == "camera_stale"
        assert ros.task_state()["type"] == "TASK_STARTED"
        assert all(
            json.loads(message.parameter) == ZERO
            for message in ros.published[SPORT][candidate_start:]
        )
        ros.now = 3.21
        ros.camera_ready()
        node.publish_state()
        assert ros.metrics()["drive_reason"] == "tracking"
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] == 0.1

    ros.run(scenario)


@pytest.mark.parametrize("verifying", [False, True])
def test_stale_stream_hard_stops_and_aborts_without_unsafe_timeout(ros, verifying):
    def scenario(node):
        ros.establish_tracking()
        if verifying:
            ros.now = 2.2
            ros.detect(tag_id=7)
        ros.now = 3.21
        stop_start = len(ros.events)
        node.publish_state()
        ros.hard_stop_before_task_state(
            stop_start, "TASK_ABORTED", "apriltag_detections_stale"
        )
        assert node.tasks.active is None
        ros.now = 3.3
        ros.detect()
        ros.camera_ready()
        node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert ros.task_state()["type"] == "TASK_ABORTED"

    ros.run(scenario, "--apriltag-confirm-window-sec", "2.0")


@pytest.mark.parametrize("phase", ["startup", "verifying", "tracking"])
def test_competing_publisher_hard_stops_immediately_in_every_active_phase(ros, phase):
    def scenario(node):
        if phase == "tracking":
            ros.establish_tracking()
        else:
            ros.start()
            if phase == "verifying":
                ros.detect(tag_id=7)
        ros.now += 0.1
        ros.external_publishers = 1
        stop_start = len(ros.events)
        node.publish_state()
        ros.hard_stop_before_task_state(
            stop_start, "TASK_ABORTED", "multiple_control_publishers"
        )
        assert node.tasks.active is None

    ros.run(scenario)


def test_completion_precedes_new_competing_publisher_at_window_deadline(ros):
    def scenario(node):
        ros.start()
        for frame, stamp in enumerate((0.0, 0.1, 0.2), start=1):
            ros.now = stamp
            ros.detect(tag_id=7, frame=frame)
        ros.now = 1.0
        ros.external_publishers = 1
        stop_start = len(ros.events)
        node.publish_state()
        ros.hard_stop_before_task_state(
            stop_start, "TASK_COMPLETED", "apriltag_confirmed:7"
        )

    ros.run(scenario)


@pytest.mark.parametrize("candidate", [False, True])
def test_received_then_lost_stream_aborts_even_before_tracking(ros, candidate):
    def scenario(node):
        ros.start()
        ros.detect(tag_id=7 if candidate else None)
        ros.now = 1.01
        stop_start = len(ros.events)
        node.publish_state()
        ros.hard_stop_before_task_state(
            stop_start, "TASK_ABORTED", "apriltag_detections_stale"
        )

    ros.run(scenario, "--apriltag-confirm-window-sec", "2.0")


def test_never_received_stream_gets_the_full_startup_hold(ros):
    def scenario(node):
        ros.start()
        ros.now = 1.99
        node.publish_state()
        assert ros.task_state()["type"] == "TASK_STARTED"
        ros.now = 2.0
        node.publish_state()
        assert ros.task_state()["type"] == "TASK_ABORTED"
        assert ros.task_state()["reason"] == "startup:apriltag_detections_stale"
        assert all(
            json.loads(message.parameter) == ZERO
            for message in ros.published[SPORT]
            if message.header.identity.api_id == 1008
        )

    ros.run(scenario)


@pytest.mark.parametrize("source", ["server", "sigterm"])
def test_server_abort_and_sigterm_use_ordered_hard_stop(ros, source):
    def scenario(node):
        ros.start()
        stop_start = len(ros.events)
        if source == "server":
            ros.subscriptions["/task_event"](
                Message(json.dumps({"type": "TASK_ABORTED", "task_id": "tag-stop-1"}))
            )
            reason = "task_aborted_by_server"
        else:
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
            reason = "sigterm"
        ros.hard_stop_before_task_state(stop_start, "TASK_ABORTED", reason)
        assert node.tasks.active is None

    ros.run(scenario)


def test_tag_metrics_and_overlay_report_verification_and_confirmation(ros, monkeypatch):
    texts = []
    put_text = debug.cv2.putText

    def capture_text(frame, text, *args, **kwargs):
        texts.append(text)
        return put_text(frame, text, *args, **kwargs)

    monkeypatch.setattr(debug.cv2, "putText", capture_text)

    def scenario(node):
        node.publish_state()
        assert ros.metrics()["apriltag"] == {
            "topic": "/detections",
            "stream_ready": False,
            "message_age_sec": None,
            "state": "no_tag",
            "hit_counts": {},
            "confirmed_id": None,
            "window_elapsed_sec": None,
        }
        ros.start()
        ros.detect(tag_id=7, frame=1)
        ros.now = 0.1
        ros.detect(tag_id=7, frame=2)
        ros.camera_ready()
        node.publish_state()
        assert ros.metrics()["apriltag"] == {
            "topic": "/detections",
            "stream_ready": True,
            "message_age_sec": 0.0,
            "state": "verifying",
            "hit_counts": {"7": 2},
            "confirmed_id": None,
            "window_elapsed_sec": 0.1,
        }
        assert "APRILTAG VERIFY 2/3" in texts
        assert ros.published["/line_tracking/swin_l/overlay"][-1].frame.shape == (
            360,
            640,
            3,
        )
        ros.now = 0.2
        ros.detect(tag_id=7, frame=3)
        ros.now = 1.0
        node.publish_state()
        assert ros.metrics()["apriltag"]["state"] == "confirmed"
        assert ros.metrics()["apriltag"]["confirmed_id"] == 7
        assert ros.metrics()["apriltag"]["message_age_sec"] == pytest.approx(0.8)
        assert "APRILTAG CONFIRMED ID 7" in texts
        assert "lidar" not in ros.metrics()

    ros.run(scenario)


@pytest.mark.parametrize(
    ("frames", "expected_type"),
    [((9, 9, 9), "TASK_STARTED"), ((0, 0, 0), "TASK_COMPLETED")],
)
def test_header_frame_identity_deduplicates_and_zero_stamps_use_callback_sequence(
    ros, frames, expected_type
):
    def scenario(node):
        ros.start()
        for stamp, frame in zip((0.0, 0.1, 0.2), frames):
            ros.now = stamp
            ros.detect(tag_id=7, frame=frame)
        ros.now = 1.0
        ros.detect(frame=10)
        node.publish_state()
        assert ros.task_state()["type"] == expected_type
        assert all(
            json.loads(message.parameter) == ZERO
            for message in ros.published[SPORT]
            if message.header.identity.api_id == 1008
        )

    ros.run(scenario)


@pytest.mark.parametrize("tag_id", [None, 7])
def test_idle_detections_only_seed_verification_once_a_task_starts(ros, tag_id):
    def scenario(node):
        ros.detect(tag_id=tag_id)
        assert SPORT not in ros.published
        node.publish_state()
        assert ros.metrics()["apriltag"]["state"] == "no_tag"
        ros.start()
        ros.now = 0.1
        node.publish_state()
        assert ros.metrics()["apriltag"]["state"] == (
            "no_tag" if tag_id is None else "verifying"
        )
        assert ros.metrics()["apriltag"]["stream_ready"] is True
        assert ros.metrics()["drive_reason"] == (
            "startup_hold" if tag_id is None else "apriltag_verifying"
        )

    ros.run(scenario)


def test_release_hold_blocks_registration_then_new_task_preserves_heartbeat(ros):
    def scenario(node):
        ros.start()
        for frame, stamp in enumerate((0.0, 0.1, 0.2), start=1):
            ros.now = stamp
            ros.detect(tag_id=7, frame=frame)
        ros.now = 1.0
        node.publish_state()
        ros.subscriptions["/task_event"](
            Message(
                json.dumps(
                    {
                        "type": "TASK_REGISTERED",
                        "action_name": "LINE_TRACKING",
                        "task_id": "hold-rejected",
                        "payload": {"duration_sec": 30},
                    }
                )
            )
        )
        assert ros.task_state()["type"] == "TASK_REJECTED"
        assert ros.task_state()["reason"] == "control_release_pending"
        ros.now = 1.9
        ros.detect()
        ros.now = 2.0
        node.publish_state()
        assert node.command_publisher is None
        ros.start("new-task")
        ros.camera_ready()
        node.publish_state()
        assert ros.metrics()["ready_reason"] == "tracking"
        assert ros.metrics()["apriltag"]["state"] == "no_tag"
        assert ros.metrics()["apriltag"]["message_age_sec"] == pytest.approx(0.1)
        assert ros.metrics()["drive_reason"] == "startup_hold"

    ros.run(scenario)


def test_cached_candidate_stops_on_start_and_observes_a_new_full_window(ros):
    def scenario(node):
        ros.detect(tag_id=7, frame=9)
        ros.now = 0.5
        ros.start()
        assert [m.header.identity.api_id for m in ros.published[SPORT]] == [1003, 1008]
        ros.now = 0.6
        ros.detect(tag_id=7, frame=9)
        node.publish_state()
        assert ros.metrics()["apriltag"]["hit_counts"] == {"7": 1}
        for frame, stamp in ((10, 0.7), (11, 0.8)):
            ros.now = stamp
            ros.detect(tag_id=7, frame=frame)
        ros.now = 1.49
        node.publish_state()
        assert ros.task_state()["type"] == "TASK_STARTED"
        ros.now = 1.5
        node.publish_state()
        assert ros.task_state()["type"] == "TASK_COMPLETED"
        assert ros.task_state()["reason"] == "apriltag_confirmed:7"

    ros.run(scenario)


def test_cached_candidate_does_not_refresh_detector_heartbeat(ros):
    def scenario(node):
        ros.detect(tag_id=7)
        ros.now = 0.8
        ros.start()
        ros.now = 1.01
        stop_start = len(ros.events)
        node.publish_state()
        ros.hard_stop_before_task_state(
            stop_start, "TASK_ABORTED", "apriltag_detections_stale"
        )

    ros.run(scenario)


@pytest.mark.parametrize("confirmed", [False, True])
def test_task_duration_waits_for_verification_then_uses_the_correct_completion(
    ros, confirmed
):
    def scenario(node):
        ros.establish_tracking(duration=3)
        candidate_start = len(ros.published[SPORT])
        for frame, stamp in enumerate((2.5, 2.6, 2.7), start=1):
            ros.now = stamp
            ros.detect(tag_id=7 if confirmed or frame == 1 else None, frame=frame)
        ros.now = 3.0
        node.publish_state()
        assert ros.task_state()["type"] == "TASK_STARTED"
        assert ros.metrics()["drive_reason"] == "apriltag_verifying"
        assert all(
            json.loads(message.parameter) == ZERO
            for message in ros.published[SPORT][candidate_start:]
            if message.header.identity.api_id == 1008
        )
        ros.now = 3.5
        if not confirmed:
            ros.detect(frame=4)
            ros.camera_ready()
        node.publish_state()
        assert ros.task_state()["type"] == "TASK_COMPLETED"
        assert ros.task_state().get("reason") == (
            "apriltag_confirmed:7" if confirmed else None
        )
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(scenario)
