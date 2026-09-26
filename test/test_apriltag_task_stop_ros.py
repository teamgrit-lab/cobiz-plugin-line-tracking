"""Exercise the live node with generated ROS and model boundaries replaced."""

import json
import signal
import sys
import threading
import time
from dataclasses import replace
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
        self.data = bytes(12) if data is None else data
        self.encoding = "rgb8"
        self.height, self.width, self.step = 2, 2, 6
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
                    encoding_to_dtype_with_channels=lambda encoding: {
                        "rgb8": ("uint8", 3), "bgr8": ("uint8", 3)
                    }[encoding],
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
                segment=lambda _frame, **_kwargs: SimpleNamespace(
                    selected_mask=np.full((360, 640), 2, np.uint8),
                    inference_seconds=0.01,
                ),
            ),
        )

    def clock_ns(self):
        return 100_000_000_000 + round(self.now * 1_000_000_000)

    def stamp(self):
        seconds, nanoseconds = divmod(self.clock_ns(), 1_000_000_000)
        return SimpleNamespace(sec=seconds, nanosec=nanoseconds)

    def run(self, scenario, *args, allow_errors=False):
        def spin(node):
            scenario(node)
            if not allow_errors:
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
            mask_class = self.node.tasks.active.selected_mask if self.node.tasks.active else 2
            path, decision = self.node.drive_readiness(mask_class, self.now)
            if decision.reason == "tracking":
                assert path is not None
                return
            assert time.perf_counter() < deadline, decision.reason
            time.sleep(0.001)

    def establish_tracking(self, duration=30, *, detection_heartbeat=True):
        self.start(duration=duration)
        self.now = 2.1
        if detection_heartbeat:
            self.detect()
        self.camera_ready()
        self.node.publish_state()
        assert self.metrics()["drive_reason"] == "tracking"
        assert json.loads(self.published[SPORT][-1].parameter)["x"] == 0.5


@pytest.fixture
def ros(monkeypatch):
    return RosHarness(monkeypatch)


def test_r50_task_drive_accepts_720p_rgb_and_keeps_camera_stop(ros, monkeypatch):
    configurations = []
    frames = []

    def segment(frame, *, color_order):
        assert frame.shape == (720, 1280, 3)
        assert color_order == "rgb"
        assert frame[0, 0].tolist() == [20, 30, 40]
        frames.append(frame)
        return SimpleNamespace(
            selected_mask=np.full((360, 640), 2, np.uint8),
            inference_seconds=0.01,
        )

    def create_segmenter(config):
        configurations.append(config)
        return SimpleNamespace(device=SimpleNamespace(type="cuda"), segment=segment)

    monkeypatch.setattr(debug, "BestSoFarSegmenter", create_segmenter)

    def scenario(node):
        camera = Message(bytes([20, 30, 40]) * 720 * 1280)
        camera.height, camera.width, camera.step = 720, 1280, 1280 * 3
        camera.header.stamp = ros.stamp()
        node.on_image(camera)
        deadline = time.perf_counter() + 3.0
        while True:
            node.publish_state()
            if ros.metrics()["inference_count"]:
                break
            assert time.perf_counter() < deadline, ros.errors
            time.sleep(0.001)

        assert ros.metrics()["profile"] == debug.R50_PROFILE
        assert ros.metrics()["path_tracked"] is True
        assert ros.published["/line_tracking/swin_l/local_path"][-1].poses
        assert SPORT not in ros.published
        ros.start()
        ros.now = 2.1
        node.publish_state()
        assert ros.metrics()["drive_reason"] == "tracking"
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] == 0.5

        ros.now = 6.0
        node.publish_state()
        assert ros.metrics()["drive_reason"] == "camera_stale"
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(
        scenario, "--profile", debug.R50_PROFILE, "--backend", "pytorch",
        "--path-mask-class", "0",
    )
    assert len(configurations) == len(frames) == 1
    assert configurations[0].profile == debug.R50_PROFILE
    assert configurations[0].backend == "pytorch"


def test_live_callback_queues_messages_and_only_latest_frame_is_decoded(ros, monkeypatch):
    release_worker = threading.Event()
    original_queue = debug.LatestFrameQueue
    decoded = []
    decode = debug.camera_image_rgb

    class GatedQueue(original_queue):
        def get_latest_at(self, ready_at_sec):
            assert release_worker.wait(timeout=3.0)
            return super().get_latest_at(ready_at_sec)

        def close(self):
            release_worker.set()
            super().close()

    def record_decode(message, bridge):
        decoded.append(message)
        return decode(message, bridge)

    monkeypatch.setattr(debug, "LatestFrameQueue", GatedQueue)
    monkeypatch.setattr(debug, "camera_image_rgb", record_decode)

    def scenario(node):
        for index in range(12):
            ros.now = 2.1 + index * 0.01
            message = Message()
            message.header.stamp = ros.stamp()
            node.on_image(message)
        assert decoded == []
        node.publish_state()
        assert ros.metrics()["queue_overwritten"] == 11
        release_worker.set()
        deadline = time.perf_counter() + 3.0
        while True:
            node.publish_state()
            if ros.metrics()["inference_count"] == 1:
                break
            assert time.perf_counter() < deadline, "latest frame was not processed"
            time.sleep(0.001)
        assert len(decoded) == 1 and decoded[0] is message

    ros.run(scenario)


@pytest.mark.parametrize("invalid", [
    {"width": 0}, {"step": 1}, {"data": bytes(1)}, {"encoding": "invalid"},
])
def test_invalid_image_metadata_stops_before_queueing(ros, monkeypatch, invalid):
    def scenario(node):
        ros.establish_tracking(detection_heartbeat=False)

        def unexpected_enqueue(*_args):
            pytest.fail("invalid metadata must not enter the inference queue")

        monkeypatch.setattr(debug.LatestFrameQueue, "put", unexpected_enqueue)
        ros.now = 2.5
        message = Message()
        message.header.stamp = ros.stamp()
        message.__dict__.update(invalid)
        node.on_image(message)
        assert node.last_valid_yaw_rate is None
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        node.publish_state()
        assert ros.metrics()["drive_reason"] == "camera_stale"

    ros.run(scenario, allow_errors=True)


@pytest.fixture
def path_bypass(ros, monkeypatch):
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_BYPASS_PATH_STOPS", "true")
    state = SimpleNamespace(lost=False, lateral=0.4)
    original = debug.extract_sidewalk_centerline

    def estimate(mask, config):
        if state.lost:
            return None
        result = original(mask, config)
        if result is not None:
            points = result.points_xy.copy()
            points[:, 1] = state.lateral
            return replace(result, points_xy=points)
        return result

    monkeypatch.setattr(debug, "extract_sidewalk_centerline", estimate)

    def refresh(*, publish=True):
        mask_class = ros.node.tasks.active.selected_mask if ros.node.tasks.active else 2
        ros.node.drive_readiness(mask_class, ros.now)
        previous_losses = ros.node.path_unavailable_inferences
        camera = Message()
        camera.header.stamp = ros.stamp()
        ros.node.on_image(camera)
        deadline = time.perf_counter() + 3.0
        while True:
            path, decision = ros.node.drive_readiness(mask_class, ros.now)
            if (
                state.lost and path is None
                and ros.node.path_unavailable_inferences == previous_losses + 1
            ) or (
                not state.lost and path is not None
                and path.age_sec == 0.0
                and abs(float(path.points_xy[0, 1]) - state.lateral) < 1e-6
            ):
                break
            assert time.perf_counter() < deadline, decision.reason
            time.sleep(0.001)
        if publish:
            ros.node.publish_state()

    state.refresh = refresh
    return state


def test_master_bypass_holds_last_yaw_and_never_reuses_it_in_another_task(ros, path_bypass):
    def scenario(node):
        ros.establish_tracking(detection_heartbeat=False)
        previous = json.loads(ros.published[SPORT][-1].parameter)
        assert abs(previous["z"]) > 0.05
        path_bypass.lost = True
        for now in (2.4, 3.4, 4.4, 5.4):
            ros.now = now
            path_bypass.refresh()
            assert ros.metrics()["drive_reason"] == "tracking_path_hold"
            assert ros.metrics()["path_yaw_held"] is True
            assert ros.metrics()["path_stop_bypass"] is True
            assert ros.metrics()["path_tracked"] is False
            assert ros.published["/line_tracking/swin_l/local_path"][-1].poses == []
            assert json.loads(ros.published[SPORT][-1].parameter) == previous
            assert node.tasks.active is not None
        assert ros.metrics()["stop_checks"] == {
            "camera_freshness": True, "inference_freshness": True,
            "path_available": False, "low_confidence": False,
            "lateral_target": False, "apriltag": True,
        }

        # A new usable path replaces the saved turn.
        path_bypass.lost = False
        path_bypass.lateral = -0.4
        ros.now = 5.6
        path_bypass.refresh()
        assert ros.metrics()["drive_reason"] == "tracking"
        assert ros.metrics()["path_yaw_held"] is False
        assert json.loads(ros.published[SPORT][-1].parameter)["z"] == pytest.approx(-previous["z"])
        path_bypass.lost = True
        ros.now = 5.8
        path_bypass.refresh()

        node.on_task_event(Message(json.dumps({"type": "TASK_ABORTED", "task_id": "tag-stop-1"})))
        assert node.last_valid_yaw_rate is None
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        ros.now = 7.0
        node.publish_state()
        ros.start(task_id="new-task")
        assert node.last_valid_yaw_rate is None
        ros.now = 9.1
        node.publish_state()
        assert ros.task_state()["reason"] == "startup:path_unavailable"
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(scenario)


def test_master_bypass_stops_and_clears_held_yaw_when_task_duration_ends(ros, path_bypass):
    def scenario(node):
        ros.establish_tracking(duration=3, detection_heartbeat=False)
        path_bypass.lost = True
        ros.now = 2.4
        path_bypass.refresh()
        assert ros.metrics()["path_yaw_held"] is True
        ros.now = 3.1
        node.publish_state()
        assert ros.task_state()["type"] == "TASK_COMPLETED"
        assert ros.metrics()["drive_reason"] == "task_complete"
        assert ros.metrics()["path_yaw_held"] is False
        assert node.last_valid_yaw_rate is None
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(scenario)


@pytest.mark.parametrize("recover", [False, True])
def test_fifth_missing_path_inference_stops_then_recovers_or_aborts(ros, path_bypass, recover):
    def scenario(node):
        ros.establish_tracking(detection_heartbeat=False)
        path_bypass.lost = True
        for count in range(1, 6):
            ros.now = 2.1 + count * 0.2
            path_bypass.refresh()
            assert ros.metrics()["path_unavailable_inferences"] == count
            assert ros.metrics()["path_unavailable_limit"] == 5
            expected = "tracking_path_hold" if count < 5 else "path_unavailable"
            assert ros.metrics()["drive_reason"] == expected
            # Repeated 10 Hz control output is not another completed inference.
            for _ in range(10):
                node.publish_state()
            assert ros.metrics()["path_unavailable_inferences"] == count
            assert ros.metrics()["drive_reason"] == expected
        assert node.tasks.active is not None
        assert ros.metrics()["stop_checks"]["path_available"] is True
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        if recover:
            ros.now = 3.3
            path_bypass.lost = False
            path_bypass.refresh()
            assert ros.metrics()["path_unavailable_inferences"] == 0
            assert ros.metrics()["drive_reason"] == "tracking"
            assert node.tasks.unsafe_since is None
            path_bypass.lost = True
            ros.now = 3.5
            path_bypass.refresh()
            assert ros.metrics()["path_unavailable_inferences"] == 1
            assert ros.metrics()["drive_reason"] == "tracking_path_hold"
        else:
            ros.now = 5.2
            node.publish_state()
            assert ros.task_state()["type"] == "TASK_ABORTED"
            assert ros.task_state()["reason"] == "unsafe:path_unavailable"
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(scenario)


def test_missing_path_streak_counts_inferences_between_control_ticks_and_resets(ros, path_bypass):
    def scenario(node):
        ros.establish_tracking(detection_heartbeat=False)
        for lost in ([True] * 4 + [False] + [True] * 4):
            path_bypass.lost = lost
            ros.now += 0.2
            path_bypass.refresh(publish=False)
        node.publish_state()
        assert ros.metrics()["path_unavailable_inferences"] == 4
        assert ros.metrics()["drive_reason"] == "tracking_path_hold"
        ros.now += 0.2
        path_bypass.refresh(publish=False)
        node.publish_state()
        assert ros.metrics()["path_unavailable_inferences"] == 5
        assert ros.metrics()["drive_reason"] == "path_unavailable"
        node.on_task_event(Message(json.dumps({"type": "TASK_ABORTED", "task_id": "tag-stop-1"})))
        ros.now += 1.1
        node.publish_state()
        ros.start(task_id="new-task")
        assert node.path_unavailable_inferences == 0
        ros.now += 0.2
        path_bypass.refresh()
        assert ros.metrics()["path_unavailable_inferences"] == 1
        assert ros.metrics()["ready_reason"] == "path_unavailable"
        assert node.last_valid_yaw_rate is None

    ros.run(scenario)


@pytest.mark.parametrize("mask_class", [0, 1, 2])
def test_missing_path_streak_uses_the_task_selected_surface(ros, monkeypatch, mask_class):
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_BYPASS_PATH_STOPS", "true")
    monkeypatch.setitem(debug.ENV, "SWIN_L_PATH_MASK_CLASS", str(mask_class))
    initial_label = 1 if mask_class == 1 else 2
    mask = np.full((360, 640), initial_label, np.uint8)
    monkeypatch.setattr(
        debug, "BestSoFarSegmenter",
        lambda _config: SimpleNamespace(
            device=SimpleNamespace(type="cuda"),
            segment=lambda _frame, **_kwargs: SimpleNamespace(selected_mask=mask.copy()),
        ),
    )

    def scenario(node):
        ros.establish_tracking(detection_heartbeat=False)
        # The other surface remains valid for road-only / sidewalk-only tasks.
        mask[:] = 0 if mask_class == 0 else 3 - mask_class
        for count in range(1, 6):
            ros.now += 0.2
            camera = Message()
            camera.header.stamp = ros.stamp()
            node.on_image(camera)
            deadline = time.perf_counter() + 3.0
            while True:
                node.drive_readiness(mask_class, ros.now)
                if node.path_unavailable_inferences == count:
                    break
                assert time.perf_counter() < deadline
                time.sleep(0.001)
        node.publish_state()
        assert ros.metrics()["path_mask_class"] == mask_class
        assert ros.metrics()["path_unavailable_inferences"] == 5
        assert ros.metrics()["drive_reason"] == "path_unavailable"
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(scenario)


@pytest.mark.parametrize("fault", [
    "camera_stale", "inference_stale", "camera_timestamp_invalid", "camera_conversion_error",
])
def test_master_bypass_preserves_all_four_sensor_stops(ros, path_bypass, monkeypatch, fault):
    def scenario(node):
        ros.establish_tracking(detection_heartbeat=False)
        path_bypass.lost = True
        ros.now = 2.4
        path_bypass.refresh()
        assert ros.metrics()["drive_reason"] == "tracking_path_hold"
        reasons = []
        publish = node.publish_drive

        def record(decision):
            reasons.append(decision.reason)
            publish(decision)

        monkeypatch.setattr(node, "publish_drive", record)
        if fault == "camera_stale":
            ros.now = 7.5
            node.publish_state()
        elif fault == "inference_stale":
            ros.now = 7.5
            # A fresh camera frame arrives, but the inference worker has stalled.
            monkeypatch.setattr(debug.LatestFrameQueue, "put", lambda *_args: True)
            camera = Message()
            camera.header.stamp = ros.stamp()
            node.on_image(camera)
            node.publish_state()
        else:
            if fault == "camera_conversion_error":
                ros.now = 2.5

                def fail(*_args, **_kwargs):
                    raise RuntimeError("test conversion fault")

                monkeypatch.setattr(node.bridge, "imgmsg_to_cv2", fail)
            # Unchanged source stamp exercises the duplicate-frame rejection.
            camera = Message()
            camera.header.stamp = ros.stamp()
            if fault == "camera_conversion_error":
                # Native rgb8 no longer needs a bridge conversion. Fail the
                # selected frame's fallback decoder in the worker instead.
                camera.encoding = "bgr8"
            node.on_image(camera)
            if fault == "camera_conversion_error":
                deadline = time.perf_counter() + 3.0
                while fault not in reasons:
                    assert time.perf_counter() < deadline, "conversion fault not reported"
                    time.sleep(0.001)
        assert fault in reasons
        assert node.last_valid_yaw_rate is None
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(scenario, allow_errors=fault == "camera_conversion_error")


def test_master_bypass_preserves_apriltag_stop_during_held_yaw(ros, path_bypass):
    def scenario(node):
        ros.establish_tracking(detection_heartbeat=False)
        path_bypass.lost = True
        ros.now = 2.4
        path_bypass.refresh()
        ros.detect(tag_id=7, frame=1)
        node.publish_state()
        assert ros.metrics()["drive_reason"] == "apriltag_verifying"
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        ros.now = 2.6
        ros.detect(tag_id=7, frame=2)
        ros.now = 2.8
        ros.detect(tag_id=7, frame=3)
        ros.now = 3.5
        node.publish_state()
        assert ros.task_state()["type"] == "TASK_COMPLETED"
        assert node.last_valid_yaw_rate is None
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(scenario)


@pytest.mark.parametrize("from_payload", [False, True])
def test_combined_surface_tracks_either_class_and_stops_on_background(
    ros, monkeypatch, from_payload
):
    mask = np.ones((360, 640), np.uint8)
    monkeypatch.setitem(debug.ENV, "SWIN_L_PATH_MASK_CLASS", "2" if from_payload else "0")
    monkeypatch.setattr(
        debug,
        "BestSoFarSegmenter",
        lambda _config: SimpleNamespace(
            device=SimpleNamespace(type="cuda"),
            segment=lambda _frame, **_kwargs: SimpleNamespace(selected_mask=mask.copy()),
        ),
    )

    def scenario(node):
        payload = {"duration_sec": 30}
        if from_payload:
            payload["selected_mask"] = 0
        node.on_task_event(Message(json.dumps({
            "type": "TASK_REGISTERED", "action_name": "LINE_TRACKING",
            "task_id": "combined-1", "payload": payload,
        })))
        assert ros.task_state()["type"] == "TASK_STARTED"
        assert node.tasks.active.selected_mask == 0
        node.publish_state()

        for surface in (1, 2, "mixed", 0):
            if surface == "mixed":
                mask[:, :320], mask[:, 320:] = 1, 2
            else:
                mask.fill(surface)
            ros.now += 2.1
            count = ros.metrics()["inference_count"]
            camera = Message()
            camera.header.stamp = ros.stamp()
            node.on_image(camera)
            deadline = time.perf_counter() + 3.0
            while True:
                assert time.perf_counter() < deadline, "inference did not finish"
                # Let the first frame finish before evaluating the startup hold.
                if count == 0:
                    _, decision = node.drive_readiness(0, ros.now)
                    if decision.reason != "tracking":
                        time.sleep(0.001)
                        continue
                node.publish_state()
                metrics = ros.metrics()
                if metrics["inference_count"] > count:
                    break
                time.sleep(0.001)

            assert metrics["inference_count"] == count + 1
            assert metrics["path_mask_class"] == 0
            assert metrics["path_surface"] == "ROAD_OR_SIDEWALK"
            path = ros.published["/line_tracking/swin_l/local_path"][-1]
            move = json.loads(ros.published[SPORT][-1].parameter)
            if surface == 0:
                assert metrics["drive_reason"] == "path_unavailable"
                assert path.poses == []
                assert move == ZERO
            else:
                assert metrics["drive_reason"] == "tracking"
                assert len(path.poses) == 20
                assert move["x"] == 0.5
                assert abs(move["z"]) <= 0.18

    ros.run(scenario)


def test_available_path_keeps_moving_between_slow_inference_updates(ros):
    def scenario(node):
        ros.establish_tracking(detection_heartbeat=False)
        updated_at = ros.now
        inference_count = ros.metrics()["inference_count"]
        for gap in (0.1, 0.45, 2.0 / 3.0, 1.0, 4.9):
            ros.now = updated_at + gap
            node.publish_state()
            metrics = ros.metrics()
            assert metrics["path_age_sec"] == pytest.approx(gap)
            assert metrics["inference_count"] == inference_count
            assert metrics["drive_reason"] == "tracking"
            assert json.loads(ros.published[SPORT][-1].parameter)["x"] == 0.5
            assert node.tasks.active is not None

        ros.now = updated_at + 5.1
        node.publish_state()
        assert ros.metrics()["drive_reason"] == "camera_stale"
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

    ros.run(scenario)


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


def test_optional_stops_can_be_disabled_but_camera_loss_and_cancel_still_stop(
    ros, monkeypatch
):
    for check in ("LOW_CONFIDENCE", "LATERAL_TARGET", "APRILTAG"):
        monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_" + check, "false")

    def scenario(node):
        ros.detect(tag_id=7, frame=1)
        ros.establish_tracking(detection_heartbeat=False)
        command_start = len(ros.published[SPORT])
        for frame, stamp in ((2, 2.2), (3, 2.7), (4, 3.2), (5, 3.3)):
            ros.now = stamp
            ros.detect(tag_id=7, frame=frame)
            node.publish_state()
            assert ros.metrics()["drive_reason"] == "tracking"
            assert ros.metrics()["apriltag"]["stop_enabled"] is False
            assert ros.metrics()["apriltag"]["stream_ready"] is True
            assert ros.task_state()["type"] == "TASK_STARTED"
        assert all(
            message.header.identity.api_id == 1008
            and json.loads(message.parameter)["x"] == 0.5
            for message in ros.published[SPORT][command_start:]
        )
        assert ros.metrics()["stop_checks"] == {
            "camera_freshness": True,
            "inference_freshness": True,
            "path_available": True,
            "low_confidence": False,
            "lateral_target": False,
            "apriltag": False,
        }

        ros.now = 7.2
        node.publish_state()
        assert ros.metrics()["drive_reason"] == "camera_stale"
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

        # Restore perception, then verify explicit cancellation still takes control.
        ros.now = 7.3
        ros.camera_ready()
        node.publish_state()
        assert ros.metrics()["drive_reason"] == "tracking"
        node.on_task_event(Message(json.dumps({
            "type": "TASK_ABORTED", "task_id": "tag-stop-1",
            "action_name": "LINE_TRACKING",
        })))
        assert ros.task_state()["type"] == "TASK_ABORTED"
        assert node.tasks.active is None
        assert ros.published[SPORT][-2].header.identity.api_id == 1003
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO

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


def test_absent_detection_publisher_permits_real_path_tracking(ros):
    def scenario(_node):
        ros.establish_tracking(detection_heartbeat=False)
        assert ros.metrics()["apriltag"]["stream_ready"] is False
        assert ros.metrics()["apriltag"]["message_age_sec"] is None

    ros.run(scenario)


def test_false_positive_resumes_only_after_camera_and_path_recover(ros):
    def scenario(node):
        # Expire the camera inside the tag window; the runtime default is 5 s.
        node.drive_config = replace(node.drive_config, max_camera_age_sec=0.5)
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
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] == 0.5

    ros.run(scenario)


@pytest.mark.parametrize("verifying", [False, True])
def test_stale_stream_does_not_abort_or_block_tracking(ros, verifying):
    def scenario(node):
        ros.establish_tracking()
        if verifying:
            ros.now = 2.2
            ros.detect(tag_id=7)
        ros.now = 3.21
        ros.camera_ready()
        node.publish_state()
        assert node.tasks.active is not None
        assert ros.task_state()["type"] == "TASK_STARTED"
        assert ros.metrics()["apriltag"]["stream_ready"] is False
        assert ros.metrics()["drive_reason"] == (
            "apriltag_verifying" if verifying else "tracking"
        )
        if verifying:
            ros.now = 4.2
            ros.camera_ready()
            node.publish_state()
            assert ros.metrics()["drive_reason"] == "tracking"
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] == 0.5

    ros.run(scenario, "--apriltag-confirm-window-sec", "2.0")


@pytest.mark.parametrize("phase", ["startup", "verifying", "tracking"])
def test_competing_publisher_does_not_abort_an_active_task(ros, phase):
    def scenario(node):
        if phase == "tracking":
            ros.establish_tracking()
        else:
            ros.start()
            if phase == "verifying":
                ros.detect(tag_id=7)
        ros.now += 0.1
        ros.external_publishers = 5
        node.publish_state()
        assert node.tasks.active is not None
        assert ros.task_state()["type"] == "TASK_STARTED"
        expected_reason = {
            "startup": "startup_hold",
            "verifying": "apriltag_verifying",
            "tracking": "tracking",
        }[phase]
        assert ros.metrics()["drive_reason"] == expected_reason
        if phase == "tracking":
            assert json.loads(ros.published[SPORT][-1].parameter)["x"] == 0.5

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
def test_received_then_lost_stream_does_not_block_startup(ros, candidate):
    def scenario(node):
        ros.start()
        ros.detect(tag_id=7 if candidate else None)
        ros.now = 1.01
        node.publish_state()
        assert node.tasks.active is not None
        assert ros.metrics()["apriltag"]["stream_ready"] is False
        ros.now = 2.1 if not candidate else 2.01
        ros.camera_ready()
        node.publish_state()
        assert node.tasks.active is not None
        assert ros.metrics()["drive_reason"] == "tracking"

    ros.run(scenario, "--apriltag-confirm-window-sec", "2.0")


def test_never_received_stream_tracks_after_the_startup_hold(ros):
    def scenario(node):
        ros.start()
        ros.now = 1.99
        node.publish_state()
        assert ros.task_state()["type"] == "TASK_STARTED"
        ros.camera_ready()
        ros.now = 2.0
        node.publish_state()
        assert ros.task_state()["type"] == "TASK_STARTED"
        assert ros.metrics()["drive_reason"] == "tracking"
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] == 0.5

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


def test_tag_metrics_report_verification_and_confirmation(ros):
    def scenario(node):
        node.publish_state()
        assert ros.metrics()["apriltag"] == {
            "topic": "/detections",
            "stop_enabled": True,
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
            "stop_enabled": True,
            "stream_ready": True,
            "message_age_sec": 0.0,
            "state": "verifying",
            "hit_counts": {"7": 2},
            "confirmed_id": None,
            "window_elapsed_sec": 0.1,
        }
        assert "/line_tracking/swin_l/overlay" not in ros.published
        ros.now = 0.2
        ros.detect(tag_id=7, frame=3)
        ros.now = 1.0
        node.publish_state()
        assert ros.metrics()["apriltag"]["state"] == "confirmed"
        assert ros.metrics()["apriltag"]["confirmed_id"] == 7
        assert ros.metrics()["apriltag"]["message_age_sec"] == pytest.approx(0.8)
        assert "lidar" not in ros.metrics()

    ros.run(scenario)


def test_callback_confirmation_state_survives_detections_until_publisher_release(ros):
    def scenario(node):
        ros.start()
        for frame, stamp in enumerate((0.0, 0.1, 0.2), start=1):
            ros.now = stamp
            ros.detect(tag_id=7, frame=frame)
        ros.camera_ready()
        ros.now = 1.0
        ros.detect(frame=4)
        assert ros.task_state()["type"] == "TASK_COMPLETED"
        assert ros.task_state()["reason"] == "apriltag_confirmed:7"
        assert node.tasks.active is None
        completion_commands = len(ros.published[SPORT])
        ros.now = 1.03
        ros.detect(frame=5)
        ros.now = 1.05
        node.publish_state()
        assert node.command_publisher is not None
        assert ros.metrics()["apriltag"]["state"] == "confirmed"
        assert ros.metrics()["apriltag"]["confirmed_id"] == 7
        assert ros.metrics()["apriltag"]["message_age_sec"] == pytest.approx(0.02)
        assert ros.metrics()["apriltag"]["stream_ready"] is True
        ros.now = 1.99
        ros.detect(frame=6)
        node.publish_state()
        assert node.command_publisher is not None
        assert ros.metrics()["apriltag"]["state"] == "confirmed"
        ros.now = 2.0
        node.publish_state()
        assert node.command_publisher is None
        assert ros.metrics()["apriltag"]["state"] == "no_tag"
        assert ros.metrics()["apriltag"]["confirmed_id"] is None
        assert ros.metrics()["drive_reason"] == "task_idle"
        assert "/line_tracking/swin_l/overlay" not in ros.published
        assert len(ros.published["/task_state"]) == 2
        assert all(
            message.header.identity.api_id == 1008
            and json.loads(message.parameter) == ZERO
            for message in ros.published[SPORT][completion_commands:]
        )

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


def test_cached_candidate_stale_heartbeat_does_not_abort_task(ros):
    def scenario(node):
        ros.detect(tag_id=7)
        ros.now = 0.8
        ros.start()
        ros.now = 1.01
        node.publish_state()
        assert node.tasks.active is not None
        assert ros.metrics()["drive_reason"] == "apriltag_verifying"
        assert ros.metrics()["apriltag"]["stream_ready"] is False
        ros.now = 1.8
        ros.camera_ready()
        node.publish_state()
        assert node.tasks.active is not None
        assert ros.metrics()["drive_reason"] == "startup_hold"
        ros.now = 2.81
        ros.camera_ready()
        node.publish_state()
        assert ros.metrics()["drive_reason"] == "tracking"

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


@pytest.mark.parametrize("phase", ["startup", "tracking"])
def test_chained_failed_windows_cannot_postpone_lifecycle_deadlines(ros, phase):
    def scenario(node):
        if phase == "tracking":
            ros.establish_tracking(duration=3)
            first_candidate = 2.5
            deadline = 3.5
        else:
            ros.start()
            first_candidate = 0.0
            deadline = 2.0
        command_start = len(ros.published[SPORT])
        for frame in range(1, 8):
            ros.now = first_candidate + (frame - 1) * 0.5
            ros.detect(tag_id=7, frame=frame)
            node.publish_state()
            if ros.now >= deadline:
                break
            assert node.tasks.active is not None
        assert node.tasks.active is None
        assert ros.task_state()["type"] == "TASK_ABORTED"
        assert ros.task_state()["reason"] == (
            "startup:apriltag_verifying"
            if phase == "startup"
            else "tracking_unavailable:apriltag_verifying"
        )
        assert all(
            json.loads(message.parameter) == ZERO
            for message in ros.published[SPORT][command_start:]
            if message.header.identity.api_id == 1008
        )
        ros.now += 1.0
        node.publish_state()
        assert node.command_publisher is None

    ros.run(scenario)


@pytest.mark.parametrize("timer_at_boundary", [None, "before", "after"])
@pytest.mark.parametrize("confirmed", [False, True])
def test_old_window_boundary_cannot_shorten_the_next_confirmation_window(
    ros, timer_at_boundary, confirmed
):
    def scenario(node):
        # Keep input expiry explicit instead of depending on the removed path timer.
        node.drive_config = replace(node.drive_config, max_camera_age_sec=0.5)
        ros.establish_tracking(duration=3.5)
        command_start = len(ros.published[SPORT])
        ros.now = 2.45
        ros.detect(tag_id=7, frame=1)
        ros.now = 2.95
        ros.detect(tag_id=7, frame=2)
        ros.now = 3.45
        if timer_at_boundary == "before":
            node.publish_state()
        ros.detect(tag_id=7, frame=3)
        if timer_at_boundary == "after":
            node.publish_state()
        for frame, stamp in ((4, 3.46), (5, 3.47)):
            ros.now = stamp
            ros.detect(tag_id=7 if confirmed else None, frame=frame)
        for stamp in (3.5, 4.44):
            ros.now = stamp
            node.publish_state()
            assert node.tasks.active is not None
            assert ros.task_state()["type"] == "TASK_STARTED"
            assert ros.metrics()["drive_reason"] == "apriltag_verifying"
        ros.now = 4.45
        if not confirmed:
            ros.detect(frame=6)
        node.publish_state()
        assert node.tasks.active is None
        assert ros.task_state()["type"] == (
            "TASK_COMPLETED" if confirmed else "TASK_ABORTED"
        )
        assert ros.task_state()["reason"] == (
            "apriltag_confirmed:7"
            if confirmed
            else "tracking_unavailable:camera_stale"
        )
        assert all(
            json.loads(message.parameter) == ZERO
            for message in ros.published[SPORT][command_start:]
            if message.header.identity.api_id == 1008
        )

    ros.run(scenario)


@pytest.mark.parametrize("source", ["candidate", "confirmation", "abort", "shutdown"])
@pytest.mark.parametrize("failed_ids", [{1003}, {1008}, {1003, 1008}])
def test_stop_publication_failure_attempts_zero_and_deactivates_task(
    ros, source, failed_ids
):
    attempts = []

    def scenario(node):
        ros.establish_tracking()
        if source == "confirmation":
            for frame, stamp in enumerate((2.5, 2.6, 2.7), start=1):
                ros.now = stamp
                ros.detect(tag_id=7, frame=frame)
        original_publish = node.command_publisher.publish

        def failing_publish(message):
            api_id = message.header.identity.api_id
            attempts.append(api_id)
            if api_id in failed_ids:
                raise RuntimeError(f"injected publication failure {api_id}")
            original_publish(message)

        node.command_publisher.publish = failing_publish
        if source == "candidate":
            ros.detect(tag_id=7)
        elif source == "confirmation":
            ros.now = 3.5
            node.publish_state()
        elif source == "abort":
            node.abort_active_task("test_abort")
        else:
            return  # Exercise run_ros2's real finally/shutdown path.
        assert node.tasks.active is None

    ros.run(scenario, allow_errors=True)
    assert attempts[:2] == [1003, 1008]
    assert ros.node.tasks.active is None
    assert ros.task_state()["type"] == "TASK_ABORTED"
    assert ros.errors
    if 1008 not in failed_ids:
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO


def test_abort_report_failure_cannot_keep_a_task_active(ros):
    def scenario(node):
        ros.establish_tracking()

        def fail_report(_message):
            raise RuntimeError("task-state transport unavailable")

        node.task_state_publisher.publish = fail_report
        node.abort_active_task("test_abort")
        assert node.tasks.active is None
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert any("TASK_ABORTED report failed" in error for error in ros.errors)

    ros.run(scenario, allow_errors=True)


@pytest.mark.parametrize("source", ["expired", "server"])
def test_verification_state_clears_without_refreshing_heartbeat(ros, source):
    def scenario(node):
        ros.establish_tracking()
        ros.now = 2.2
        ros.detect(tag_id=7)
        node.publish_state()
        assert ros.metrics()["apriltag"]["state"] == "verifying"
        assert ros.metrics()["drive_reason"] == "apriltag_verifying"
        ros.now = 3.21 if source == "server" else 4.2
        if source == "server":
            ros.subscriptions["/task_event"](
                Message(json.dumps({"type": "TASK_ABORTED", "task_id": "tag-stop-1"}))
            )
        else:
            ros.camera_ready()
        node.publish_state()
        assert ros.task_state()["type"] == (
            "TASK_STARTED" if source == "expired" else "TASK_ABORTED"
        )
        if source == "server":
            assert ros.task_state()["reason"] == "task_aborted_by_server"
        assert ros.metrics()["task_active"] is (source == "expired")
        assert ros.metrics()["apriltag"]["state"] == "no_tag"
        assert ros.metrics()["apriltag"]["message_age_sec"] == pytest.approx(
            2.0 if source == "expired" else 1.01
        )
        assert ros.metrics()["apriltag"]["stream_ready"] is False
        assert "/line_tracking/swin_l/overlay" not in ros.published

    ros.run(scenario, "--apriltag-confirm-window-sec", "2.0")
