"""Optional two-checkout test using navigation's real coordinator and this worker.

Set TEAMGRIT_NAVIGATION_SOURCE to a local navigation checkout. No ROS services,
network, model downloads or robot commands are used.
"""
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from test_apriltag_task_stop_ros import RosHarness, debug


@pytest.mark.skipif(not os.environ.get("TEAMGRIT_NAVIGATION_SOURCE"), reason="requires navigation checkout")
def test_sidewalk_stop_spin_nav2_and_road_reuse_one_real_worker(monkeypatch):
    source = Path(os.environ["TEAMGRIT_NAVIGATION_SOURCE"]) / "nav/src/cobiz/scripts/surface_navigation_core.py"
    spec = importlib.util.spec_from_file_location("integration_surface_navigation_core", source)
    navigation = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, navigation)
    spec.loader.exec_module(navigation)
    ros = RosHarness(monkeypatch)
    loads = []
    def load(_config):
        loads.append(True)
        return SimpleNamespace(device=SimpleNamespace(type="cuda"), reset=lambda: None,
            segment=lambda _frame, **kwargs: SimpleNamespace(
                selected_mask=np.full((360, 640), kwargs["path_mask_class"], np.uint8), inference_seconds=.01))
    monkeypatch.setattr(debug, "BestSoFarSegmenter", load)
    core = navigation.SurfaceCoordinator()
    events, status_count = [], 0

    def goal(req_id, mode, start, end, **extra):
        return dict(req_id=req_id, execution_session="ingress", Mode=mode,
            frame="prior_map", goal_checker_id="waypoint_goal_checker",
            waypoints=[dict(x=start, y=0., z=0., yaw=0.), dict(x=end, y=0., z=0., yaw=0.)], **extra)

    def transport():
        nonlocal status_count
        while True:
            pending = core.drain()
            for channel, body in pending:
                if channel == "line":
                    ros.subscriptions["/line_tracking/segment_command"](SimpleNamespace(data=json.dumps(body)))
                else:
                    events.append((channel, body))
            published = ros.published["/line_tracking/segment_status"]
            for message in published[status_count:]:
                core.line_status(json.loads(message.data), ros.now, ros.clock_ns()/1e9)
            changed = status_count != len(published)
            status_count = len(published)
            if not core.events and not changed:
                break

    def tick(now, pose):
        ros.now = now
        stamp = ros.clock_ns()/1e9
        core.update_pose(pose, stamp)
        core.lease(dict(req_id=core.goal["req_id"], execution_session="ingress", stamp_sec=stamp), now, stamp)
        core.tick(now, stamp)
        transport()

    def scenario(node):
        core.update_pose((0, 0, 0, 0), 100.)
        core.start(goal(1, "sidewalk", 0., 10., next_target={"x": 9.6, "y": 10.}), 0., 100.)
        transport()
        assert node.tasks.active.selected_mask == 2
        ros.camera_ready()
        node.publish_state()
        transport()
        node.publish_state()
        packet = json.loads(ros.published["/line_tracking/command"][-1].data)
        assert core.line_velocity(packet, 0., 100.)[0] > 0
        old_packet = dict(packet)

        tick(.1, (9.6, 0, 0, 0))
        node.publish_state()
        transport()
        tick(.45, (9.6, 0, 0, 0))
        assert node.tasks.active is None
        assert core.phase == "SPIN_START"
        spin = next(body for channel, body in events if channel == "spin")
        assert spin["target_yaw"] == pytest.approx(math.pi / 2)
        assert not core.allow("line", .45, 100.45)
        core.spin_accepted(core.session, True, .45, 100.45)
        core.update_pose((9.6, 0, 0, math.pi / 2), 100.5)
        core.spin_result(core.session, True, .5, 100.5)
        transport()
        assert any(channel == "status" and body["status"] == "REACHED" for channel, body in events)

        # Omitted Mode is Nav2 on the next destination; no engine restart.
        native_goal = goal(2, "nav2", 9.6, 20.)
        native_goal.pop("Mode")
        core.start(native_goal, .5, 100.5)
        transport()
        native = next(body for channel, body in reversed(events) if channel == "nav2_goal")
        core.nav2_status(dict(req_id=native["req_id"], status="ACCEPTED"), .5, 100.5)
        assert core.allow("nav2", .5, 100.5)
        core.nav2_status(dict(req_id=native["req_id"], status="REACHED"), .6, 100.6)
        transport()

        ros.now = 1.
        core.update_pose((20, 0, 0, 0), 101.)
        core.start(goal(3, "road", 20., 30.), 1., 101.)
        transport()
        assert node.tasks.active.selected_mask == 1
        ros.camera_ready()
        node.publish_state()
        transport()
        assert core.phase == "LINE_ACTIVE"
        assert core.line_velocity(old_packet, 1., 101.) is None
        assert loads == [True]
        assert "/api/sport/request" not in ros.published
        assert not ros.published["/task_state"]

    ros.rclpy.spin = scenario
    assert debug.run_ros2(debug.parse_args(["navigation-worker", "--inference-hz", "1000"])) == 0
    assert not ros.errors


@pytest.mark.skipif(not os.environ.get("TEAMGRIT_NAVIGATION_SOURCE"), reason="requires navigation checkout")
def test_legacy_sidewalk_spin_nav2_road_and_legacy_share_one_model(monkeypatch):
    from test_navigation_worker_ros import legacy
    directory = Path(os.environ["TEAMGRIT_NAVIGATION_SOURCE"]) / "nav/src/cobiz/scripts"
    def load_module(name, path):
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        monkeypatch.setitem(sys.modules, name, module)
        spec.loader.exec_module(module)
        return module
    navigation = load_module("surface_navigation_core", directory / "surface_navigation_core.py")
    ownership = load_module("integration_standalone_core", directory / "standalone_control_core.py")
    control, core = ownership.StandaloneArbiter(0.), navigation.SurfaceCoordinator()
    ros = RosHarness(monkeypatch)
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_" + check.upper(), "false")
    loads = []
    def load(_config):
        loads.append(True)
        return SimpleNamespace(device=SimpleNamespace(type="cuda"), reset=lambda: None,
            segment=lambda _frame, **kwargs: SimpleNamespace(selected_mask=np.full(
                (360, 640), kwargs["path_mask_class"] or 2, np.uint8), inference_seconds=.01))
    monkeypatch.setattr(debug, "BestSoFarSegmenter", load)
    counts = {"request": 0, "status": 0}
    events = []

    def transport():
        while True:
            changed = False
            requests = ros.published["/line_tracking/task_request"]
            for msg in requests[counts["request"]:]:
                response = control.receive(json.loads(msg.data), ros.now, ros.clock_ns()/1e9,
                                           busy=core.goal is not None)
                if response:
                    ros.node.on_task_control(SimpleNamespace(data=json.dumps(response)))
                changed = True
            counts["request"] = len(requests)
            for channel, body in core.drain():
                changed = True
                if channel == "line":
                    ros.node.on_navigation_command(SimpleNamespace(data=json.dumps(body)))
                else:
                    events.append((channel, body))
            statuses = ros.published["/line_tracking/segment_status"]
            for msg in statuses[counts["status"]:]:
                changed = True
                core.line_status(json.loads(msg.data), ros.now, ros.clock_ns()/1e9)
            counts["status"] = len(statuses)
            if not changed:
                return

    def tick(now, pose):
        ros.now = now
        stamp = ros.clock_ns()/1e9
        core.update_pose(pose, stamp)
        core.lease(dict(req_id=core.goal["req_id"], execution_session="ingress", stamp_sec=stamp), now, stamp)
        core.tick(now, stamp)
        transport()

    def start(req, mode, x, end, **extra):
        stamp = ros.clock_ns()/1e9
        core.update_pose((x, 0, 0, 0), stamp)
        core.start(dict(req_id=req, Mode=mode, execution_session="ingress", frame="prior_map",
            goal_checker_id="waypoint_goal_checker", waypoints=[
                dict(x=x, y=0, z=0, yaw=0), dict(x=end, y=0, z=0, yaw=0)], **extra), ros.now, stamp)
        transport()

    def scenario(node):
        ros.now = 3.
        legacy(ros)
        transport()
        assert ros.task_state()["type"] == "TASK_STARTED"
        ros.camera_ready()
        node.publish_state()
        transport()
        assert json.loads(ros.published["/api/sport/request"][-1].parameter)["x"] > 0
        ros.now = 3.1
        legacy(ros, "TASK_ABORTED")
        transport()
        assert control.held(4.)
        ros.now = 5.2
        node.publish_state()
        assert not control.held(ros.now)
        sport_count = len(ros.published["/api/sport/request"])
        start(1, "sidewalk", 0., 10., next_target=dict(x=9.6, y=10.))
        ros.camera_ready()
        node.publish_state()
        transport()
        assert core.phase == "LINE_ACTIVE"
        tick(5.3, (9.6, 0, 0, 0))
        node.publish_state()
        transport()
        tick(5.65, (9.6, 0, 0, 0))
        assert core.phase == "SPIN_START"
        core.spin_accepted(core.session, True, ros.now, ros.clock_ns()/1e9)
        core.update_pose((9.6, 0, 0, math.pi/2), ros.clock_ns()/1e9)
        core.spin_result(core.session, True, ros.now, ros.clock_ns()/1e9)
        transport()
        start(2, "nav2", 9.6, 20.)
        native = next(body for channel, body in reversed(events) if channel == "nav2_goal")
        core.nav2_status(dict(req_id=native["req_id"], status="ACCEPTED"), ros.now, ros.clock_ns()/1e9)
        assert core.allow("nav2", ros.now, ros.clock_ns()/1e9)
        core.nav2_status(dict(req_id=native["req_id"], status="REACHED"), ros.now, ros.clock_ns()/1e9)
        transport()
        start(3, "road", 20., 30.)
        ros.camera_ready()
        node.publish_state()
        transport()
        assert node.tasks.active.selected_mask == 1 and core.phase == "LINE_ACTIVE"
        assert len(ros.published["/api/sport/request"]) == sport_count
        core.cancel(ros.now, ros.clock_ns()/1e9)
        transport()
        assert core.goal is None and node.navigation.active is None
        legacy(ros, task="last-legacy", mask=1)
        transport()
        ros.camera_ready()
        node.publish_state()
        transport()
        assert ros.task_state()["type"] == "TASK_STARTED"
        assert json.loads(ros.published["/api/sport/request"][-1].parameter)["x"] > 0
        assert len(loads) == 1

    ros.rclpy.spin = scenario
    assert debug.run_ros2(debug.parse_args(["navigation-worker", "--inference-hz", "1000"])) == 0
    assert not ros.errors
