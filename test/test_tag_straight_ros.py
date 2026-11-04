"""COBIZ payload through the real ROS node, with transport/camera/model fakes."""
import json
import math
import sys
import threading
from types import SimpleNamespace as NS

import pytest

from test_apriltag_task_stop_ros import RosHarness, Message, SPORT, ZERO
import swin_l_local_path_debug as debug


class Camera:
    def __init__(self, *args, **kwargs):
        self.idle, self.packet = threading.Event(), None
        self.idle.set()
        self.session_id = None
    def start(self, session_id, spec):
        self.session_id, self.spec = session_id, spec
        self.idle.clear()
    def stop(self):
        self.packet = None
        self.idle.set()
    def latest(self):
        return self.packet
    def close(self):
        self.stop()


@pytest.fixture
def ros(monkeypatch):
    harness = RosHarness(monkeypatch)
    monkeypatch.setitem(sys.modules, 'unitree_go.msg', NS(SportModeState=Message))
    monkeypatch.setattr(debug, 'TagRouteCamera', Camera)
    return harness


def register(h, payload=None):
    h.subscriptions['/task_event'](Message(json.dumps(dict(type='TASK_REGISTERED',
        action_name='LINE_TRACKING', task_id='tag-route',
        payload=payload or dict(selected_mask=2, start_tag_id=7, end_tag_id=8)))))


def inputs(h, node, now, tags=(), yaw_deg=0., position=(0., 0.)):
    h.now = now
    q = math.radians(yaw_deg) / 2
    h.subscriptions['/lf/sportmodestate'](NS(imu_state=NS(
        quaternion=[math.cos(q), 0., 0., math.sin(q)], rpy=[0., 0., 0.]),
        position=[*position, 0.], stamp=h.stamp()))
    node.tag_route_camera.packet = dict(session_id=node.tasks.active.key,
        stamp_ns=h.clock_ns(), detections=list(tags))


def tag(tag_id=7, error=20., x=.2):
    return dict(tag_id=tag_id, pose_status='ok', parallel_error_deg=error,
                range_m=1.5, position_body_m=[x, 1.4, 0.])


def begin_tracking(h, node):
    register(h)
    assert h.task_state()['type'] == 'TASK_STARTED'
    inputs(h, node, 2.1)
    h.camera_ready()
    node.publish_state()
    assert h.metrics()['drive_reason'] == 'tracking'


def begin_straight(h, node):
    begin_tracking(h, node)
    for t in (2.2, 2.4, 2.6):
        inputs(h, node, t, [tag()], yaw_deg=20.)
        node.publish_state()
    assert node.tag_route.phase == 'align'
    assert json.loads(h.published[SPORT][-1].parameter)['z'] < 0
    for t in (2.8, 3., 3.2):
        inputs(h, node, t)
        node.publish_state()
    inputs(h, node, 3.3)
    node.publish_state()
    assert node.tag_route.phase == 'straight'
    assert json.loads(h.published[SPORT][-1].parameter)['x'] == .3


def test_route_payload_is_rejected_when_hardware_feature_is_disabled(ros):
    def scenario(node):
        register(ros)
        assert ros.task_state()['type'] == 'TASK_REJECTED'
        assert ros.task_state()['reason'] == 'tag_route_disabled'
        assert node.command_publisher is None
    ros.run(scenario)


def test_enabled_feature_without_tag_ids_keeps_sidewalk_without_opening_tag_camera(ros):
    def scenario(node):
        ros.establish_tracking()
        assert ros.metrics()['drive_reason'] == 'tracking'
        assert node.tag_route is None and node.tag_route_camera is None
        assert node.tag_route_pose_subscription is None
    ros.run(scenario, '--tag-route-enabled')


def test_tag_id_outside_selected_family_is_rejected_before_acquiring_motion(ros):
    def scenario(node):
        register(ros, dict(selected_mask=2, start_tag_id=9999, end_tag_id=8))
        assert ros.task_state()['reason'] == 'tag_id_outside_family'
        assert node.command_publisher is None and node.tag_route_camera is None
    ros.run(scenario, '--tag-route-enabled')


def test_start_tag_already_visible_waits_for_first_sidewalk_inference(ros):
    def scenario(node):
        register(ros)
        for t in (.1, .2, .3):
            inputs(ros, node, t, [tag(error=20)], yaw_deg=20)
            node.publish_state()
        assert node.tasks.active is not None
        assert node.tag_route.phase == 'align'
        assert node.last_ready_reason == 'tag_route_sidewalk_inputs_pending'
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
    ros.run(scenario, '--tag-route-enabled')


def test_start_parallel_turn_straight_and_arrival_use_one_task_and_stop_before_completion(ros):
    def scenario(node):
        begin_straight(ros, node)
        # Global stop-on-any-tag is enabled in this harness, but the route owns
        # its two IDs and must not complete on the start or an unrelated tag.
        ros.detect(tag_id=7, frame=1)
        ros.detect(tag_id=9, frame=2)
        assert node.tasks.active is not None
        event_start = len(ros.events)
        for t in (3.4, 3.6, 3.8):
            inputs(ros, node, t, [tag(tag_id=8)])
            node.publish_state()
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert ros.task_state()['type'] == 'TASK_COMPLETED'
        assert ros.task_state()['reason'] == 'tag_route_arrived:8'
        events = ros.events[event_start:]
        completion = next(i for i, (topic, msg) in enumerate(events)
            if topic == '/task_state' and json.loads(msg.data)['type'] == 'TASK_COMPLETED')
        assert any(topic == SPORT and msg.header.identity.api_id == 1003 for topic, msg in events[:completion])
        assert node.tag_route is None and node.tag_route_camera.idle.is_set()
    ros.run(scenario, '--tag-route-enabled')


def test_pose_loss_aborts_and_final_publication_does_not_reuse_a_straight_command(ros):
    def scenario(node):
        begin_straight(ros, node)
        previous = node.drive_readiness(2, ros.now)[1]
        ros.now = 3.9
        # The camera is alive, while body feedback has stopped.
        node.tag_route_camera.packet = dict(session_id=node.tasks.active.key,
            stamp_ns=ros.clock_ns(), detections=[])
        decision = node.publish_drive(previous)
        assert decision.vx == decision.yaw_rate == 0
        node.publish_state()
        assert ros.task_state()['type'] == 'TASK_ABORTED'
        assert ros.task_state()['reason'] == 'tag_route_pose_stale'
    ros.run(scenario, '--tag-route-enabled')


def test_cancel_closes_route_camera_and_late_destination_cannot_complete_task(ros):
    def scenario(node):
        begin_straight(ros, node)
        worker = node.tag_route_camera
        ros.subscriptions['/task_event'](Message(json.dumps(dict(type='TASK_ABORTED', task_id='tag-route'))))
        assert ros.task_state()['type'] == 'TASK_ABORTED'
        assert node.tag_route is None and worker.idle.is_set()
        worker.packet = dict(session_id='tag-route', stamp_ns=ros.clock_ns(), detections=[tag(tag_id=8)])
        node.publish_state()
        assert ros.task_state()['type'] == 'TASK_ABORTED'
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
    ros.run(scenario, '--tag-route-enabled')


def test_destination_observed_during_sidewalk_does_not_complete_or_switch_modes(ros):
    def scenario(node):
        begin_tracking(ros, node)
        for t in (2.2, 2.4, 2.6):
            inputs(ros, node, t, [tag(tag_id=8)])
            node.publish_state()
        assert node.tasks.active is not None and node.tag_route.phase == 'sidewalk'
        assert ros.metrics()['drive_reason'] == 'tracking'
    ros.run(scenario, '--tag-route-enabled')


def test_navigation_worker_opens_camera_only_after_grant_and_closes_on_lease_loss(ros):
    from test_navigation_worker_ros import grant_legacy
    def scenario(node):
        register(ros)
        assert node.tag_route_camera is None
        assert not ros.published['/task_state']
        grant_legacy(ros)
        assert node.tag_route is not None and node.tag_route_camera.session_id == 'tag-route'
        assert ros.task_state()['type'] == 'TASK_STARTED'
        ros.now = 1.01
        node.publish_state()
        assert ros.task_state()['type'] == 'TASK_ABORTED'
        assert node.tag_route_camera.idle.is_set()
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
    ros.rclpy.spin = scenario
    assert debug.run_ros2(debug.parse_args(['navigation-worker', '--tag-route-enabled',
                                          '--inference-hz', '1000'])) == 0
    assert not ros.errors
