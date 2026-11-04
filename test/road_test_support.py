"""Shared task events, model identity and camera-owned FCN input fixtures."""
import json
from types import SimpleNamespace

from test_apriltag_task_stop_ros import Message, SPORT, ZERO

PIN = 'a' * 64


def details():
    from surround_camera import load_profile, profile_sha256
    return dict(backend='flyvis360-road-policy', weights_sha256=PIN,
                road_sha256='b'*64, policy_sha256='c'*64, encoder_sha256='d'*64,
                camera_views=['front', 'left', 'rear', 'right'],
                camera_profile_sha256=profile_sha256(load_profile()),
                uses_ground_truth_at_runtime=False, pose_input=False, goal_input=False,
                trained_max_speed_mps=10., head_device='cpu')


def start(ros, mode, task_id):
    ros.node.on_task_event(Message(json.dumps({
        'type': 'TASK_REGISTERED', 'action_name': 'LINE_TRACKING', 'task_id': task_id,
        'payload': {'selected_mask': mode, 'duration_sec': 60},
    })))
    assert ros.task_state()['type'] == 'TASK_STARTED'
    assert ros.node.tasks.active.selected_mask == mode


def cancel(ros, task_id):
    ros.node.on_task_event(Message(json.dumps({'type': 'TASK_ABORTED', 'task_id': task_id})))
    assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
    ros.now += 1.1
    ros.node.publish_state()


class FcnInput:
    """Replace the HTTP camera boundary while keeping the real FCN controller."""
    def __init__(self, monkeypatch, ros):
        import direct_camera_client
        self.ros, self.capture = ros, None
        monkeypatch.setattr(direct_camera_client, 'DirectCameraClient', lambda *a, **k: self)

    def infer(self, stream, not_before_ns, after_ns, max_age, now_ns):
        capture, self.capture = self.capture, None
        if capture is None:
            return None
        from fcn360_road import apply_front_robot_prior
        grids, stamp = capture
        return SimpleNamespace(masks=apply_front_robot_prior(grids), capture_stamp_ns=stamp, inference_seconds=.01)

    def send(self, grids):
        self.capture = (grids.copy(), self.ros.clock_ns())
        self.ros.node.request_direct_road()
