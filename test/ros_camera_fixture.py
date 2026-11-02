"""Feed a real-size front image and wait for the camera inference worker in ROS tests."""
import time

from test_apriltag_task_stop_ros import Message


def image(node, ros):
    message = Message(bytes(360 * 640 * 3))
    message.width, message.height, message.step = 640, 360, 640 * 3
    message.header.stamp = ros.stamp()
    message.header.frame_id = "camera_optical_frame"
    node.on_image(message)


def complete_inference(node, ros):
    node.publish_state()
    previous = ros.metrics()["inference_count"]
    image(node, ros)
    deadline = time.perf_counter() + 4
    while True:
        node.publish_state()
        if ros.metrics()["inference_count"] > previous:
            return ros.metrics()
        assert time.perf_counter() < deadline, ros.errors
        time.sleep(0.001)
