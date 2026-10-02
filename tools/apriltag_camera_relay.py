#!/usr/bin/env python3
"""Pair each raw image with matching, cached camera calibration for apriltag_ros.

Detection only: no rectification or pose estimation. The detector's image_rect
input name is retained, but downstream pose estimation must stay disabled.
"""
from copy import deepcopy
import math
import time


def camera_info_for_image(image, camera_info):
    """Reuse calibration only at its native size, without changing the source."""
    if camera_info is None or (image.width, image.height) != (
        camera_info.width, camera_info.height
    ):
        return None
    paired = deepcopy(camera_info)
    paired.header = deepcopy(image.header)
    return paired


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    class CameraRelay(Node):
        def __init__(self):
            super().__init__("line_tracking_apriltag_camera")
            image_topic = self.declare_parameter(
                "input_image_topic", "/a2/front_camera/res_720p/image_raw"
            ).value
            info_topic = self.declare_parameter(
                "input_camera_info_topic", "/a2/front_camera/res_720p/camera_info"
            ).value
            self.info = None
            rate = float(self.declare_parameter("max_hz", 10.0).value)
            if not math.isfinite(rate) or rate <= 0:
                raise ValueError("AprilTag max_hz must be finite and positive")
            self.period = 1.0 / rate
            self.last_image_at = -math.inf
            self.image_pub = self.create_publisher(Image, "image_rect", qos_profile_sensor_data)
            self.info_pub = self.create_publisher(CameraInfo, "camera_info", qos_profile_sensor_data)
            self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
            self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
            self.get_logger().info(f"AprilTag input: image={image_topic} info={info_topic}")

        def on_info(self, message):
            self.info = message

        def on_image(self, image):
            now = time.monotonic()
            if now - self.last_image_at < self.period:
                return
            info = camera_info_for_image(image, self.info)
            if info is None:
                self.get_logger().warning(
                    "AprilTag waiting for CameraInfo with matching image dimensions",
                    throttle_duration_sec=5.0,
                )
                return
            self.last_image_at = now
            self.info_pub.publish(info)
            self.image_pub.publish(image)

    rclpy.init()
    node = CameraRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
