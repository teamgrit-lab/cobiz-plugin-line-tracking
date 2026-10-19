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


class CameraRelayState:
    """Share untouched input independently of the AprilTag sampling cadence."""

    def __init__(self, period, publish_tag, publish_raw=None, publish_info=None):
        self.period = period
        self.publish_tag = publish_tag
        self.publish_raw = publish_raw
        self.publish_info = publish_info
        self.info = None
        self.last_image_at = -math.inf

    def on_info(self, message):
        self.info = message
        if self.publish_info is not None:
            self.publish_info(message)

    def on_image(self, image, now):
        if self.publish_raw is not None:
            self.publish_raw(image)
        if now - self.last_image_at < self.period:
            return True
        info = camera_info_for_image(image, self.info)
        if info is None:
            return False
        self.last_image_at = now
        self.publish_tag(image, info)
        return True


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
            rate = float(self.declare_parameter("max_hz", 10.0).value)
            if not math.isfinite(rate) or rate <= 0:
                raise ValueError("AprilTag max_hz must be finite and positive")
            raw_topic = self.declare_parameter("shared_image_topic", "").value
            raw_info_topic = self.declare_parameter("shared_camera_info_topic", "").value
            for output, sources in ((raw_topic, (image_topic, "image_rect")),
                                    (raw_info_topic, (info_topic, "camera_info"))):
                if output and self.resolve_topic_name(output) in {
                    self.resolve_topic_name(source) for source in sources
                }:
                    raise ValueError("shared camera output must differ from input and tag output")
            self.raw_pub = (self.create_publisher(Image, raw_topic, qos_profile_sensor_data)
                            if raw_topic else None)
            self.raw_info_pub = (self.create_publisher(CameraInfo, raw_info_topic, qos_profile_sensor_data)
                                 if raw_info_topic else None)
            self.image_pub = self.create_publisher(Image, "image_rect", qos_profile_sensor_data)
            self.info_pub = self.create_publisher(CameraInfo, "camera_info", qos_profile_sensor_data)
            self.state = CameraRelayState(
                1.0 / rate, self.publish_tag,
                self.raw_pub.publish if self.raw_pub else None,
                self.raw_info_pub.publish if self.raw_info_pub else None,
            )
            self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
            self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
            self.get_logger().info(f"AprilTag input: image={image_topic} info={info_topic}")

        def on_info(self, message):
            self.state.on_info(message)

        def on_image(self, image):
            if not self.state.on_image(image, time.monotonic()):
                self.get_logger().warning(
                    "AprilTag waiting for CameraInfo with matching image dimensions",
                    throttle_duration_sec=5.0,
                )

        def publish_tag(self, image, info):
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
