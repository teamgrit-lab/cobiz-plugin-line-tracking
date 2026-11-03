"""Publish timestamped four-view RGB from /dev/video91. No motion publisher."""
import argparse
import math

from connectome360_runtime import VIEWS_TOPIC, views_frame_id
from surround_capture import Camera
from surround_camera import load_profile, project_panorama, view_atlas


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device', default='/dev/video91')
    p.add_argument('--hz', type=float, default=5.)
    args = p.parse_args()
    if not math.isfinite(args.hz) or not .1 <= args.hz <= 30:
        p.error('camera publication rate must be 0.1..30 Hz')
    profile = load_profile()
    rclpy.init()
    node = rclpy.create_node('connectome360_camera')
    publisher = node.create_publisher(Image, VIEWS_TOPIC, qos_profile_sensor_data)
    camera = None

    def tick():
        nonlocal camera
        if not publisher.get_subscription_count():
            if camera is not None:
                camera.close()
                camera = None
            return
        if camera is None:
            camera = Camera(args.device)
        capture = camera.read(node.get_clock())
        if capture is None:
            return
        panorama, stamp = capture
        # Same projection/pixel centers/heading/order as the training dataset.
        atlas = view_atlas(project_panorama(panorama, profile))
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, views_frame_id()
        msg.height, msg.width, msg.step, msg.encoding = 540, 960, 960*3, 'bgr8'
        msg.data = atlas.tobytes()
        publisher.publish(msg)

    node.create_timer(1/args.hz, tick)
    try:
        rclpy.spin(node)
    finally:
        if camera is not None:
            camera.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
