"""ROS 2 node for vision line tracking."""

import json
import math
import time
from typing import Any, Iterable, Tuple

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Float32, String

from .controller import ControllerConfig, LineTrackingController, VelocityCommand
from .joy_command import command_to_joy_axes, released_buttons
from .segmentation import YolopConfig, YolopSegmenter
from .vision import VisionConfig, VisionResult, YellowLineVision


def _as_tuple(value: Iterable[Any], caster: Any) -> Tuple[Any, ...]:
    return tuple(caster(item) for item in value)


class LineTrackingNode(Node):
    """Subscribe to a camera and publish confidence-gated A2 Joy commands.

    ``test_mode`` turns the node into a safe inspection mode: it still runs
    the complete segmentation and centerline fitting pipeline, but publishes
    a zero Joy command unless explicitly enabled otherwise.  The inspection
    topics use standard ROS messages so they can be viewed with
    ``rqt_image_view``, ``ros2 topic echo`` and RViz2.
    """

    def __init__(self) -> None:
        super().__init__("line_tracking")
        self._declare_parameters()
        vision_config = self._vision_config()
        controller_config = self._controller_config()
        model_path = str(self.get_parameter("segmentation_model_path").value).strip()
        segmenter = None
        if model_path:
            try:
                segmenter = YolopSegmenter(
                    YolopConfig(
                        model_path=model_path,
                        input_width=int(
                            self.get_parameter("segmentation_input_width").value
                        ),
                        input_height=int(
                            self.get_parameter("segmentation_input_height").value
                        ),
                        road_threshold=float(
                            self.get_parameter("road_threshold").value
                        ),
                        line_threshold=float(
                            self.get_parameter("line_threshold").value
                        ),
                        road_gate_kernel=int(
                            self.get_parameter("road_gate_kernel").value
                        ),
                    )
                )
            except Exception as error:
                raise RuntimeError(
                    f"failed to load road/line ONNX segmentation model: {error}"
                ) from error
            self.get_logger().info(f"using YOLOP road/line model: {model_path}")
        else:
            self.get_logger().warning(
                "segmentation_model_path is empty; using the legacy color fallback. "
                "Set SEGMENTATION_MODEL_PATH to a YOLOP ONNX file for production."
            )
        self._vision = YellowLineVision(vision_config, segmenter=segmenter)
        self._controller = LineTrackingController(controller_config)
        self._bridge = CvBridge()
        self._test_mode = bool(self.get_parameter("test_mode").value)
        self._publish_control_in_test_mode = bool(
            self.get_parameter("publish_control_in_test_mode").value
        )
        self._last_frame_monotonic = 0.0
        self._last_control_monotonic = 0.0
        self._watchdog_stopped = True

        input_reliability = str(self.get_parameter("input_reliability").value).lower()
        reliability = (
            ReliabilityPolicy.RELIABLE
            if input_reliability == "reliable"
            else ReliabilityPolicy.BEST_EFFORT
        )
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(self.get_parameter("input_qos_depth").value),
            reliability=reliability,
        )

        control_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._command_publisher = self.create_publisher(
            Joy,
            str(self.get_parameter("joy_topic").value),
            control_qos,
        )
        self._confidence_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("confidence_topic").value),
            10,
        )
        self._debug_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("debug_image_topic").value),
            image_qos,
        )
        self._mask_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("mask_topic").value),
            image_qos,
        )
        self._test_debug_publisher = None
        self._test_road_mask_publisher = None
        self._test_raw_line_mask_publisher = None
        self._test_line_mask_publisher = None
        self._test_birdseye_mask_publisher = None
        self._test_centerline_publisher = None
        self._test_metrics_publisher = None
        if self._test_mode:
            self._test_debug_publisher = self.create_publisher(
                Image,
                str(self.get_parameter("test_debug_image_topic").value),
                image_qos,
            )
            self._test_road_mask_publisher = self.create_publisher(
                Image,
                str(self.get_parameter("test_road_mask_topic").value),
                image_qos,
            )
            self._test_raw_line_mask_publisher = self.create_publisher(
                Image,
                str(self.get_parameter("test_raw_line_mask_topic").value),
                image_qos,
            )
            self._test_line_mask_publisher = self.create_publisher(
                Image,
                str(self.get_parameter("test_line_mask_topic").value),
                image_qos,
            )
            self._test_birdseye_mask_publisher = self.create_publisher(
                Image,
                str(self.get_parameter("test_birdseye_mask_topic").value),
                image_qos,
            )
            self._test_centerline_publisher = self.create_publisher(
                Path,
                str(self.get_parameter("test_centerline_topic").value),
                10,
            )
            self._test_metrics_publisher = self.create_publisher(
                String,
                str(self.get_parameter("test_metrics_topic").value),
                10,
            )
        self._subscription = self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._image_callback,
            image_qos,
        )
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_callback)

        self.get_logger().info(
            "line tracking started | image=%s joy=%s profile=%s model_input=%sx%s "
            "reliability=%s test_mode=%s"
            % (
                self.get_parameter("image_topic").value,
                self.get_parameter("joy_topic").value,
                self.get_parameter("camera_profile").value,
                self.get_parameter("segmentation_input_width").value,
                self.get_parameter("segmentation_input_height").value,
                input_reliability,
                self._test_mode,
            )
        )
        if self._test_mode:
            self.get_logger().warning(
                "segmentation test mode enabled; zero Joy commands are published "
                "unless publish_control_in_test_mode is true"
            )

    def _declare_parameters(self) -> None:
        parameters = {
            "image_topic": "/a2/front_camera/image_raw",
            "joy_topic": "/a2_control",
            "joy_frame_id": "line_tracking",
            "debug_image_topic": "/line_tracking/debug_image",
            "mask_topic": "/line_tracking/mask",
            "confidence_topic": "/line_tracking/confidence",
            "camera_profile": "360p",
            "input_reliability": "best_effort",
            "input_qos_depth": 5,
            "frame_timeout_sec": 0.5,
            "publish_debug": True,
            "test_mode": False,
            "publish_control_in_test_mode": False,
            "test_debug_image_topic": "/line_tracking/test/debug_image",
            "test_road_mask_topic": "/line_tracking/test/road_mask",
            "test_raw_line_mask_topic": "/line_tracking/test/raw_line_mask",
            "test_line_mask_topic": "/line_tracking/test/line_mask",
            "test_birdseye_mask_topic": "/line_tracking/test/birdseye_mask",
            "test_centerline_topic": "/line_tracking/test/centerline",
            "test_metrics_topic": "/line_tracking/test/metrics",
            "centerline_frame_id": "base_link",
            "segmentation_model_path": "",
            "segmentation_input_width": 640,
            "segmentation_input_height": 360,
            "road_threshold": 0.50,
            "line_threshold": 0.50,
            "road_gate_kernel": 21,
            "hsv_lower": [14, 45, 40],
            "hsv_upper": [42, 255, 255],
            "lab_b_min": 135,
            "adaptive_lab_percentile": 85.0,
            "clahe_clip_limit": 2.0,
            "clahe_grid_size": 8,
            "roi_polygon": [0.10, 1.00, 0.90, 1.00, 0.65, 0.45, 0.35, 0.45],
            "perspective_source": [
                0.10,
                1.00,
                0.90,
                1.00,
                0.65,
                0.45,
                0.35,
                0.45,
            ],
            "birdseye_width": 400,
            "birdseye_height": 600,
            "near_distance_m": 0.30,
            "far_distance_m": 2.00,
            "half_width_m": 1.00,
            "open_kernel": 3,
            "close_kernel": 7,
            "min_component_area_px": 80,
            "sample_rows": 7,
            "sample_band_height_px": 30,
            "min_pixels_per_band": 20,
            "polynomial_degree": 2,
            "max_fit_residual_m": 0.20,
            "target_mask_ratio": 0.025,
            "lookahead_min_m": 0.80,
            "lookahead_speed_gain": 1.0,
            "nominal_vx": 0.30,
            "min_vx": 0.05,
            "max_vx": 0.30,
            "lateral_kp": 0.8,
            "lateral_kd": 0.05,
            "yaw_kp": 1.2,
            "curvature_speed_gain": 2.0,
            "lateral_speed_gain": 1.5,
            "heading_speed_gain": 1.0,
            "max_vy": 0.10,
            "max_yaw_rate": 0.40,
            "filter_alpha": 0.20,
            "max_vx_rate": 0.30,
            "max_vy_rate": 0.25,
            "max_yaw_rate_change": 0.80,
            "slow_confidence": 0.70,
            "stop_confidence": 0.40,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _vision_config(self) -> VisionConfig:
        def value(name: str) -> Any:
            return self.get_parameter(name).value

        return VisionConfig(
            hsv_lower=_as_tuple(value("hsv_lower"), int),
            hsv_upper=_as_tuple(value("hsv_upper"), int),
            lab_b_min=int(value("lab_b_min")),
            adaptive_lab_percentile=float(value("adaptive_lab_percentile")),
            clahe_clip_limit=float(value("clahe_clip_limit")),
            clahe_grid_size=int(value("clahe_grid_size")),
            roi_polygon=_as_tuple(value("roi_polygon"), float),
            perspective_source=_as_tuple(value("perspective_source"), float),
            birdseye_width=int(value("birdseye_width")),
            birdseye_height=int(value("birdseye_height")),
            near_distance_m=float(value("near_distance_m")),
            far_distance_m=float(value("far_distance_m")),
            half_width_m=float(value("half_width_m")),
            open_kernel=int(value("open_kernel")),
            close_kernel=int(value("close_kernel")),
            min_component_area_px=int(value("min_component_area_px")),
            sample_rows=int(value("sample_rows")),
            sample_band_height_px=int(value("sample_band_height_px")),
            min_pixels_per_band=int(value("min_pixels_per_band")),
            polynomial_degree=int(value("polynomial_degree")),
            max_fit_residual_m=float(value("max_fit_residual_m")),
            target_mask_ratio=float(value("target_mask_ratio")),
        )

    def _controller_config(self) -> ControllerConfig:
        def value(name: str) -> Any:
            return self.get_parameter(name).value

        return ControllerConfig(
            near_distance_m=float(value("near_distance_m")),
            lookahead_min_m=float(value("lookahead_min_m")),
            lookahead_speed_gain=float(value("lookahead_speed_gain")),
            nominal_vx=float(value("nominal_vx")),
            min_vx=float(value("min_vx")),
            max_vx=float(value("max_vx")),
            lateral_kp=float(value("lateral_kp")),
            lateral_kd=float(value("lateral_kd")),
            yaw_kp=float(value("yaw_kp")),
            curvature_speed_gain=float(value("curvature_speed_gain")),
            lateral_speed_gain=float(value("lateral_speed_gain")),
            heading_speed_gain=float(value("heading_speed_gain")),
            max_vy=float(value("max_vy")),
            max_yaw_rate=float(value("max_yaw_rate")),
            filter_alpha=float(value("filter_alpha")),
            max_vx_rate=float(value("max_vx_rate")),
            max_vy_rate=float(value("max_vy_rate")),
            max_yaw_rate_change=float(value("max_yaw_rate_change")),
            slow_confidence=float(value("slow_confidence")),
            stop_confidence=float(value("stop_confidence")),
        )

    def _publish_command(self, command: VelocityCommand) -> None:
        if self._test_mode and not self._publish_control_in_test_mode:
            command = VelocityCommand(0.0, 0.0, 0.0)
        message = Joy()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("joy_frame_id").value)
        message.axes = command_to_joy_axes(command)
        message.buttons = released_buttons()
        self._command_publisher.publish(message)

    def _build_debug_frame(self, frame: Any, result: VisionResult) -> Any:
        """Draw road/line segmentation and the fitted centerline on a frame."""

        debug = frame.copy()
        if result.road_mask is not None:
            road_overlay = debug.copy()
            road_overlay[result.road_mask > 0] = (0, 180, 0)
            debug = cv2.addWeighted(debug, 0.80, road_overlay, 0.20, 0.0)

        if result.raw_line_mask is not None:
            raw_line_overlay = debug.copy()
            raw_line_overlay[result.raw_line_mask > 0] = (0, 0, 255)
            debug = cv2.addWeighted(debug, 0.85, raw_line_overlay, 0.15, 0.0)

        line_overlay = debug.copy()
        line_overlay[result.mask > 0] = (0, 255, 255)
        debug = cv2.addWeighted(debug, 0.70, line_overlay, 0.30, 0.0)
        cv2.polylines(debug, [result.roi_polygon_px], True, (255, 0, 255), 2)

        if len(result.centerline_points_px) >= 2:
            centerline = np.rint(result.centerline_points_px).astype(np.int32)
            centerline = centerline.reshape(-1, 1, 2)
            cv2.polylines(debug, [centerline], False, (255, 255, 255), 3)
            near_point = tuple(int(value) for value in centerline[0, 0])
            cv2.circle(debug, near_point, 7, (0, 0, 255), -1)
            tracking_text = "centerline: TRACKED"
        else:
            tracking_text = "centerline: LOST"

        confidence = result.estimate.confidence if result.estimate is not None else 0.0
        cv2.putText(
            debug,
            f"line confidence: {confidence:.2f}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0) if confidence >= 0.7 else (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug,
            tracking_text,
            (20, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255) if len(result.centerline_points_px) >= 2 else (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        if self._test_mode:
            cv2.putText(
                debug,
                "TEST | road=green line=yellow center=white",
                (20, 101),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        return debug

    def _publish_test_path(self, source_message: Image, result: VisionResult) -> None:
        if self._test_centerline_publisher is None:
            return

        path = Path()
        path.header = source_message.header
        path.header.frame_id = str(
            self.get_parameter("centerline_frame_id").value
        )
        estimate = result.estimate
        if estimate is not None:
            for forward_x, lateral_y in estimate.points_xy:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(forward_x)
                pose.pose.position.y = float(lateral_y)
                yaw = math.atan(estimate.slope(float(forward_x)))
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                path.poses.append(pose)
        self._test_centerline_publisher.publish(path)

    def _publish_test_outputs(
        self,
        source_message: Image,
        frame: Any,
        result: VisionResult,
        command: VelocityCommand,
    ) -> None:
        """Publish inspectable segmentation, path and scalar test outputs."""

        if not self._test_mode:
            return

        if self._test_debug_publisher is not None:
            debug_message = self._bridge.cv2_to_imgmsg(
                self._build_debug_frame(frame, result), encoding="bgr8"
            )
            debug_message.header = source_message.header
            self._test_debug_publisher.publish(debug_message)

        road_mask = (
            result.road_mask
            if result.road_mask is not None
            else np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
        )
        for publisher, mask in (
            (self._test_road_mask_publisher, road_mask),
            (
                self._test_raw_line_mask_publisher,
                result.raw_line_mask
                if result.raw_line_mask is not None
                else np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8),
            ),
            (self._test_line_mask_publisher, result.mask),
            (self._test_birdseye_mask_publisher, result.birdseye_mask),
        ):
            if publisher is None:
                continue
            mask_message = self._bridge.cv2_to_imgmsg(mask, encoding="mono8")
            mask_message.header = source_message.header
            publisher.publish(mask_message)

        self._publish_test_path(source_message, result)

        if self._test_metrics_publisher is not None:
            estimate = result.estimate
            metrics = {
                "line_detected": estimate is not None,
                "centerline_tracked": len(result.centerline_points_px) >= 2,
                "confidence": float(estimate.confidence) if estimate else 0.0,
                "fit_residual_m": float(estimate.residual_m) if estimate else None,
                "lateral_error_m": float(command.lateral_error),
                "heading_error_rad": float(command.heading_error),
                "curvature": float(command.curvature),
                "sampled_points": int(len(estimate.points_xy)) if estimate else 0,
                "camera_width": int(frame.shape[1]),
                "camera_height": int(frame.shape[0]),
                "camera_profile": str(self.get_parameter("camera_profile").value),
            }
            self._test_metrics_publisher.publish(String(data=json.dumps(metrics)))

    def _publish_debug(
        self, source_message: Image, frame: Any, result: VisionResult
    ) -> None:
        if not bool(self.get_parameter("publish_debug").value) and not self._test_mode:
            return
        debug = self._build_debug_frame(frame, result)
        debug_message = self._bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_message.header = source_message.header
        mask_message = self._bridge.cv2_to_imgmsg(result.mask, encoding="mono8")
        mask_message.header = source_message.header
        if bool(self.get_parameter("publish_debug").value):
            self._debug_publisher.publish(debug_message)
            self._mask_publisher.publish(mask_message)

    def _image_callback(self, message: Image) -> None:
        now = time.monotonic()
        dt = (
            now - self._last_control_monotonic
            if self._last_control_monotonic
            else 1.0 / 20.0
        )
        self._last_frame_monotonic = now
        self._last_control_monotonic = now
        self._watchdog_stopped = False

        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            result = self._vision.process(frame)
        except Exception as error:  # Stop safely on conversion or vision failures.
            self.get_logger().error(f"line detection failed: {error}")
            self._publish_command(self._controller.stop())
            return

        if result.estimate is None:
            command = self._controller.stop()
            confidence = 0.0
        else:
            command = self._controller.generate(result.estimate, dt)
            confidence = result.estimate.confidence

        self._publish_command(command)
        self._confidence_publisher.publish(Float32(data=float(confidence)))
        self._publish_debug(message, frame, result)
        self._publish_test_outputs(message, frame, result, command)

    def _watchdog_callback(self) -> None:
        if self._watchdog_stopped or self._last_frame_monotonic == 0.0:
            return
        timeout = float(self.get_parameter("frame_timeout_sec").value)
        if time.monotonic() - self._last_frame_monotonic > timeout:
            self._publish_command(self._controller.stop())
            self._confidence_publisher.publish(Float32(data=0.0))
            self._watchdog_stopped = True
            self.get_logger().warning("camera frame timeout; published stop command")


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = LineTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node._publish_command(node._controller.stop())
            node.destroy_node()
        except KeyboardInterrupt:
            # ros2 launch can forward a second SIGINT while cleanup is running.
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
