#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = [
#   "mcap==1.4.0",
#   "mcap-ros2-support==0.5.7",
#   "numpy==2.5.2",
#   "opencv-python-headless==5.0.0.93",
#   "pillow==12.3.0",
#   "scipy==1.18.1",
#   "torch==2.13.0",
#   "torchvision==0.28.0",
#   "transformers==5.16.1",
# ]
# ///
"""Debug Swin-L sidewalk segmentation, local path smoothing and LiDAR gating.

The ``mcap`` mode replays the supplied rosbag without ROS 2 and writes a
camera-rate MP4 overlay.  Swin-L is intentionally scheduled at a lower rate;
the smoothed path is reused between inference frames.  With ``--overlay-mode
sidewalk``, every camera frame is inferred without path or LiDAR processing.
The ``ros2`` mode publishes only path, metrics and safety-stop topics. The
separate ``drive`` mode also publishes fail-closed, low-speed A2 Joy commands;
it requires explicit drive enablement and confirmed camera/LiDAR calibration.
Both live modes require a Jetson ROS/PyTorch environment.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict, dataclass, replace
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
from typing import Any, Sequence

import cv2
import numpy as np

from best_so_far_runtime import (
    DEFAULT_EVALUATION_SIZE,
    PROFILE_NAMES,
    SWIN_L_ASPECT_PROFILE,
    resolve_profile,
    BestSoFarConfig,
    BestSoFarResult,
    BestSoFarSegmenter,
)
from evaluate_mapillary_temporal import upscale_mask
from local_path import (
    DEFAULT_ROI_POLYGON,
    LidarSafetyConfig,
    LidarSafetyMonitor,
    LidarSafetyResult,
    LocalPathConfig,
    LocalPathEstimate,
    LocalPathSmoother,
    SmoothedPath,
    extract_sidewalk_centerline,
    ground_to_pixel,
    normalized_polygon_pixels,
    pixel_to_ground_homography,
    pointcloud2_xyz,
)
from swin_l_drive_control import (
    DriveConfig,
    DriveDecision,
    decide_drive,
    lidar_frame_matches_base,
)
from cobiz_line_tracking_task import (
    ActiveTask,
    LineTrackingTasks,
    TaskPolicy,
    requested_selected_mask,
)


DEFAULT_IMAGE_TOPIC = "/a2/front_camera/res_360p/image_raw"
DEFAULT_OVERLAY_TOPIC = "/line_tracking/swin_l/overlay"
DEFAULT_LOCAL_PATH_TOPIC = "/line_tracking/swin_l/local_path"
DEFAULT_SAFETY_STOP_TOPIC = "/line_tracking/swin_l/safety_stop"
DEFAULT_CLEARANCE_TOPIC = "/line_tracking/swin_l/clearance_m"
DEFAULT_METRICS_TOPIC = "/line_tracking/swin_l/metrics"
PATH_MASK_CLASSES = {1: "ROAD", 2: "SIDEWALK"}


def _load_dotenv_values() -> dict[str, str]:
    """Read a local .env for direct script use; process env wins later."""

    candidates = [Path.cwd() / ".env", Path(__file__).resolve().parents[1] / ".env"]
    values: dict[str, str] = {}
    for path in candidates:
        if not path.is_file():
            continue
        for raw_line in path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            name, raw_value = line.split("=", 1)
            name = name.strip()
            value = raw_value.strip()
            if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
                value = value[1:-1]
            values[name] = value
    values.update({key: value for key, value in os.environ.items()})
    return values


ENV = _load_dotenv_values()


def _env(name: str, default: str) -> str:
    return str(ENV.get(name, default)).strip()


def _env_float(name: str, default: float) -> float:
    return float(_env(name, str(default)))


def _env_int(name: str, default: int) -> int:
    return int(_env(name, str(default)))


def selected_path_region(selected_mask: np.ndarray, path_mask_class: int) -> np.ndarray:
    """Select only the configured drivable class; background is never a path."""

    if path_mask_class not in PATH_MASK_CLASSES:
        raise ValueError("SWIN_L_PATH_MASK_CLASS must be 1 (road) or 2 (sidewalk)")
    return selected_mask == path_mask_class


def active_path_mask_class(active_task: ActiveTask | None, default: int) -> int:
    """Use the Cobiz task's class only for the lifetime of that task."""

    return active_task.selected_mask if active_task is not None else default


def update_path_smoothers(
    selected_mask: np.ndarray,
    smoothers: dict[int, LocalPathSmoother],
    config: LocalPathConfig,
    timestamp_sec: float,
) -> dict[int, LocalPathEstimate | None]:
    """Keep both task-mode path candidates current for safe task preflight."""

    estimates = {}
    for mask_class, smoother in smoothers.items():
        estimate = extract_sidewalk_centerline(
            selected_path_region(selected_mask, mask_class), config
        )
        smoother.update(estimate, timestamp_sec)
        estimates[mask_class] = estimate
    return estimates


def _parse_polygon(value: str) -> tuple[float, ...]:
    try:
        values = tuple(
            float(item.strip()) for item in value.replace(";", ",").split(",")
        )
    except ValueError as error:
        raise ValueError("SWIN_L_ROI_POLYGON must be comma-separated floats") from error
    if len(values) != 8:
        raise ValueError("SWIN_L_ROI_POLYGON must contain eight values")
    return values


def _local_path_config_from_args(args: argparse.Namespace) -> LocalPathConfig:
    polygon = (
        args.roi_polygon
        if args.roi_polygon is not None
        else _parse_polygon(
            _env("SWIN_L_ROI_POLYGON", ",".join(map(str, DEFAULT_ROI_POLYGON)))
        )
    )
    return LocalPathConfig(
        near_distance_m=args.near_distance_m,
        far_distance_m=args.far_distance_m,
        ground_half_width_m=args.ground_half_width_m,
        search_half_width_m=args.search_half_width_m,
        roi_polygon=polygon,
        path_points=args.path_points,
        bev_width_px=args.bev_width_px,
        bev_height_px=args.bev_height_px,
        min_valid_ratio=args.min_valid_ratio,
        min_sidewalk_width_m=args.min_sidewalk_width_m,
        close_kernel_px=args.close_kernel_px,
        smoothing_time_constant_sec=args.smoothing_time_constant_sec,
        max_lateral_update_m=args.max_lateral_update_m,
        path_hold_sec=args.path_hold_sec,
        path_duration_sec=args.path_duration_sec,
    )


def _lidar_config_from_args(args: argparse.Namespace) -> LidarSafetyConfig:
    return LidarSafetyConfig(
        topic=args.lidar_topic,
        timeout_sec=args.lidar_timeout_sec,
        obstacle_distance_m=args.obstacle_distance_m,
        stop_distance_m=args.stop_distance_m,
        corridor_half_width_m=args.corridor_half_width_m,
        z_min_m=args.lidar_z_min_m,
        z_max_m=args.lidar_z_max_m,
        min_obstacle_points=args.min_obstacle_points,
    )


def _runtime_config(args: argparse.Namespace) -> BestSoFarConfig:
    return BestSoFarConfig(
        profile=args.profile,
        model_id=args.model_id,
        model_revision=args.model_revision,
        evaluation_height=args.evaluation_size[0],
        evaluation_width=args.evaluation_size[1],
        device=args.device,
    )


def decode_ros_image(decoded: Any) -> np.ndarray:
    """Decode a sensor_msgs/Image-like object to contiguous BGR8."""

    encoding = str(decoded.encoding).lower()
    formats = {
        "rgb8": (3, cv2.COLOR_RGB2BGR),
        "bgr8": (3, None),
        "rgba8": (4, cv2.COLOR_RGBA2BGR),
        "bgra8": (4, cv2.COLOR_BGRA2BGR),
        "mono8": (1, cv2.COLOR_GRAY2BGR),
    }
    if encoding not in formats:
        raise ValueError(f"unsupported camera encoding: {encoding}")
    channels, conversion = formats[encoding]
    width = int(decoded.width)
    height = int(decoded.height)
    step = int(decoded.step)
    row_bytes = width * channels
    if width <= 0 or height <= 0 or step < row_bytes:
        raise ValueError("invalid sensor_msgs/Image dimensions or step")
    raw = np.frombuffer(bytes(decoded.data), dtype=np.uint8)
    if raw.size < step * height:
        raise ValueError("sensor_msgs/Image data is shorter than step * height")
    rows = raw[: step * height].reshape(height, step)[:, :row_bytes]
    image = (
        rows.reshape(height, width, channels)
        if channels > 1
        else rows.reshape(height, width)
    )
    if conversion is not None:
        image = cv2.cvtColor(image, conversion)
    return np.ascontiguousarray(image)


@dataclass(frozen=True)
class FramePacket:
    frame_bgr: np.ndarray
    timestamp_sec: float
    sequence: int
    source_header: Any = None


class LatestFrameQueue:
    """Depth-one queue: old camera frames are replaced, never accumulated."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._item: FramePacket | None = None
        self._closed = False
        self.overwritten = 0

    def put(self, item: FramePacket) -> bool:
        with self._condition:
            if self._closed:
                return False
            if self._item is not None:
                self.overwritten += 1
            self._item = item
            self._condition.notify()
            return True

    def get(self) -> FramePacket | None:
        with self._condition:
            while self._item is None and not self._closed:
                self._condition.wait()
            if self._item is None:
                return None
            item = self._item
            self._item = None
            return item

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()


def _draw_polyline(
    image: np.ndarray, points: np.ndarray, color: tuple[int, int, int], width: int
) -> None:
    if points.shape[0] < 2:
        return
    valid = np.all(np.isfinite(points), axis=1)
    points = points[valid]
    if points.shape[0] < 2:
        return
    cv2.polylines(
        image,
        [np.rint(points).astype(np.int32).reshape(-1, 1, 2)],
        False,
        color,
        width,
        cv2.LINE_AA,
    )


def render_local_path_overlay(
    frame_bgr: np.ndarray,
    selected_mask: np.ndarray,
    estimate: LocalPathEstimate | None,
    path: SmoothedPath | None,
    safety: LidarSafetyResult,
    config: LocalPathConfig,
    *,
    frame_index: int,
    inference_count: int,
    inference_hz: float,
    path_mask_class: int = 2,
) -> np.ndarray:
    """Render segmentation, raw/final path and LiDAR state on the camera frame."""

    overlay = frame_bgr.copy()
    mask = upscale_mask(selected_mask, frame_bgr)
    road = overlay.copy()
    road[mask == 1] = (40, 180, 40)
    sidewalk = overlay.copy()
    sidewalk[mask == 2] = (220, 60, 220)
    overlay = cv2.addWeighted(overlay, 0.68, road, 0.32, 0.0)
    overlay = cv2.addWeighted(overlay, 0.70, sidewalk, 0.30, 0.0)

    roi = normalized_polygon_pixels(config.roi_polygon, frame_bgr.shape[:2])
    cv2.polylines(overlay, [roi.reshape(-1, 1, 2)], True, (255, 180, 0), 2)
    if estimate is not None:
        homography = pixel_to_ground_homography(frame_bgr.shape[:2], config)
        raw_pixels = ground_to_pixel(estimate.points_xy, homography)
        _draw_polyline(overlay, raw_pixels, (0, 165, 255), 3)
    if path is not None:
        homography = pixel_to_ground_homography(frame_bgr.shape[:2], config)
        smoothed_pixels = ground_to_pixel(path.points_xy, homography)
        _draw_polyline(overlay, smoothed_pixels, (255, 255, 255), 5)
        for pixel in smoothed_pixels[:: max(1, len(smoothed_pixels) // 6)]:
            if np.all(np.isfinite(pixel)):
                cv2.circle(
                    overlay, tuple(np.rint(pixel).astype(int)), 4, (255, 255, 255), -1
                )

    if safety.stop:
        safety_text = f"LIDAR STOP: {safety.reason}"
        safety_color = (0, 0, 255)
    elif safety.lidar_available:
        clearance = "--" if safety.clearance_m is None else f"{safety.clearance_m:.1f}m"
        safety_text = f"LIDAR CLEAR: {clearance}"
        safety_color = (0, 220, 0)
    else:
        safety_text = f"LIDAR: {safety.reason}"
        safety_color = (0, 165, 255)
    lines = (
        f"SWIN-L {PATH_MASK_CLASSES[path_mask_class]} LOCAL PATH",
        f"raw={estimate.confidence:.2f} valid={estimate.valid_ratio:.2f}"
        if estimate
        else "raw=--",
        f"path={'TRACKED' if path else 'LOST'} hold={path.age_sec:.2f}s"
        if path
        else "path=LOST",
        f"inference={inference_hz:.2f}Hz updates={inference_count} frame={frame_index}",
        safety_text,
        "white=smoothed path orange=raw | magenta=sidewalk",
    )
    panel_height = 22 * len(lines) + 12
    cv2.rectangle(overlay, (10, 10), (500, 10 + panel_height), (0, 0, 0), -1)
    for row, line in enumerate(lines):
        color = safety_color if row == 4 else (255, 255, 255)
        cv2.putText(
            overlay,
            line,
            (20, 31 + row * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            1,
            cv2.LINE_AA,
        )
    return overlay


def _build_writer(output: Path, frame: np.ndarray, fps: float) -> cv2.VideoWriter:
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output),
        cv2.VideoWriter_fourcc(*"mp4v"),
        max(1.0, fps),
        (frame.shape[1], frame.shape[0]),
    )
    if not writer.isOpened():
        raise RuntimeError(f"could not open MP4 writer: {output}")
    return writer


def _iter_mcap_events(path: Path, topics: Sequence[str], start_time_ns: int = 0):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    with path.open("rb") as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        yield from reader.iter_decoded_messages(
            topics=list(topics), start_time=start_time_ns
        )


def run_mcap(args: argparse.Namespace) -> int:
    path = args.input.expanduser().resolve()
    if not path.is_file() or path.suffix.lower() != ".mcap":
        raise FileNotFoundError(f"MCAP input does not exist: {path}")
    with_local_path = args.overlay_mode == "local-path"
    local_config = _local_path_config_from_args(args) if with_local_path else None
    lidar_config = _lidar_config_from_args(args) if with_local_path else None
    segmenter = BestSoFarSegmenter(_runtime_config(args))
    smoother = LocalPathSmoother(local_config) if local_config else None
    lidar = LidarSafetyMonitor(lidar_config) if lidar_config else None
    writer: cv2.VideoWriter | None = None
    inference_period = 1.0 / args.inference_hz
    next_inference = -math.inf
    frame_count = 0
    inference_count = 0
    previous_estimate: LocalPathEstimate | None = None
    previous_mask = np.zeros((0, 0), dtype=np.uint8)
    inference_times: deque[float] = deque(maxlen=32)
    start_ns = 0
    if args.start_offset > 0.0:
        from mcap.reader import make_reader

        with path.open("rb") as stream:
            summary = make_reader(stream).get_summary()
        if summary is None or summary.statistics is None:
            raise RuntimeError("MCAP has no readable summary")
        start_ns = int(summary.statistics.message_start_time + args.start_offset * 1e9)

    try:
        topics = (
            (args.image_topic, args.lidar_topic)
            if with_local_path
            else (args.image_topic,)
        )
        for schema, channel, message, decoded in _iter_mcap_events(
            path, topics, start_time_ns=start_ns
        ):
            if lidar is not None and channel.topic == args.lidar_topic:
                try:
                    lidar.update(pointcloud2_xyz(decoded), message.log_time / 1e9)
                except ValueError:
                    continue
                continue
            if channel.topic != args.image_topic:
                continue
            if schema.name != "sensor_msgs/msg/Image":
                raise ValueError(f"camera topic type is {schema.name}, expected Image")
            timestamp_sec = message.log_time / 1e9
            frame = decode_ros_image(decoded)
            if writer is None:
                writer = _build_writer(
                    args.output.expanduser().resolve(), frame, args.output_fps
                )
            if not with_local_path:
                # Preserve the quality baseline: infer every original camera
                # frame, with no MP4 re-encoding or lower-rate mask reuse.
                result = segmenter.segment(frame)
                inference_count += 1
                overlay = segmenter.render_overlay(
                    frame,
                    result.selected_mask,
                    frame_index=frame_count,
                    fps=args.output_fps,
                )
            else:
                assert (
                    local_config is not None
                    and smoother is not None
                    and lidar is not None
                )
                estimate = previous_estimate
                if timestamp_sec >= next_inference:
                    result = segmenter.segment(frame)
                    previous_mask = result.selected_mask
                    estimate = extract_sidewalk_centerline(
                        selected_path_region(
                            result.selected_mask, args.path_mask_class
                        ),
                        local_config,
                    )
                    previous_estimate = estimate
                    smoother.update(estimate, timestamp_sec)
                    inference_count += 1
                    inference_times.append(result.total_seconds)
                    next_inference = timestamp_sec + inference_period
                path_now = smoother.current(timestamp_sec)
                safety = lidar.evaluate(path_now, timestamp_sec)
                overlay = render_local_path_overlay(
                    frame,
                    previous_mask,
                    estimate,
                    path_now,
                    safety,
                    local_config,
                    frame_index=frame_count,
                    inference_count=inference_count,
                    inference_hz=(
                        1.0 / float(np.mean(inference_times))
                        if inference_times
                        else 0.0
                    ),
                    path_mask_class=args.path_mask_class,
                )
            assert writer is not None
            writer.write(overlay)
            frame_count += 1
            if frame_count % 100 == 0:
                print(
                    f"MCAP_PROGRESS frames={frame_count} updates={inference_count}",
                    flush=True,
                )
            if args.max_frames and frame_count >= args.max_frames:
                break
    finally:
        if writer is not None:
            writer.release()
    if frame_count == 0:
        raise RuntimeError(f"no camera frames found on {args.image_topic}")
    if args.report is not None:
        report = {
            "source": str(path),
            "overlay_mode": args.overlay_mode,
            "start_offset_sec": args.start_offset,
            "output_fps": args.output_fps,
            "inference_policy": "bag_time_rate" if with_local_path else "every_frame",
            "requested_inference_hz": args.inference_hz if with_local_path else None,
            "image_topic": args.image_topic,
            "lidar_topic": args.lidar_topic if with_local_path else None,
            "frames_written": frame_count,
            "swin_l_updates": inference_count,
            "output": str(args.output.expanduser().resolve()),
            "model": segmenter.metadata(),
            "path_mask_class": args.path_mask_class if with_local_path else None,
            "local_path": asdict(local_config) if local_config else None,
            "lidar_safety": asdict(lidar_config) if lidar_config else None,
        }
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        f"MCAP_COMPLETE frames={frame_count} swin_l_updates={inference_count} output={args.output}"
    )
    return 0


def _now_stamp(node: Any) -> Any:
    return node.get_clock().now().to_msg()


def _stamp_ns(header: Any) -> int:
    """Require a usable source timestamp for live driving freshness checks."""

    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _live_source_stamp(
    source_stamp_ns: int, current_stamp_ns: int, max_age_sec: float
) -> bool:
    """Reject old rosbag frames and future-dated sensor frames in drive mode."""

    age_sec = (current_stamp_ns - source_stamp_ns) / 1_000_000_000
    return source_stamp_ns > 0 and -0.05 <= age_sec <= max_age_sec


def _effective_source_age_sec(
    arrival_sec: float | None,
    source_stamp_ns: int | None,
    current_monotonic_sec: float,
    current_stamp_ns: int,
) -> float | None:
    """Keep both arrival and original sensor age in the drive watchdog."""

    if arrival_sec is None or source_stamp_ns is None:
        return None
    return max(
        current_monotonic_sec - arrival_sec,
        (current_stamp_ns - source_stamp_ns) / 1_000_000_000,
    )


def _path_message(path: SmoothedPath | None, header: Any, frame_id: str) -> Any:
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path

    message = Path()
    message.header.stamp = header.stamp
    message.header.frame_id = frame_id
    if path is None:
        return message
    for (forward_x, lateral_y), next_point in zip(path.points_xy, path.points_xy[1:]):
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x = float(forward_x)
        pose.pose.position.y = float(lateral_y)
        yaw = math.atan2(
            float(next_point[1] - lateral_y), float(next_point[0] - forward_x)
        )
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.poses.append(pose)
    if path.points_xy.shape[0] >= 1:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x = float(path.points_xy[-1, 0])
        pose.pose.position.y = float(path.points_xy[-1, 1])
        message.poses.append(pose)
    return message


def _validate_drive_preflight(
    args: argparse.Namespace, *, require_arm: bool = True
) -> None:
    """Reject live control unless the selected model and calibration are explicit."""

    enabled = _env("SWIN_L_DRIVE_ENABLED", "false").lower() in {"1", "true", "yes"}
    calibrated = _env("SWIN_L_CALIBRATION_CONFIRMED", "false").lower() in {
        "1",
        "true",
        "yes",
    }
    if require_arm and (not enabled or not calibrated):
        raise RuntimeError(
            "drive mode requires SWIN_L_DRIVE_ENABLED=true and "
            "SWIN_L_CALIBRATION_CONFIRMED=true after camera/LiDAR frame validation"
        )
    pinned = resolve_profile(SWIN_L_ASPECT_PROFILE)
    if args.profile != SWIN_L_ASPECT_PROFILE:
        raise ValueError("drive mode is pinned to swin-l-aspect-224x384")
    if args.model_id not in (None, pinned.model_id) or args.model_revision not in (
        None,
        pinned.model_revision,
    ):
        raise ValueError("drive mode cannot override the pinned checkpoint")
    if tuple(args.evaluation_size) != DEFAULT_EVALUATION_SIZE:
        raise ValueError("drive mode requires a 360x640 score map")
    if args.path_frame_id != "base_link":
        raise ValueError("drive mode requires a calibrated base_link path")
    if args.output_hz < 10.0:
        raise ValueError("drive mode requires at least 10 Hz zero-command updates")


def run_ros2(args: argparse.Namespace) -> int:
    try:
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image, Joy, PointCloud2
        from std_msgs.msg import Bool, Float32, String
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 mode requires sourced rclpy/cv_bridge/sensor_msgs/std_msgs "
            "and a Jetson-compatible PyTorch/Transformers environment"
        ) from error

    task_mode = args.mode == "task-drive"
    drive_mode = args.mode in ("drive", "task-drive")
    # Inspection mode keeps only lightweight path, safety and metrics outputs.
    extended_diagnostics = drive_mode
    if drive_mode:
        _validate_drive_preflight(args, require_arm=not task_mode)
    task_armed = _env("SWIN_L_DRIVE_ENABLED", "false").lower() in {
        "1",
        "true",
        "yes",
    } and _env("SWIN_L_CALIBRATION_CONFIRMED", "false").lower() in {"1", "true", "yes"}
    local_config = _local_path_config_from_args(args)
    lidar_config = _lidar_config_from_args(args)
    segmenter = BestSoFarSegmenter(_runtime_config(args))
    if drive_mode and segmenter.device.type != "cuda":
        raise RuntimeError("Swin-L drive mode requires a CUDA model device")
    smoothers = {
        mask_class: LocalPathSmoother(local_config)
        for mask_class in ((1, 2) if task_mode else (args.path_mask_class,))
    }
    lidar = LidarSafetyMonitor(lidar_config)
    latest = LatestFrameQueue()
    state_lock = threading.Lock()
    state: dict[str, Any] = {
        "frame": None,
        "selected_mask": None,
        "estimates": {},
        "header": None,
        "sequence": 0,
        "inference_count": 0,
        "inference_times": deque(maxlen=32),
        "last_image_at": None,
        "last_inference_at": None,
        "last_image_stamp_ns": None,
        "last_inference_stamp_ns": None,
        "last_lidar_stamp_ns": None,
    }
    worker_error: list[BaseException] = []

    class DebugNode(Node):
        def __init__(self) -> None:
            super().__init__(
                "swin_l_drive" if drive_mode else "swin_l_local_path_debug"
            )
            if (
                drive_mode
                and self.has_parameter("use_sim_time")
                and bool(self.get_parameter("use_sim_time").value)
            ):
                raise RuntimeError("drive mode requires live system time, not /clock")
            self.bridge = CvBridge()
            self.drive_config = DriveConfig() if drive_mode else None
            self.tasks = (
                LineTrackingTasks(
                    TaskPolicy(
                        default_duration_sec=args.default_task_duration_sec,
                        max_duration_sec=args.max_task_duration_sec,
                        unsafe_timeout_sec=args.unsafe_timeout_sec,
                        default_selected_mask=args.path_mask_class,
                    )
                )
                if task_mode
                else None
            )
            self.last_ready_reason = "inputs_not_ready"
            self.stop_until = 0.0
            if self.drive_config is not None:
                self.drive_config.validate()
            reliability = (
                ReliabilityPolicy.RELIABLE
                if args.reliability == "reliable"
                else ReliabilityPolicy.BEST_EFFORT
            )
            input_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=reliability,
            )
            output_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.image_subscription = self.create_subscription(
                Image, args.image_topic, self.on_image, input_qos
            )
            self.lidar_subscription = self.create_subscription(
                PointCloud2, args.lidar_topic, self.on_lidar, input_qos
            )
            self.path_publisher = self.create_publisher(
                __import__("nav_msgs.msg", fromlist=["Path"]).Path,
                args.local_path_topic,
                output_qos,
            )
            self.overlay_publisher = (
                self.create_publisher(Image, args.overlay_topic, input_qos)
                if extended_diagnostics
                else None
            )
            self.safety_publisher = self.create_publisher(
                Bool, args.safety_stop_topic, output_qos
            )
            self.clearance_publisher = (
                self.create_publisher(Float32, args.clearance_topic, output_qos)
                if extended_diagnostics
                else None
            )
            self.metrics_publisher = self.create_publisher(
                String, args.metrics_topic, output_qos
            )
            self.command_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self.command_publisher = (
                self.create_publisher(Joy, args.joy_topic, self.command_qos)
                if drive_mode and not task_mode
                else None
            )
            self.task_state_publisher = (
                self.create_publisher(String, args.task_state_topic, output_qos)
                if task_mode
                else None
            )
            self.task_event_subscription = (
                self.create_subscription(
                    String, args.task_event_topic, self.on_task_event, output_qos
                )
                if task_mode
                else None
            )
            self.timer = self.create_timer(1.0 / args.output_hz, self.publish_state)
            self.started = time.monotonic()
            self.get_logger().info(
                "Swin-L local path debug started | image=%s lidar=%s profile=%s "
                "path_surface=%s inference_hz=%.2f output_hz=%.2f near=%.1fm far=%.1fm"
                % (
                    args.image_topic,
                    args.lidar_topic,
                    args.profile,
                    PATH_MASK_CLASSES[args.path_mask_class],
                    args.inference_hz,
                    args.output_hz,
                    local_config.near_distance_m,
                    local_config.far_distance_m,
                )
            )
            self.get_logger().warning(
                "LiDAR x/y are evaluated in the PointCloud2 frame; configure the "
                "safety thresholds after confirming hesai_lidar-to-base_link alignment"
            )
            if task_mode:
                self.get_logger().info(
                    "Cobiz LINE_TRACKING task listener ready; Joy publisher is absent until a safe task is accepted"
                )
            elif drive_mode:
                self.get_logger().warning(
                    "Swin-L DRIVE mode armed for low-speed Joy output; "
                    "camera/LiDAR calibration and exclusive control ownership "
                    "must have been verified before starting"
                )

        def publish_task_state(self, body: dict[str, Any]) -> None:
            if self.task_state_publisher is not None:
                self.task_state_publisher.publish(
                    String(data=json.dumps(body, separators=(",", ":")))
                )

        def release_task_control(self, reason: str) -> None:
            if self.command_publisher is not None:
                self.publish_drive(DriveDecision.stop(reason))
                self.stop_until = time.monotonic() + 1.0

        def abort_active_task(self, reason: str) -> None:
            if self.tasks is None:
                return
            body = self.tasks.finish("TASK_ABORTED", reason)
            if body is not None:
                self.release_task_control(reason)
                self.publish_task_state(body)

        def drive_readiness(
            self, mask_class: int, now: float
        ) -> tuple[SmoothedPath | None, LidarSafetyResult, DriveDecision]:
            with state_lock:
                last_image_at = state["last_image_at"]
                last_inference_at = state["last_inference_at"]
                last_image_stamp_ns = state["last_image_stamp_ns"]
                last_inference_stamp_ns = state["last_inference_stamp_ns"]
                last_lidar_stamp_ns = state["last_lidar_stamp_ns"]
            path = smoothers[mask_class].current(now)
            safety = lidar.evaluate(path, now)
            clock_now_ns = self.get_clock().now().nanoseconds
            camera_age_sec = _effective_source_age_sec(
                last_image_at, last_image_stamp_ns, now, clock_now_ns
            )
            inference_age_sec = _effective_source_age_sec(
                last_inference_at, last_inference_stamp_ns, now, clock_now_ns
            )
            if safety.lidar_available and last_lidar_stamp_ns is not None:
                safety = replace(
                    safety,
                    age_sec=max(
                        safety.age_sec or 0.0,
                        (clock_now_ns - last_lidar_stamp_ns) / 1_000_000_000,
                    ),
                )
            publisher_count = self.count_publishers(args.joy_topic)
            other_publishers = publisher_count > (
                1 if self.command_publisher is not None else 0
            )
            decision = decide_drive(
                path,
                safety,
                camera_age_sec=camera_age_sec,
                inference_age_sec=inference_age_sec,
                other_control_publishers=other_publishers,
                enabled=True,
                calibrated=task_armed if task_mode else True,
                config=self.drive_config,
            )
            return path, safety, decision

        def on_task_event(self, message: Any) -> None:
            if self.tasks is None:
                return
            try:
                event = json.loads(message.data)
            except (ValueError, TypeError):
                self.get_logger().warning("ignored malformed /task_event JSON")
                return
            now = time.monotonic()
            ready_reason = "inputs_not_ready"
            if (
                isinstance(event, dict)
                and event.get("type") == "TASK_REGISTERED"
                and event.get("action_name") == "LINE_TRACKING"
                and self.tasks.active is None
            ):
                try:
                    mask_class = requested_selected_mask(
                        event, self.tasks.policy.default_selected_mask
                    )
                except ValueError:
                    pass  # The task lifecycle reports the precise payload error.
                else:
                    ready_reason = self.drive_readiness(mask_class, now)[2].reason
            if self.command_publisher is not None and self.tasks.active is None:
                ready_reason = "control_release_pending"
            previous_task = self.tasks.active
            body = self.tasks.handle_event(event, now=now, ready_reason=ready_reason)
            if body is None:
                return
            if body["type"] == "TASK_STARTED":
                try:
                    self.command_publisher = self.create_publisher(
                        Joy, args.joy_topic, self.command_qos
                    )
                    self.publish_drive(DriveDecision.stop("startup_hold"))
                except Exception as error:  # noqa: BLE001 - never report start without control.
                    self.get_logger().error(f"failed to acquire Joy control: {error}")
                    failed = self.tasks.finish(
                        "TASK_REJECTED", "control_publisher_error"
                    )
                    if self.command_publisher is not None:
                        self.destroy_publisher(self.command_publisher)
                        self.command_publisher = None
                    if failed is not None:
                        self.publish_task_state(failed)
                    return
            elif previous_task is not None and self.tasks.active is None:
                self.release_task_control("task_aborted_by_server")
            self.publish_task_state(body)

        def publish_drive(self, decision: DriveDecision) -> None:
            if self.command_publisher is None:
                return
            message = Joy()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "swin_l_drive"
            message.axes = decision.joy_axes()
            message.buttons = [0] * 10
            self.command_publisher.publish(message)

        def on_image(self, message: Any) -> None:
            try:
                source_stamp_ns = _stamp_ns(message.header) if drive_mode else None
                if drive_mode and (
                    not _live_source_stamp(
                        source_stamp_ns,
                        self.get_clock().now().nanoseconds,
                        self.drive_config.max_camera_age_sec,
                    )
                    or (
                        state["last_image_stamp_ns"] is not None
                        and source_stamp_ns <= state["last_image_stamp_ns"]
                    )
                ):
                    with state_lock:
                        state["last_image_at"] = None
                    self.publish_drive(DriveDecision.stop("camera_timestamp_invalid"))
                    return
                frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                accepted = latest.put(
                    FramePacket(
                        frame_bgr=np.ascontiguousarray(frame),
                        timestamp_sec=time.monotonic(),
                        sequence=int(state["sequence"]),
                        source_header=message.header,
                    )
                )
                if accepted:
                    with state_lock:
                        state["sequence"] += 1
                        state["last_image_at"] = time.monotonic()
                        if drive_mode:
                            state["last_image_stamp_ns"] = source_stamp_ns
            except Exception as error:  # noqa: BLE001 - safe debug boundary.
                if drive_mode:
                    with state_lock:
                        state["last_image_at"] = None
                    self.publish_drive(DriveDecision.stop("camera_conversion_error"))
                self.get_logger().error(f"camera conversion failed: {error}")

        def on_lidar(self, message: Any) -> None:
            try:
                source_stamp_ns = _stamp_ns(message.header) if drive_mode else None
                if drive_mode and (
                    not _live_source_stamp(
                        source_stamp_ns,
                        self.get_clock().now().nanoseconds,
                        self.drive_config.max_lidar_age_sec,
                    )
                    or (
                        state["last_lidar_stamp_ns"] is not None
                        and source_stamp_ns <= state["last_lidar_stamp_ns"]
                    )
                ):
                    lidar.invalidate()
                    self.publish_drive(DriveDecision.stop("lidar_timestamp_invalid"))
                    return
                if drive_mode and not lidar_frame_matches_base(
                    str(message.header.frame_id), args.path_frame_id
                ):
                    self.get_logger().warning(
                        "LiDAR scan is not in base_link; no transform is applied "
                        "and drive remains stopped"
                    )
                    lidar.invalidate()
                    self.publish_drive(DriveDecision.stop("lidar_frame_invalid"))
                    return
                points = pointcloud2_xyz(message)
                if drive_mode and not np.any(np.all(np.isfinite(points), axis=1)):
                    lidar.invalidate()
                    self.publish_drive(DriveDecision.stop("lidar_empty_scan"))
                    return
                lidar.update(points, time.monotonic())
                if drive_mode:
                    with state_lock:
                        state["last_lidar_stamp_ns"] = source_stamp_ns
            except Exception as error:  # noqa: BLE001 - ignore malformed scan.
                if drive_mode:
                    lidar.invalidate()
                    self.publish_drive(DriveDecision.stop("lidar_decode_error"))
                self.get_logger().warning(f"LiDAR decode failed: {error}")

        def publish_state(self) -> None:
            try:
                self._publish_state()
            except Exception as error:  # noqa: BLE001 - stop on any timer fault.
                try:
                    self.publish_drive(DriveDecision.stop("publish_error"))
                except Exception:  # noqa: BLE001 - still report task failure.
                    pass
                self.abort_active_task("publish_error")
                self.get_logger().error(f"Swin-L output failed: {error}")

        def _publish_state(self) -> None:
            task_active = self.tasks.active if self.tasks is not None else None
            mask_class = active_path_mask_class(task_active, args.path_mask_class)
            with state_lock:
                frame = (
                    state["frame"].copy()
                    if extended_diagnostics and state["frame"] is not None
                    else None
                )
                selected = (
                    state["selected_mask"].copy()
                    if extended_diagnostics and state["selected_mask"] is not None
                    else None
                )
                estimate = (
                    state["estimates"].get(mask_class) if extended_diagnostics else None
                )
                header = state["header"]
                sequence = int(state["sequence"]) if extended_diagnostics else 0
                inference_count = int(state["inference_count"])
                inference_times = (
                    list(state["inference_times"]) if extended_diagnostics else []
                )
            now = time.monotonic()
            drive_decision = None
            if self.drive_config is not None:
                path, safety, ready_decision = self.drive_readiness(mask_class, now)
                self.last_ready_reason = ready_decision.reason
                startup_hold = (
                    task_active is not None
                    and now - task_active.started_at
                    < self.tasks.policy.startup_hold_sec
                )
                drive_decision = (
                    DriveDecision.stop("startup_hold")
                    if startup_hold
                    else ready_decision
                )
                if task_mode:
                    if task_active is not None:
                        terminal = self.tasks.tick(
                            now=now, drive_reason=drive_decision.reason
                        )
                        if terminal is not None:
                            self.release_task_control(
                                terminal.get("reason", "task_complete")
                            )
                            self.publish_task_state(terminal)
                        else:
                            self.publish_drive(drive_decision)
                    elif self.command_publisher is not None:
                        drive_decision = DriveDecision.stop("task_idle")
                        self.publish_drive(drive_decision)
                        if now >= self.stop_until:
                            self.destroy_publisher(self.command_publisher)
                            self.command_publisher = None
                    else:
                        drive_decision = DriveDecision.stop("task_idle")
                else:
                    if now - self.started < 2.0:
                        drive_decision = DriveDecision.stop("startup_hold")
                    self.publish_drive(drive_decision)
            else:
                path = smoothers[mask_class].current(now)
                safety = lidar.evaluate(path, now)
            self.safety_publisher.publish(Bool(data=bool(safety.stop)))
            if self.clearance_publisher is not None:
                self.clearance_publisher.publish(
                    Float32(data=float(safety.clearance_m or 0.0))
                )
            metrics = {
                "profile": args.profile,
                "camera_topic": args.image_topic,
                "lidar_topic": args.lidar_topic,
                "path_mask_class": mask_class,
                "path_surface": PATH_MASK_CLASSES[mask_class],
                "path_tracked": path is not None,
                "path_confidence": float(path.confidence) if path else 0.0,
                "path_age_sec": float(path.age_sec) if path else None,
                "path_duration_sec": local_config.path_duration_sec,
                "near_distance_m": local_config.near_distance_m,
                "far_distance_m": local_config.far_distance_m,
                "lidar": asdict(safety),
                "queue_overwritten": latest.overwritten,
                "inference_count": inference_count,
                "drive_reason": drive_decision.reason if drive_decision else None,
                "ready_reason": self.last_ready_reason if task_mode else None,
                "task_active": self.tasks.active is not None if self.tasks else None,
            }
            self.metrics_publisher.publish(String(data=json.dumps(metrics)))
            if header is None:
                return
            path_message = _path_message(path, header, args.path_frame_id)
            self.path_publisher.publish(path_message)
            if self.overlay_publisher is None or frame is None or selected is None:
                return
            mean_total = float(np.mean(inference_times)) if inference_times else 0.0
            inference_hz = 1.0 / mean_total if mean_total > 0.0 else 0.0
            overlay = render_local_path_overlay(
                frame,
                selected,
                estimate,
                path,
                safety,
                local_config,
                frame_index=sequence,
                inference_count=inference_count,
                inference_hz=inference_hz,
                path_mask_class=mask_class,
            )
            overlay_message = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            overlay_message.header = header
            self.overlay_publisher.publish(overlay_message)

    rclpy.init(args=[])
    node = DebugNode()
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def stop_on_sigterm(_signum: int, _frame: Any) -> None:
        if drive_mode:
            try:
                node.publish_drive(DriveDecision.stop("sigterm"))
                node.abort_active_task("sigterm")
            except Exception as error:  # noqa: BLE001 - still terminate the process.
                node.get_logger().error(f"SIGTERM stop publish failed: {error}")
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, stop_on_sigterm)

    def worker() -> None:
        next_allowed = time.monotonic()
        try:
            while rclpy.ok():
                packet = latest.get()
                if packet is None:
                    break
                delay = next_allowed - time.monotonic()
                if delay > 0.0:
                    time.sleep(delay)
                started = time.perf_counter() if extended_diagnostics else 0.0
                result: BestSoFarResult = segmenter.segment(packet.frame_bgr)
                with state_lock:
                    estimates = update_path_smoothers(
                        result.selected_mask,
                        smoothers,
                        local_config,
                        packet.timestamp_sec,
                    )
                    if extended_diagnostics:
                        state["frame"] = packet.frame_bgr
                        state["selected_mask"] = result.selected_mask
                        state["estimates"] = estimates
                        state["inference_times"].append(time.perf_counter() - started)
                    state["header"] = packet.source_header
                    state["inference_count"] += 1
                    state["last_inference_at"] = time.monotonic()
                    if drive_mode:
                        state["last_inference_stamp_ns"] = _stamp_ns(
                            packet.source_header
                        )
                next_allowed = time.monotonic() + 1.0 / args.inference_hz
        except BaseException as error:  # noqa: BLE001 - forward to main thread.
            worker_error.append(error)
            if drive_mode:
                try:
                    node.publish_drive(DriveDecision.stop("inference_error"))
                    node.abort_active_task("inference_error")
                except Exception:  # noqa: BLE001 - shutdown must still proceed.
                    pass
            if rclpy.ok():
                rclpy.shutdown()

    thread = threading.Thread(target=worker, name="swin-l-inference", daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if drive_mode:
            try:
                node.publish_drive(DriveDecision.stop("shutdown"))
                node.abort_active_task("shutdown")
            except Exception as error:  # noqa: BLE001 - complete shutdown regardless.
                node.get_logger().error(f"shutdown stop publish failed: {error}")
        latest.close()
        thread.join(timeout=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_sigterm)
    if worker_error:
        raise worker_error[0]
    return 0


def _add_common_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--profile",
        choices=PROFILE_NAMES,
        default=_env("SWIN_L_PROFILE", SWIN_L_ASPECT_PROFILE),
    )
    parser.add_argument("--model-id", default=_env("SWIN_L_MODEL_ID", "") or None)
    parser.add_argument(
        "--model-revision", default=_env("SWIN_L_MODEL_REVISION", "") or None
    )
    parser.add_argument("--device", default=_env("SWIN_L_DEVICE", "auto"))
    parser.add_argument(
        "--evaluation-size",
        type=int,
        nargs=2,
        default=(
            _env_int("SWIN_L_EVALUATION_HEIGHT", DEFAULT_EVALUATION_SIZE[0]),
            _env_int("SWIN_L_EVALUATION_WIDTH", DEFAULT_EVALUATION_SIZE[1]),
        ),
        metavar=("HEIGHT", "WIDTH"),
    )
    parser.add_argument(
        "--image-topic", default=_env("SWIN_L_IMAGE_TOPIC", DEFAULT_IMAGE_TOPIC)
    )
    parser.add_argument(
        "--lidar-topic", default=_env("SWIN_L_LIDAR_TOPIC", LidarSafetyConfig.topic)
    )
    parser.add_argument(
        "--path-mask-class",
        type=int,
        choices=tuple(PATH_MASK_CLASSES),
        default=_env("SWIN_L_PATH_MASK_CLASS", "2"),
        help="1=road, 2=sidewalk (default from SWIN_L_PATH_MASK_CLASS)",
    )
    parser.add_argument(
        "--near-distance-m",
        type=float,
        default=_env_float("SWIN_L_NEAR_DISTANCE_M", 3.0),
    )
    parser.add_argument(
        "--far-distance-m", type=float, default=_env_float("SWIN_L_FAR_DISTANCE_M", 8.0)
    )
    parser.add_argument(
        "--ground-half-width-m",
        type=float,
        default=_env_float("SWIN_L_GROUND_HALF_WIDTH_M", 4.0),
    )
    parser.add_argument(
        "--search-half-width-m",
        type=float,
        default=_env_float("SWIN_L_SEARCH_HALF_WIDTH_M", 3.5),
    )
    parser.add_argument(
        "--path-points", type=int, default=_env_int("SWIN_L_PATH_POINTS", 20)
    )
    parser.add_argument(
        "--bev-width-px", type=int, default=_env_int("SWIN_L_BEV_WIDTH_PX", 280)
    )
    parser.add_argument(
        "--bev-height-px", type=int, default=_env_int("SWIN_L_BEV_HEIGHT_PX", 160)
    )
    parser.add_argument(
        "--min-valid-ratio",
        type=float,
        default=_env_float("SWIN_L_MIN_VALID_RATIO", 0.35),
    )
    parser.add_argument(
        "--min-sidewalk-width-m",
        type=float,
        default=_env_float("SWIN_L_MIN_SIDEWALK_WIDTH_M", 0.12),
    )
    parser.add_argument(
        "--close-kernel-px", type=int, default=_env_int("SWIN_L_CLOSE_KERNEL_PX", 5)
    )
    parser.add_argument(
        "--smoothing-time-constant-sec",
        type=float,
        default=_env_float("SWIN_L_SMOOTHING_TIME_CONSTANT_SEC", 0.80),
    )
    parser.add_argument(
        "--max-lateral-update-m",
        type=float,
        default=_env_float("SWIN_L_MAX_LATERAL_UPDATE_M", 0.35),
    )
    parser.add_argument(
        "--path-hold-sec", type=float, default=_env_float("SWIN_L_PATH_HOLD_SEC", 0.90)
    )
    parser.add_argument(
        "--path-duration-sec",
        type=float,
        default=_env_float("SWIN_L_PATH_DURATION_SEC", 1.50),
    )
    parser.add_argument(
        "--inference-hz", type=float, default=_env_float("SWIN_L_INFERENCE_HZ", 4.0)
    )
    parser.add_argument(
        "--output-fps", type=float, default=_env_float("SWIN_L_OUTPUT_FPS", 20.0)
    )
    parser.add_argument(
        "--obstacle-distance-m",
        type=float,
        default=_env_float("SWIN_L_OBSTACLE_DISTANCE_M", 8.0),
    )
    parser.add_argument(
        "--stop-distance-m",
        type=float,
        default=_env_float("SWIN_L_STOP_DISTANCE_M", 3.0),
    )
    parser.add_argument(
        "--corridor-half-width-m",
        type=float,
        default=_env_float("SWIN_L_CORRIDOR_HALF_WIDTH_M", 0.55),
    )
    parser.add_argument(
        "--lidar-timeout-sec",
        type=float,
        default=_env_float("SWIN_L_LIDAR_TIMEOUT_SEC", 0.35),
    )
    parser.add_argument(
        "--lidar-z-min-m", type=float, default=_env_float("SWIN_L_LIDAR_Z_MIN_M", -0.40)
    )
    parser.add_argument(
        "--lidar-z-max-m", type=float, default=_env_float("SWIN_L_LIDAR_Z_MAX_M", 0.80)
    )
    parser.add_argument(
        "--min-obstacle-points",
        type=int,
        default=_env_int("SWIN_L_MIN_OBSTACLE_POINTS", 3),
    )
    parser.add_argument(
        "--roi-polygon",
        type=float,
        nargs=8,
        default=None,
        metavar=("BL_X", "BL_Y", "BR_X", "BR_Y", "TR_X", "TR_Y", "TL_X", "TL_Y"),
    )


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="mode", required=True)
    mcap = subparsers.add_parser("mcap", help="replay an MCAP and write an overlay MP4")
    _add_common_arguments(mcap)
    mcap.add_argument("--input", type=Path, required=True)
    mcap.add_argument("--output", type=Path, required=True)
    mcap.add_argument("--report", type=Path, default=None)
    mcap.add_argument("--start-offset", type=float, default=0.0)
    mcap.add_argument("--max-frames", type=int, default=200)
    mcap.add_argument(
        "--overlay-mode",
        choices=("sidewalk", "local-path"),
        default="local-path",
        help="sidewalk infers every camera frame; local-path also smooths paths and checks LiDAR",
    )
    for mode, help_text in (
        ("ros2", "publish live ROS 2 path and debug topics without control"),
        ("drive", "publish fail-closed, low-speed Joy commands after preflight"),
        ("task-drive", "wait for a Cobiz LINE_TRACKING task before Joy control"),
    ):
        live = subparsers.add_parser(mode, help=help_text)
        _add_common_arguments(live)
        if mode != "ros2":
            live.add_argument(
                "--overlay-topic",
                default=_env("SWIN_L_OVERLAY_TOPIC", DEFAULT_OVERLAY_TOPIC),
            )
        live.add_argument(
            "--local-path-topic",
            default=_env("SWIN_L_LOCAL_PATH_TOPIC", DEFAULT_LOCAL_PATH_TOPIC),
        )
        live.add_argument(
            "--safety-stop-topic",
            default=_env("SWIN_L_SAFETY_STOP_TOPIC", DEFAULT_SAFETY_STOP_TOPIC),
        )
        if mode != "ros2":
            live.add_argument(
                "--clearance-topic",
                default=_env("SWIN_L_CLEARANCE_TOPIC", DEFAULT_CLEARANCE_TOPIC),
            )
        live.add_argument(
            "--metrics-topic",
            default=_env("SWIN_L_METRICS_TOPIC", DEFAULT_METRICS_TOPIC),
        )
        live.add_argument(
            "--path-frame-id", default=_env("SWIN_L_PATH_FRAME_ID", "base_link")
        )
        live.add_argument(
            "--output-hz", type=float, default=_env_float("SWIN_L_OUTPUT_HZ", 10.0)
        )
        live.add_argument(
            "--reliability",
            choices=("best_effort", "reliable"),
            default=_env("SWIN_L_INPUT_RELIABILITY", "best_effort"),
        )
        if mode in ("drive", "task-drive"):
            live.add_argument("--joy-topic", default=_env("JOY_TOPIC", "/a2_control"))
        if mode == "task-drive":
            live.add_argument(
                "--task-event-topic",
                default=_env("LINE_TRACKING_TASK_EVENT_TOPIC", "/task_event"),
            )
            live.add_argument(
                "--task-state-topic",
                default=_env("LINE_TRACKING_TASK_STATE_TOPIC", "/task_state"),
            )
            live.add_argument(
                "--default-task-duration-sec",
                type=float,
                default=_env_float("LINE_TRACKING_DEFAULT_DURATION_SEC", 60.0),
            )
            live.add_argument(
                "--max-task-duration-sec",
                type=float,
                default=_env_float("LINE_TRACKING_MAX_DURATION_SEC", 300.0),
            )
            live.add_argument(
                "--unsafe-timeout-sec",
                type=float,
                default=_env_float("LINE_TRACKING_UNSAFE_TIMEOUT_SEC", 2.0),
            )
    args = parser.parse_args(argv)
    if args.path_mask_class not in PATH_MASK_CLASSES:
        parser.error("SWIN_L_PATH_MASK_CLASS must be 1 (road) or 2 (sidewalk)")
    if args.inference_hz <= 0.0 or args.output_fps <= 0.0:
        parser.error("inference/output FPS must be positive")
    if args.mode in ("ros2", "drive", "task-drive") and args.output_hz <= 0.0:
        parser.error("output-hz must be positive")
    if args.mode == "mcap" and (args.start_offset < 0.0 or args.max_frames < 0):
        parser.error("start-offset and max-frames must be non-negative")
    if args.mode == "mcap":
        args.output = args.output.expanduser().resolve()
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.mode == "mcap":
        return run_mcap(args)
    return run_ros2(args)


if __name__ == "__main__":
    raise SystemExit(main())
