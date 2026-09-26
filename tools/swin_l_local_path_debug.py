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
"""Run Mapillary surface segmentation and local path smoothing.

The ``mcap`` mode replays the supplied rosbag without ROS 2 and writes a
camera-rate MP4 overlay.  Swin-L is intentionally scheduled at a lower rate;
the smoothed path is reused between inference frames.  With ``--overlay-mode
sidewalk``, every camera frame is inferred without path processing.
The ``ros2`` mode publishes only path and metrics topics. The
task-driven mode publishes fail-closed, low-speed Unitree Sport Move requests
only for an accepted Cobiz task with fresh perception inputs.
Both live modes require a Jetson ROS/PyTorch environment.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Sequence

import cv2
import numpy as np
import torch
from apriltag_stop import AprilTagDecision, AprilTagPolicy, AprilTagStopMonitor
from best_so_far_runtime import (
    DEFAULT_EVALUATION_SIZE,
    INFERENCE_BACKENDS,
    PROFILE_NAMES,
    R50_PROFILE,
    SWIN_L_ASPECT_FP16_PROFILE,
    BestSoFarConfig,
    BestSoFarResult,
    BestSoFarSegmenter,
    resolve_profile,
)
from cobiz_line_tracking_task import (
    ActiveTask,
    LineTrackingTasks,
    TaskPolicy,
)
from evaluate_mapillary_temporal import upscale_mask
from local_path import (
    DEFAULT_ROI_POLYGON,
    PATH_MASK_CLASSES,
    LocalPathConfig,
    LocalPathEstimate,
    LocalPathSmoother,
    SmoothedPath,
    extract_sidewalk_centerline,
    ground_to_pixel,
    normalized_polygon_pixels,
    pixel_to_ground_homography,
    selected_path_region,
)
from swin_l_drive_control import (
    MAX_PATH_UNAVAILABLE_INFERENCES,
    DriveConfig,
    DriveDecision,
    decide_drive,
    path_target_lateral,
)
from unitree_sport_api import (
    drive_to_sport_move,
    populate_move_request,
    populate_stop_move_request,
)

DEFAULT_IMAGE_TOPIC = "/a2/front_camera/res_360p/image_raw"
DEFAULT_LOCAL_PATH_TOPIC = "/line_tracking/swin_l/local_path"
DEFAULT_METRICS_TOPIC = "/line_tracking/swin_l/metrics"
PERFORMANCE_WARMUP_FRAMES = 10
PERFORMANCE_SAMPLE_WINDOW = 512


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


def _env_bool(name: str, default: bool) -> bool:
    value = _env(name, "true" if default else "false").lower()
    if value in {"1", "true", "yes", "on"}:
        return True
    if value in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{name} must be a boolean value")


def summarize_performance(
    inference_seconds: Sequence[float],
    processing_seconds: Sequence[float],
    completion_times: Sequence[float],
) -> dict[str, float | int | None]:
    """Summarize synchronized live-pipeline samples after warm-up."""

    inference = np.asarray(inference_seconds, dtype=np.float64)
    processing = np.asarray(processing_seconds, dtype=np.float64)
    completion = np.asarray(completion_times, dtype=np.float64)

    def milliseconds(values: np.ndarray, percentile: float) -> float | None:
        return (
            float(np.percentile(values, percentile) * 1000.0) if values.size else None
        )

    mean_processing = float(np.mean(processing)) if processing.size else 0.0
    elapsed = float(completion[-1] - completion[0]) if completion.size >= 2 else 0.0
    return {
        "sample_count": int(inference.size),
        "warmup_frames": PERFORMANCE_WARMUP_FRAMES,
        "inference_mean_ms": (
            float(np.mean(inference) * 1000.0) if inference.size else None
        ),
        "inference_p95_ms": milliseconds(inference, 95.0),
        "inference_p99_ms": milliseconds(inference, 99.0),
        "processing_mean_ms": float(mean_processing * 1000.0)
        if processing.size
        else None,
        "processing_p95_ms": milliseconds(processing, 95.0),
        "processing_p99_ms": milliseconds(processing, 99.0),
        "completion_gap_max_ms": (
            float(np.max(np.diff(completion)) * 1000.0)
            if completion.size >= 2
            else None
        ),
        "processing_capacity_fps": 1.0 / mean_processing
        if mean_processing > 0.0
        else None,
        "completion_fps": (completion.size - 1) / elapsed if elapsed > 0.0 else None,
    }


def cuda_memory_metrics(device: torch.device) -> dict[str, float] | None:
    """Return memory visible to PyTorch's CUDA allocator."""

    if device.type != "cuda" or not torch.cuda.is_available():
        return None
    scale = 1024.0 * 1024.0
    return {
        "allocated_mib": torch.cuda.memory_allocated(device) / scale,
        "reserved_mib": torch.cuda.memory_reserved(device) / scale,
        "max_allocated_mib": torch.cuda.max_memory_allocated(device) / scale,
        "max_reserved_mib": torch.cuda.max_memory_reserved(device) / scale,
    }


def active_path_mask_class(active_task: ActiveTask | None, default: int) -> int:
    """Use the Cobiz task's class only for the lifetime of that task."""

    return active_task.selected_mask if active_task is not None else default


def extract_path_estimates(
    selected_mask: np.ndarray,
    mask_classes: Sequence[int],
    config: LocalPathConfig,
) -> dict[int, LocalPathEstimate | None]:
    """Compute surface candidates without locking shared live control state."""

    return {
        mask_class: extract_sidewalk_centerline(
            selected_path_region(selected_mask, mask_class), config
        )
        for mask_class in mask_classes
    }


def update_path_smoothers(
    selected_mask: np.ndarray,
    smoothers: dict[int, LocalPathSmoother],
    config: LocalPathConfig,
    timestamp_sec: float,
) -> dict[int, LocalPathEstimate | None]:
    """Keep all configured surface candidates current from one inference."""

    estimates = extract_path_estimates(selected_mask, tuple(smoothers), config)
    for mask_class, smoother in smoothers.items():
        smoother.update(estimates[mask_class], timestamp_sec)
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
        unrestricted_path_mode=args.unrestricted_path_mode,
    )


def _drive_config_from_args(args: argparse.Namespace) -> DriveConfig:
    config = DriveConfig(
        max_forward_mps=args.max_forward_mps,
        max_target_heading_deg=args.max_target_heading_deg,
        min_confidence=(
            0.0 if args.unrestricted_path_mode else DriveConfig.min_confidence
        ),
        stop_on_low_confidence=args.stop_on_low_confidence,
        stop_on_lateral_target=args.stop_on_lateral_target,
        bypass_path_stops=args.bypass_path_stops,
    )
    config.validate()
    return config


def _runtime_config(args: argparse.Namespace) -> BestSoFarConfig:
    return BestSoFarConfig(
        profile=args.profile,
        model_id=args.model_id,
        model_revision=args.model_revision,
        evaluation_height=args.evaluation_size[0],
        evaluation_width=args.evaluation_size[1],
        device=args.device,
        backend=args.backend,
        tensorrt_engine_path=args.tensorrt_engine,
        tensorrt_manifest_path=args.tensorrt_manifest,
        allow_backend_fallback=args.allow_backend_fallback,
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
    # Keep the ROS message (and its backing buffer) alive until inference ends.
    image_message: Any
    sequence: int
    source_header: Any = None


def validate_camera_image(message: Any, bridge: Any) -> None:
    """Check each incoming message without decoding/copying its pixel buffer."""

    dtype, channels = bridge.encoding_to_dtype_with_channels(message.encoding)
    row_bytes = message.width * channels * np.dtype(dtype).itemsize
    if message.width <= 0 or message.height <= 0 or message.step < row_bytes:
        raise ValueError("invalid sensor_msgs/Image dimensions or step")
    if len(message.data) < message.step * message.height:
        raise ValueError("sensor_msgs/Image data is shorter than step * height")


def camera_image_rgb(message: Any, bridge: Any) -> np.ndarray:
    """Decode only the selected message, preserving native RGB and row stride."""

    if message.encoding == "rgb8":
        frame = np.ndarray(
            shape=(message.height, message.width, 3),
            dtype=np.uint8,
            buffer=message.data,
            strides=(message.step, 3, 1),
        )
    else:
        frame = bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
    return np.ascontiguousarray(frame)


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

    def get_latest_at(self, ready_at_sec: float) -> FramePacket | None:
        """Return the freshest frame once the rate-limit deadline is reached.

        Waiting happens while the frame remains in the depth-one queue, so a
        newer camera callback can replace it. Closing the queue interrupts the
        wait immediately.
        """

        with self._condition:
            while not self._closed:
                if self._item is None:
                    self._condition.wait()
                    continue
                remaining = ready_at_sec - time.monotonic()
                if remaining > 0.0:
                    self._condition.wait(timeout=remaining)
                    continue
                item = self._item
                self._item = None
                return item
            return None

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
    config: LocalPathConfig,
    *,
    frame_index: int,
    inference_count: int,
    inference_hz: float,
    path_mask_class: int = 2,
    status_text: str | None = None,
) -> np.ndarray:
    """Render segmentation, raw/final path and optional status on the camera frame."""

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

    lines = (
        f"SWIN-L {PATH_MASK_CLASSES[path_mask_class]} LOCAL PATH",
        f"raw={estimate.confidence:.2f} valid={estimate.valid_ratio:.2f}"
        if estimate
        else "raw=--",
        f"path={'TRACKED' if path else 'LOST'} hold={path.age_sec:.2f}s"
        if path
        else "path=LOST",
        f"inference={inference_hz:.2f}Hz updates={inference_count} frame={frame_index}",
        *((status_text,) if status_text else ()),
        "white=smoothed path orange=raw | magenta=sidewalk",
    )
    panel_height = 22 * len(lines) + 12
    cv2.rectangle(overlay, (10, 10), (500, 10 + panel_height), (0, 0, 0), -1)
    for row, line in enumerate(lines):
        cv2.putText(
            overlay,
            line,
            (20, 31 + row * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
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
    segmenter = BestSoFarSegmenter(_runtime_config(args))
    smoother = LocalPathSmoother(local_config) if local_config else None
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
        for schema, channel, message, decoded in _iter_mcap_events(
            path, (args.image_topic,), start_time_ns=start_ns
        ):
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
                assert local_config is not None and smoother is not None
                estimate = previous_estimate
                if args.unrestricted_path_mode or timestamp_sec >= next_inference:
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
                    next_inference = (
                        -math.inf
                        if args.unrestricted_path_mode
                        else timestamp_sec + inference_period
                    )
                path_now = smoother.current(timestamp_sec)
                overlay = render_local_path_overlay(
                    frame,
                    previous_mask,
                    estimate,
                    path_now,
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
            "inference_policy": (
                "every_frame"
                if args.unrestricted_path_mode or not with_local_path
                else "bag_time_rate"
            ),
            "requested_inference_hz": (
                args.inference_hz
                if with_local_path and not args.unrestricted_path_mode
                else None
            ),
            "image_topic": args.image_topic,
            "frames_written": frame_count,
            "swin_l_updates": inference_count,
            "output": str(args.output.expanduser().resolve()),
            "model": segmenter.metadata(),
            "path_mask_class": args.path_mask_class if with_local_path else None,
            "local_path": asdict(local_config) if local_config else None,
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
    """Reject old and future-dated sensor frames in live task mode."""

    age_sec = (current_stamp_ns - source_stamp_ns) / 1_000_000_000
    return source_stamp_ns > 0 and -0.05 <= age_sec <= max_age_sec


def _effective_source_age_sec(
    arrival_sec: float | None,
    source_stamp_ns: int | None,
    current_monotonic_sec: float,
    current_stamp_ns: int,
) -> float | None:
    """Keep both arrival and original sensor age in the live watchdog."""

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


def _validate_task_drive_preflight(args: argparse.Namespace) -> None:
    """Reject task-driven control unless its model and calibration are explicit."""

    if args.profile not in (SWIN_L_ASPECT_FP16_PROFILE, R50_PROFILE):
        raise ValueError(
            "task-drive mode requires a pinned FP16 deployment profile: "
            f"{SWIN_L_ASPECT_FP16_PROFILE} or {R50_PROFILE}"
        )
    pinned = resolve_profile(args.profile)
    if args.model_id not in (None, pinned.model_id) or args.model_revision not in (
        None,
        pinned.model_revision,
    ):
        raise ValueError("task-drive mode cannot override the pinned checkpoint")
    if args.profile == R50_PROFILE and args.backend != "pytorch":
        raise ValueError("R50 task-drive mode requires the pytorch backend")
    if tuple(args.evaluation_size) != DEFAULT_EVALUATION_SIZE:
        raise ValueError("task-drive mode requires a 360x640 score map")
    if args.path_frame_id != "base_link":
        raise ValueError("task-drive mode requires a calibrated base_link path")
    if args.output_hz < 10.0:
        raise ValueError("task-drive mode requires at least 10 Hz zero-command updates")
    if args.allow_backend_fallback:
        raise ValueError(
            "task-drive mode prohibits automatic inference-backend fallback"
        )
    _drive_config_from_args(args)


def run_ros2(args: argparse.Namespace) -> int:
    try:
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 mode requires sourced rclpy/cv_bridge/sensor_msgs/std_msgs "
            "and a Jetson-compatible PyTorch/Transformers environment"
        ) from error

    task_mode = args.mode == "task-drive"
    Request = None
    if task_mode:
        try:
            from apriltag_msgs.msg import AprilTagDetectionArray
            from rclpy.qos import DurabilityPolicy
            from unitree_api.msg import Request
        except ImportError as error:
            raise RuntimeError(
                "task-drive mode requires the sourced unitree_api and apriltag_msgs "
                "ROS interfaces"
            ) from error
    if task_mode:
        _validate_task_drive_preflight(args)
    local_config = _local_path_config_from_args(args)
    segmenter = BestSoFarSegmenter(_runtime_config(args))
    if task_mode and segmenter.device.type != "cuda":
        raise RuntimeError("task-drive mode requires a CUDA model device")
    smoothers = {
        mask_class: LocalPathSmoother(local_config)
        for mask_class in (PATH_MASK_CLASSES if task_mode else (args.path_mask_class,))
    }
    latest = LatestFrameQueue()
    state_lock = threading.Lock()
    state: dict[str, Any] = {
        "header": None,
        "sequence": 0,
        "inference_count": 0,
        "path_unavailable_inferences": {mask_class: 0 for mask_class in smoothers},
        "performance_inference_seconds": deque(maxlen=PERFORMANCE_SAMPLE_WINDOW),
        "performance_processing_seconds": deque(maxlen=PERFORMANCE_SAMPLE_WINDOW),
        "performance_completion_times": deque(maxlen=PERFORMANCE_SAMPLE_WINDOW),
        "last_image_at": None,
        "last_inference_at": None,
        "last_image_stamp_ns": None,
        "last_inference_stamp_ns": None,
    }
    worker_error: list[BaseException] = []

    class DebugNode(Node):
        def __init__(self) -> None:
            super().__init__(
                "swin_l_task_drive" if task_mode else "swin_l_local_path_debug"
            )
            if (
                task_mode
                and self.has_parameter("use_sim_time")
                and bool(self.get_parameter("use_sim_time").value)
            ):
                raise RuntimeError(
                    "task-drive mode requires live system time, not /clock"
                )
            self.bridge = CvBridge()
            self.drive_config = _drive_config_from_args(args) if task_mode else None
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
            self.last_valid_yaw_rate: float | None = None
            self.last_valid_forward_mps: float | None = None
            self.path_unavailable_inferences = 0
            self.stop_until = 0.0
            self.apriltags = (
                AprilTagStopMonitor(
                    AprilTagPolicy(
                        max_age_sec=args.apriltag_max_age_sec,
                        confirm_window_sec=args.apriltag_confirm_window_sec,
                        min_hits=args.apriltag_confirm_min_hits,
                    )
                )
                if task_mode
                else None
            )
            self.apriltag_callback_sequence = 0
            self.latest_apriltag_ids: tuple[int, ...] = ()
            self.latest_apriltag_frame_key = 0
            self.terminal_apriltag_status: AprilTagDecision | None = None
            self.apriltag_window_resolved_at: float | None = None
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
            self.path_publisher = self.create_publisher(
                __import__("nav_msgs.msg", fromlist=["Path"]).Path,
                args.local_path_topic,
                output_qos,
            )
            self.metrics_publisher = self.create_publisher(
                String, args.metrics_topic, output_qos
            )
            self.command_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.command_publisher = None
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
            if task_mode:
                apriltag_qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                )
                self.apriltag_subscription = self.create_subscription(
                    AprilTagDetectionArray,
                    args.apriltag_detections_topic,
                    self.on_apriltag_detections,
                    apriltag_qos,
                )
            self.timer = self.create_timer(1.0 / args.output_hz, self.publish_state)
            self.started = time.monotonic()
            self.get_logger().info(
                "Swin-L local path debug started | image=%s profile=%s "
                "path_surface=%s inference_hz=%.2f output_hz=%.2f near=%.1fm far=%.1fm"
                % (
                    args.image_topic,
                    args.profile,
                    PATH_MASK_CLASSES[args.path_mask_class],
                    args.inference_hz,
                    args.output_hz,
                    local_config.near_distance_m,
                    local_config.far_distance_m,
                )
            )
            if args.unrestricted_path_mode:
                self.get_logger().warning(
                    "SWIN_L_UNRESTRICTED_PATH_MODE is enabled: path valid-ratio/"
                    "confidence gates and temporal smoothing are bypassed; "
                    "inference rate limiting remains enabled"
                )
            if task_mode:
                self.get_logger().info(
                    "Cobiz LINE_TRACKING task listener ready; direct Sport request "
                    "publisher is absent until a safe task is accepted"
                )

        def publish_task_state(self, body: dict[str, Any]) -> None:
            if self.task_state_publisher is not None:
                self.task_state_publisher.publish(
                    String(data=json.dumps(body, separators=(",", ":")))
                )

        def publish_zero_move(self, reason: str) -> None:
            self.publish_drive(DriveDecision.stop(reason))

        def publish_hard_stop(self, reason: str) -> bool:
            if self.command_publisher is None:
                return True
            assert Request is not None
            succeeded = True
            try:
                self.command_publisher.publish(populate_stop_move_request(Request()))
            except Exception as error:  # noqa: BLE001 - still attempt zero Move.
                succeeded = False
                self.get_logger().error(f"StopMove failed ({reason}): {error}")
            try:
                self.publish_zero_move(reason)
            except Exception as error:  # noqa: BLE001 - task termination must proceed.
                succeeded = False
                self.get_logger().error(f"zero Move failed ({reason}): {error}")
            return succeeded

        def complete_apriltag_task(self, tag_id: int) -> None:
            if not args.stop_on_apriltag:
                return
            reason = f"apriltag_confirmed:{tag_id}"
            if not self.publish_hard_stop(reason):
                self.abort_active_task("stop_publish_error")
                return
            body = self.tasks.finish("TASK_COMPLETED", reason)
            self.last_valid_yaw_rate = None
            self.last_valid_forward_mps = None
            if body is not None:
                self.terminal_apriltag_status = self.apriltags.snapshot(
                    now=time.monotonic()
                )
                self.publish_task_state(body)
            self.stop_until = time.monotonic() + 1.0

        def on_apriltag_detections(self, message: Any) -> None:
            now = time.monotonic()
            stamp_ns = _stamp_ns(message.header)
            self.apriltag_callback_sequence += 1
            frame_key = stamp_ns if stamp_ns > 0 else self.apriltag_callback_sequence
            self.latest_apriltag_ids = tuple(
                int(detection.id) for detection in message.detections
            )
            self.latest_apriltag_frame_key = frame_key
            decision = self.apriltags.observe(
                ids=self.latest_apriltag_ids,
                frame_key=frame_key,
                now=now,
                task_active=args.stop_on_apriltag and self.tasks.active is not None,
            )
            # Evaluate deadlines at this boundary, not a later timer's time:
            # a newly seeded window still owns its full confirmation period.
            if decision.false_positive:
                self.apriltag_window_resolved_at = now
            if decision.stop_now and not self.publish_hard_stop("apriltag_verifying"):
                self.abort_active_task("stop_publish_error")
                return
            if decision.just_confirmed:
                self.complete_apriltag_task(decision.confirmed_id)

        def release_task_control(self, reason: str) -> None:
            self.last_valid_yaw_rate = None
            self.last_valid_forward_mps = None
            if self.command_publisher is not None:
                self.publish_hard_stop(reason)
                self.stop_until = time.monotonic() + 1.0

        def abort_active_task(self, reason: str) -> None:
            if self.tasks is None:
                return
            self.release_task_control(reason)
            body = self.tasks.finish("TASK_ABORTED", reason)
            if body is not None:
                self.apriltags.reset_task()
                self.apriltag_window_resolved_at = None
                self.terminal_apriltag_status = None
                try:
                    self.publish_task_state(body)
                except Exception as error:  # noqa: BLE001 - already deactivated locally.
                    self.get_logger().error(f"TASK_ABORTED report failed: {error}")

        def drive_readiness(
            self, mask_class: int, now: float
        ) -> tuple[SmoothedPath | None, DriveDecision]:
            with state_lock:
                last_image_at = state["last_image_at"]
                last_inference_at = state["last_inference_at"]
                last_image_stamp_ns = state["last_image_stamp_ns"]
                last_inference_stamp_ns = state["last_inference_stamp_ns"]
                path = smoothers[mask_class].current(now)
                self.path_unavailable_inferences = state["path_unavailable_inferences"][
                    mask_class
                ]
            clock_now_ns = self.get_clock().now().nanoseconds
            camera_age_sec = _effective_source_age_sec(
                last_image_at, last_image_stamp_ns, now, clock_now_ns
            )
            inference_age_sec = _effective_source_age_sec(
                last_inference_at, last_inference_stamp_ns, now, clock_now_ns
            )
            decision = decide_drive(
                path,
                camera_age_sec=camera_age_sec,
                inference_age_sec=inference_age_sec,
                config=self.drive_config,
                last_valid_yaw_rate=self.last_valid_yaw_rate,
                last_valid_forward_mps=self.last_valid_forward_mps,
                path_unavailable_inferences=self.path_unavailable_inferences,
            )
            if decision.reason in ("camera_stale", "inference_stale"):
                self.last_valid_yaw_rate = None
                self.last_valid_forward_mps = None
            elif self.tasks.active is not None and decision.reason in (
                "tracking", "tracking_slow_turn"
            ):
                self.last_valid_yaw_rate = decision.yaw_rate
                self.last_valid_forward_mps = decision.vx
            return path, decision

        def on_task_event(self, message: Any) -> None:
            if self.tasks is None:
                return
            try:
                event = json.loads(message.data)
            except (ValueError, TypeError):
                self.get_logger().warning("ignored malformed /task_event JSON")
                return
            now = time.monotonic()
            rejection_reason = None
            if self.command_publisher is not None and self.tasks.active is None:
                rejection_reason = "control_release_pending"
            previous_task = self.tasks.active
            body = self.tasks.handle_event(
                event, now=now, rejection_reason=rejection_reason
            )
            if body is None:
                return
            if body["type"] == "TASK_STARTED":
                self.last_valid_yaw_rate = None
                self.last_valid_forward_mps = None
                self.path_unavailable_inferences = 0
                with state_lock:
                    state["path_unavailable_inferences"] = {
                        mask_class: 0 for mask_class in smoothers
                    }
                self.terminal_apriltag_status = None
                self.apriltag_window_resolved_at = None
                try:
                    assert Request is not None
                    self.command_publisher = self.create_publisher(
                        Request, args.sport_request_topic, self.command_qos
                    )
                    tag_status = self.apriltags.begin_task(
                        ids=self.latest_apriltag_ids if args.stop_on_apriltag else (),
                        frame_key=self.latest_apriltag_frame_key,
                        now=now,
                    )
                    if tag_status.stop_now:
                        if not self.publish_hard_stop("apriltag_verifying"):
                            raise RuntimeError("initial hard stop publication failed")
                    else:
                        self.publish_zero_move("startup_hold")
                except Exception as error:  # noqa: BLE001 - never report start without control.
                    self.get_logger().error(
                        f"failed to acquire direct Sport control: {error}"
                    )
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
                self.apriltags.reset_task()
                self.apriltag_window_resolved_at = None
                self.release_task_control("task_aborted_by_server")
            self.publish_task_state(body)

        def publish_drive(self, decision: DriveDecision) -> None:
            if self.command_publisher is None:
                return
            assert Request is not None
            message = populate_move_request(
                Request(),
                drive_to_sport_move(
                    vx=decision.vx,
                    vy=decision.vy,
                    yaw_rate=decision.yaw_rate,
                ),
            )
            self.command_publisher.publish(message)

        def on_image(self, message: Any) -> None:
            try:
                source_stamp_ns = _stamp_ns(message.header) if task_mode else None
                if task_mode and (
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
                    self.last_valid_yaw_rate = None
                    self.last_valid_forward_mps = None
                    self.publish_drive(DriveDecision.stop("camera_timestamp_invalid"))
                    return
                validate_camera_image(message, self.bridge)
                accepted = latest.put(
                    FramePacket(
                        image_message=message,
                        sequence=int(state["sequence"]),
                        source_header=message.header,
                    )
                )
                if accepted:
                    with state_lock:
                        state["sequence"] += 1
                        state["last_image_at"] = time.monotonic()
                        if task_mode:
                            state["last_image_stamp_ns"] = source_stamp_ns
            except Exception as error:  # noqa: BLE001 - safe debug boundary.
                self.camera_conversion_failed(error)

        def camera_conversion_failed(self, error: Exception) -> None:
            # Also used by the worker after deferred image conversion fails.
            if task_mode:
                with state_lock:
                    state["last_image_at"] = None
                self.last_valid_yaw_rate = None
                self.last_valid_forward_mps = None
                self.publish_drive(DriveDecision.stop("camera_conversion_error"))
            self.get_logger().error(f"camera conversion failed: {error}")

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
            now = time.monotonic()
            tag_status = None
            if self.apriltags is not None:
                tag_status = (
                    self.apriltags.tick(now=now, task_active=args.stop_on_apriltag)
                    if self.tasks.active is not None
                    else self.apriltags.snapshot(now=now)
                )
                if tag_status.just_confirmed:
                    self.complete_apriltag_task(tag_status.confirmed_id)
                tag_status = self.apriltags.snapshot(now=now)
            task_active = self.tasks.active if self.tasks is not None else None
            mask_class = active_path_mask_class(task_active, args.path_mask_class)
            with state_lock:
                header = state["header"]
                inference_count = int(state["inference_count"])
                performance = summarize_performance(
                    list(state["performance_inference_seconds"]),
                    list(state["performance_processing_seconds"]),
                    list(state["performance_completion_times"]),
                )
            drive_decision = None
            if self.drive_config is not None:
                path, ready_decision = self.drive_readiness(mask_class, now)
                self.last_ready_reason = ready_decision.reason
                startup_hold = (
                    task_active is not None
                    and now - task_active.started_at
                    < self.tasks.policy.startup_hold_sec
                )
                drive_decision = (
                    DriveDecision.stop("apriltag_verifying")
                    if tag_status.state == "verifying"
                    else DriveDecision.stop("startup_hold")
                    if startup_hold
                    else ready_decision
                )
                if task_mode:
                    if task_active is not None:
                        lifecycle_now = (
                            self.apriltag_window_resolved_at
                            if tag_status.state == "verifying"
                            else now
                        )
                        terminal = (
                            None
                            if lifecycle_now is None
                            else self.tasks.tick(
                                now=lifecycle_now, drive_reason=drive_decision.reason
                            )
                        )
                        self.apriltag_window_resolved_at = None
                        if terminal is not None:
                            self.apriltags.reset_task()
                            tag_status = self.apriltags.snapshot(now=now)
                            drive_decision = DriveDecision.stop(
                                terminal.get("reason", "task_complete")
                            )
                            self.release_task_control(drive_decision.reason)
                            self.publish_task_state(terminal)
                        else:
                            self.publish_drive(drive_decision)
                    elif self.command_publisher is not None:
                        drive_decision = DriveDecision.stop("task_idle")
                        self.publish_drive(drive_decision)
                        if now >= self.stop_until:
                            self.destroy_publisher(self.command_publisher)
                            self.command_publisher = None
                            self.apriltags.reset_task()
                            self.terminal_apriltag_status = None
                            tag_status = self.apriltags.snapshot(now=now)
                    else:
                        drive_decision = DriveDecision.stop("task_idle")
                else:
                    if now - self.started < 2.0:
                        drive_decision = DriveDecision.stop("startup_hold")
                    self.publish_drive(drive_decision)
            else:
                path = smoothers[mask_class].current(now)
            # Real callbacks keep refreshing liveness after completion. Retain
            # the terminal display independently until control is released.
            if self.terminal_apriltag_status is not None:
                tag_status = self.terminal_apriltag_status
            metrics = {
                "profile": args.profile,
                "camera_topic": args.image_topic,
                "path_mask_class": mask_class,
                "path_surface": PATH_MASK_CLASSES[mask_class],
                "path_tracked": path is not None,
                "path_confidence": float(path.confidence) if path else 0.0,
                "path_age_sec": float(path.age_sec) if path else None,
                "path_duration_sec": local_config.path_duration_sec,
                "unrestricted_path_mode": args.unrestricted_path_mode,
                "inference_target_hz": args.inference_hz,
                "near_distance_m": local_config.near_distance_m,
                "far_distance_m": local_config.far_distance_m,
                "queue_overwritten": latest.overwritten,
                "inference_count": inference_count,
                "drive_reason": drive_decision.reason if drive_decision else None,
                "ready_reason": self.last_ready_reason if task_mode else None,
                "task_active": self.tasks.active is not None if self.tasks else None,
                "performance": performance,
                "cuda_memory": cuda_memory_metrics(segmenter.device),
            }
            if tag_status is not None:
                metrics["apriltag"] = {
                    "topic": args.apriltag_detections_topic,
                    "stop_enabled": args.stop_on_apriltag,
                    "stream_ready": self.apriltags.stream_ready(now),
                    "message_age_sec": self.apriltags.message_age_sec(now),
                    "state": tag_status.state,
                    "hit_counts": dict(tag_status.hit_counts),
                    "confirmed_id": tag_status.confirmed_id,
                    "window_elapsed_sec": tag_status.window_elapsed_sec,
                }
            if self.drive_config is not None:
                lateral = path_target_lateral(path, self.drive_config.lookahead_m)
                metrics["turn_speed_control"] = {
                    "target_heading_deg": (
                        math.degrees(math.atan2(lateral, self.drive_config.lookahead_m))
                        if lateral is not None else None
                    ),
                    "max_target_heading_deg": self.drive_config.max_target_heading_deg,
                    "command_forward_mps": drive_decision.vx,
                    "command_yaw_deg_sec": math.degrees(drive_decision.yaw_rate),
                }
                metrics["path_stop_bypass"] = self.drive_config.bypass_path_stops
                metrics["path_unavailable_inferences"] = self.path_unavailable_inferences
                metrics["path_unavailable_limit"] = MAX_PATH_UNAVAILABLE_INFERENCES
                metrics["path_yaw_held"] = (
                    drive_decision is not None
                    and drive_decision.reason == "tracking_path_hold"
                )
                metrics["stop_checks"] = {
                    "camera_freshness": True,
                    "inference_freshness": True,
                    "path_available": (
                        not self.drive_config.bypass_path_stops
                        or self.path_unavailable_inferences
                        >= MAX_PATH_UNAVAILABLE_INFERENCES
                    ),
                    "low_confidence": (
                        not self.drive_config.bypass_path_stops
                        and self.drive_config.stop_on_low_confidence
                        and self.drive_config.min_confidence > 0.0
                    ),
                    "lateral_target": (
                        not self.drive_config.bypass_path_stops
                        and self.drive_config.stop_on_lateral_target
                    ),
                    "apriltag": args.stop_on_apriltag,
                }
            self.metrics_publisher.publish(String(data=json.dumps(metrics)))
            if header is None:
                return
            path_message = _path_message(path, header, args.path_frame_id)
            self.path_publisher.publish(path_message)

    rclpy.init(args=[])
    node = DebugNode()
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def stop_on_sigterm(_signum: int, _frame: Any) -> None:
        if task_mode:
            try:
                node.abort_active_task("sigterm")
            except Exception as error:  # noqa: BLE001 - still terminate the process.
                node.get_logger().error(f"SIGTERM stop publish failed: {error}")
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, stop_on_sigterm)

    def worker() -> None:
        # Path acceptance settings must never disable the live GPU budget.
        inference_period = 1.0 / args.inference_hz
        next_allowed = time.monotonic()
        try:
            while rclpy.ok():
                packet = latest.get_latest_at(next_allowed)
                if packet is None:
                    break
                inference_started_at = time.monotonic()
                # Limit start-to-start frequency, including conversion failures.
                # Slow inference must not incur an extra fixed-period sleep.
                next_allowed = inference_started_at + inference_period
                try:
                    frame_rgb = camera_image_rgb(packet.image_message, node.bridge)
                except Exception as error:  # noqa: BLE001 - retain camera fault handling.
                    node.camera_conversion_failed(error)
                    continue
                result: BestSoFarResult = segmenter.segment(
                    frame_rgb, color_order="rgb"
                )
                # The path becomes usable when this result is available. Using
                # the camera-arrival timestamp here can expire a path before it
                # is ever published when inference or rate limiting is slow.
                path_updated_at = time.monotonic()
                estimates = extract_path_estimates(
                    result.selected_mask, tuple(smoothers), local_config
                )
                with state_lock:
                    # Publish paths, loss streaks and source freshness as one
                    # completed inference. Timer ticks never advance a streak.
                    for mask_class, smoother in smoothers.items():
                        smoother.update(estimates[mask_class], path_updated_at)
                    inference_finished_at = time.monotonic()
                    if task_mode:
                        counts = state["path_unavailable_inferences"]
                        for mask_class, smoother in smoothers.items():
                            available = (
                                path_target_lateral(
                                    smoother.current(inference_finished_at),
                                    node.drive_config.lookahead_m,
                                ) is not None
                            )
                            counts[mask_class] = 0 if available else counts[mask_class] + 1
                    state["header"] = packet.source_header
                    completed_before = int(state["inference_count"])
                    state["inference_count"] += 1
                    state["last_inference_at"] = inference_finished_at
                    if completed_before >= PERFORMANCE_WARMUP_FRAMES:
                        state["performance_inference_seconds"].append(
                            result.inference_seconds
                        )
                        state["performance_processing_seconds"].append(
                            inference_finished_at - inference_started_at
                        )
                        state["performance_completion_times"].append(
                            inference_finished_at
                        )
                    if task_mode:
                        state["last_inference_stamp_ns"] = _stamp_ns(
                            packet.source_header
                        )
        except BaseException as error:  # noqa: BLE001 - forward to main thread.
            worker_error.append(error)
            if task_mode:
                try:
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
        if task_mode:
            try:
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
        default=_env("SWIN_L_PROFILE", SWIN_L_ASPECT_FP16_PROFILE),
    )
    parser.add_argument("--model-id", default=_env("SWIN_L_MODEL_ID", "") or None)
    parser.add_argument(
        "--model-revision", default=_env("SWIN_L_MODEL_REVISION", "") or None
    )
    parser.add_argument("--device", default=_env("SWIN_L_DEVICE", "auto"))
    parser.add_argument(
        "--backend",
        choices=INFERENCE_BACKENDS,
        default=_env("SWIN_L_BACKEND", "pytorch"),
    )
    parser.add_argument(
        "--tensorrt-engine",
        default=_env("SWIN_L_TRT_ENGINE", "") or None,
    )
    parser.add_argument(
        "--tensorrt-manifest",
        default=_env("SWIN_L_TRT_MANIFEST", "") or None,
    )
    parser.add_argument(
        "--allow-backend-fallback",
        action=argparse.BooleanOptionalAction,
        default=_env_bool("SWIN_L_ALLOW_BACKEND_FALLBACK", False),
    )
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
        "--path-mask-class",
        type=int,
        choices=tuple(PATH_MASK_CLASSES),
        default=_env("SWIN_L_PATH_MASK_CLASS", "2"),
        help="0=road or sidewalk, 1=road, 2=sidewalk (default from SWIN_L_PATH_MASK_CLASS)",
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
        "--unrestricted-path-mode",
        action=argparse.BooleanOptionalAction,
        default=_env_bool("SWIN_L_UNRESTRICTED_PATH_MODE", True),
        help=(
            "bypass path valid-ratio, confidence, and temporal smoothing "
            "restrictions; live inference still obeys --inference-hz"
        ),
    )
    parser.add_argument(
        "--output-fps", type=float, default=_env_float("SWIN_L_OUTPUT_FPS", 20.0)
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
        help="sidewalk infers every camera frame; local-path also smooths paths",
    )
    for mode, help_text in (
        ("ros2", "publish live ROS 2 path and debug topics without control"),
        (
            "task-drive",
            "wait for a Cobiz LINE_TRACKING task before direct Unitree Sport control",
        ),
    ):
        live = subparsers.add_parser(mode, help=help_text)
        _add_common_arguments(live)
        live.add_argument(
            "--local-path-topic",
            default=_env("SWIN_L_LOCAL_PATH_TOPIC", DEFAULT_LOCAL_PATH_TOPIC),
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
        if mode == "task-drive":
            live.add_argument(
                "--bypass-path-stops",
                action=argparse.BooleanOptionalAction,
                default=_env_bool("LINE_TRACKING_BYPASS_PATH_STOPS", False),
                help="Bypass path-based stops while preserving camera/inference checks",
            )
            for check in ("low_confidence", "lateral_target", "apriltag"):
                live.add_argument(
                    "--stop-on-" + check.replace("_", "-"),
                    action=argparse.BooleanOptionalAction,
                    default=_env_bool("LINE_TRACKING_STOP_ON_" + check.upper(), True),
                    help="Enable or disable this automatic stop check only",
                )
            live.add_argument(
                "--apriltag-detections-topic",
                default=_env("SWIN_L_APRILTAG_DETECTIONS_TOPIC", "/detections"),
            )
            live.add_argument(
                "--apriltag-max-age-sec",
                type=float,
                default=_env_float("SWIN_L_APRILTAG_MAX_AGE_SEC", 1.0),
            )
            live.add_argument(
                "--apriltag-confirm-window-sec",
                type=float,
                default=_env_float("SWIN_L_APRILTAG_CONFIRM_WINDOW_SEC", 1.0),
            )
            live.add_argument(
                "--apriltag-confirm-min-hits",
                type=int,
                default=_env_int("SWIN_L_APRILTAG_CONFIRM_MIN_HITS", 3),
            )
            live.add_argument(
                "--sport-request-topic",
                default=_env("LINE_TRACKING_SPORT_REQUEST_TOPIC", "/api/sport/request"),
            )
            live.add_argument(
                "--max-forward-mps",
                type=float,
                default=_env_float("LINE_TRACKING_MAX_FORWARD_MPS", 0.50),
            )
            live.add_argument(
                "--max-target-heading-deg",
                type=float,
                default=_env_float("LINE_TRACKING_MAX_TARGET_HEADING_DEG", 60.0),
                help="Maximum absolute path target bearing; independent of yaw rate",
            )
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
                default=_env_float("LINE_TRACKING_DEFAULT_DURATION_SEC", 500.0),
            )
            live.add_argument(
                "--max-task-duration-sec",
                type=float,
                default=_env_float("LINE_TRACKING_MAX_DURATION_SEC", 10000.0),
            )
            live.add_argument(
                "--unsafe-timeout-sec",
                type=float,
                default=_env_float("LINE_TRACKING_UNSAFE_TIMEOUT_SEC", 2.0),
            )
    args = parser.parse_args(argv)
    if args.path_mask_class not in PATH_MASK_CLASSES:
        parser.error(
            "SWIN_L_PATH_MASK_CLASS must be 0 (road or sidewalk), "
            "1 (road), or 2 (sidewalk)"
        )
    if not all(
        math.isfinite(rate) and rate > 0.0
        for rate in (args.inference_hz, args.output_fps)
    ):
        parser.error("inference/output FPS must be finite and positive")
    if args.mode in ("ros2", "task-drive") and args.output_hz <= 0.0:
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
