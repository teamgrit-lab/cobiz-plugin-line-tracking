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
camera-rate MP4 overlay.  Surface is intentionally scheduled at a lower rate;
the smoothed path is reused between inference frames.  With ``--overlay-mode
sidewalk``, every camera frame is inferred without path processing.
The ``ros2`` mode publishes only path and metrics topics. The
task-driven mode retains the loaded model but only processes camera frames
while a Cobiz task is accepted, and publishes direct Unitree Sport Move requests.
Live modes require Jetson ROS/PyTorch.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import threading
import time
import uuid
from collections import deque
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any, Sequence

import cv2
import numpy as np
import torch
from apriltag_stop import (
    AprilTagDecision,
    AprilTagGate,
    AprilTagPolicy,
    AprilTagStopMonitor,
    camera_model_from_info,
    gate_detections,
    parse_tag_ids,
)
from segmentation_runtime import (
    DEFAULT_EVALUATION_SIZE,
    FINETUNED_PROFILE,
    INFERENCE_BACKENDS,
    PROFILE_NAMES,
    R50_PROFILE,
    R50_PROFILES,
    DEFAULT_PROFILE,
    SegmentationConfig,
    SegmentationModel,
    resolve_profile,
)
from cobiz_line_tracking_task import (
    TASK_STOP_CHECKS,
    ActiveTask,
    LineTrackingTasks,
    TaskPolicy,
    requested_selected_mask,
)
from connectome_client import ConnectomeClient, ConnectomePathTracker, reference_stop_reason
from connectome_surface import PlannerConfig
from segmentation_utils import upscale_mask
from local_path import (
    DEFAULT_ROI_POLYGON,
    DEFAULT_ROAD_ROI_POLYGON,
    DEFAULT_ROAD_ROI_EXCLUSION_POLYGON,
    PATH_MASK_CLASSES,
    LocalPathConfig,
    LocalPathEstimate,
    LocalPathSmoother,
    SmoothedPath,
    extract_sidewalk_centerline,
    ground_to_pixel,
    pixel_to_ground_homography,
    path_config_for_mode,
    path_roi_mask,
    roi_exclusion_mask,
    selected_path_region,
)
from perception_process import PerceptionProcess
from surface_path_planner import SurfacePathPlanner, uses_connectome
from fcn360_road import FcnRoadPlanner, mask_topic, read_masks, side_recovery_yaw
from surface_segmenter import FrontRoadInferenceError, SurfaceSegmenter
from r50_checkpoint import validate_checkpoint
from drive_control import (
    DRIVE_STOP_CHECKS,
    MAX_PATH_UNAVAILABLE_INFERENCES,
    MAX_PATH_RECOVERY_INFERENCES,
    DriveConfig,
    DriveDecision,
    PathLossRecovery,
    decide_drive,
    path_target_lateral,
)
from unitree_sport_api import (
    drive_to_sport_move,
    populate_move_request,
    populate_stop_move_request,
)


def _segmenter_from_args(args):
    if getattr(args, 'connectome360_url', ''):
        from connectome360_client import Connectome360Segmenter
        override = ({'detector_sha256': args.connectome360_detector_sha256}
                    if getattr(args, 'connectome360_detector_sha256', '') else {})
        return SurfaceSegmenter(SegmentationModel(_runtime_config(args)),
            lambda: Connectome360Segmenter(args.connectome360_url,
                args.connectome360_sha256, args.connectome360_timeout_sec, **override))
    if getattr(args, 'front_connectome_url', ''):
        from front_connectome_client import FrontConnectomeSegmenter
        return SurfaceSegmenter(SegmentationModel(_runtime_config(args)),
            lambda: FrontConnectomeSegmenter(args.front_connectome_url,
                args.front_connectome_sha256, args.front_connectome_timeout_sec))
    return SegmentationModel(_runtime_config(args))


def uses_learned_road_policy(args, mask_class):
    return bool((getattr(args, 'front_connectome_url', '')
                 or getattr(args, 'connectome360_url', '')) and mask_class == 1)


def learned_road_backend(args):
    if getattr(args, 'connectome360_url', '') and getattr(args, 'connectome360_detector_sha256', ''):
        return 'r50_360_policy'
    return 'connectome360' if getattr(args, 'connectome360_url', '') else 'front_connectome'


def direct_camera_config(args):
    if args.connectome360_url:
        return (args.connectome360_url.rsplit('/', 1)[0]+'/direct-road', 'connectome360',
                args.connectome360_sha256, args.connectome360_timeout_sec)
    if args.front_connectome_url:
        return (args.front_connectome_url.rsplit('/', 1)[0]+'/direct-road', 'front_connectome',
                args.front_connectome_sha256, args.front_connectome_timeout_sec)
    return args.fcn_url, 'fcn360', args.fcn_sha256, args.fcn_timeout_sec

DEFAULT_IMAGE_TOPIC = "/a2/front_camera/res_360p/image_raw"
# Published by apriltag_camera_relay, paired with each detector frame.
DEFAULT_APRILTAG_CAMERA_INFO_TOPIC = "/line_tracking/apriltag/camera_info"
DEFAULT_LOCAL_PATH_TOPIC = "/line_tracking/local_path"
DEFAULT_METRICS_TOPIC = "/line_tracking/metrics"
PERFORMANCE_WARMUP_FRAMES = 10
PERFORMANCE_SAMPLE_WINDOW = 512
AUTOMATIC_STOP_CHECKS = (*DRIVE_STOP_CHECKS, "apriltag", *TASK_STOP_CHECKS)


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


def extract_path_estimates(selected_mask, mask_classes, config):
    """Semantic reference paths are independent of obstacle sensing."""
    return {key: extract_sidewalk_centerline(selected_path_region(selected_mask, key), path_config_for_mode(config, key))
            for key in mask_classes}


def _connectome_client_from_args(args, local_config):
    if args.path_planner != 'connectome':
        return None
    return ConnectomeClient(args.connectome_url, local_config,
        PlannerConfig(half_width_m=args.robot_half_width_m,
                      margin_m=args.path_margin_m,
                      road_corridor_smoothing_m=args.road_corridor_smoothing_m,
                      road_building_cost_weight=args.road_building_cost_weight,
                      road_building_distance_m=args.road_building_distance_m), timeout=args.connectome_timeout_sec)


def _surface_planner_from_args(args, local_config):
    client = _connectome_client_from_args(args, local_config)
    return SurfacePathPlanner(client, local_config) if client is not None else None


def _path_smoother(mask_class, args, config, metrics=None):
    if getattr(args, 'fcn_sha256', '') and mask_class == 1:
        return LocalPathSmoother(replace(config, branch_preference='center', unrestricted_path_mode=False))
    if uses_connectome(mask_class, args.path_planner == 'connectome', metrics):
        return ConnectomePathTracker(args.connectome_max_age_sec)
    return LocalPathSmoother(path_config_for_mode(config, mask_class))


def _waiting_planner_status(args, mask_class):
    backend = 'connectome' if uses_connectome(mask_class, args.path_planner == 'connectome') else 'local'
    return {'backend': backend, 'ready': False, 'reason': backend+'_waiting'}


def _parse_polygon(value: str) -> tuple[float, ...]:
    try:
        values = tuple(
            float(item.strip()) for item in value.replace(";", ",").split(",")
        )
    except ValueError as error:
        raise ValueError("LINE_TRACKING_ROI_POLYGON must be comma-separated floats") from error
    if len(values) != 8:
        raise ValueError("LINE_TRACKING_ROI_POLYGON must contain eight values")
    return values


def _parse_exclusion_polygon(value: str) -> tuple[float, ...]:
    if not value.strip():
        return ()
    try:
        return tuple(float(item.strip()) for item in value.replace(';', ',').split(','))
    except ValueError as error:
        raise ValueError('LINE_TRACKING_ROAD_ROI_EXCLUSION_POLYGON must be comma-separated floats') from error


def _local_path_config_from_args(args: argparse.Namespace) -> LocalPathConfig:
    polygon = (
        args.roi_polygon
        if args.roi_polygon is not None
        else _parse_polygon(
            _env("LINE_TRACKING_ROI_POLYGON", ",".join(map(str, DEFAULT_ROI_POLYGON)))
        )
    )
    return LocalPathConfig(
        near_distance_m=args.near_distance_m,
        far_distance_m=args.far_distance_m,
        ground_half_width_m=args.ground_half_width_m,
        search_half_width_m=args.search_half_width_m,
        roi_polygon=polygon,
        road_roi_polygon=tuple(args.road_roi_polygon) if args.road_roi_polygon is not None else _parse_polygon(
            _env("LINE_TRACKING_ROAD_ROI_POLYGON", ",".join(map(str, DEFAULT_ROAD_ROI_POLYGON)))),
        road_roi_exclusion_polygon=(tuple(args.road_roi_exclusion_polygon)
            if args.road_roi_exclusion_polygon is not None else _parse_exclusion_polygon(
                _env('LINE_TRACKING_ROAD_ROI_EXCLUSION_POLYGON',
                     ','.join(map(str, DEFAULT_ROAD_ROI_EXCLUSION_POLYGON))))),
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
        branch_preference=args.branch_preference,
        branch_min_width_m=args.branch_min_width_m,
        branch_margin_m=args.branch_margin_m,
        branch_confirm_frames=args.branch_confirm_frames,
        branch_hold_sec=args.branch_hold_sec,
    )


def _drive_config_from_args(args: argparse.Namespace) -> DriveConfig:
    config = DriveConfig(
        max_forward_mps=args.max_forward_mps,
        path_loss_recovery_enabled=args.path_loss_recovery_enabled,
        path_recovery_wait_sec=args.path_recovery_wait_sec,
        path_recovery_yaw_rps=args.path_recovery_yaw_rps,
        path_recovery_confirm_frames=args.path_recovery_confirm_frames,
        path_recovery_first_direction=args.path_recovery_first_direction,
        **{
            "stop_on_" + name: getattr(args, "stop_on_" + name)
            for name in DRIVE_STOP_CHECKS
        },
    )
    config.validate()
    return config


def _runtime_config(args: argparse.Namespace) -> SegmentationConfig:
    return SegmentationConfig(
        profile=args.profile,
        model_id=args.model_id,
        model_revision=args.model_revision,
        checkpoint_manifest_sha256=args.checkpoint_manifest_sha256,
        evaluation_height=args.evaluation_size[0],
        evaluation_width=args.evaluation_size[1],
        device=args.device,
        backend=args.backend,
        cuda_graph=args.cuda_graph,
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
    generation: int = 0
    fcn_masks: bool = False
    direct_request: tuple | None = None


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

    def __init__(self, *, enabled: bool = True) -> None:
        self._condition = threading.Condition()
        self._item: FramePacket | None = None
        self._closed = False
        self._enabled = enabled
        self._generation = 0
        self.overwritten = 0

    @property
    def enabled(self) -> bool:
        with self._condition:
            return self._enabled and not self._closed

    def set_enabled(self, enabled: bool) -> None:
        """Start a fresh session or suspend it, invalidating pending/in-flight frames."""
        with self._condition:
            self._generation += 1
            self._enabled = enabled
            self._item = None
            self._condition.notify_all()

    def is_current(self, packet: FramePacket) -> bool:
        with self._condition:
            return (
                not self._closed and self._enabled
                and packet.generation == self._generation
            )

    def put(self, item: FramePacket) -> bool:
        with self._condition:
            if self._closed or not self._enabled:
                return False
            if self._item is not None:
                self.overwritten += 1
            self._item = replace(item, generation=self._generation)
            self._condition.notify()
            return True

    def get_latest(self) -> FramePacket | None:
        """Wait for input, then return the latest frame without a rate delay."""
        with self._condition:
            while not self._closed:
                if not self._enabled or self._item is None:
                    self._condition.wait()
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
    metric_projector: Any = None,
) -> np.ndarray:
    """Render segmentation, raw/final path and optional status on the camera frame."""

    overlay = frame_bgr.copy()
    config = path_config_for_mode(config, path_mask_class)
    mask = upscale_mask(selected_mask, frame_bgr)
    road = overlay.copy()
    road[mask == 1] = (40, 180, 40)
    sidewalk = overlay.copy()
    sidewalk[mask == 2] = (220, 60, 220)
    overlay = cv2.addWeighted(overlay, 0.68, road, 0.32, 0.0)
    overlay = cv2.addWeighted(overlay, 0.70, sidewalk, 0.30, 0.0)

    if metric_projector is None:
        shape = frame_bgr.shape[:2]
        roi = path_roi_mask(shape, config)
        contours, _ = cv2.findContours(roi, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if config.roi_exclusion_polygon:
            excluded = roi_exclusion_mask(shape, config) != 0
            overlay[excluded] = (overlay[excluded] * .45).astype(np.uint8)
        cv2.drawContours(overlay, contours, -1,
                         (0, 165, 255) if path_mask_class == 1 else (255, 180, 0), 2)
        homography = pixel_to_ground_homography(frame_bgr.shape[:2], config)
    if estimate is not None:
        raw_pixels = (metric_projector(estimate.points_xy) if metric_projector
                      else ground_to_pixel(estimate.points_xy, homography))
        _draw_polyline(overlay, raw_pixels, (0, 165, 255), 3)
    if path is not None:
        smoothed_pixels = (metric_projector(path.points_xy) if metric_projector
                           else ground_to_pixel(path.points_xy, homography))
        _draw_polyline(overlay, smoothed_pixels, (255, 255, 255), 5)
        for pixel in smoothed_pixels[:: max(1, len(smoothed_pixels) // 6)]:
            if np.all(np.isfinite(pixel)):
                cv2.circle(
                    overlay, tuple(np.rint(pixel).astype(int)), 4, (255, 255, 255), -1
                )

    lines = (
        f"SURFACE {PATH_MASK_CLASSES[path_mask_class]} LOCAL PATH",
        f"raw={estimate.confidence:.2f} valid={estimate.valid_ratio:.2f}"
        if estimate
        else "raw=--",
        f"path={path.stop_reason or 'TRACKED'} hold={path.age_sec:.2f}s"
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


def run_mcap(args):
    path = args.input.expanduser().resolve()
    if not path.is_file() or path.suffix.lower() != ".mcap":
        raise FileNotFoundError(f"MCAP input does not exist: {path}")
    with_path = args.overlay_mode == "local-path"
    local_config = _local_path_config_from_args(args) if with_path else None
    segmenter = _segmenter_from_args(args)
    planner = _surface_planner_from_args(args, local_config) if with_path else None
    smoother = _path_smoother(args.path_mask_class, args, local_config) if with_path else None
    start_ns = 0
    if args.start_offset > 0:
        from mcap.reader import make_reader
        with path.open("rb") as stream:
            summary = make_reader(stream).get_summary()
        if summary is None or summary.statistics is None:
            raise RuntimeError("MCAP has no readable summary")
        start_ns = int(summary.statistics.message_start_time + args.start_offset * 1e9)
    writer = None
    count = updates = 0
    next_inference = -math.inf
    previous_mask = None
    times = deque(maxlen=32)
    try:
        for schema, channel, message, decoded in _iter_mcap_events(path, (args.image_topic,), start_time_ns=start_ns):
            now = message.log_time / 1e9
            if channel.topic != args.image_topic:
                continue
            if schema.name != "sensor_msgs/msg/Image":
                raise ValueError(f"camera topic type is {schema.name}, expected Image")
            frame = decode_ros_image(decoded)
            if writer is None:
                writer = _build_writer(args.output.expanduser().resolve(), frame, args.output_fps)
            if not with_path:
                result = segmenter.segment(frame)
                updates += 1
                overlay = segmenter.render_overlay(frame, result.selected_mask, frame_index=count, fps=args.output_fps)
            else:
                if args.unrestricted_path_mode or now >= next_inference:
                    extra = {'source_stamp': now} if getattr(args, 'front_connectome_url', '') else {}
                    result = segmenter.segment(frame, detect_road_hazards=planner is not None,
                                               path_mask_class=args.path_mask_class, **extra)
                    previous_mask = result.selected_mask
                    if planner:
                        estimates = planner.plan(frame, previous_mask, result.road_hazard_mask,
                                                 now, 0, count+1, mode=args.path_mask_class,
                                                 building=getattr(result, 'road_building_mask', None))
                        estimate = estimates[args.path_mask_class]
                        if uses_connectome(args.path_mask_class, True, planner.metrics) != isinstance(smoother, ConnectomePathTracker):
                            smoother = _path_smoother(args.path_mask_class, args, local_config, planner.metrics)
                    else:
                        estimate = extract_path_estimates(previous_mask, (args.path_mask_class,), local_config)[args.path_mask_class]
                    smoother.update(estimate, now)
                    updates += 1
                    times.append(result.total_seconds)
                    next_inference = now + 1 / args.sample_hz
                current = smoother.current(now)
                overlay = render_local_path_overlay(frame, previous_mask, estimate, current, local_config,
                    frame_index=count, inference_count=updates,
                    inference_hz=1 / float(np.mean(times)) if times and np.mean(times) > 0 else 0.,
                    path_mask_class=args.path_mask_class)
            writer.write(overlay)
            count += 1
            if count % 100 == 0:
                print(f"MCAP_PROGRESS frames={count} updates={updates}", flush=True)
            if args.max_frames and count >= args.max_frames:
                break
    finally:
        if writer is not None:
            writer.release()
    if count == 0:
        raise RuntimeError(f"No images found on {args.image_topic}")
    if args.report:
        report = {"mode": "mcap", "source": str(path), "overlay_mode": args.overlay_mode,
            "start_offset_sec": args.start_offset, "output_fps": args.output_fps,
            "inference_policy": "every_frame" if args.unrestricted_path_mode or not with_path else "bag_time_rate",
            "requested_sample_hz": args.sample_hz if with_path and not args.unrestricted_path_mode else None,
            "image_topic": args.image_topic, "frames_written": count, "segmentation_updates": updates,
            "output": str(args.output.expanduser().resolve()), "model": segmenter.metadata(),
            "path_mask_class": args.path_mask_class if with_path else None,
            "local_path": asdict(local_config) if local_config else None,
            "path_planner": planner.metrics if planner else {"backend": "local"}}
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2)+"\n")
    print(f"MCAP_COMPLETE frames={count} segmentation_updates={updates} output={args.output}")
    return 0


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


def _path_message(path: SmoothedPath | None, header: Any, frame_id: str, *, stamp=None) -> Any:
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path

    message = Path()
    message.header.stamp = stamp if stamp is not None else header.stamp
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

    # The original model remains active for Sidewalk, including its pin checks.
    if args.profile not in R50_PROFILES:
        raise ValueError(
            "task-drive mode requires a pinned FP16 deployment profile: "
            f"{R50_PROFILE} or {FINETUNED_PROFILE}"
        )
    pinned = resolve_profile(args.profile)
    if args.profile == FINETUNED_PROFILE:
        if args.model_revision is not None:
            raise ValueError("local R50 task-drive cannot override model_revision")
    elif args.model_id not in (None, pinned.model_id) or args.model_revision not in (
        None,
        pinned.model_revision,
    ):
        raise ValueError("task-drive mode cannot override the pinned checkpoint")
    if args.profile in R50_PROFILES and args.backend != "pytorch":
        raise ValueError("R50 task-drive mode requires the pytorch backend")
    if tuple(args.evaluation_size) != DEFAULT_EVALUATION_SIZE:
        raise ValueError("task-drive mode requires a 360x640 score map")
    if args.path_frame_id != "base_link":
        raise ValueError("task-drive mode requires a calibrated base_link path")
    if args.output_hz < 10.0:
        raise ValueError("task-drive mode requires at least 10 Hz zero-command updates")
    _drive_config_from_args(args)
    if args.profile == FINETUNED_PROFILE:
        validate_checkpoint(
            args.model_id or pinned.model_id,
            args.checkpoint_manifest_sha256,
            require_pin=True,
        )


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
    planner = _surface_planner_from_args(args, local_config)
    # Morphology must not create an all-core OpenCV pool alongside the ROS
    # sensor executor and CUDA worker. Match the existing CPU thread budget.
    cv2.setNumThreads(2)
    segmenter = (PerceptionProcess(_runtime_config(args), {
        'url': planner.client.url, 'local_config': local_config,
        'planner_config': planner.client.config, 'timeout': planner.client.timeout})
        if args.inference_process else _segmenter_from_args(args))
    if task_mode and segmenter.device.type != "cuda":
        if args.inference_process: segmenter.close()
        raise RuntimeError("task-drive mode requires a CUDA model device")
    smoothers = {
        mask_class: _path_smoother(mask_class, args, local_config)
        for mask_class in (PATH_MASK_CLASSES if task_mode else (args.path_mask_class,))
    }
    fcn = FcnRoadPlanner(local_config, args.fcn_height_m) if args.fcn_sha256 else None
    direct_input = args.road_direct_input
    latest = LatestFrameQueue(enabled=not task_mode)
    # SIGTERM can interrupt a ROS callback already reading this state.
    state_lock = threading.RLock()
    state: dict[str, Any] = {
        "header": None,
        "sequence": 0,
        "inference_count": 0,
        "path_unavailable_inferences": {mask_class: 0 for mask_class in smoothers},
        "path_recovery_inferences": {key: 0 for key in smoothers},
        "semantic_path_missing": {key: False for key in smoothers},
        "performance_inference_seconds": deque(maxlen=PERFORMANCE_SAMPLE_WINDOW),
        "performance_processing_seconds": deque(maxlen=PERFORMANCE_SAMPLE_WINDOW),
        "performance_completion_times": deque(maxlen=PERFORMANCE_SAMPLE_WINDOW),
        "last_image_at": None,
        "camera_received": 0,
        "camera_gap_max_sec": 0.,
        "camera_gaps_over_sec": 0,
        "camera_fault_reason": None,
        "camera_fault_min_sequence": 0,
        "last_inference_at": None,
        "last_image_stamp_ns": None,
        "last_inference_stamp_ns": None,
        "planner_status": _waiting_planner_status(args, args.path_mask_class),
        "pipeline_timing": {},
        "fcn_metrics": {},
        "front_command": None,
        "front_metrics": {},
    }
    worker_error: list[BaseException] = []

    class DebugNode(Node):
        def __init__(self) -> None:
            super().__init__(
                "line_tracking_task_drive" if task_mode else "line_tracking_runtime"
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
            self.next_telemetry_at = 0.
            self.control_times = deque(maxlen=PERFORMANCE_SAMPLE_WINDOW)
            self.path_search = (
                PathLossRecovery(self.drive_config, output_hz=args.output_hz)
                if task_mode else None
            )
            self.tasks = (
                LineTrackingTasks(
                    TaskPolicy(
                        default_duration_sec=args.default_task_duration_sec,
                        max_duration_sec=args.max_task_duration_sec,
                        default_selected_mask=args.path_mask_class,
                        **{
                            "stop_on_" + name: getattr(args, "stop_on_" + name)
                            for name in TASK_STOP_CHECKS
                        },
                    )
                )
                if task_mode
                else None
            )
            self.last_ready_reason = "inputs_not_ready"
            self.last_valid_yaw_rate: float | None = None
            self.last_valid_forward_mps: float | None = None
            self.path_unavailable_inferences = 0
            self.path_recovery_inferences = 0
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
            self.apriltag_gate = (
                AprilTagGate(
                    max_range_m=args.apriltag_max_range_m,
                    min_decision_margin=args.apriltag_min_decision_margin,
                    allowed_ids=parse_tag_ids(args.apriltag_allowed_ids),
                    tag_size_m=args.apriltag_tag_size_m,
                )
                if task_mode
                else None
            )
            if self.apriltag_gate is not None:
                self.apriltag_gate.validate()
            self.apriltag_camera = None
            self.latest_apriltag_ranges: dict[int, float | None] = {}
            self.apriltag_confirmed_range_m: float | None = None
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
            self.front_input_qos = input_qos
            self.image_subscription = (None if direct_input else self.create_subscription(
                Image, args.image_topic, self.on_image, input_qos))
            self.fcn_subscription = None
            self.fcn_session_start_ns = None
            self.connectome360_subscription = None
            self.connectome360_session_start_ns = None
            self.direct_session = None
            self.fcn_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                      reliability=ReliabilityPolicy.BEST_EFFORT)
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
            self.slam_localization = None
            self.last_slam_stop_at = None
            if task_mode:
                self.slam_command_publisher = self.create_publisher(
                    String, "/grit_slam/localization/command", self.command_qos)
                self.create_subscription(
                    String, "/grit_slam/localization/status", self.on_slam_localization,
                    QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL))
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
                if args.apriltag_max_range_m > 0.0:
                    from sensor_msgs.msg import CameraInfo

                    # The relay publishes this paired with each detector frame.
                    self.apriltag_camera_info_subscription = self.create_subscription(
                        CameraInfo,
                        args.apriltag_camera_info_topic,
                        self.on_apriltag_camera_info,
                        QoSProfile(
                            history=HistoryPolicy.KEEP_LAST,
                            depth=1,
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                        ),
                    )
            self.control_hz = args.output_hz
            self.timer = self.create_timer(1.0 / self.control_hz, self.publish_state)
            self.started = time.monotonic()
            self.get_logger().info(
                "Surface local path debug started | image=%s profile=%s "
                "path_surface=%s inference=latest_frame output_hz=%.2f near=%.1fm far=%.1fm"
                % (
                    args.image_topic,
                    args.profile,
                    PATH_MASK_CLASSES[args.path_mask_class],
                    self.control_hz,
                    local_config.near_distance_m,
                    local_config.far_distance_m,
                )
            )
            if args.unrestricted_path_mode:
                self.get_logger().warning(
                    "LINE_TRACKING_UNRESTRICTED_PATH_MODE is enabled: path valid-ratio "
                    "and temporal smoothing are bypassed; explicit drive stop checks "
                    "still apply"
                )
            if task_mode:
                self.get_logger().info(
                    "Cobiz LINE_TRACKING task listener ready; model loaded, camera "
                    "inference paused until task acceptance; Sport publisher absent"
                )

        def publish_task_state(self, body: dict[str, Any]) -> None:
            if self.task_state_publisher is not None:
                self.task_state_publisher.publish(
                    String(data=json.dumps(body, separators=(",", ":")))
                )

        def on_slam_localization(self, message: Any) -> None:
            try:
                body = json.loads(message.data)
            except (TypeError, ValueError):
                return
            if not isinstance(body, dict):
                return
            with state_lock:
                if body.get("state") in ("STOPPED", "ERROR"):
                    self.slam_localization = None
                    self.last_slam_stop_at = None
                    return
                if (body.get("state") not in ("STARTING", "READY")
                        or not isinstance(body.get("task_id"), str)
                        or not body["task_id"].strip()
                        or not isinstance(body.get("map_uuid"), str)
                        or not body["map_uuid"].strip()):
                    return
                self.slam_localization = {
                    "task_id": body["task_id"], "map_uuid": body["map_uuid"]}
                try:
                    self.stop_slam_localization(force=True)
                except Exception as error:
                    self.get_logger().error(f"SLAM STOP publication failed: {error}")
                    self.abort_active_task("slam_stop_publish_error")

        def stop_slam_localization(self, *, force: bool = False) -> None:
            # STOP must name SLAM's retained localization session, not this task.
            now = time.monotonic()
            if (self.tasks is None or self.tasks.active is None or self.slam_localization is None):
                return
            if (not force and self.last_slam_stop_at is not None
                    and now - self.last_slam_stop_at < 1.0):
                return
            self.slam_command_publisher.publish(String(data=json.dumps({
                "command": "STOP", **self.slam_localization, "grace_sec": 0})))
            self.last_slam_stop_at = now


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
            distance = self.apriltag_confirmed_range_m
            self.get_logger().info(
                "AprilTag %d confirmed at %s camera range; stopping line tracking"
                % (tag_id, "unchecked" if distance is None else f"{distance:.2f} m")
            )
            if not self.publish_hard_stop(reason):
                self.abort_active_task("stop_publish_error")
                return
            body = self.tasks.finish("TASK_COMPLETED", reason)
            self.set_task_inference(False)
            self.last_valid_yaw_rate = None
            self.last_valid_forward_mps = None
            if body is not None:
                self.terminal_apriltag_status = self.apriltags.snapshot(
                    now=time.monotonic()
                )
                self.publish_task_state(body)
            self.stop_until = time.monotonic() + 1.0

        def on_apriltag_camera_info(self, message: Any) -> None:
            camera = camera_model_from_info(message)
            if camera is None:
                self.get_logger().warning(
                    "AprilTag range gate cannot use camera_info distortion model %r; "
                    "tags will not stop tracking" % getattr(message, "distortion_model", "")
                )
            self.apriltag_camera = camera

        def on_apriltag_detections(self, message: Any) -> None:
            now = time.monotonic()
            stamp_ns = _stamp_ns(message.header)
            self.apriltag_callback_sequence += 1
            frame_key = stamp_ns if stamp_ns > 0 else self.apriltag_callback_sequence
            # Only a tag TAG_APPROACH could take over from may stop tracking:
            # a far, faint or foreign tag is treated as no tag at all.
            self.latest_apriltag_ranges = gate_detections(
                message.detections, self.apriltag_gate, self.apriltag_camera
            )
            self.latest_apriltag_ids = tuple(sorted(self.latest_apriltag_ranges))
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
                self.apriltag_confirmed_range_m = self.latest_apriltag_ranges.get(
                    decision.confirmed_id
                )
                self.complete_apriltag_task(decision.confirmed_id)

        def set_task_inference(self, enabled: bool) -> None:
            # Use the same lock as result publication so a cancelled frame can
            # never repopulate paths after this reset. Model/EMA reset belongs
            # to the worker, which alone owns the segmenter during forward().
            with state_lock:
                latest.set_enabled(enabled)
                self.direct_session = ((uuid.uuid4().hex, self.get_clock().now().nanoseconds)
                    if direct_input and enabled and self.tasks.active and self.tasks.active.selected_mask == 1 else None)
                if direct_input:
                    if self.image_subscription is not None:
                        self.destroy_subscription(self.image_subscription)
                        self.image_subscription = None
                    if enabled and self.tasks.active and self.tasks.active.selected_mask == 2:
                        self.image_subscription = self.create_subscription(
                            Image, args.image_topic, self.on_image, self.front_input_qos)
                if args.connectome360_url and not direct_input:
                    from connectome360_runtime import VIEWS_TOPIC
                    self.connectome360_session_start_ns = self.get_clock().now().nanoseconds if enabled else None
                    if self.connectome360_subscription is not None:
                        self.destroy_subscription(self.connectome360_subscription)
                        self.connectome360_subscription = None
                    if enabled and self.tasks.active and self.tasks.active.selected_mask == 1:
                        self.connectome360_subscription = self.create_subscription(
                            Image, VIEWS_TOPIC, self.on_connectome360_image, self.fcn_qos)
                if fcn is not None and not direct_input:
                    self.fcn_session_start_ns = self.get_clock().now().nanoseconds if enabled else None
                    if self.fcn_subscription is not None:
                        self.destroy_subscription(self.fcn_subscription)
                        self.fcn_subscription = None
                    if enabled and self.tasks.active and self.tasks.active.selected_mask == 1:
                        self.fcn_subscription = self.create_subscription(
                            Image, mask_topic(args.fcn_sha256), self.on_fcn_masks, self.fcn_qos)
                if fcn is not None:
                    state['fcn_metrics'] = {}
                state['front_command'] = None
                state['front_metrics'] = {}
                # Replace path histories instead of locking an old smoother:
                # SIGTERM may have interrupted its current() call on this thread.
                for mask_class in smoothers:
                    smoothers[mask_class] = _path_smoother(mask_class, args, local_config)
                if planner or fcn:
                    state['planner_status'] = _waiting_planner_status(args,
                        active_path_mask_class(self.tasks.active if self.tasks else None, args.path_mask_class))
                for key in (
                    "last_image_at", "last_image_stamp_ns", "last_inference_at",
                    "last_inference_stamp_ns", "camera_fault_reason",
                ):
                    state[key] = None
                if enabled:
                    state["header"] = None
                state["camera_fault_min_sequence"] = state["sequence"]
                state["path_unavailable_inferences"] = {
                    mask_class: 0 for mask_class in smoothers
                }
                state["path_recovery_inferences"] = {key: 0 for key in smoothers}
                state["semantic_path_missing"] = {key: False for key in smoothers}
                for key in (
                    "performance_inference_seconds", "performance_processing_seconds",
                    "performance_completion_times",
                ):
                    state[key].clear()
            self.last_valid_yaw_rate = None
            self.last_valid_forward_mps = None
            self.path_unavailable_inferences = 0
            self.path_recovery_inferences = 0
            if self.path_search is not None:
                self.path_search.reset()
            if self.direct_session is not None:
                self.request_direct_road()

        def request_direct_road(self):
            # Task activation or worker completion schedules one request token.
            # The same generation queue rejects late results after cancellation.
            with state_lock:
                if not self.direct_session or not latest.enabled:
                    return
                after = state['last_image_stamp_ns'] or 0
                if latest.put(FramePacket(None, int(state['sequence']),
                        direct_request=(*self.direct_session, after))):
                    state['sequence'] += 1

        def release_task_control(self, reason: str) -> None:
            self.set_task_inference(False)
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
                camera_fault_reason = state["camera_fault_reason"]
                last_inference_at = state["last_inference_at"]
                last_image_stamp_ns = state["last_image_stamp_ns"]
                last_inference_stamp_ns = state["last_inference_stamp_ns"]
                path = smoothers[mask_class].current(now)
                fcn_speed_fraction = state['fcn_metrics'].get('speed_fraction', 0.)
                fcn_metrics = dict(state['fcn_metrics'])
                self.path_recovery_inferences = state["path_recovery_inferences"][mask_class]
                recovering = (self.drive_config.path_loss_recovery_enabled and (
                    state["semantic_path_missing"][mask_class]
                    or self.path_recovery_inferences >= MAX_PATH_RECOVERY_INFERENCES
                ))
                inference_id = int(state["inference_count"])
                planner_status = state['planner_status']
                if recovering and (path is None or path.stop_reason is None):
                    path = None
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
            if fcn is not None and mask_class == 1:
                reason = self.fcn_stop_reason(path, camera_age_sec, inference_age_sec)
                if reason:
                    return None, DriveDecision.stop(reason)
                if path is None or not fcn_metrics.get('front_path_valid'):
                    yaw = side_recovery_yaw(fcn_metrics, self.drive_config.max_yaw_rps,
                                            age_sec=max(camera_age_sec, inference_age_sec))
                    return None, (DriveDecision(0., 0., yaw, 'fcn_side_recovery') if yaw
                                  else DriveDecision.stop('fcn_front_path_unavailable'))
            if uses_learned_road_policy(args, mask_class):
                from front_connectome_client import front_drive_decision, limit_front_command
                with state_lock:
                    learned = state['front_command']
                decision = front_drive_decision(learned, mask_class,
                    (camera_age_sec, inference_age_sec), args.connectome_max_age_sec, camera_fault_reason,
                    backend=learned_road_backend(args))
                decision = limit_front_command(decision, self.drive_config.max_forward_mps,
                                               self.drive_config.max_yaw_rps)
                return path, decision
            connectome_path = uses_connectome(mask_class, planner is not None, planner_status)
            if connectome_path:
                # This gate also overrides optional legacy path-loss recovery:
                # no stale reference, straight fallback or scan after failure.
                reason = reference_stop_reason(path, planner_status.get('ready'),
                    (camera_age_sec, inference_age_sec), args.connectome_max_age_sec)
                if reason:
                    self.last_valid_forward_mps = self.last_valid_yaw_rate = None
                    return None, DriveDecision.stop(reason)
            decision = decide_drive(
                path,
                camera_age_sec=camera_age_sec,
                inference_age_sec=inference_age_sec,
                config=self.drive_config,
                last_valid_yaw_rate=self.last_valid_yaw_rate,
                last_valid_forward_mps=self.last_valid_forward_mps,
                path_unavailable_inferences=self.path_unavailable_inferences,
                path_recovery_inferences=self.path_recovery_inferences if recovering else None,
            )
            if fcn is not None and mask_class == 1:
                decision = replace(decision, vx=decision.vx * fcn_speed_fraction)
            if camera_fault_reason is not None and decision.reason not in (
                "camera_stale", "inference_stale"
            ):
                decision = DriveDecision.stop(camera_fault_reason)
            if (self.tasks.active is not None and not connectome_path
                    and not (fcn is not None and mask_class == 1)):
                sensor_reason = None
                # Searching requires live observations even when normal driving's
                # optional stale-sensor stops are disabled.
                for name, age, maximum in (
                    ("camera", camera_age_sec, self.drive_config.max_camera_age_sec),
                    ("inference", inference_age_sec, self.drive_config.max_inference_age_sec),
                ):
                    if age is None or not math.isfinite(age) or not 0 <= age <= maximum:
                        sensor_reason = name + "_stale"
                        break
                decision = self.path_search.decide(
                    decision, now=now, inference_id=inference_id,
                    inference_source_at=(now-inference_age_sec
                                         if inference_age_sec is not None else None),
                    sensors_ready=sensor_reason is None,
                    sensor_reason=sensor_reason or "inputs_not_ready",
                    path_confidence=path.confidence if path is not None else None,
                    path=path,
                )
                if self.path_search.active and (path is None or path.stop_reason is None):
                    path = None
            if decision.reason in (
                "camera_stale", "inference_stale",
                "camera_conversion_error",
            ) or decision.reason.startswith("branch_"):
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
            if args.front_connectome_url or args.connectome360_url or fcn is not None:
                try:
                    if requested_selected_mask(event, args.path_mask_class) not in (1, 2):
                        rejection_reason = ('fcn_requires_road_or_sidewalk' if fcn else learned_road_backend(args) + '_requires_road_or_sidewalk')
                except ValueError:
                    rejection_reason = 'invalid_selected_mask'
            if self.command_publisher is not None and self.tasks.active is None:
                rejection_reason = "control_release_pending"
            previous_task = self.tasks.active
            body = self.tasks.handle_event(
                event, now=now, rejection_reason=rejection_reason
            )
            if body is None:
                return
            if body["type"] == "TASK_STARTED":
                self.set_task_inference(False)
                self.terminal_apriltag_status = None
                self.apriltag_confirmed_range_m = None
                self.apriltag_window_resolved_at = None
                try:
                    assert Request is not None
                    self.stop_slam_localization(force=True)
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
                    elif self.tasks.policy.stop_on_startup_hold:
                        self.publish_zero_move("startup_hold")
                    else:
                        _, ready = self.drive_readiness(
                            self.tasks.active.selected_mask, now
                        )
                        self.publish_drive(ready)
                    # Wake only after initial control publication succeeds.
                    # Idle frames were discarded; wait for a new camera image.
                    self.set_task_inference(True)
                except Exception as error:  # noqa: BLE001 - never report start without control.
                    self.get_logger().error(
                        f"failed to acquire direct Sport control: {error}"
                    )
                    # A failed publication may still have reached the robot;
                    # attempt both stop commands before releasing the publisher.
                    self.release_task_control("control_publisher_error")
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

        def publish_drive(self, decision: DriveDecision) -> DriveDecision:
            if self.command_publisher is None:
                return decision
            active = self.tasks.active if self.tasks is not None else None
            front_road = uses_learned_road_policy(args, active_path_mask_class(active, args.path_mask_class))
            if task_mode and not rclpy.ok():
                # A callback interrupted by SIGTERM can resume after its stop.
                decision = DriveDecision.stop("shutdown")
            if (fcn is not None and active is not None and active.selected_mask == 1
                    and (decision.vx or decision.vy or decision.yaw_rate)):
                now = time.monotonic()
                clock_ns = self.get_clock().now().nanoseconds
                with state_lock:
                    ages = [_effective_source_age_sec(state[a], state[s], now, clock_ns)
                            for a, s in (('last_image_at', 'last_image_stamp_ns'),
                                         ('last_inference_at', 'last_inference_stamp_ns'))]
                    current = smoothers[1].current(now)
                    reason = self.fcn_stop_reason(current, *ages)
                    metrics = dict(state['fcn_metrics'])
                if reason:
                    decision = DriveDecision.stop(reason)
                elif current is None or not metrics.get('front_path_valid'):
                    yaw = side_recovery_yaw(metrics, self.drive_config.max_yaw_rps, age_sec=max(ages))
                    decision = (DriveDecision(0., 0., yaw, 'fcn_side_recovery') if yaw
                                else DriveDecision.stop('fcn_front_path_unavailable'))
                elif decision.reason == 'fcn_side_recovery':
                    decision = DriveDecision.stop('fcn_path_recovered')
            if front_road and (decision.vx or decision.vy or decision.yaw_rate):
                from front_connectome_client import front_drive_decision, limit_front_command
                now = time.monotonic(); clock_ns = self.get_clock().now().nanoseconds
                with state_lock:
                    active = self.tasks.active if self.tasks is not None else None
                    mask_class = active_path_mask_class(active, args.path_mask_class)
                    ages = [_effective_source_age_sec(state[received], state[stamp], now, clock_ns)
                        for received, stamp in [('last_image_at', 'last_image_stamp_ns'),
                                                ('last_inference_at', 'last_inference_stamp_ns')]]
                    guard = front_drive_decision(state['front_command'], mask_class, ages,
                        args.connectome_max_age_sec, state['camera_fault_reason'],
                        backend=learned_road_backend(args))
                if guard.reason != 'tracking':
                    decision = guard
                decision = limit_front_command(decision, self.drive_config.max_forward_mps,
                                               self.drive_config.max_yaw_rps)
            if planner and (decision.vx or decision.vy or decision.yaw_rate):
                now = time.monotonic()
                clock_ns = self.get_clock().now().nanoseconds
                with state_lock:
                    active = self.tasks.active if self.tasks is not None else None
                    mask_class = active_path_mask_class(active, args.path_mask_class)
                    current = smoothers[mask_class].current(now)
                    ages = [_effective_source_age_sec(state[received], state[stamp], now, clock_ns)
                            for received, stamp in [('last_image_at', 'last_image_stamp_ns'),
                                                    ('last_inference_at', 'last_inference_stamp_ns')]]
                    reason = state['camera_fault_reason']
                    if uses_connectome(mask_class, True, state['planner_status']):
                        reason = reason or reference_stop_reason(
                            current, state['planner_status'].get('ready'), ages, args.connectome_max_age_sec)
                if reason:
                    decision = DriveDecision.stop(reason)
            assert Request is not None
            move = drive_to_sport_move(vx=decision.vx, vy=decision.vy, yaw_rate=decision.yaw_rate)
            message = populate_move_request(
                Request(),
                move,
            )
            self.command_publisher.publish(message)
            self.path_search.record_command(decision, now=time.monotonic())
            return decision

        def fcn_stop_reason(self, path, camera_age, inference_age):
            if state['camera_fault_reason']:
                return state['camera_fault_reason']
            if any(age is None or not 0 <= age <= args.fcn_max_age_sec for age in (camera_age, inference_age)):
                return "fcn_masks_stale"
            if path is None or not state['fcn_metrics'].get('front_path_valid'):
                if side_recovery_yaw(state['fcn_metrics'], self.drive_config.max_yaw_rps,
                                     age_sec=max(camera_age, inference_age)):
                    return None
                status = state['fcn_metrics'].get('path_status')
                if status in ('turn_confirmation', 'reobserve'):
                    return 'fcn_' + status
                return "fcn_front_path_unavailable"
            return path.stop_reason

        def on_fcn_masks(self, message):
            if direct_input:
                return
            with state_lock:
                if not latest.enabled or self.tasks.active is None or self.tasks.active.selected_mask != 1:
                    return
                stamp = _stamp_ns(message.header)
                if self.fcn_session_start_ns is None or stamp < self.fcn_session_start_ns:
                    return  # A queued DDS sample from before this task.
                if not _live_source_stamp(stamp, self.get_clock().now().nanoseconds, args.fcn_max_age_sec):
                    return
                if state['last_image_stamp_ns'] is not None and stamp <= state['last_image_stamp_ns']:
                    return
                self.accept_image(message, stamp, is_fcn=True)

        def on_connectome360_image(self, message):
            if direct_input:
                return
            from connectome360_runtime import views_frame_id
            with state_lock:
                if not latest.enabled or self.tasks.active is None or self.tasks.active.selected_mask != 1:
                    return
                stamp = _stamp_ns(message.header)
                if (self.connectome360_session_start_ns is None or stamp < self.connectome360_session_start_ns
                        or message.header.frame_id != views_frame_id()
                        or (message.height, message.width, message.encoding) != (540, 960, 'bgr8')
                        or not _live_source_stamp(stamp, self.get_clock().now().nanoseconds, args.connectome_max_age_sec)
                        or (state['last_image_stamp_ns'] is not None and stamp <= state['last_image_stamp_ns'])):
                    return
                try:
                    validate_camera_image(message, self.bridge)
                    self.accept_image(message, stamp)
                except Exception as error:
                    self.camera_conversion_failed(error)

        def on_image(self, message: Any) -> None:
            if direct_input and self.tasks.active and self.tasks.active.selected_mask == 1:
                return
            if args.connectome360_url and self.tasks.active and self.tasks.active.selected_mask == 1:
                return
            if fcn is not None and self.tasks.active and self.tasks.active.selected_mask == 1:
                return
            if task_mode and not latest.enabled:
                return
            try:
                source_stamp_ns = _stamp_ns(message.header) if task_mode else None
                if fcn is not None and self.fcn_session_start_ns is not None and source_stamp_ns < self.fcn_session_start_ns:
                    return
                validate_camera_image(message, self.bridge)
                self.accept_image(message, source_stamp_ns)
            except Exception as error:  # noqa: BLE001 - safe debug boundary.
                self.camera_conversion_failed(error)

        def accept_image(self, message, source_stamp_ns, *, is_fcn=False):
            with state_lock:
                accepted = latest.put(
                    FramePacket(
                        image_message=message,
                        sequence=int(state["sequence"]),
                        source_header=message.header,
                        fcn_masks=is_fcn,
                    )
                )
                if accepted:
                    arrival = time.monotonic()
                    if state['last_image_at'] is not None:
                        gap = arrival-state['last_image_at']
                        state['camera_gap_max_sec'] = max(state['camera_gap_max_sec'], gap)
                        state['camera_gaps_over_sec'] += int(gap > 1.)
                    state['camera_received'] += 1
                    state["sequence"] += 1
                    state["last_image_at"] = arrival
                    if task_mode:
                        state["last_image_stamp_ns"] = source_stamp_ns

        def camera_conversion_failed(
            self, error: Exception, *, packet: FramePacket | None = None
        ) -> None:
            # Also used by the worker after deferred image conversion fails.
            if task_mode:
                with state_lock:
                    if packet is not None and not latest.is_current(packet):
                        return
                    state["last_image_at"] = None
                    state["camera_fault_reason"] = "camera_conversion_error"
                    state["camera_fault_min_sequence"] = state["sequence"]
                    self.last_valid_yaw_rate = None
                    self.last_valid_forward_mps = None
                    self.publish_drive(DriveDecision.stop("camera_conversion_error"))
            self.get_logger().error(f"camera conversion failed: {error}")

        def publish_state(self) -> None:
            try:
                # A frame commit must not appear in telemetry halfway through
                # issuing a command based on the previous reference. Use the
                # existing reentrant lock for one control/publication snapshot.
                with state_lock:
                    self._publish_state()
            except Exception as error:  # noqa: BLE001 - stop on any timer fault.
                try:
                    self.publish_drive(DriveDecision.stop("publish_error"))
                except Exception:  # noqa: BLE001 - still report task failure.
                    pass
                self.abort_active_task("publish_error")
                self.get_logger().error(f"Surface output failed: {error}")

        def _publish_state(self) -> None:
            now = time.monotonic()
            self.stop_slam_localization()
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
            drive_decision = None
            if self.drive_config is not None:
                path, ready_decision = self.drive_readiness(mask_class, now)
                self.last_ready_reason = ready_decision.reason
                startup_hold = (
                    task_active is not None
                    and self.tasks.policy.stop_on_startup_hold
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
                            path = None
                            self.publish_task_state(terminal)
                        else:
                            drive_decision = self.publish_drive(drive_decision)
                    elif self.command_publisher is not None:
                        drive_decision = DriveDecision.stop("task_idle")
                        drive_decision = self.publish_drive(drive_decision)
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
                    drive_decision = self.publish_drive(drive_decision)
            else:
                path = smoothers[mask_class].current(now)
            # Real callbacks keep refreshing liveness after completion. Retain
            # the terminal display independently until control is released.
            if self.terminal_apriltag_status is not None:
                tag_status = self.terminal_apriltag_status
            # Command publication gates above always run at control
            # cadence. Debug serialization/ROS Pose construction must not
            # starve either sensor callbacks or the inference worker's GIL.
            self.control_times.append(time.monotonic())
            if args.telemetry_hz > 0 and now < self.next_telemetry_at:
                return
            if args.telemetry_hz > 0:
                self.next_telemetry_at = now+1/args.telemetry_hz
            with state_lock:
                header = state['header']
                inference_count = int(state['inference_count'])
                pipeline_timing = dict(state['pipeline_timing'])
                planner_status = dict(state['planner_status'])
                camera_at = state['last_image_at']
                camera_input = {'received': state['camera_received'],
                    'arrival_age_sec': max(0., now-camera_at) if camera_at is not None else None,
                    'gap_max_sec': state['camera_gap_max_sec'],
                    'gaps_over_1_sec': state['camera_gaps_over_sec']}
                performance = summarize_performance(
                    list(state["performance_inference_seconds"]),
                    list(state["performance_processing_seconds"]),
                    list(state["performance_completion_times"]),
                )
            periods = np.diff(self.control_times)
            metrics = {
                'camera_input': camera_input,
                'control_loop': {'target_hz': self.control_hz,
                    'observed_hz': float(1/np.mean(periods)) if len(periods) and np.mean(periods) > 0 else None,
                    'gap_max_sec': float(np.max(periods)) if len(periods) else None,
                    'telemetry_hz': args.telemetry_hz or self.control_hz},
                "path_planner": planner_status,
                "profile": args.profile,
                "fcn360": dict(state['fcn_metrics']) if fcn and mask_class == 1 else None,
                "inference_backend": (f'fcn{args.fcn_input_width}-360' if fcn and mask_class == 1 else learned_road_backend(args).replace('_', '-') if uses_learned_road_policy(args, mask_class) else 'r50'),
                "front_connectome": (dict(state['front_metrics'], command=state['front_command'])
                    if args.front_connectome_url and mask_class == 1 else None),
                "connectome360": (dict(state['front_metrics'], command=state['front_command'])
                    if args.connectome360_url and mask_class == 1 else None),
                "camera_transport": ('direct' if direct_input and mask_class == 1 else 'ros'),
                "camera_topic": (None if direct_input and mask_class == 1 else mask_topic(args.fcn_sha256) if fcn and mask_class == 1 else
                    '/line_tracking/connectome360/views' if args.connectome360_url and mask_class == 1 else args.image_topic),
                "path_mask_class": mask_class,
                "path_roi_polygon": [] if fcn and mask_class == 1 else list(path_config_for_mode(local_config, mask_class).roi_polygon),
                "path_roi_exclusion_polygon": [] if fcn and mask_class == 1 else list(path_config_for_mode(local_config, mask_class).roi_exclusion_polygon),
                "path_surface": PATH_MASK_CLASSES[mask_class],
                "path_tracked": path is not None and path.stop_reason is None,
                "branch_selection": path.branch_status if path else {
                    "preference": local_config.branch_preference, "state": "idle",
                    "reason": "no_path", "branch_detected": False,
                },
                "path_confidence": float(path.confidence) if path else 0.0,
                "path_age_sec": float(path.age_sec) if path else None,
                "path_duration_sec": local_config.path_duration_sec,
                "unrestricted_path_mode": False if fcn and mask_class == 1 else args.unrestricted_path_mode,
                "inference_schedule": "latest_frame",
                "inference_enabled": latest.enabled,
                "near_distance_m": local_config.near_distance_m,
                "far_distance_m": local_config.far_distance_m,
                "queue_overwritten": latest.overwritten,
                "inference_count": inference_count,
                "drive_reason": drive_decision.reason if drive_decision else None,
                "ready_reason": self.last_ready_reason if task_mode else None,
                "task_active": self.tasks.active is not None if self.tasks else None,
                "performance": performance,
                "pipeline_timing": pipeline_timing,
                "cuda_memory": (dict(segmenter.cuda_memory) if args.inference_process
                                else cuda_memory_metrics(segmenter.device)),
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
                    "max_range_m": args.apriltag_max_range_m,
                    "camera_info_ready": self.apriltag_camera is not None,
                    "candidate_ranges_m": {
                        str(tag_id): distance
                        for tag_id, distance in self.latest_apriltag_ranges.items()
                    },
                    "confirmed_range_m": (
                        self.apriltag_confirmed_range_m
                        if tag_status.confirmed_id is not None
                        else None
                    ),
                }
            if self.drive_config is not None:
                lateral = path_target_lateral(path, self.drive_config.lookahead_m)
                metrics["turn_speed_control"] = {
                    "target_heading_deg": (
                        math.degrees(math.atan2(lateral, self.drive_config.lookahead_m))
                        if lateral is not None else None
                    ),
                    "command_forward_mps": drive_decision.vx,
                    "command_yaw_deg_sec": math.degrees(drive_decision.yaw_rate),
                }
                metrics["path_unavailable_inferences"] = self.path_unavailable_inferences
                metrics["path_unavailable_limit"] = MAX_PATH_UNAVAILABLE_INFERENCES
                metrics["path_recovery"] = {
                    "enabled": self.drive_config.path_loss_recovery_enabled,
                    "failed_inferences": self.path_recovery_inferences,
                    "limit": MAX_PATH_RECOVERY_INFERENCES,
                    "active": self.path_search.active,
                    "phase": self.path_search.phase,
                    "wait_sec": self.drive_config.path_recovery_wait_sec,
                    "first_direction": self.path_search.first_direction,
                    "configured_first_direction": self.drive_config.path_recovery_first_direction,
                    "direction_source": self.path_search.direction_source,
                    "last_path_heading_deg": (
                        math.degrees(self.path_search.last_path_heading_rad)
                        if self.path_search.last_path_heading_rad is not None else None
                    ),
                    "yaw_rps": self.drive_config.path_recovery_yaw_rps,
                    "estimated_angle_deg": math.degrees(self.path_search.angle_rad),
                    "confirmed_frames": self.path_search.confirmed_frames,
                    "confirm_frames": self.drive_config.path_recovery_confirm_frames,
                    "exhausted": self.path_search.phase == "exhausted",
                }
                metrics["path_yaw_held"] = (
                    drive_decision is not None
                    and drive_decision.reason == "tracking_path_hold"
                )
                metrics["stop_checks"] = {
                    name: getattr(args, "stop_on_" + name)
                    for name in AUTOMATIC_STOP_CHECKS
                }
                metrics["stop_checks"].update({
                    "camera_freshness": self.drive_config.stop_on_camera_stale,
                    "inference_freshness": self.drive_config.stop_on_inference_stale,
                    "path_available": (
                        self.drive_config.stop_on_path_loss_limit
                        and self.path_unavailable_inferences >= MAX_PATH_UNAVAILABLE_INFERENCES
                    ),
                })
                metrics["camera_fault_reason"] = state["camera_fault_reason"]
            self.metrics_publisher.publish(String(data=json.dumps(metrics)))
            if header is None:
                return
            path_message = _path_message(path, header, args.path_frame_id)
            self.path_publisher.publish(path_message)

    rclpy.init(args=[])
    try:
        node = DebugNode()
    except BaseException:
        if args.inference_process: segmenter.close()
        if rclpy.ok(): rclpy.shutdown()
        raise
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
        previous_generation: int | None = None
        previous_packet: FramePacket | None = None
        direct_client = None
        try:
            while rclpy.ok():
                # Direct Road owns capture at the inference service. Start its
                # next request only after this worker has consumed the result
                # (including camera_pending), never from a periodic timer.
                if previous_packet is not None and previous_packet.direct_request is not None:
                    with state_lock:
                        if latest.is_current(previous_packet):
                            node.request_direct_road()
                packet = latest.get_latest()
                previous_packet = packet
                if packet is None:
                    break
                with state_lock:
                    if not latest.is_current(packet):
                        continue
                if task_mode and packet.generation != previous_generation:
                    segmenter.reset()
                    direct_client = None
                    if fcn is not None:
                        fcn.reset()
                    previous_generation = packet.generation
                inference_started_at = time.monotonic()
                if packet.direct_request is not None:
                    try:
                        from direct_camera_client import DirectCameraClient
                        if direct_client is None:
                            direct_client = DirectCameraClient(*direct_camera_config(args), input_width=args.fcn_input_width,
                                detector_sha256=args.connectome360_detector_sha256)
                        max_age = args.fcn_max_age_sec if fcn else args.connectome_max_age_sec
                        result = direct_client.infer(*packet.direct_request, max_age,
                            now_ns=lambda: node.get_clock().now().nanoseconds)
                    except Exception as error:
                        node.get_logger().warning(f'direct Road inference failed: {error}')
                        with state_lock:
                            if latest.is_current(packet):
                                backend = learned_road_backend(args) if uses_learned_road_policy(args, 1) else direct_camera_config(args)[1]
                                node.abort_active_task(backend+'_unavailable')
                        continue
                    with state_lock:
                        if not latest.is_current(packet) or result is None:
                            continue
                        # Only the camera's original capture stamp refreshes
                        # freshness. Arrival of an HTTP response is not a frame.
                        header = Image().header
                        header.stamp.sec, header.stamp.nanosec = divmod(result.capture_stamp_ns, 1_000_000_000)
                        header.frame_id = direct_camera_config(args)[1]+'_camera'
                        packet = replace(packet, source_header=header, fcn_masks=fcn is not None)
                        arrival = time.monotonic()
                        if state['last_image_at'] is not None:
                            gap = arrival-state['last_image_at']
                            state['camera_gap_max_sec'] = max(state['camera_gap_max_sec'], gap)
                            state['camera_gaps_over_sec'] += int(gap > 1.)
                        state['camera_received'] += 1
                        state['last_image_at'], state['last_image_stamp_ns'] = arrival, result.capture_stamp_ns
                try:
                    if packet.direct_request is not None:
                        if fcn is not None:
                            masks = result.masks
                    elif packet.fcn_masks:
                        masks = read_masks(packet.image_message, args.fcn_input_width)
                    else:
                        frame_rgb = camera_image_rgb(packet.image_message, node.bridge)
                except Exception as error:  # noqa: BLE001 - retain camera fault handling.
                    node.camera_conversion_failed(error, packet=packet)
                    continue
                with state_lock:
                    if not latest.is_current(packet):
                        continue
                    requested_mode = active_path_mask_class(
                        node.tasks.active if node.tasks else None, args.path_mask_class)
                kwargs = {"color_order": "rgb", "path_mask_class": requested_mode}
                if uses_learned_road_policy(args, requested_mode):
                    kwargs['source_stamp'] = _stamp_ns(packet.source_header) / 1e9
                if planner:
                    kwargs['detect_road_hazards'] = True
                if packet.fcn_masks:
                    estimate, fcn_metrics = fcn.plan(masks, stamp=_stamp_ns(packet.source_header)/1e9,
                                                     max_gap=args.fcn_max_age_sec)
                    fcn_metrics['front_path_valid'] = estimate is not None
                    estimates = {key: estimate if key == 1 else None for key in smoothers}
                    segmentation_finished_at = time.monotonic()
                elif packet.direct_request is not None:
                    segmentation_finished_at = time.monotonic()
                elif args.inference_process:
                    response = segmenter.infer(frame_rgb, _stamp_ns(packet.source_header)/1e9,
                                               packet.generation, packet.sequence, mode=requested_mode)
                    result = response['result']
                    segmentation_finished_at = response['segmentation_finished_at']
                    estimates = response['estimates']
                    planner.metrics = response['planner']
                else:
                    try:
                        result = segmenter.segment(frame_rgb, **kwargs)
                    except FrontRoadInferenceError as error:
                        node.get_logger().warning(f'front Road inference failed: {error}')
                        with state_lock:
                            # A failed request from a cancelled task must not
                            # abort a newer Sidewalk (or Road) task.
                            if latest.is_current(packet):
                                if task_mode:
                                    node.abort_active_task(learned_road_backend(args) + '_unavailable')
                                else:
                                    state['front_command'] = None
                                    state['front_metrics'] = {}
                        continue
                    segmentation_finished_at = time.monotonic()
                with state_lock:
                    if not latest.is_current(packet):
                        continue
                # The path becomes usable when this result is available. Using
                # the camera-arrival timestamp here can expire a path before it
                # is ever published when inference or rate limiting is slow.
                if packet.fcn_masks or args.inference_process:
                    pass  # The FCN geometry or the child process already supplied the path.
                elif planner:
                    estimates = planner.plan(cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR),
                        result.selected_mask, getattr(result, 'road_hazard_mask', np.zeros_like(result.selected_mask)),
                        _stamp_ns(packet.source_header)/1e9, packet.generation, packet.sequence, mode=requested_mode,
                        building=getattr(result, 'road_building_mask', None))
                elif (args.connectome360_url or direct_input) and requested_mode == 1:
                    # The actor supplies velocity directly. Do not derive a
                    # geometric path from an atlas or invent one for telemetry.
                    estimates = {key: None for key in smoothers}
                else:
                    estimates = extract_path_estimates(result.selected_mask, tuple(smoothers), local_config)
                path_updated_at = time.monotonic()
                recovery_enabled = task_mode and node.drive_config.path_loss_recovery_enabled
                semantic_missing = {key: recovery_enabled and estimates[key] is None for key in smoothers}
                with state_lock:
                    if not latest.is_current(packet):
                        continue
                    if planner:
                        state['planner_status'] = dict(planner.metrics)
                    if packet.fcn_masks:
                        state['fcn_metrics'] = fcn_metrics
                        state['planner_status'] = {'backend': 'fcn360-local', 'ready': estimate is not None,
                                                   'reason': fcn_metrics['path_status']}
                    state['front_command'] = result.front_command if uses_learned_road_policy(args, requested_mode) else None
                    state['front_metrics'] = result.front_metrics if uses_learned_road_policy(args, requested_mode) else {}
                    for mask_class, smoother in smoothers.items():
                        if (uses_connectome(mask_class, planner is not None, state['planner_status'])
                                != isinstance(smoother, ConnectomePathTracker)):
                            smoother = smoothers[mask_class] = _path_smoother(
                                mask_class, args, local_config, state['planner_status'])
                        if estimates[mask_class] is None:
                            smoother.reset(preserve_branch=True)
                        smoother.update(estimates[mask_class], path_updated_at)
                    inference_finished_at = time.monotonic()
                    if task_mode:
                        if packet.sequence >= state["camera_fault_min_sequence"]:
                            state["camera_fault_reason"] = None
                        counts = state["path_unavailable_inferences"]
                        for mask_class, smoother in smoothers.items():
                            available = (
                                path_target_lateral(
                                    smoother.current(inference_finished_at),
                                    node.drive_config.lookahead_m,
                                ) is not None
                            )
                            current = smoother.current(inference_finished_at)
                            missing = semantic_missing[mask_class] and (
                                current is None or current.stop_reason is None
                            )
                            state["semantic_path_missing"][mask_class] = missing
                            recovery_counts = state["path_recovery_inferences"]
                            if available and current.stop_reason is None:
                                recovery_counts[mask_class] = 0
                            elif missing:
                                recovery_counts[mask_class] += 1
                            counts[mask_class] = 0 if available else counts[mask_class] + 1
                    state["header"] = packet.source_header
                    state['pipeline_timing'] = {
                        'segmentation_ms': (segmentation_finished_at-inference_started_at)*1000,
                        'planner_ms': (path_updated_at-segmentation_finished_at)*1000,
                        'source_age_at_completion_sec': ((node.get_clock().now().nanoseconds-
                            _stamp_ns(packet.source_header))/1e9 if planner or packet.fcn_masks or packet.direct_request else None),
                        'camera_transport': 'direct' if packet.direct_request else 'ros',
                        'cuda_graph': args.cuda_graph,
                        'inference_process': args.inference_process,
                    }
                    completed_before = int(state["inference_count"])
                    state["inference_count"] += 1
                    state["last_inference_at"] = inference_finished_at
                    if completed_before >= PERFORMANCE_WARMUP_FRAMES:
                        state["performance_inference_seconds"].append(
                            segmentation_finished_at - inference_started_at if packet.fcn_masks else result.inference_seconds
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

    thread = threading.Thread(target=worker, name="line-tracking-inference", daemon=True)
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
        if args.inference_process:
            segmenter.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_sigterm)
    if worker_error:
        raise worker_error[0]
    return 0


def _add_common_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument('--road-direct-input', action=argparse.BooleanOptionalAction,
                        default=_env('LINE_TRACKING_ROAD_DIRECT_INPUT', 'false').lower() == 'true')
    parser.add_argument('--fcn-url', default=_env('LINE_TRACKING_FCN_URL', 'http://127.0.0.1:8094/v1/direct-road'))
    parser.add_argument('--fcn-timeout-sec', type=float, default=_env_float('LINE_TRACKING_FCN_TIMEOUT_SEC', 2.))
    parser.add_argument('--connectome360-url', default=_env('LINE_TRACKING_CONNECTOME360_URL', ''))
    parser.add_argument('--connectome360-sha256', default=_env('LINE_TRACKING_CONNECTOME360_SHA256', ''))
    parser.add_argument('--connectome360-detector-sha256',
                        default=_env('LINE_TRACKING_CONNECTOME360_DETECTOR_SHA256', ''),
                        help='Explicit R50 manifest pin; keep the 360 package actor unchanged')
    parser.add_argument('--connectome360-timeout-sec', type=float,
                        default=_env_float('LINE_TRACKING_CONNECTOME360_TIMEOUT_SEC', 2.))
    parser.add_argument('--fcn-sha256', default=_env('LINE_TRACKING_FCN_SHA256', ''))
    parser.add_argument('--fcn-input-width', type=int, choices=(512, 1024),
                        default=int(_env('LINE_TRACKING_FCN_INPUT_WIDTH', '1024')))
    parser.add_argument('--fcn-height-m', type=float, default=_env_float('LINE_TRACKING_FCN_HEIGHT_M', 1.6))
    parser.add_argument('--fcn-max-age-sec', type=float, default=_env_float('LINE_TRACKING_FCN_MAX_AGE_SEC', 1.0))
    parser.add_argument('--front-connectome-url', default=_env('LINE_TRACKING_FRONT_CONNECTOME_URL', ''))
    parser.add_argument('--front-connectome-sha256', default=_env('LINE_TRACKING_FRONT_CONNECTOME_SHA256', ''))
    parser.add_argument('--front-connectome-timeout-sec', type=float,
        default=_env_float('LINE_TRACKING_FRONT_CONNECTOME_TIMEOUT_SEC', 1.))
    parser.add_argument('--path-planner', choices=('local', 'connectome'),
                        default=_env('LINE_TRACKING_PATH_PLANNER', 'local'))
    parser.add_argument('--connectome-url', default=_env('LINE_TRACKING_CONNECTOME_URL', 'http://127.0.0.1:8091/v1/plan'))
    parser.add_argument('--connectome-timeout-sec', type=float, default=_env_float('LINE_TRACKING_CONNECTOME_TIMEOUT_SEC', .8))
    parser.add_argument('--connectome-max-age-sec', type=float, default=_env_float('LINE_TRACKING_CONNECTOME_MAX_AGE_SEC', 1.))
    parser.add_argument('--road-corridor-smoothing-m', type=float,
        default=_env_float('LINE_TRACKING_ROAD_CORRIDOR_SMOOTHING_M', .30))
    parser.add_argument('--road-building-cost-weight', type=float,
        default=_env_float('LINE_TRACKING_ROAD_BUILDING_COST_WEIGHT', 2.))
    parser.add_argument('--road-building-distance-m', type=float,
        default=_env_float('LINE_TRACKING_ROAD_BUILDING_DISTANCE_M', 2.))
    parser.add_argument("--path-frame-id", default=_env("LINE_TRACKING_PATH_FRAME_ID", "base_link"))
    parser.add_argument("--robot-half-width-m", type=float, default=_env_float("LINE_TRACKING_ROBOT_HALF_WIDTH_M", .30))
    parser.add_argument("--path-margin-m", type=float, default=_env_float("LINE_TRACKING_PATH_MARGIN_M", .15))
    parser.add_argument(
        "--profile",
        choices=PROFILE_NAMES,
        default=_env("LINE_TRACKING_PROFILE", DEFAULT_PROFILE),
    )
    parser.add_argument("--model-id", default=_env("LINE_TRACKING_MODEL_ID", "") or None)
    parser.add_argument(
        "--model-revision", default=_env("LINE_TRACKING_MODEL_REVISION", "") or None
    )
    parser.add_argument(
        "--checkpoint-sha256",
        dest="checkpoint_manifest_sha256",
        default=_env("LINE_TRACKING_CHECKPOINT_SHA256", "") or None,
        help="Local R50 checkpoint manifest SHA-256; required for task-drive",
    )
    parser.add_argument("--device", default=_env("LINE_TRACKING_DEVICE", "auto"))
    parser.add_argument('--cuda-graph', action=argparse.BooleanOptionalAction,
                        default=_env_bool('LINE_TRACKING_CUDA_GRAPH', False))
    parser.add_argument('--telemetry-hz', type=float, default=_env_float('LINE_TRACKING_TELEMETRY_HZ', 0.),
                        help='Path/metrics publication rate; 0 follows the command timer')
    parser.add_argument('--inference-process', action=argparse.BooleanOptionalAction,
                        default=_env_bool('LINE_TRACKING_INFERENCE_PROCESS', False))
    parser.add_argument(
        "--backend",
        choices=INFERENCE_BACKENDS,
        default=_env("LINE_TRACKING_BACKEND", "pytorch"),
    )
    parser.add_argument(
        "--evaluation-size",
        type=int,
        nargs=2,
        default=(
            _env_int("LINE_TRACKING_EVALUATION_HEIGHT", DEFAULT_EVALUATION_SIZE[0]),
            _env_int("LINE_TRACKING_EVALUATION_WIDTH", DEFAULT_EVALUATION_SIZE[1]),
        ),
        metavar=("HEIGHT", "WIDTH"),
    )
    parser.add_argument(
        "--image-topic", default=_env("LINE_TRACKING_IMAGE_TOPIC", DEFAULT_IMAGE_TOPIC)
    )
    parser.add_argument(
        "--path-mask-class",
        type=int,
        choices=tuple(PATH_MASK_CLASSES),
        default=_env("LINE_TRACKING_PATH_MASK_CLASS", "2"),
        help="0=road or sidewalk, 1=road, 2=sidewalk (default from LINE_TRACKING_PATH_MASK_CLASS)",
    )
    parser.add_argument(
        "--near-distance-m",
        type=float,
        default=_env_float("LINE_TRACKING_NEAR_DISTANCE_M", 3.0),
    )
    parser.add_argument(
        "--far-distance-m", type=float, default=_env_float("LINE_TRACKING_FAR_DISTANCE_M", 8.0)
    )
    parser.add_argument(
        "--ground-half-width-m",
        type=float,
        default=_env_float("LINE_TRACKING_GROUND_HALF_WIDTH_M", 4.0),
    )
    parser.add_argument(
        "--search-half-width-m",
        type=float,
        default=_env_float("LINE_TRACKING_SEARCH_HALF_WIDTH_M", 3.5),
    )
    parser.add_argument(
        "--path-points", type=int, default=_env_int("LINE_TRACKING_PATH_POINTS", 20)
    )
    parser.add_argument(
        "--bev-width-px", type=int, default=_env_int("LINE_TRACKING_BEV_WIDTH_PX", 280)
    )
    parser.add_argument(
        "--bev-height-px", type=int, default=_env_int("LINE_TRACKING_BEV_HEIGHT_PX", 160)
    )
    parser.add_argument(
        "--min-valid-ratio",
        type=float,
        default=_env_float("LINE_TRACKING_MIN_VALID_RATIO", 0.35),
    )
    parser.add_argument(
        "--min-sidewalk-width-m",
        type=float,
        default=_env_float("LINE_TRACKING_MIN_SIDEWALK_WIDTH_M", 0.12),
    )
    parser.add_argument(
        "--close-kernel-px", type=int, default=_env_int("LINE_TRACKING_CLOSE_KERNEL_PX", 5)
    )
    parser.add_argument(
        "--smoothing-time-constant-sec",
        type=float,
        default=_env_float("LINE_TRACKING_SMOOTHING_TIME_CONSTANT_SEC", 0.80),
    )
    parser.add_argument(
        "--max-lateral-update-m",
        type=float,
        default=_env_float("LINE_TRACKING_MAX_LATERAL_UPDATE_M", 0.35),
    )
    parser.add_argument(
        "--path-hold-sec", type=float, default=_env_float("LINE_TRACKING_PATH_HOLD_SEC", 0.90)
    )
    parser.add_argument(
        "--path-duration-sec",
        type=float,
        default=_env_float("LINE_TRACKING_PATH_DURATION_SEC", 1.50),
    )
    parser.add_argument(
        "--unrestricted-path-mode",
        action=argparse.BooleanOptionalAction,
        default=_env_bool("LINE_TRACKING_UNRESTRICTED_PATH_MODE", True),
        help=(
            "bypass path valid-ratio and temporal smoothing "
            "restrictions"
        ),
    )
    parser.add_argument(
        "--branch-preference", choices=("center", "left", "right"),
        default=_env("LINE_TRACKING_BRANCH_PREFERENCE", "right"),
        help="prefer a confirmed connected left/right fork; center preserves legacy extraction",
    )
    parser.add_argument(
        "--branch-min-width-m", type=float,
        default=_env_float("LINE_TRACKING_BRANCH_MIN_WIDTH_M", 0.60),
    )
    parser.add_argument(
        "--branch-margin-m", type=float,
        default=_env_float("LINE_TRACKING_BRANCH_MARGIN_M", 0.10),
    )
    parser.add_argument(
        "--branch-confirm-frames", type=int,
        default=_env_int("LINE_TRACKING_BRANCH_CONFIRM_FRAMES", 2),
    )
    parser.add_argument(
        "--branch-hold-sec", type=float,
        default=_env_float("LINE_TRACKING_BRANCH_HOLD_SEC", 1.50),
    )
    parser.add_argument(
        "--output-fps", type=float, default=_env_float("LINE_TRACKING_OUTPUT_FPS", 20.0)
    )
    parser.add_argument(
        "--roi-polygon",
        type=float,
        nargs=8,
        default=None,
        metavar=("BL_X", "BL_Y", "BR_X", "BR_Y", "TR_X", "TR_Y", "TL_X", "TL_Y"),
    )
    parser.add_argument(
        "--road-roi-polygon", type=float, nargs=8, default=None,
        help="Mode 1 ROI only; Mode 0/2 keep --roi-polygon",
    )
    parser.add_argument(
        '--road-roi-exclusion-polygon', type=float, nargs='*', default=None,
        help='Mode 1 camera-body image exclusion, independent of metric projection; empty disables it',
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
        "--sample-hz", type=float, default=4.0,
        help="MCAP local-path replay sampling only; ignored in unrestricted mode",
    )
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
            default=_env("LINE_TRACKING_LOCAL_PATH_TOPIC", DEFAULT_LOCAL_PATH_TOPIC),
        )
        live.add_argument(
            "--metrics-topic",
            default=_env("LINE_TRACKING_METRICS_TOPIC", DEFAULT_METRICS_TOPIC),
        )
        live.add_argument(
            "--output-hz", type=float, default=_env_float("LINE_TRACKING_OUTPUT_HZ", 10.0)
        )
        live.add_argument(
            "--reliability",
            choices=("best_effort", "reliable"),
            default=_env("LINE_TRACKING_INPUT_RELIABILITY", "best_effort"),
        )
        if mode == "task-drive":
            for check in AUTOMATIC_STOP_CHECKS:
                live.add_argument(
                    "--stop-on-" + check.replace("_", "-"),
                    action=argparse.BooleanOptionalAction,
                    default=_env_bool("LINE_TRACKING_STOP_ON_" + check.upper(), False),
                    help="Enable this automatic stop check (default: disabled)",
                )
            live.add_argument(
                "--apriltag-detections-topic",
                default=_env("LINE_TRACKING_APRILTAG_DETECTIONS_TOPIC", "/detections"),
            )
            live.add_argument(
                "--apriltag-max-age-sec",
                type=float,
                default=_env_float("LINE_TRACKING_APRILTAG_MAX_AGE_SEC", 1.0),
            )
            live.add_argument(
                "--apriltag-confirm-window-sec",
                type=float,
                default=_env_float("LINE_TRACKING_APRILTAG_CONFIRM_WINDOW_SEC", 1.0),
            )
            live.add_argument(
                "--apriltag-confirm-min-hits",
                type=int,
                default=_env_int("LINE_TRACKING_APRILTAG_CONFIRM_MIN_HITS", 3),
            )
            live.add_argument(
                "--apriltag-max-range-m",
                type=float,
                default=_env_float("LINE_TRACKING_APRILTAG_MAX_RANGE_M", 3.0),
                help="Only tags within this camera range stop tracking (0 disables)",
            )
            live.add_argument(
                "--apriltag-min-decision-margin",
                type=float,
                default=_env_float("LINE_TRACKING_APRILTAG_MIN_DECISION_MARGIN", 15.0),
            )
            live.add_argument(
                "--apriltag-allowed-ids",
                default=_env("LINE_TRACKING_APRILTAG_ALLOWED_IDS", ""),
                help='Ids that may stop tracking, e.g. "0-30"; blank means any',
            )
            live.add_argument(
                "--apriltag-tag-size-m",
                type=float,
                default=_env_float("LINE_TRACKING_APRILTAG_TAG_SIZE_M", 0.167),
            )
            live.add_argument(
                "--apriltag-camera-info-topic",
                default=_env(
                    "LINE_TRACKING_APRILTAG_RANGE_CAMERA_INFO_TOPIC",
                    DEFAULT_APRILTAG_CAMERA_INFO_TOPIC,
                ),
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
                "--path-loss-recovery-enabled", action=argparse.BooleanOptionalAction,
                default=_env_bool("LINE_TRACKING_PATH_LOSS_RECOVERY_ENABLED", True),
                help="Retain straight recovery until the third visual path failure; wait after stopping, then scan +/-90 degrees",
            )
            live.add_argument(
                "--path-recovery-wait-sec", type=float,
                default=_env_float("LINE_TRACKING_PATH_RECOVERY_WAIT_SEC", 1.0),
            )
            live.add_argument(
                "--path-recovery-first-direction", choices=("left", "right"),
                default=_env("LINE_TRACKING_PATH_RECOVERY_FIRST_DIRECTION", "left"),
                help="Fallback first search side when the last confident path has no clear bend",
            )
            live.add_argument(
                "--path-recovery-yaw-rps", type=float,
                default=_env_float("LINE_TRACKING_PATH_RECOVERY_YAW_RPS", 0.18),
            )
            live.add_argument(
                "--path-recovery-confirm-frames", type=int,
                default=_env_int("LINE_TRACKING_PATH_RECOVERY_CONFIRM_FRAMES", 2),
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
    from migrate_tracking_config import validate_environment
    try:
        validate_environment(ENV)
    except ValueError as error:
        parser.error(str(error))
    args = parser.parse_args(argv)
    if args.road_direct_input:
        if (args.mode != 'task-drive' or not (args.connectome360_url or args.front_connectome_url or args.fcn_sha256)):
            parser.error('direct input requires a task-drive Road inference service')
        url, _, _, timeout = direct_camera_config(args)
        if (not url.startswith(('http://127.0.0.1:', 'http://localhost:'))
                or not math.isfinite(timeout) or timeout <= 0):
            parser.error('direct camera requires a loopback URL and a positive timeout')
    if args.connectome360_detector_sha256:
        if (not args.connectome360_url or not args.road_direct_input
                or len(args.connectome360_detector_sha256) != 64
                or any(c not in '0123456789abcdef' for c in args.connectome360_detector_sha256)):
            parser.error('R50 detector override requires direct 360 Road input and a lowercase manifest SHA256')
    if args.connectome360_url:
        if (args.mode != 'task-drive' or args.path_mask_class not in (1, 2)
                or args.path_planner != 'local' or args.inference_process
                or args.front_connectome_url or args.fcn_sha256):
            parser.error('360 Connectome requires task-drive, mask 1 or 2, local planner and an exclusive Road backend')
        if (len(args.connectome360_sha256) != 64
                or any(c not in '0123456789abcdef' for c in args.connectome360_sha256)
                or not math.isfinite(args.connectome360_timeout_sec) or args.connectome360_timeout_sec <= 0
                or not args.connectome360_url.startswith(('http://127.0.0.1:', 'http://localhost:'))):
            parser.error('360 Connectome requires a pinned model.json SHA256, loopback URL and positive timeout')
    if args.fcn_sha256:
        if (args.fcn_input_width not in (512, 1024) or len(args.fcn_sha256) != 64 or any(c not in '0123456789abcdef' for c in args.fcn_sha256)
                or not 0 < args.fcn_height_m < 5 or not math.isfinite(args.fcn_max_age_sec) or args.fcn_max_age_sec <= 0):
            parser.error('FCN requires a lowercase SHA256, camera height (0..5 m) and positive maximum age')
        if (args.mode != 'task-drive' or args.path_mask_class not in (1, 2) or args.path_planner != 'local'
                or args.inference_process or args.front_connectome_url):
            parser.error('FCN requires task-drive, Road or Sidewalk, local planner, and exclusive Road backend')
    if args.front_connectome_url:
        if args.path_mask_class not in (1, 2) or args.path_planner != 'local' or args.inference_process:
            parser.error('front/sidewalk routing requires explicit mask 1 or 2, local planner and no inference process')
        if not math.isfinite(args.front_connectome_timeout_sec) or args.front_connectome_timeout_sec <= 0:
            parser.error('positive front connectome timeout required')
    if not math.isfinite(args.telemetry_hz) or not 0 <= args.telemetry_hz <= 50:
        parser.error('telemetry rate must be finite and 0..50 Hz')
    if args.inference_process and (args.path_planner != 'connectome' or args.mode == 'mcap'):
        parser.error('inference process requires live ros2/task-drive with connectome')
    if not all(math.isfinite(x) and x > 0 for x in (args.connectome_timeout_sec, args.connectome_max_age_sec)):
        parser.error('connectome timeout and max age must be finite and positive')
    try:
        PlannerConfig(half_width_m=args.robot_half_width_m, margin_m=args.path_margin_m,
                      road_corridor_smoothing_m=args.road_corridor_smoothing_m,
                      road_building_cost_weight=args.road_building_cost_weight,
                      road_building_distance_m=args.road_building_distance_m).validate()
    except ValueError as error:
        parser.error(str(error))
    if args.mode != "mcap" or args.overlay_mode == "local-path":
        try:
            _local_path_config_from_args(args).validate()
        except ValueError as error:
            parser.error(str(error))
    if args.mode == "task-drive" and "LINE_TRACKING_BYPASS_PATH_STOPS" in ENV:
        parser.error(
            "LINE_TRACKING_BYPASS_PATH_STOPS was removed; remove it. "
            "The separate LINE_TRACKING_STOP_ON_PATH_LOSS_LIMIT option remains available."
        )
    if args.path_mask_class not in PATH_MASK_CLASSES:
        parser.error(
            "LINE_TRACKING_PATH_MASK_CLASS must be 0 (road or sidewalk), "
            "1 (road), or 2 (sidewalk)"
        )
    if not math.isfinite(args.output_fps) or args.output_fps <= 0.0:
        parser.error("output FPS must be finite and positive")
    if args.mode == "mcap" and (not math.isfinite(args.sample_hz) or args.sample_hz <= 0.0):
        parser.error("sample-hz must be finite and positive")
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
