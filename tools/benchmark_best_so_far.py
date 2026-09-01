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
"""Measure best-so-far segmentation Hz from MCAP or a live ROS 2 image topic.

MCAP mode is portable and needs no ROS installation. ``realtime`` playback
uses a latest-frame queue, so it reproduces the frame replacement behavior of
a low-latency live subscriber when segmentation is slower than the camera.

ROS 2 mode subscribes directly to ``sensor_msgs/msg/Image``. Run that mode with
the ROS distro's Python after installing the model dependencies; ``rclpy`` and
``cv_bridge`` are supplied by ROS and are intentionally not PyPI dependencies.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import statistics
import tempfile
import threading
import time
from collections.abc import Iterable, Iterator, Sequence
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
from best_so_far_runtime import (
    DEFAULT_EVALUATION_SIZE,
    DEFAULT_PROFILE,
    PROFILE_NAMES,
    BestSoFarConfig,
    BestSoFarResult,
    BestSoFarSegmenter,
)

RAW_IMAGE_TYPE = "sensor_msgs/msg/Image"
DEFAULT_TOPIC = "/a2/front_camera/res_360p/image_raw"


@dataclass(frozen=True)
class FramePacket:
    frame_bgr: np.ndarray
    source_timestamp_ns: int
    arrival_monotonic: float
    sequence: int
    source_header: Any = None


class LatestFrameQueue:
    """A thread-safe, depth-one queue that counts overwritten frames."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._item: FramePacket | None = None
        self._closed = False
        self.overwritten = 0

    def put(self, packet: FramePacket) -> bool:
        with self._condition:
            if self._closed:
                return False
            if self._item is not None:
                self.overwritten += 1
            self._item = packet
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


def _percentile(values: Sequence[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(ordered[lower])
    fraction = position - lower
    return float(ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction)


def _latency_summary(seconds: Sequence[float]) -> dict[str, float | None]:
    milliseconds = [value * 1000.0 for value in seconds]
    return {
        "mean": statistics.fmean(milliseconds) if milliseconds else None,
        "p50": _percentile(milliseconds, 0.50),
        "p95": _percentile(milliseconds, 0.95),
        "p99": _percentile(milliseconds, 0.99),
        "max": max(milliseconds) if milliseconds else None,
    }


def _timestamp_hz(timestamps_ns: Sequence[int]) -> float | None:
    deltas = [
        current - previous
        for previous, current in pairwise(timestamps_ns)
        if current > previous
    ]
    if not deltas:
        return None
    return 1_000_000_000.0 / statistics.median(deltas)


class BenchmarkStats:
    """Thread-safe counters and latency samples for one source stream."""

    def __init__(self, *, warmup_frames: int, expected_input_hz: float) -> None:
        self.warmup_frames = warmup_frames
        self.expected_input_hz = expected_input_hz
        self.lock = threading.Lock()
        self.received = 0
        self.processed = 0
        self.errors = 0
        self.source_timestamps_ns: list[int] = []
        self.arrival_times: list[float] = []
        self.measured_completion_times: list[float] = []
        self.processing_seconds: list[float] = []
        self.inference_seconds: list[float] = []
        self.postprocess_seconds: list[float] = []
        self.queue_seconds: list[float] = []
        self.end_to_end_seconds: list[float] = []
        self.hold_ratios: list[float] = []
        self.road_area_ratios: list[float] = []
        self.sidewalk_area_ratios: list[float] = []

    def record_received(self, packet: FramePacket) -> None:
        with self.lock:
            self.received += 1
            self.source_timestamps_ns.append(packet.source_timestamp_ns)
            self.arrival_times.append(packet.arrival_monotonic)

    def record_processed(
        self,
        packet: FramePacket,
        result: BestSoFarResult,
        *,
        started: float,
        completed: float,
    ) -> None:
        with self.lock:
            self.processed += 1
            if self.processed <= self.warmup_frames:
                return
            self.measured_completion_times.append(completed)
            self.processing_seconds.append(result.total_seconds)
            self.inference_seconds.append(result.inference_seconds)
            self.postprocess_seconds.append(result.postprocess_seconds)
            self.queue_seconds.append(max(0.0, started - packet.arrival_monotonic))
            self.end_to_end_seconds.append(
                max(0.0, completed - packet.arrival_monotonic)
            )
            self.hold_ratios.append(result.hysteresis_hold_ratio)
            self.road_area_ratios.append(result.road_area_ratio)
            self.sidewalk_area_ratios.append(result.sidewalk_area_ratio)

    def record_error(self) -> None:
        with self.lock:
            self.errors += 1

    def report(self, *, overwritten: int) -> dict[str, Any]:
        with self.lock:
            source_hz = _timestamp_hz(self.source_timestamps_ns)
            if source_hz is None and self.expected_input_hz > 0.0:
                source_hz = self.expected_input_hz
            arrival_hz = None
            if len(self.arrival_times) >= 2:
                duration = self.arrival_times[-1] - self.arrival_times[0]
                if duration > 0.0:
                    arrival_hz = (len(self.arrival_times) - 1) / duration
            segmentation_hz = (
                len(self.processing_seconds) / sum(self.processing_seconds)
                if self.processing_seconds and sum(self.processing_seconds) > 0.0
                else None
            )
            effective_output_hz = None
            if len(self.measured_completion_times) >= 2:
                duration = (
                    self.measured_completion_times[-1]
                    - self.measured_completion_times[0]
                )
                if duration > 0.0:
                    effective_output_hz = (
                        len(self.measured_completion_times) - 1
                    ) / duration
            measured = len(self.processing_seconds)
            dropped = max(overwritten, self.received - self.processed - self.errors)
            reference_hz = source_hz or self.expected_input_hz or None
            realtime_factor = (
                segmentation_hz / reference_hz
                if segmentation_hz is not None and reference_hz
                else None
            )
            return {
                "counters": {
                    "received_frames": self.received,
                    "processed_frames": self.processed,
                    "measured_frames": measured,
                    "warmup_frames": min(self.processed, self.warmup_frames),
                    "overwritten_frames": overwritten,
                    "dropped_frames": dropped,
                    "processing_errors": self.errors,
                    "drop_ratio": dropped / self.received if self.received else 0.0,
                },
                "rates_hz": {
                    "source_timestamp": source_hz,
                    "wall_arrival": arrival_hz,
                    "segmentation_compute": segmentation_hz,
                    "effective_output": effective_output_hz,
                    "realtime_factor": realtime_factor,
                },
                "latency_ms": {
                    "processing": _latency_summary(self.processing_seconds),
                    "inference": _latency_summary(self.inference_seconds),
                    "postprocess": _latency_summary(self.postprocess_seconds),
                    "queue_wait": _latency_summary(self.queue_seconds),
                    "arrival_to_output": _latency_summary(self.end_to_end_seconds),
                },
                "mask_summary": {
                    "mean_hysteresis_hold_ratio": (
                        statistics.fmean(self.hold_ratios) if self.hold_ratios else None
                    ),
                    "mean_road_area_ratio": (
                        statistics.fmean(self.road_area_ratios)
                        if self.road_area_ratios
                        else None
                    ),
                    "mean_sidewalk_area_ratio": (
                        statistics.fmean(self.sidewalk_area_ratios)
                        if self.sidewalk_area_ratios
                        else None
                    ),
                },
                "verdict": {
                    "reference_input_hz": reference_hz,
                    "can_keep_up": (
                        segmentation_hz >= reference_hz
                        if segmentation_hz is not None and reference_hz
                        else None
                    ),
                },
            }


def decoded_image_to_bgr(message: Any) -> np.ndarray:
    """Convert common ROS ``Image`` encodings without requiring cv_bridge."""

    encoding = str(message.encoding).lower()
    formats = {
        "rgb8": (3, cv2.COLOR_RGB2BGR),
        "bgr8": (3, None),
        "rgba8": (4, cv2.COLOR_RGBA2BGR),
        "bgra8": (4, cv2.COLOR_BGRA2BGR),
        "mono8": (1, cv2.COLOR_GRAY2BGR),
    }
    if encoding not in formats:
        raise ValueError(
            f"unsupported image encoding '{encoding}'; "
            f"supported encodings: {', '.join(sorted(formats))}"
        )
    channels, conversion = formats[encoding]
    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    row_bytes = width * channels
    if width <= 0 or height <= 0 or step < row_bytes:
        raise ValueError(
            f"invalid image layout: {width=} {height=} {step=} {encoding=}"
        )
    data = np.frombuffer(bytes(message.data), dtype=np.uint8)
    required = step * height
    if data.size < required:
        raise ValueError(f"image has {data.size} bytes; expected at least {required}")
    rows = data[:required].reshape(height, step)[:, :row_bytes]
    image = (
        rows.reshape(height, width, channels)
        if channels > 1
        else rows.reshape(height, width)
    )
    if conversion is None:
        return np.ascontiguousarray(image)
    return cv2.cvtColor(image, conversion)


def resolve_mcap_files(inputs: Iterable[Path]) -> list[Path]:
    resolved: list[Path] = []
    for raw_path in inputs:
        path = raw_path.expanduser().resolve()
        if path.is_file():
            if path.suffix.lower() != ".mcap":
                raise ValueError(f"input is not an MCAP file: {path}")
            resolved.append(path)
            continue
        if not path.is_dir():
            raise FileNotFoundError(f"MCAP file or bag directory not found: {path}")
        files = sorted(path.glob("*.mcap"))
        if not files:
            raise ValueError(f"bag directory contains no MCAP files: {path}")
        resolved.extend(files)
    if not resolved:
        raise ValueError("at least one MCAP input is required")
    return resolved


def inspect_mcap(path: Path, topic: str) -> dict[str, Any]:
    from mcap.reader import make_reader

    with path.open("rb") as stream:
        summary = make_reader(stream).get_summary()
    if summary is None or summary.statistics is None:
        raise RuntimeError(f"MCAP has no readable summary: {path}")
    matching = [
        channel for channel in summary.channels.values() if channel.topic == topic
    ]
    if not matching:
        available = sorted({channel.topic for channel in summary.channels.values()})
        raise ValueError(
            f"topic '{topic}' is absent from {path}; available topics: {available}"
        )
    channel = matching[0]
    schema = summary.schemas.get(channel.schema_id)
    count = int(summary.statistics.channel_message_counts.get(channel.id, 0))
    if schema is None or schema.name != RAW_IMAGE_TYPE:
        raise ValueError(
            f"topic '{topic}' has type '{schema.name if schema else None}', "
            f"expected '{RAW_IMAGE_TYPE}'"
        )
    start_ns = int(summary.statistics.message_start_time)
    end_ns = int(summary.statistics.message_end_time)
    return {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "topic": topic,
        "message_type": schema.name,
        "topic_message_count": count,
        "bag_start_ns": start_ns,
        "bag_end_ns": end_ns,
        "bag_duration_seconds": (end_ns - start_ns) / 1_000_000_000.0,
    }


def iter_mcap_packets(
    path: Path,
    topic: str,
    *,
    start_offset_seconds: float,
    max_frames: int,
) -> Iterator[FramePacket]:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    profile = inspect_mcap(path, topic)
    start_time = profile["bag_start_ns"] + int(start_offset_seconds * 1_000_000_000)
    with path.open("rb") as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        for sequence, (schema, _channel, message, decoded) in enumerate(
            reader.iter_decoded_messages(topics=[topic], start_time=start_time)
        ):
            if max_frames and sequence >= max_frames:
                break
            if schema.name != RAW_IMAGE_TYPE:
                raise ValueError(
                    f"topic '{topic}' changed to unsupported type '{schema.name}'"
                )
            arrival = time.perf_counter()
            yield FramePacket(
                frame_bgr=decoded_image_to_bgr(decoded),
                source_timestamp_ns=int(message.log_time),
                arrival_monotonic=arrival,
                sequence=sequence,
            )


def _progress_line(stats: BenchmarkStats, overwritten: int) -> str:
    report = stats.report(overwritten=overwritten)
    counters = report["counters"]
    rates = report["rates_hz"]
    latency = report["latency_ms"]["processing"]
    source_hz = rates["source_timestamp"] or 0.0
    compute_hz = rates["segmentation_compute"] or 0.0
    p50 = latency["p50"] or 0.0
    return (
        f"BENCHMARK_PROGRESS received={counters['received_frames']} "
        f"processed={counters['processed_frames']} "
        f"dropped={counters['dropped_frames']} source_hz={source_hz:.3f} "
        f"segmentation_hz={compute_hz:.3f} processing_p50_ms={p50:.1f}"
    )


def _run_throughput(
    segmenter: BestSoFarSegmenter,
    packets: Iterable[FramePacket],
    stats: BenchmarkStats,
    *,
    report_interval: float,
) -> tuple[int, tuple[np.ndarray, np.ndarray] | None]:
    last_report = time.perf_counter()
    last_output: tuple[np.ndarray, np.ndarray] | None = None
    for packet in packets:
        stats.record_received(packet)
        started = time.perf_counter()
        try:
            result = segmenter.segment(packet.frame_bgr)
        except Exception:
            stats.record_error()
            raise
        completed = time.perf_counter()
        stats.record_processed(packet, result, started=started, completed=completed)
        last_output = (packet.frame_bgr, result.selected_mask)
        if completed - last_report >= report_interval:
            print(_progress_line(stats, 0), flush=True)
            last_report = completed
    return 0, last_output


def _run_realtime(
    segmenter: BestSoFarSegmenter,
    packets: Iterable[FramePacket],
    stats: BenchmarkStats,
    *,
    playback_rate: float,
    report_interval: float,
) -> tuple[int, tuple[np.ndarray, np.ndarray] | None]:
    latest = LatestFrameQueue()
    producer_error: list[BaseException] = []

    def produce() -> None:
        first_source_ns: int | None = None
        first_wall = 0.0
        try:
            for original in packets:
                if first_source_ns is None:
                    first_source_ns = original.source_timestamp_ns
                    first_wall = time.perf_counter()
                target = first_wall + (
                    (original.source_timestamp_ns - first_source_ns)
                    / 1_000_000_000.0
                    / playback_rate
                )
                delay = target - time.perf_counter()
                if delay > 0.0:
                    time.sleep(delay)
                packet = FramePacket(
                    frame_bgr=original.frame_bgr,
                    source_timestamp_ns=original.source_timestamp_ns,
                    arrival_monotonic=time.perf_counter(),
                    sequence=original.sequence,
                )
                stats.record_received(packet)
                latest.put(packet)
        except Exception as error:  # noqa: BLE001 - forward producer failures.
            producer_error.append(error)
        finally:
            latest.close()

    producer = threading.Thread(target=produce, name="mcap-producer", daemon=True)
    producer.start()
    last_report = time.perf_counter()
    last_output: tuple[np.ndarray, np.ndarray] | None = None
    while True:
        packet = latest.get()
        if packet is None:
            break
        started = time.perf_counter()
        try:
            result = segmenter.segment(packet.frame_bgr)
        except Exception:
            stats.record_error()
            latest.close()
            producer.join()
            raise
        completed = time.perf_counter()
        stats.record_processed(packet, result, started=started, completed=completed)
        last_output = (packet.frame_bgr, result.selected_mask)
        if completed - last_report >= report_interval:
            print(_progress_line(stats, latest.overwritten), flush=True)
            last_report = completed
    producer.join()
    if producer_error:
        raise producer_error[0]
    return latest.overwritten, last_output


def _atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    output = path.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.", suffix=".tmp", dir=output.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary_name, output)
    except BaseException:
        Path(temporary_name).unlink(missing_ok=True)
        raise


def _environment_metadata(segmenter: BestSoFarSegmenter) -> dict[str, Any]:
    return {
        "platform": platform.platform(),
        "machine": platform.machine(),
        "python": platform.python_version(),
        "torch": torch.__version__,
        **segmenter.metadata(),
    }


def run_mcap(args: argparse.Namespace, segmenter: BestSoFarSegmenter) -> dict[str, Any]:
    files = resolve_mcap_files(args.input)
    runs: list[dict[str, Any]] = []
    for index, path in enumerate(files, start=1):
        segmenter.reset()
        profile = inspect_mcap(path, args.topic)
        print(
            f"MCAP_START index={index}/{len(files)} path={path} "
            f"topic={args.topic} mode={args.playback_mode}",
            flush=True,
        )
        packets = iter_mcap_packets(
            path,
            args.topic,
            start_offset_seconds=args.start_offset,
            max_frames=args.max_frames,
        )
        stats = BenchmarkStats(
            warmup_frames=args.warmup_frames,
            expected_input_hz=args.expected_input_hz,
        )
        started = time.perf_counter()
        if args.playback_mode == "realtime":
            overwritten, last_output = _run_realtime(
                segmenter,
                packets,
                stats,
                playback_rate=args.playback_rate,
                report_interval=args.report_interval,
            )
        else:
            overwritten, last_output = _run_throughput(
                segmenter,
                packets,
                stats,
                report_interval=args.report_interval,
            )
        report = stats.report(overwritten=overwritten)
        report.update(
            {
                "source": profile,
                "playback_mode": args.playback_mode,
                "playback_rate": args.playback_rate,
                "start_offset_seconds": args.start_offset,
                "wall_elapsed_seconds": time.perf_counter() - started,
            }
        )
        runs.append(report)
        if args.snapshot_dir is not None and last_output is not None:
            frame_bgr, selected = last_output
            source_hz = report["rates_hz"]["source_timestamp"] or args.expected_input_hz
            overlay = segmenter.render_overlay(
                frame_bgr,
                selected,
                frame_index=max(stats.processed - 1, 0),
                fps=source_hz,
            )
            snapshot_dir = args.snapshot_dir.expanduser().resolve()
            snapshot_dir.mkdir(parents=True, exist_ok=True)
            snapshot_path = snapshot_dir / f"{path.stem}-{segmenter.profile.name}.jpg"
            if not cv2.imwrite(str(snapshot_path), overlay):
                raise RuntimeError(f"could not write snapshot: {snapshot_path}")
            report["snapshot"] = str(snapshot_path)
        print(_progress_line(stats, overwritten), flush=True)
        print(
            f"MCAP_COMPLETE index={index}/{len(files)} "
            f"can_keep_up={report['verdict']['can_keep_up']}",
            flush=True,
        )
    return {
        "schema_version": 1,
        "source_mode": "mcap",
        "environment": _environment_metadata(segmenter),
        "runs": runs,
    }


def run_ros2(args: argparse.Namespace, segmenter: BestSoFarSegmenter) -> dict[str, Any]:
    """Run the same depth-one benchmark against a live ROS 2 image topic."""

    try:
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 mode requires a sourced ROS environment with rclpy, "
            "cv_bridge, sensor_msgs and std_msgs. MCAP mode does not require ROS."
        ) from error

    segmenter.reset()
    latest = LatestFrameQueue()
    stats = BenchmarkStats(
        warmup_frames=args.warmup_frames,
        expected_input_hz=args.expected_input_hz,
    )
    stop_requested = threading.Event()
    worker_error: list[BaseException] = []

    class BenchmarkNode(Node):
        def __init__(self) -> None:
            super().__init__("best_so_far_segmentation_benchmark")
            self.bridge = CvBridge()
            reliability = (
                ReliabilityPolicy.RELIABLE
                if args.reliability == "reliable"
                else ReliabilityPolicy.BEST_EFFORT
            )
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=reliability,
            )
            self.subscription = self.create_subscription(
                Image,
                args.topic,
                self.on_image,
                qos,
            )
            self.overlay_publisher = (
                self.create_publisher(Image, args.overlay_topic, qos)
                if args.overlay_topic
                else None
            )
            self.metrics_publisher = (
                self.create_publisher(String, args.metrics_topic, 10)
                if args.metrics_topic
                else None
            )
            self.started = time.perf_counter()
            self.timer = self.create_timer(0.2, self.check_stop)
            self.get_logger().info(
                f"benchmark started | topic={args.topic} "
                f"reliability={args.reliability} "
                f"expected_hz={args.expected_input_hz:.3f}"
            )

        def on_image(self, message: Any) -> None:
            arrival = time.perf_counter()
            try:
                frame_bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            except Exception as error:  # noqa: BLE001 - cv_bridge error boundary.
                stats.record_error()
                self.get_logger().error(f"image conversion failed: {error}")
                return
            source_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
                message.header.stamp.nanosec
            )
            packet = FramePacket(
                frame_bgr=np.ascontiguousarray(frame_bgr),
                source_timestamp_ns=source_ns,
                arrival_monotonic=arrival,
                sequence=stats.received,
                source_header=message.header,
            )
            stats.record_received(packet)
            latest.put(packet)
            if args.max_frames and stats.received >= args.max_frames:
                stop_requested.set()

        def publish_result(
            self,
            packet: FramePacket,
            result: BestSoFarResult,
        ) -> None:
            if self.overlay_publisher is not None:
                source_hz = (
                    stats.report(overwritten=latest.overwritten)["rates_hz"][
                        "source_timestamp"
                    ]
                    or args.expected_input_hz
                )
                overlay = segmenter.render_overlay(
                    packet.frame_bgr,
                    result.selected_mask,
                    frame_index=packet.sequence,
                    fps=source_hz,
                )
                message = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                message.header = packet.source_header
                self.overlay_publisher.publish(message)

        def publish_metrics(self) -> None:
            if self.metrics_publisher is None:
                return
            payload = stats.report(overwritten=latest.overwritten)
            self.metrics_publisher.publish(String(data=json.dumps(payload)))

        def check_stop(self) -> None:
            if (
                args.duration > 0.0
                and time.perf_counter() - self.started >= args.duration
            ):
                stop_requested.set()
            if stop_requested.is_set() and rclpy.ok():
                rclpy.shutdown()

    # argparse has already consumed this tool's CLI. Avoid presenting those
    # non-ROS arguments to rclpy's own argument parser.
    rclpy.init(args=[])
    node = BenchmarkNode()

    def process_latest() -> None:
        last_report = time.perf_counter()
        try:
            while True:
                packet = latest.get()
                if packet is None:
                    break
                started = time.perf_counter()
                result = segmenter.segment(packet.frame_bgr)
                completed = time.perf_counter()
                stats.record_processed(
                    packet,
                    result,
                    started=started,
                    completed=completed,
                )
                node.publish_result(packet, result)
                if completed - last_report >= args.report_interval:
                    line = _progress_line(stats, latest.overwritten)
                    node.get_logger().info(line)
                    node.publish_metrics()
                    last_report = completed
        except Exception as error:  # noqa: BLE001 - worker failure boundary.
            stats.record_error()
            worker_error.append(error)
            stop_requested.set()
            if rclpy.ok():
                rclpy.shutdown()

    worker = threading.Thread(
        target=process_latest,
        name="segmentation-worker",
        daemon=True,
    )
    worker.start()
    started = time.perf_counter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop_requested.set()
    finally:
        latest.close()
        worker.join()
        final_metrics = stats.report(overwritten=latest.overwritten)
        try:
            node.publish_metrics()
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    if worker_error:
        raise worker_error[0]
    return {
        "schema_version": 1,
        "source_mode": "ros2",
        "environment": _environment_metadata(segmenter),
        "run": {
            "source": {
                "topic": args.topic,
                "message_type": RAW_IMAGE_TYPE,
                "reliability": args.reliability,
            },
            "wall_elapsed_seconds": time.perf_counter() - started,
            **final_metrics,
        },
    }


def _add_model_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--profile",
        choices=PROFILE_NAMES,
        default=DEFAULT_PROFILE,
        help=(
            "named runtime profile; use swin-l-best-so-far to restore the "
            "retained quality baseline"
        ),
    )
    parser.add_argument(
        "--model-id",
        default=None,
        help="optional checkpoint override for the selected profile family",
    )
    parser.add_argument(
        "--model-revision",
        default=None,
        help="optional pinned revision override",
    )
    parser.add_argument(
        "--evaluation-size",
        type=int,
        nargs=2,
        metavar=("HEIGHT", "WIDTH"),
        default=DEFAULT_EVALUATION_SIZE,
    )
    parser.add_argument("--temporal-alpha", type=float, default=None)
    parser.add_argument("--temporal-hysteresis-margin", type=float, default=None)
    parser.add_argument(
        "--device",
        default="auto",
        help="auto, cpu, cuda, mps, or another torch device string",
    )
    parser.add_argument("--warmup-frames", type=int, default=2)
    parser.add_argument("--expected-input-hz", type=float, default=20.0)
    parser.add_argument("--report-interval", type=float, default=2.0)
    parser.add_argument(
        "--output-report",
        type=Path,
        required=True,
        help="JSON report path (written atomically)",
    )


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="source_mode", required=True)

    mcap_parser = subparsers.add_parser(
        "mcap",
        help="read an MCAP directly, without ROS 2",
    )
    _add_model_arguments(mcap_parser)
    mcap_parser.add_argument(
        "--input",
        type=Path,
        nargs="+",
        required=True,
        help="one or more MCAP files or rosbag directories",
    )
    mcap_parser.add_argument("--topic", default=DEFAULT_TOPIC)
    mcap_parser.add_argument(
        "--playback-mode",
        choices=("realtime", "throughput"),
        default="realtime",
        help="realtime paces by bag timestamps; throughput measures maximum speed",
    )
    mcap_parser.add_argument("--playback-rate", type=float, default=1.0)
    mcap_parser.add_argument("--start-offset", type=float, default=0.0)
    mcap_parser.add_argument(
        "--max-frames",
        type=int,
        default=200,
        help="source frames per MCAP; 0 reads every frame",
    )
    mcap_parser.add_argument(
        "--snapshot-dir",
        type=Path,
        help="optional directory for one final segmentation JPEG per MCAP",
    )

    ros_parser = subparsers.add_parser(
        "ros2",
        help="subscribe to a live sensor_msgs/msg/Image topic",
    )
    _add_model_arguments(ros_parser)
    ros_parser.add_argument("--topic", default=DEFAULT_TOPIC)
    ros_parser.add_argument(
        "--reliability",
        choices=("best_effort", "reliable"),
        default="best_effort",
    )
    ros_parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="measurement duration in seconds; 0 waits until Ctrl-C",
    )
    ros_parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="stop after this many received frames; 0 disables the limit",
    )
    ros_parser.add_argument(
        "--overlay-topic",
        default="/best_so_far/benchmark/overlay",
        help="set to an empty string to disable overlay publishing",
    )
    ros_parser.add_argument(
        "--metrics-topic",
        default="/best_so_far/benchmark/metrics",
        help="set to an empty string to disable JSON metric publishing",
    )

    args = parser.parse_args(argv)
    if args.warmup_frames < 0:
        parser.error("--warmup-frames must be non-negative")
    if args.expected_input_hz <= 0.0:
        parser.error("--expected-input-hz must be positive")
    if args.report_interval <= 0.0:
        parser.error("--report-interval must be positive")
    if any(value <= 0 for value in args.evaluation_size):
        parser.error("--evaluation-size values must be positive")
    if args.source_mode == "mcap":
        if args.playback_rate <= 0.0:
            parser.error("--playback-rate must be positive")
        if args.start_offset < 0.0 or args.max_frames < 0:
            parser.error("--start-offset and --max-frames must be non-negative")
    else:
        if args.duration < 0.0 or args.max_frames < 0:
            parser.error("--duration and --max-frames must be non-negative")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    config = BestSoFarConfig(
        profile=args.profile,
        model_id=args.model_id,
        model_revision=args.model_revision,
        evaluation_height=args.evaluation_size[0],
        evaluation_width=args.evaluation_size[1],
        temporal_alpha=args.temporal_alpha,
        temporal_hysteresis_margin=args.temporal_hysteresis_margin,
        device=args.device,
    )
    print(
        f"MODEL_LOAD_START profile={args.profile} requested_device={args.device}",
        flush=True,
    )
    segmenter = BestSoFarSegmenter(config)
    print(
        f"MODEL_LOAD_COMPLETE device={segmenter.device} "
        f"elapsed_seconds={segmenter.model_load_seconds:.3f}",
        flush=True,
    )
    report = (
        run_mcap(args, segmenter)
        if args.source_mode == "mcap"
        else run_ros2(args, segmenter)
    )
    _atomic_write_json(args.output_report, report)
    print(
        f"REPORT_WRITTEN path={args.output_report.expanduser().resolve()}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
