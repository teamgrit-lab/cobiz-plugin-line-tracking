#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = [
#   "numpy==2.5.2",
#   "opencv-python-headless==5.0.0.93",
#   "pillow==12.3.0",
#   "scipy==1.18.1",
#   "torch==2.13.0",
#   "torchvision==0.28.0",
#   "transformers==5.16.1",
# ]
# ///
"""Offline full-video Road/Sidewalk segmentation and hypothetical path debug.

The video has no LiDAR, pose, or calibrated camera extrinsics. This tool never
publishes ROS topics or A2 commands; its steering annotation is illustrative.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, replace
import math
import os
from pathlib import Path
import signal
import time

import cv2
import numpy as np

from best_so_far_runtime import (
    BestSoFarConfig,
    BestSoFarSegmenter,
    SWIN_L_ASPECT_PROFILE,
)
from local_path import (
    LocalPathConfig,
    LocalPathEstimate,
    LocalPathSmoother,
    SmoothedPath,
    extract_sidewalk_centerline,
    ground_to_pixel,
    normalized_polygon_pixels,
    pixel_to_ground_homography,
)
from segment_sidewalk_road import atomic_write_json, utc_now
from swin_l_drive_control import DriveConfig


def choose_surface(
    selected_mask: np.ndarray,
    path_config: LocalPathConfig,
    drive_config: DriveConfig,
) -> tuple[str, LocalPathEstimate] | None:
    """Prefer a plausible sidewalk; use a plausible road when none is available."""

    for label, name in ((2, "SIDEWALK"), (1, "ROAD")):
        estimate = extract_sidewalk_centerline(selected_mask == label, path_config)
        if estimate is None or estimate.confidence < drive_config.min_confidence:
            continue
        raw_x = estimate.raw_points_xy[:, 0]
        if (
            float(raw_x.min()) > path_config.near_distance_m + 0.5
            or float(raw_x.max()) < drive_config.lookahead_m
        ):
            continue
        lateral = float(
            np.interp(
                drive_config.lookahead_m,
                estimate.points_xy[:, 0],
                estimate.points_xy[:, 1],
            )
        )
        if (
            not math.isfinite(lateral)
            or abs(lateral) > drive_config.max_lateral_target_m
        ):
            continue
        return name, estimate
    return None


def preview_yaw(path: SmoothedPath | None, config: DriveConfig) -> float | None:
    """Return logical steering only; this is not a motion authorization."""

    if path is None or path.confidence < config.min_confidence:
        return None
    if path.age_sec > config.max_path_age_sec:
        return None
    lateral = float(
        np.interp(config.lookahead_m, path.points_xy[:, 0], path.points_xy[:, 1])
    )
    if not math.isfinite(lateral) or abs(lateral) > config.max_lateral_target_m:
        return None
    return float(
        np.clip(
            config.heading_gain * math.atan2(lateral, config.lookahead_m),
            -config.max_yaw_rps,
            config.max_yaw_rps,
        )
    )


def video_path_config(roi_top: float) -> LocalPathConfig:
    """Restrict the video-only ROI to pixels below the visible horizon."""

    if not 0.0 < roi_top < 1.0:
        raise ValueError("roi_top must be in (0, 1)")
    base = LocalPathConfig()
    roi = list(base.roi_polygon)
    roi[5] = roi_top
    roi[7] = roi_top
    return replace(base, roi_polygon=tuple(roi))


def draw_debug(
    segmentation: np.ndarray,
    path: SmoothedPath | None,
    source: str | None,
    yaw: float | None,
    path_config: LocalPathConfig,
    frame_index: int,
    fps: float,
) -> np.ndarray:
    """Draw one offline route preview over the full-resolution mask overlay."""

    image = segmentation.copy()
    roi = normalized_polygon_pixels(path_config.roi_polygon, image.shape[:2])
    cv2.polylines(image, [roi.reshape(-1, 1, 2)], True, (255, 180, 0), 2)
    if path is not None:
        homography = pixel_to_ground_homography(image.shape[:2], path_config)
        points = ground_to_pixel(path.points_xy, homography)
        valid = np.all(np.isfinite(points), axis=1)
        if np.count_nonzero(valid) >= 2:
            pixels = np.rint(points[valid]).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(image, [pixels], False, (255, 255, 255), 4, cv2.LINE_AA)
            lookahead_y = float(
                np.interp(4.0, path.points_xy[:, 0], path.points_xy[:, 1])
            )
            target = ground_to_pixel(
                np.asarray([[4.0, lookahead_y]], dtype=np.float32), homography
            )[0]
            if np.all(np.isfinite(target)):
                cv2.arrowedLine(
                    image,
                    (image.shape[1] // 2, image.shape[0] - 12),
                    tuple(np.rint(target).astype(int)),
                    (0, 200, 255),
                    3,
                    cv2.LINE_AA,
                    tipLength=0.12,
                )

    direction = (
        "STOP / NO ROUTE"
        if yaw is None
        else "LEFT"
        if yaw > 0.035
        else "RIGHT"
        if yaw < -0.035
        else "STRAIGHT"
    )
    lines = (
        "OFFLINE SURFACE FOLLOW DEBUG - NO LIDAR/TF",
        f"surface={source or 'NONE'}  direction={direction}",
        f"preview_yaw={yaw:+.3f}rad/s  confidence={path.confidence:.2f}"
        if yaw is not None and path is not None
        else "preview_yaw=--  ROBOT COMMAND=DISABLED",
        f"video_time={frame_index / fps:.1f}s  frame={frame_index}",
    )
    panel_top = image.shape[0] - 118
    cv2.rectangle(image, (8, panel_top), (660, image.shape[0] - 8), (0, 0, 0), -1)
    for row, line in enumerate(lines):
        cv2.putText(
            image,
            line,
            (17, panel_top + 25 + 26 * row),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 180, 255) if row == 0 else (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--inference-hz", type=float, default=4.0)
    parser.add_argument("--output-size", type=int, nargs=2, default=(960, 540))
    parser.add_argument("--roi-top", type=float, default=0.55)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--progress-every", type=int, default=100)
    return parser.parse_args()


def _writer(path: Path, fps: float, size: tuple[int, int]) -> cv2.VideoWriter:
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
    if not writer.isOpened():
        raise RuntimeError(f"could not open MP4 writer: {path}")
    return writer


def main() -> int:
    args = parse_args()
    if args.inference_hz <= 0 or args.max_frames < 0 or args.progress_every <= 0:
        raise ValueError("invalid inference/frame/progress limits")
    size = tuple(args.output_size)
    if len(size) != 2 or min(size) <= 0:
        raise ValueError("output size must be positive WIDTH HEIGHT")
    source = args.input.expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    segmentation_path = output_dir / "segmentation-full.mp4"
    follow_path = output_dir / "surface-follow-debug-full.mp4"
    report_path = output_dir / "surface-follow-report.json"
    segmentation_tmp = output_dir / ".segmentation-full.tmp.mp4"
    follow_tmp = output_dir / ".surface-follow-debug-full.tmp.mp4"
    for path in (
        segmentation_path,
        follow_path,
        report_path,
        segmentation_tmp,
        follow_tmp,
    ):
        if path.exists():
            raise FileExistsError(f"refusing to overwrite {path}")

    path_config = video_path_config(args.roi_top)
    drive_config = DriveConfig()
    path_config.validate()
    drive_config.validate()
    segmenter = BestSoFarSegmenter(
        BestSoFarConfig(profile=SWIN_L_ASPECT_PROFILE, device=args.device)
    )
    smoother = LocalPathSmoother(path_config)
    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise RuntimeError(f"could not open video: {source}")
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    if not math.isfinite(fps) or fps <= 0:
        capture.release()
        raise ValueError("source video has no valid frame rate")
    source_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    expected_frames = source_frames
    if args.max_frames:
        expected_frames = min(expected_frames, args.max_frames)
    segmentation_writer = _writer(segmentation_tmp, fps, size)
    follow_writer = _writer(follow_tmp, fps, size)
    stop_requested = False

    def stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    previous_sigint = signal.signal(signal.SIGINT, stop)
    previous_sigterm = signal.signal(signal.SIGTERM, stop)
    frame_count = 0
    read_eof = False
    inference_count = 0
    next_inference_sec = 0.0
    selected_mask = np.zeros((360, 640), dtype=np.uint8)
    source_name: str | None = None
    surface_counts: Counter[str] = Counter()
    movement_counts: Counter[str] = Counter()
    road_area: list[float] = []
    sidewalk_area: list[float] = []
    transitions: list[dict[str, float | str | None]] = []
    started = time.perf_counter()
    print(
        f"VIDEO_START frames={expected_frames} fps={fps:.4f} "
        f"profile={segmenter.profile.name} device={segmenter.device}",
        flush=True,
    )
    try:
        while not stop_requested and frame_count < expected_frames:
            ok, frame = capture.read()
            if not ok:
                read_eof = True
                break
            timestamp = frame_count / fps
            if timestamp + 1e-9 >= next_inference_sec:
                result = segmenter.segment(frame)
                selected_mask = result.selected_mask
                road_area.append(result.road_area_ratio)
                sidewalk_area.append(result.sidewalk_area_ratio)
                chosen = choose_surface(selected_mask, path_config, drive_config)
                if chosen is not None:
                    new_source, estimate = chosen
                    if new_source != source_name:
                        transitions.append(
                            {"time_sec": round(timestamp, 3), "surface": new_source}
                        )
                        smoother.reset()
                    source_name = new_source
                    smoother.update(estimate, timestamp)
                else:
                    smoother.update(None, timestamp)
                inference_count += 1
                next_inference_sec += 1.0 / args.inference_hz
                while next_inference_sec <= timestamp:
                    next_inference_sec += 1.0 / args.inference_hz
            path = smoother.current(timestamp)
            if path is None:
                source_name = None
            yaw = preview_yaw(path, drive_config)
            surface_counts[source_name or "NONE"] += 1
            movement_counts[
                "STOP"
                if yaw is None
                else "LEFT"
                if yaw > 0.035
                else "RIGHT"
                if yaw < -0.035
                else "STRAIGHT"
            ] += 1
            resized = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
            segmentation = segmenter.render_overlay(
                resized, selected_mask, frame_index=frame_count, fps=fps
            )
            segmentation_writer.write(segmentation)
            follow_writer.write(
                draw_debug(
                    segmentation, path, source_name, yaw, path_config, frame_count, fps
                )
            )
            frame_count += 1
            if frame_count % args.progress_every == 0:
                elapsed = time.perf_counter() - started
                rate = frame_count / elapsed if elapsed else 0.0
                remaining = (expected_frames - frame_count) / rate if rate else 0.0
                print(
                    f"VIDEO_PROGRESS frames={frame_count}/{expected_frames} "
                    f"updates={inference_count} rate={rate:.2f}fps "
                    f"eta_seconds={remaining:.0f}",
                    flush=True,
                )
    finally:
        capture.release()
        segmentation_writer.release()
        follow_writer.release()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)

    metadata_frame_shortfall = max(0, expected_frames - frame_count)
    complete = (
        not stop_requested
        and frame_count > 0
        and (
            frame_count == expected_frames
            or (read_eof and metadata_frame_shortfall == 1)
        )
    )
    status = (
        "complete_with_metadata_shortfall"
        if complete and metadata_frame_shortfall
        else "complete"
        if complete
        else "interrupted_or_truncated"
    )
    if complete:
        os.replace(segmentation_tmp, segmentation_path)
        os.replace(follow_tmp, follow_path)
    report = {
        "status": status,
        "updated_at": utc_now(),
        "source": str(source),
        "source_frames": source_frames,
        "source_fps": fps,
        "frames_written": frame_count,
        "metadata_frame_shortfall": metadata_frame_shortfall,
        "inference_updates": inference_count,
        "inference_policy": "periodic_model_inference_with_mask_hold",
        "requested_inference_hz": args.inference_hz,
        "output_size": list(size),
        "segmentation_video": str(segmentation_path) if complete else None,
        "surface_follow_video": str(follow_path) if complete else None,
        "model": segmenter.metadata(),
        "path_config": asdict(path_config),
        "drive_preview_config": asdict(drive_config),
        "surface_frame_counts": dict(surface_counts),
        "steering_preview_frame_counts": dict(movement_counts),
        "surface_transitions": transitions,
        "mean_road_area_ratio": float(np.mean(road_area)) if road_area else None,
        "mean_sidewalk_area_ratio": float(np.mean(sidewalk_area))
        if sidewalk_area
        else None,
        "elapsed_seconds": time.perf_counter() - started,
        "limitations": [
            "No LiDAR, odometry, or camera extrinsic calibration in the MP4.",
            "Steering is a hypothetical image-based preview; no ROS Joy is published.",
            "Source variable timestamps are approximated using average frame rate.",
        ],
    }
    atomic_write_json(report_path, report)
    print(
        f"VIDEO_COMPLETE status={report['status']} frames={frame_count} "
        f"updates={inference_count} report={report_path}",
        flush=True,
    )
    return 0 if complete else 1


if __name__ == "__main__":
    raise SystemExit(main())
