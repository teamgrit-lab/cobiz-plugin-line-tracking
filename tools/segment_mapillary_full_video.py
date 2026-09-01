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
"""Render full-length Road/Sidewalk overlays with a named Mapillary profile."""

from __future__ import annotations

import argparse
import os
import signal
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from best_so_far_runtime import (
    DEFAULT_EVALUATION_SIZE,
    DEFAULT_PROFILE,
    PROFILE_NAMES,
    BestSoFarConfig,
    BestSoFarSegmenter,
)
from evaluate_sidewalk_road_temporal import binary_iou
from segment_sidewalk_road import atomic_write_json, discover_videos, utc_now


def parse_args() -> argparse.Namespace:
    repository = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--input", type=Path, nargs="+")
    source.add_argument("--input-dir", type=Path)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            repository / "rosbag-results" / "sidewalk-road-results" / "full-three-video"
        ),
    )
    parser.add_argument("--profile", choices=PROFILE_NAMES, default=DEFAULT_PROFILE)
    parser.add_argument("--model-id", default=None)
    parser.add_argument("--model-revision", default=None)
    parser.add_argument("--temporal-alpha", type=float, default=None)
    parser.add_argument("--temporal-hysteresis-margin", type=float, default=None)
    parser.add_argument(
        "--evaluation-size",
        type=int,
        nargs=2,
        metavar=("HEIGHT", "WIDTH"),
        default=DEFAULT_EVALUATION_SIZE,
    )
    parser.add_argument(
        "--output-size",
        type=int,
        nargs=2,
        metavar=("WIDTH", "HEIGHT"),
        default=(960, 540),
    )
    parser.add_argument("--device", default="auto")
    parser.add_argument(
        "--batch-size",
        type=int,
        default=1,
        help="compatibility option; stateful temporal inference remains sequential",
    )
    parser.add_argument("--progress-every", type=int, default=100)
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Testing aid; 0 processes every readable frame.",
    )
    return parser.parse_args()


def mean(values: list[float]) -> float:
    return float(np.mean(values)) if values else 0.0


def validate_args(args: argparse.Namespace) -> None:
    if args.temporal_alpha is not None and not 0.5 <= args.temporal_alpha <= 1.0:
        raise ValueError("--temporal-alpha must be in [0.5, 1.0]")
    if (
        args.temporal_hysteresis_margin is not None
        and not 0.0 <= args.temporal_hysteresis_margin <= 1.0
    ):
        raise ValueError("--temporal-hysteresis-margin must be in [0, 1]")
    if any(value <= 0 for value in (*args.evaluation_size, *args.output_size)):
        raise ValueError("evaluation and output dimensions must be positive")
    if args.batch_size <= 0 or args.progress_every <= 0 or args.max_frames < 0:
        raise ValueError(
            "batch size and progress interval must be positive and max frames non-negative"
        )


def resolve_videos(args: argparse.Namespace) -> list[Path]:
    if args.input is not None:
        videos = [path.expanduser().resolve() for path in args.input]
        missing = [path for path in videos if not path.is_file()]
        if missing:
            raise FileNotFoundError(f"input video does not exist: {missing[0]}")
        return videos
    return discover_videos(args.input_dir.expanduser().resolve())


def main() -> int:
    args = parse_args()
    validate_args(args)
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    staging_dir = output_dir / ".staging"
    staging_dir.mkdir(parents=True, exist_ok=True)
    videos = resolve_videos(args)
    if not videos:
        raise RuntimeError("no videos found")

    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

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
        f"MODEL_LOAD_COMPLETE profile={segmenter.profile.name} "
        f"device={segmenter.device} precision="
        f"{'fp16' if segmenter.use_fp16 else 'fp32'} "
        f"elapsed_seconds={segmenter.model_load_seconds:.3f}",
        flush=True,
    )

    report_path = output_dir / "full-video-report.json"
    run_start = time.perf_counter()
    video_reports: list[dict[str, Any]] = []
    total_frames = 0

    for video_number, input_path in enumerate(videos, start=1):
        if stop_requested:
            break
        segmenter.reset()
        output_path = (
            output_dir
            / f"{input_path.stem}-{segmenter.profile.name}-segmented-full.mp4"
        )
        staging_path = staging_dir / f"{output_path.stem}.tmp.mp4"
        staging_path.unlink(missing_ok=True)
        capture = cv2.VideoCapture(str(input_path))
        if not capture.isOpened():
            raise RuntimeError(f"could not open input video: {input_path}")
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        if not np.isfinite(fps) or fps <= 0.0:
            fps = 20.0
        expected_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        if args.max_frames:
            expected_frames = min(expected_frames, args.max_frames)
        writer = cv2.VideoWriter(
            str(staging_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            tuple(args.output_size),
        )
        if not writer.isOpened():
            capture.release()
            raise RuntimeError(f"could not create output video: {staging_path}")

        print(
            f"VIDEO_START index={video_number}/{len(videos)} name={input_path.name} "
            f"expected_frames={expected_frames} fps={fps:.6f}",
            flush=True,
        )
        video_start = time.perf_counter()
        processed = 0
        previous_selected: np.ndarray | None = None
        change_ratios: list[float] = []
        road_ious: list[float] = []
        sidewalk_ious: list[float] = []
        hold_fractions: list[float] = []
        road_areas: list[float] = []
        sidewalk_areas: list[float] = []
        inference_seconds: list[float] = []
        postprocess_seconds: list[float] = []
        try:
            while not stop_requested:
                if args.max_frames and processed >= args.max_frames:
                    break
                ok, frame_bgr = capture.read()
                if not ok:
                    break
                result = segmenter.segment(frame_bgr)
                selected = result.selected_mask
                if previous_selected is not None:
                    change_ratios.append(float(np.mean(selected != previous_selected)))
                    road_ious.append(binary_iou(selected == 1, previous_selected == 1))
                    sidewalk_ious.append(
                        binary_iou(selected == 2, previous_selected == 2)
                    )
                previous_selected = selected.copy()
                hold_fractions.append(result.hysteresis_hold_ratio)
                road_areas.append(result.road_area_ratio)
                sidewalk_areas.append(result.sidewalk_area_ratio)
                inference_seconds.append(result.inference_seconds)
                postprocess_seconds.append(result.postprocess_seconds)

                overlay = segmenter.render_overlay(
                    frame_bgr,
                    selected,
                    frame_index=processed,
                    fps=fps,
                )
                overlay = cv2.resize(
                    overlay,
                    tuple(args.output_size),
                    interpolation=cv2.INTER_AREA,
                )
                writer.write(overlay)
                processed += 1
                total_frames += 1
                if processed % args.progress_every == 0:
                    elapsed = time.perf_counter() - video_start
                    rate = processed / elapsed if elapsed else 0.0
                    remaining = (
                        max(expected_frames - processed, 0) / rate if rate else 0.0
                    )
                    print(
                        f"VIDEO_PROGRESS index={video_number}/{len(videos)} "
                        f"frames={processed}/{expected_frames} rate={rate:.3f}fps "
                        f"eta_seconds={remaining:.1f}",
                        flush=True,
                    )
        finally:
            capture.release()
            writer.release()

        if stop_requested:
            staging_path.unlink(missing_ok=True)
            break
        if processed == 0:
            staging_path.unlink(missing_ok=True)
            raise RuntimeError(f"no frames decoded from {input_path}")
        os.replace(staging_path, output_path)
        elapsed = time.perf_counter() - video_start
        video_report = {
            "input": str(input_path),
            "output": str(output_path),
            "expected_frames": expected_frames,
            "processed_frames": processed,
            "input_fps": fps,
            "elapsed_seconds": elapsed,
            "processing_fps": processed / elapsed,
            "mean_inference_ms": mean(inference_seconds) * 1000.0,
            "mean_postprocess_ms": mean(postprocess_seconds) * 1000.0,
            "mean_selected_label_change_ratio": mean(change_ratios),
            "mean_road_adjacent_iou": mean(road_ious),
            "mean_sidewalk_adjacent_iou": mean(sidewalk_ious),
            "mean_hysteresis_hold_fraction": mean(hold_fractions),
            "mean_road_area_ratio": mean(road_areas),
            "mean_sidewalk_area_ratio": mean(sidewalk_areas),
        }
        video_reports.append(video_report)
        runtime = segmenter.metadata()
        atomic_write_json(
            report_path,
            {
                "schema_version": 2,
                "status": "running" if video_number < len(videos) else "complete",
                "updated_at": utc_now(),
                "runtime": runtime,
                "output_size": list(args.output_size),
                "videos": video_reports,
                "processed_frames": total_frames,
                "elapsed_seconds": time.perf_counter() - run_start,
            },
        )
        print(
            f"VIDEO_COMPLETE index={video_number}/{len(videos)} frames={processed} "
            f"elapsed_seconds={elapsed:.1f} output={output_path}",
            flush=True,
        )

    status = "interrupted" if stop_requested else "complete"
    report = {
        "schema_version": 2,
        "status": status,
        "updated_at": utc_now(),
        "runtime": segmenter.metadata(),
        "output_size": list(args.output_size),
        "videos": video_reports,
        "processed_frames": total_frames,
        "elapsed_seconds": time.perf_counter() - run_start,
    }
    atomic_write_json(report_path, report)
    print(
        f"RUN_COMPLETE status={status} frames={total_frames} "
        f"elapsed_seconds={report['elapsed_seconds']:.1f}",
        flush=True,
    )
    return 130 if stop_requested else 0


if __name__ == "__main__":
    raise SystemExit(main())
