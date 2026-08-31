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
"""Render full-length Road/Sidewalk overlays with the retained Mapillary profile."""

from __future__ import annotations

import argparse
import json
import os
import signal
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
from transformers import AutoImageProcessor, Mask2FormerForUniversalSegmentation

from evaluate_mapillary_label_aggregation import ROAD_LABELS, SIDEWALK_LABELS, resolve_ids
from evaluate_mapillary_temporal import (
    MODEL_ID,
    MODEL_REVISION,
    EVALUATION_SIZE,
    aggregated_selected_mask,
    upscale_mask,
)
from evaluate_sidewalk_road_temporal import binary_iou, remove_small_components
from segment_sidewalk_road import (
    VIDEO_EXTENSIONS,
    atomic_write_json,
    choose_device,
    discover_videos,
    render_overlay,
    utc_now,
)


def parse_args() -> argparse.Namespace:
    repository = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repository / "rosbag-results" / "sidewalk-road-results" / "full-three-video",
    )
    parser.add_argument("--model-id", default=MODEL_ID)
    parser.add_argument("--model-revision", default=MODEL_REVISION)
    parser.add_argument("--temporal-alpha", type=float, default=0.62)
    parser.add_argument("--temporal-hysteresis-margin", type=float, default=0.07)
    parser.add_argument(
        "--evaluation-size",
        type=int,
        nargs=2,
        metavar=("HEIGHT", "WIDTH"),
        default=EVALUATION_SIZE,
    )
    parser.add_argument(
        "--output-size",
        type=int,
        nargs=2,
        metavar=("WIDTH", "HEIGHT"),
        default=(960, 540),
    )
    parser.add_argument("--device", default="auto")
    parser.add_argument("--batch-size", type=int, default=1)
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
    if not 0.5 <= args.temporal_alpha <= 1.0:
        raise ValueError("--temporal-alpha must be in [0.5, 1.0]")
    if not 0.0 <= args.temporal_hysteresis_margin <= 1.0:
        raise ValueError("--temporal-hysteresis-margin must be in [0, 1]")
    if any(value <= 0 for value in (*args.evaluation_size, *args.output_size)):
        raise ValueError("evaluation and output dimensions must be positive")
    if args.batch_size <= 0 or args.progress_every <= 0 or args.max_frames < 0:
        raise ValueError(
            "batch size and progress interval must be positive and max frames non-negative"
        )


def semantic_scores_batch(
    processor: AutoImageProcessor,
    model: Mask2FormerForUniversalSegmentation,
    frames_bgr: list[np.ndarray],
    device: torch.device,
    target_size: tuple[int, int],
) -> list[torch.Tensor]:
    frames_rgb = [cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) for frame in frames_bgr]
    inputs = processor(images=frames_rgb, return_tensors="pt")
    inputs = {name: value.to(device) for name, value in inputs.items()}
    with torch.inference_mode():
        outputs = model(**inputs)
        processed = processor.post_process_semantic_segmentation(
            outputs,
            target_sizes=[target_size] * len(frames_bgr),
            return_segmentation_scores=True,
        )
    return [item["segmentation_scores"].detach().float().cpu() for item in processed]


def main() -> int:
    args = parse_args()
    validate_args(args)
    input_dir = args.input_dir.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    staging_dir = output_dir / ".staging"
    staging_dir.mkdir(parents=True, exist_ok=True)
    videos = discover_videos(input_dir)
    if not videos:
        raise RuntimeError(f"no videos found under {input_dir}")

    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    device = choose_device(args.device)
    print(f"MODEL_LOAD_START device={device}", flush=True)
    processor = AutoImageProcessor.from_pretrained(
        args.model_id, revision=args.model_revision
    )
    model = Mask2FormerForUniversalSegmentation.from_pretrained(
        args.model_id, revision=args.model_revision
    ).to(device)
    model.eval()
    road_ids = resolve_ids(model.config.id2label, ROAD_LABELS)
    sidewalk_ids = resolve_ids(model.config.id2label, SIDEWALK_LABELS)
    print("MODEL_LOAD_COMPLETE", flush=True)

    report_path = output_dir / "full-video-report.json"
    run_start = time.perf_counter()
    video_reports: list[dict[str, Any]] = []
    total_frames = 0

    for video_number, input_path in enumerate(videos, start=1):
        if stop_requested:
            break
        input_path = Path(input_path).resolve()
        output_path = output_dir / f"{input_path.stem}-segmented-full.mp4"
        staging_path = staging_dir / f"{input_path.stem}-segmented-full.tmp.mp4"
        staging_path.unlink(missing_ok=True)
        capture = cv2.VideoCapture(str(input_path))
        if not capture.isOpened():
            raise RuntimeError(f"could not open input video: {input_path}")
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        if not np.isfinite(fps) or fps <= 0.0:
            fps = 20.0
        expected_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
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
        previous_scores: torch.Tensor | None = None
        previous_selected: np.ndarray | None = None
        change_ratios: list[float] = []
        road_ious: list[float] = []
        sidewalk_ious: list[float] = []
        hold_fractions: list[float] = []
        road_areas: list[float] = []
        sidewalk_areas: list[float] = []
        try:
            while not stop_requested:
                frame_batch: list[np.ndarray] = []
                while len(frame_batch) < args.batch_size:
                    if args.max_frames and processed + len(frame_batch) >= args.max_frames:
                        break
                    ok, frame_bgr = capture.read()
                    if not ok:
                        break
                    frame_batch.append(frame_bgr)
                if not frame_batch:
                    break
                score_batch = semantic_scores_batch(
                    processor,
                    model,
                    frame_batch,
                    device,
                    tuple(args.evaluation_size),
                )
                for frame_bgr, scores in zip(frame_batch, score_batch, strict=True):
                    if stop_requested:
                        break
                    if previous_scores is None:
                        smooth_scores = scores
                    else:
                        smooth_scores = (
                            args.temporal_alpha * scores
                            + (1.0 - args.temporal_alpha) * previous_scores
                        )
                    previous_scores = smooth_scores
                    smooth_map = smooth_scores.argmax(dim=0).numpy()
                    minimum_area = max(48, int(smooth_map.size * 0.00035))
                    road_confidence = (
                        smooth_scores[road_ids].max(dim=0).values.numpy()
                    )
                    sidewalk_confidence = (
                        smooth_scores[sidewalk_ids].max(dim=0).values.numpy()
                    )
                    selected = aggregated_selected_mask(
                        smooth_map,
                        road_ids=road_ids,
                        sidewalk_ids=sidewalk_ids,
                        road_confidence=road_confidence,
                        sidewalk_confidence=sidewalk_confidence,
                        minimum_area=minimum_area,
                    )
                    if previous_selected is not None:
                        top_scores = torch.topk(smooth_scores, k=2, dim=0).values.numpy()
                        score_margin = top_scores[0] - top_scores[1]
                        hold_mask = (
                            (selected != previous_selected)
                            & (score_margin < args.temporal_hysteresis_margin)
                        )
                        selected = selected.copy()
                        selected[hold_mask] = previous_selected[hold_mask]
                        retained_road = remove_small_components(
                            selected == 1, minimum_area
                        )
                        retained_sidewalk = remove_small_components(
                            selected == 2, minimum_area
                        )
                        selected.fill(0)
                        selected[retained_road] = 1
                        selected[retained_sidewalk] = 2
                        change_ratios.append(
                            float(np.mean(selected != previous_selected))
                        )
                        road_ious.append(
                            binary_iou(selected == 1, previous_selected == 1)
                        )
                        sidewalk_ious.append(
                            binary_iou(selected == 2, previous_selected == 2)
                        )
                        hold_fractions.append(float(np.mean(hold_mask)))
                    else:
                        hold_fractions.append(0.0)
                    previous_selected = selected
                    road_areas.append(float(np.mean(selected == 1)))
                    sidewalk_areas.append(float(np.mean(selected == 2)))

                    overlay = render_overlay(
                        frame_bgr,
                        upscale_mask(selected, frame_bgr),
                        frame_index=processed,
                        fps=fps,
                    )
                    overlay = cv2.resize(
                        overlay, tuple(args.output_size), interpolation=cv2.INTER_AREA
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
            "mean_selected_label_change_ratio": mean(change_ratios),
            "mean_road_adjacent_iou": mean(road_ious),
            "mean_sidewalk_adjacent_iou": mean(sidewalk_ious),
            "mean_hysteresis_hold_fraction": mean(hold_fractions),
            "mean_road_area_ratio": mean(road_areas),
            "mean_sidewalk_area_ratio": mean(sidewalk_areas),
        }
        video_reports.append(video_report)
        atomic_write_json(
            report_path,
            {
                "schema_version": 1,
                "status": "running" if video_number < len(videos) else "complete",
                "updated_at": utc_now(),
                "model": {"id": args.model_id, "revision": args.model_revision},
                "settings": {
                    "profile": "surface-aggregate",
                    "temporal_alpha": args.temporal_alpha,
                    "temporal_hysteresis_margin": args.temporal_hysteresis_margin,
                    "evaluation_size": list(args.evaluation_size),
                    "output_size": list(args.output_size),
                    "road_labels": list(ROAD_LABELS),
                    "sidewalk_labels": list(SIDEWALK_LABELS),
                },
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
        "schema_version": 1,
        "status": status,
        "updated_at": utc_now(),
        "model": {"id": args.model_id, "revision": args.model_revision},
        "settings": {
            "profile": "surface-aggregate",
            "temporal_alpha": args.temporal_alpha,
            "temporal_hysteresis_margin": args.temporal_hysteresis_margin,
            "evaluation_size": list(args.evaluation_size),
            "output_size": list(args.output_size),
            "road_labels": list(ROAD_LABELS),
            "sidewalk_labels": list(SIDEWALK_LABELS),
        },
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
