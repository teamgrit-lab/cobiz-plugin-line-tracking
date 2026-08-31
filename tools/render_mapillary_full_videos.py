#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = [
#   "numpy==2.5.2",
#   "opencv-python-headless==5.0.0.93",
#   "scipy==1.18.1",
#   "torch==2.13.0",
#   "torchvision==0.28.0",
#   "transformers==5.16.1",
# ]
# ///
"""Render full-length Road/Sidewalk overlays with the retained Mapillary profile."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import signal
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
from transformers import AutoImageProcessor, Mask2FormerForUniversalSegmentation

from evaluate_mapillary_label_aggregation import ROAD_LABELS, SIDEWALK_LABELS, resolve_ids
from evaluate_mapillary_temporal import aggregated_selected_mask, upscale_mask
from evaluate_sidewalk_road_temporal import binary_iou, remove_small_components
from segment_sidewalk_road import (
    RunLock,
    atomic_write_json,
    choose_device,
    discover_videos,
    inspect_video,
    render_overlay,
    utc_now,
)


MODEL_ID = "facebook/mask2former-swin-large-mapillary-vistas-semantic"
MODEL_REVISION = "4772b6bf101d91f2534c106dc524d906aeb3c68a"
EVALUATION_SIZE = (360, 640)


@dataclass
class PairTotals:
    count: int = 0
    selected_change: float = 0.0
    road_iou: float = 0.0
    sidewalk_iou: float = 0.0
    max_road_area_jump: float = 0.0
    max_sidewalk_area_jump: float = 0.0

    def update(self, previous: np.ndarray, current: np.ndarray) -> None:
        previous_road = previous == 1
        current_road = current == 1
        previous_sidewalk = previous == 2
        current_sidewalk = current == 2
        self.count += 1
        self.selected_change += float(np.mean(previous != current))
        self.road_iou += binary_iou(previous_road, current_road)
        self.sidewalk_iou += binary_iou(previous_sidewalk, current_sidewalk)
        road_jump = abs(float(previous_road.mean()) - float(current_road.mean()))
        sidewalk_jump = abs(
            float(previous_sidewalk.mean()) - float(current_sidewalk.mean())
        )
        self.max_road_area_jump = max(self.max_road_area_jump, road_jump)
        self.max_sidewalk_area_jump = max(
            self.max_sidewalk_area_jump, sidewalk_jump
        )

    def summary(self) -> dict[str, float | int]:
        divisor = max(self.count, 1)
        return {
            "pair_count": self.count,
            "mean_selected_label_change_ratio": self.selected_change / divisor,
            "mean_road_iou": self.road_iou / divisor,
            "mean_sidewalk_iou": self.sidewalk_iou / divisor,
            "max_road_area_jump": self.max_road_area_jump,
            "max_sidewalk_area_jump": self.max_sidewalk_area_jump,
        }


def parse_args() -> argparse.Namespace:
    repository = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            repository
            / "rosbag-results"
            / "sidewalk-road-results"
            / "full-video-tests"
        ),
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
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--device", default="auto")
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Smoke-test limit per video; zero processes every frame.",
    )
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def batched_scores(
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
    return [
        item["segmentation_scores"].detach().float().cpu() for item in processed
    ]


def open_writer(path: Path, fps: float, width: int, height: int) -> cv2.VideoWriter:
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        raise RuntimeError(f"could not open output writer: {path}")
    return writer


def render_video(
    video_path: Path,
    output_dir: Path,
    *,
    processor: AutoImageProcessor,
    model: Mask2FormerForUniversalSegmentation,
    device: torch.device,
    road_ids: list[int],
    sidewalk_ids: list[int],
    temporal_alpha: float,
    hysteresis_margin: float,
    evaluation_size: tuple[int, int],
    batch_size: int,
    max_frames: int,
    stop_requested: list[bool],
) -> dict[str, Any]:
    info = inspect_video(video_path)
    final_path = output_dir / f"{video_path.stem}-segmentation-full.mp4"
    partial_path = output_dir / f".{video_path.stem}-segmentation-full.partial.mp4"
    if final_path.exists():
        raise FileExistsError(f"refusing to overwrite existing output: {final_path}")
    partial_path.unlink(missing_ok=True)

    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"could not open input: {video_path}")
    writer = open_writer(partial_path, info.fps, info.width, info.height)

    previous_scores: torch.Tensor | None = None
    previous_raw: np.ndarray | None = None
    previous_filtered: np.ndarray | None = None
    raw_pairs = PairTotals()
    filtered_pairs = PairTotals()
    preservation_total = 0.0
    preservation_count = 0
    hysteresis_total = 0.0
    frame_index = 0
    cloned_tail_frame_count = 0
    last_output_frame: np.ndarray | None = None
    expected_frames = (
        min(info.frame_count, max_frames) if max_frames else info.frame_count
    )
    started = time.perf_counter()

    try:
        while not stop_requested[0]:
            frames: list[np.ndarray] = []
            while len(frames) < batch_size:
                if max_frames and frame_index + len(frames) >= max_frames:
                    break
                ok, frame = capture.read()
                if not ok:
                    break
                frames.append(frame)
            if not frames:
                break

            score_batch = batched_scores(
                processor,
                model,
                frames,
                device,
                evaluation_size,
            )
            for frame, scores in zip(frames, score_batch, strict=True):
                raw_map = scores.argmax(dim=0).numpy()
                raw_road_confidence = scores[road_ids].max(dim=0).values.numpy()
                raw_sidewalk_confidence = (
                    scores[sidewalk_ids].max(dim=0).values.numpy()
                )
                raw_selected = aggregated_selected_mask(
                    raw_map,
                    road_ids=road_ids,
                    sidewalk_ids=sidewalk_ids,
                    road_confidence=raw_road_confidence,
                    sidewalk_confidence=raw_sidewalk_confidence,
                )

                smooth_scores = (
                    scores
                    if previous_scores is None
                    else temporal_alpha * scores
                    + (1.0 - temporal_alpha) * previous_scores
                )
                previous_scores = smooth_scores
                smooth_map = smooth_scores.argmax(dim=0).numpy()
                minimum_area = max(48, int(smooth_map.size * 0.00035))
                filtered_selected = aggregated_selected_mask(
                    smooth_map,
                    road_ids=road_ids,
                    sidewalk_ids=sidewalk_ids,
                    road_confidence=(
                        smooth_scores[road_ids].max(dim=0).values.numpy()
                    ),
                    sidewalk_confidence=(
                        smooth_scores[sidewalk_ids].max(dim=0).values.numpy()
                    ),
                    minimum_area=minimum_area,
                )

                if previous_filtered is not None and hysteresis_margin > 0.0:
                    top_scores = torch.topk(smooth_scores, k=2, dim=0).values.numpy()
                    score_margin = top_scores[0] - top_scores[1]
                    hold_mask = (
                        (filtered_selected != previous_filtered)
                        & (score_margin < hysteresis_margin)
                    )
                    filtered_selected = filtered_selected.copy()
                    filtered_selected[hold_mask] = previous_filtered[hold_mask]
                    retained_road = remove_small_components(
                        filtered_selected == 1, minimum_area
                    )
                    retained_sidewalk = remove_small_components(
                        filtered_selected == 2, minimum_area
                    )
                    filtered_selected.fill(0)
                    filtered_selected[retained_road] = 1
                    filtered_selected[retained_sidewalk] = 2
                    hysteresis_total += float(np.mean(hold_mask))

                if previous_raw is not None:
                    raw_pairs.update(previous_raw, raw_selected)
                if previous_filtered is not None:
                    filtered_pairs.update(previous_filtered, filtered_selected)
                preservation_total += (
                    binary_iou(raw_selected == 1, filtered_selected == 1)
                    + binary_iou(raw_selected == 2, filtered_selected == 2)
                ) / 2.0
                preservation_count += 1
                previous_raw = raw_selected
                previous_filtered = filtered_selected

                output_frame = render_overlay(
                    frame,
                    upscale_mask(filtered_selected, frame),
                    frame_index=frame_index,
                    fps=info.fps,
                )
                writer.write(output_frame)
                last_output_frame = output_frame
                frame_index += 1
                if frame_index % 100 == 0:
                    elapsed = time.perf_counter() - started
                    speed = frame_index / max(elapsed, 1e-9)
                    remaining_frames = max(info.frame_count - frame_index, 0)
                    eta = remaining_frames / max(speed, 1e-9)
                    print(
                        f"PROGRESS video={video_path.name} frames={frame_index}/"
                        f"{info.frame_count} speed_fps={speed:.3f} eta_seconds={eta:.0f}",
                        flush=True,
                    )
        shortfall = expected_frames - frame_index
        if (
            not stop_requested[0]
            and 0 < shortfall <= 2
            and last_output_frame is not None
        ):
            for _ in range(shortfall):
                writer.write(last_output_frame)
                frame_index += 1
                cloned_tail_frame_count += 1
    finally:
        capture.release()
        writer.release()

    if stop_requested[0]:
        raise InterruptedError(f"stop requested while processing {video_path}")
    if frame_index != expected_frames:
        raise RuntimeError(
            f"decoded frame count mismatch for {video_path}: "
            f"expected {expected_frames}, rendered {frame_index}"
        )

    os.replace(partial_path, final_path)
    elapsed = time.perf_counter() - started
    return {
        "input": str(video_path.resolve()),
        "output": str(final_path.resolve()),
        "input_width": info.width,
        "input_height": info.height,
        "input_fps": info.fps,
        "input_duration_seconds": info.duration_seconds,
        "input_frame_count": info.frame_count,
        "rendered_frame_count": frame_index,
        "cloned_tail_frame_count": cloned_tail_frame_count,
        "elapsed_seconds": elapsed,
        "processing_frames_per_second": frame_index / max(elapsed, 1e-9),
        "raw": raw_pairs.summary(),
        "temporal_spatial": filtered_pairs.summary(),
        "mean_raw_preservation_iou": preservation_total
        / max(preservation_count, 1),
        "mean_hysteresis_hold_fraction": hysteresis_total
        / max(frame_index, 1),
        "output_sha256": sha256(final_path),
    }


def main() -> int:
    args = parse_args()
    if not 0.5 <= args.temporal_alpha <= 1.0:
        raise ValueError("--temporal-alpha must be in [0.5, 1.0]")
    if not 0.0 <= args.temporal_hysteresis_margin <= 1.0:
        raise ValueError("--temporal-hysteresis-margin must be in [0, 1]")
    if args.batch_size < 1 or args.batch_size > 16:
        raise ValueError("--batch-size must be in [1, 16]")
    if args.max_frames < 0:
        raise ValueError("--max-frames must be non-negative")

    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    videos = discover_videos(input_dir)
    if not videos:
        raise FileNotFoundError(f"no videos found under {input_dir}")

    stop_requested = [False]

    def request_stop(_signum: int, _frame: object) -> None:
        stop_requested[0] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    device = choose_device(args.device)
    print(f"DEVICE={device}", flush=True)
    processor = AutoImageProcessor.from_pretrained(
        args.model_id, revision=args.model_revision
    )
    model = Mask2FormerForUniversalSegmentation.from_pretrained(
        args.model_id, revision=args.model_revision
    ).to(device)
    model.eval()
    road_ids = resolve_ids(model.config.id2label, ROAD_LABELS)
    sidewalk_ids = resolve_ids(model.config.id2label, SIDEWALK_LABELS)

    started = time.perf_counter()
    results: list[dict[str, Any]] = []
    with RunLock(output_dir / "run.lock"):
        for video in videos:
            print(f"START_VIDEO={video}", flush=True)
            result = render_video(
                video,
                output_dir,
                processor=processor,
                model=model,
                device=device,
                road_ids=road_ids,
                sidewalk_ids=sidewalk_ids,
                temporal_alpha=args.temporal_alpha,
                hysteresis_margin=args.temporal_hysteresis_margin,
                evaluation_size=tuple(args.evaluation_size),
                batch_size=args.batch_size,
                max_frames=args.max_frames,
                stop_requested=stop_requested,
            )
            results.append(result)
            atomic_write_json(
                output_dir / "run-summary.json",
                {
                    "schema_version": 1,
                    "status": "running",
                    "updated_at": utc_now(),
                    "model": {"id": args.model_id, "revision": args.model_revision},
                    "settings": {
                        "profile": "surface-aggregate",
                        "temporal_alpha": args.temporal_alpha,
                        "temporal_hysteresis_margin": args.temporal_hysteresis_margin,
                        "evaluation_size": list(args.evaluation_size),
                        "batch_size": args.batch_size,
                        "max_frames_per_video": args.max_frames,
                        "device": str(device),
                    },
                    "videos": results,
                },
            )
            print(
                f"COMPLETE_VIDEO={video.name} elapsed_seconds="
                f"{result['elapsed_seconds']:.3f}",
                flush=True,
            )

    total_elapsed = time.perf_counter() - started
    summary = {
        "schema_version": 1,
        "status": "complete",
        "completed_at": utc_now(),
        "model": {"id": args.model_id, "revision": args.model_revision},
        "settings": {
            "profile": "surface-aggregate",
            "road_labels": list(ROAD_LABELS),
            "sidewalk_labels": list(SIDEWALK_LABELS),
            "temporal_alpha": args.temporal_alpha,
            "temporal_hysteresis_margin": args.temporal_hysteresis_margin,
            "evaluation_size": list(args.evaluation_size),
            "batch_size": args.batch_size,
            "max_frames_per_video": args.max_frames,
            "device": str(device),
        },
        "video_count": len(results),
        "rendered_frame_count": sum(item["rendered_frame_count"] for item in results),
        "total_elapsed_seconds": total_elapsed,
        "videos": results,
    }
    atomic_write_json(output_dir / "run-summary.json", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
