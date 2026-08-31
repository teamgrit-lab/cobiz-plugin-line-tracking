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
"""Compare two named Mapillary profiles on the same consecutive video frames."""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from best_so_far_runtime import (
    PROFILE_NAMES,
    R50_PROFILE,
    SWIN_L_PROFILE,
    BestSoFarConfig,
    BestSoFarSegmenter,
)
from evaluate_sidewalk_road_temporal import binary_iou
from segment_sidewalk_road import atomic_write_json, utc_now


def parse_args() -> argparse.Namespace:
    repository = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repository / "rosbag-results" / "profile-comparisons",
    )
    parser.add_argument(
        "--profiles",
        choices=PROFILE_NAMES,
        nargs=2,
        default=(SWIN_L_PROFILE, R50_PROFILE),
        metavar=("LEFT", "RIGHT"),
    )
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--max-frames", type=int, default=200)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--progress-every", type=int, default=25)
    return parser.parse_args()


def mean(values: list[float]) -> float:
    return float(np.mean(values)) if values else 0.0


def profile_summary(
    *,
    elapsed: list[float],
    inference: list[float],
    postprocess: list[float],
    road_areas: list[float],
    sidewalk_areas: list[float],
    road_temporal_iou: list[float],
    sidewalk_temporal_iou: list[float],
) -> dict[str, float]:
    total = sum(elapsed)
    return {
        "processing_fps": len(elapsed) / total if total else 0.0,
        "mean_total_ms": mean(elapsed) * 1000.0,
        "mean_inference_ms": mean(inference) * 1000.0,
        "mean_postprocess_ms": mean(postprocess) * 1000.0,
        "mean_road_area_ratio": mean(road_areas),
        "mean_sidewalk_area_ratio": mean(sidewalk_areas),
        "mean_road_adjacent_iou": mean(road_temporal_iou),
        "mean_sidewalk_adjacent_iou": mean(sidewalk_temporal_iou),
    }


def main() -> int:
    args = parse_args()
    if args.start_frame < 0 or args.max_frames <= 0 or args.progress_every <= 0:
        raise ValueError(
            "start frame must be non-negative; frame limits must be positive"
        )
    input_path = args.input.expanduser().resolve()
    if not input_path.is_file():
        raise FileNotFoundError(f"input video does not exist: {input_path}")
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    segmenters = [
        BestSoFarSegmenter(BestSoFarConfig(profile=name, device=args.device))
        for name in args.profiles
    ]
    capture = cv2.VideoCapture(str(input_path))
    if not capture.isOpened():
        raise RuntimeError(f"could not open input video: {input_path}")
    capture.set(cv2.CAP_PROP_POS_FRAMES, args.start_frame)
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    if not np.isfinite(fps) or fps <= 0.0:
        fps = 20.0
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    stem = f"{input_path.stem}-frames-{args.start_frame}-{args.start_frame + args.max_frames - 1}"
    final_video = output_dir / f"{stem}-comparison.mp4"
    staging_video = output_dir / f".{stem}-comparison.tmp.mp4"
    staging_video.unlink(missing_ok=True)
    writer = cv2.VideoWriter(
        str(staging_video),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width * 2, height),
    )
    if not writer.isOpened():
        capture.release()
        raise RuntimeError(f"could not create comparison video: {staging_video}")

    timings: list[dict[str, list[float]]] = [
        {
            "elapsed": [],
            "inference": [],
            "postprocess": [],
            "road_areas": [],
            "sidewalk_areas": [],
            "road_temporal_iou": [],
            "sidewalk_temporal_iou": [],
        }
        for _ in segmenters
    ]
    previous: list[np.ndarray | None] = [None, None]
    road_cross_iou: list[float] = []
    sidewalk_cross_iou: list[float] = []
    selected_agreement: list[float] = []
    processed = 0
    started = time.perf_counter()
    try:
        while processed < args.max_frames:
            ok, frame = capture.read()
            if not ok:
                break
            masks: list[np.ndarray] = []
            overlays: list[np.ndarray] = []
            for index, segmenter in enumerate(segmenters):
                result = segmenter.segment(frame)
                masks.append(result.selected_mask)
                timings[index]["elapsed"].append(result.total_seconds)
                timings[index]["inference"].append(result.inference_seconds)
                timings[index]["postprocess"].append(result.postprocess_seconds)
                timings[index]["road_areas"].append(result.road_area_ratio)
                timings[index]["sidewalk_areas"].append(result.sidewalk_area_ratio)
                if previous[index] is not None:
                    timings[index]["road_temporal_iou"].append(
                        binary_iou(result.selected_mask == 1, previous[index] == 1)
                    )
                    timings[index]["sidewalk_temporal_iou"].append(
                        binary_iou(result.selected_mask == 2, previous[index] == 2)
                    )
                previous[index] = result.selected_mask.copy()
                overlay = segmenter.render_overlay(
                    frame,
                    result.selected_mask,
                    frame_index=args.start_frame + processed,
                    fps=fps,
                )
                cv2.putText(
                    overlay,
                    segmenter.profile.name,
                    (12, height - 14),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                overlays.append(overlay)
            road_cross_iou.append(binary_iou(masks[0] == 1, masks[1] == 1))
            sidewalk_cross_iou.append(binary_iou(masks[0] == 2, masks[1] == 2))
            selected_agreement.append(float(np.mean(masks[0] == masks[1])))
            writer.write(np.hstack(overlays))
            processed += 1
            if processed % args.progress_every == 0:
                print(
                    f"COMPARE_PROGRESS frames={processed}/{args.max_frames} "
                    f"elapsed_seconds={time.perf_counter() - started:.1f}",
                    flush=True,
                )
    finally:
        capture.release()
        writer.release()

    if processed == 0:
        staging_video.unlink(missing_ok=True)
        raise RuntimeError("no comparison frames were decoded")
    os.replace(staging_video, final_video)
    summaries = [profile_summary(**values) for values in timings]
    report: dict[str, Any] = {
        "schema_version": 1,
        "status": "complete",
        "updated_at": utc_now(),
        "input": str(input_path),
        "output_video": str(final_video),
        "start_frame": args.start_frame,
        "processed_frames": processed,
        "fps": fps,
        "profiles": [
            {"runtime": segmenter.metadata(), "metrics": summary}
            for segmenter, summary in zip(segmenters, summaries, strict=True)
        ],
        "cross_profile": {
            "mean_selected_label_agreement": mean(selected_agreement),
            "mean_road_iou": mean(road_cross_iou),
            "mean_sidewalk_iou": mean(sidewalk_cross_iou),
        },
        "wall_elapsed_seconds": time.perf_counter() - started,
    }
    report_path = output_dir / f"{stem}-comparison.json"
    atomic_write_json(report_path, report)
    print(f"COMPARISON_VIDEO_WRITTEN path={final_video}", flush=True)
    print(f"COMPARISON_REPORT_WRITTEN path={report_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
