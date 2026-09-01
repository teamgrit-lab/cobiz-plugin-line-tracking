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
"""Directly sweep cheap R50 post-processing against the pinned Swin-L masks."""

from __future__ import annotations

import argparse
import os
import tempfile
import time
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch

from best_so_far_runtime import (
    R50_PROFILE,
    SWIN_L_PROFILE,
    BestSoFarConfig,
    BestSoFarSegmenter,
)
from evaluate_mapillary_label_aggregation import (
    ROAD_LABELS,
    SIDEWALK_LABELS,
    resolve_ids,
)
from evaluate_mapillary_temporal import aggregated_selected_mask
from evaluate_sidewalk_road_temporal import binary_iou, remove_small_components
from segment_sidewalk_road import atomic_write_json, discover_videos, utc_now


@dataclass
class CandidateState:
    name: str
    maximum_road_component_area: int
    minimum_sidewalk_ring_ratio: float
    action: str
    semantic_mapping: str = "current"
    hysteresis_margin: float = 0.07
    previous_selected: np.ndarray | None = None


def parse_args() -> argparse.Namespace:
    repository = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-dir", type=Path, default=repository / "rosbag-results" / "full"
    )
    parser.add_argument(
        "--output-report",
        type=Path,
        default=(
            repository
            / "rosbag-results"
            / "r50-swin-experiments-20260901"
            / "direct-r50-swin-sweep-20260901.json"
        ),
    )
    parser.add_argument("--clips-per-video", type=int, default=3)
    parser.add_argument("--frames-per-clip", type=int, default=12)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--progress-every", type=int, default=24)
    return parser.parse_args()


def candidate_states() -> list[CandidateState]:
    return [
        CandidateState("baseline", 0, 0.0, "none"),
        CandidateState("drop-road-2560-ring-0.10", 2560, 0.10, "drop"),
        CandidateState("semantic-tuned", 0, 0.0, "none", "swin-tuned"),
        CandidateState(
            "semantic-tuned-drop-road-640-ring-0.10",
            640,
            0.10,
            "drop",
            "swin-tuned",
        ),
        CandidateState(
            "semantic-tuned-manhole",
            0,
            0.0,
            "none",
            "swin-tuned-manhole",
        ),
        CandidateState(
            "semantic-tuned-manhole-drop-road-2560-ring-0.05",
            2560,
            0.05,
            "drop",
            "swin-tuned-manhole",
        ),
        CandidateState(
            "semantic-tuned-manhole-drop-road-2560-ring-0.10-hysteresis-0.00",
            2560,
            0.10,
            "drop",
            "swin-tuned-manhole",
            hysteresis_margin=0.0,
        ),
        CandidateState(
            "semantic-tuned-manhole-drop-road-2560-ring-0.10-hysteresis-0.03",
            2560,
            0.10,
            "drop",
            "swin-tuned-manhole",
            hysteresis_margin=0.03,
        ),
        CandidateState(
            "semantic-tuned-manhole-drop-road-2560-ring-0.10",
            2560,
            0.10,
            "drop",
            "swin-tuned-manhole",
        ),
        CandidateState(
            "semantic-tuned-manhole-drop-road-3840-ring-0.05",
            3840,
            0.05,
            "drop",
            "swin-tuned-manhole",
        ),
        CandidateState(
            "semantic-tuned-manhole-drop-road-3840-ring-0.10",
            3840,
            0.10,
            "drop",
            "swin-tuned-manhole",
        ),
        CandidateState(
            "semantic-tuned-manhole-drop-road-5120-ring-0.10",
            5120,
            0.10,
            "drop",
            "swin-tuned-manhole",
        ),
    ]


def reset_candidates(candidates: list[CandidateState]) -> None:
    for candidate in candidates:
        candidate.previous_selected = None


def refine_road_islands(
    selected: np.ndarray,
    *,
    maximum_area: int,
    minimum_sidewalk_ring_ratio: float,
    action: str,
) -> np.ndarray:
    if action == "none":
        return selected
    refined = selected.copy()
    sidewalk = selected == 2
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        (selected == 1).astype(np.uint8), connectivity=8
    )
    kernel = np.ones((5, 5), dtype=np.uint8)
    for component in range(1, count):
        area = int(stats[component, cv2.CC_STAT_AREA])
        if area > maximum_area:
            continue
        component_mask = labels == component
        ring = cv2.dilate(component_mask.astype(np.uint8), kernel).astype(bool)
        ring &= ~component_mask
        ring_size = int(np.count_nonzero(ring))
        ratio = (
            float(np.count_nonzero(ring & sidewalk) / ring_size) if ring_size else 0.0
        )
        if ratio >= minimum_sidewalk_ring_ratio:
            refined[component_mask] = 2 if action == "reassign-sidewalk" else 0
    return refined


def base_postprocess(
    smooth_scores: torch.Tensor,
    *,
    road_ids: list[int],
    sidewalk_ids: list[int],
) -> tuple[np.ndarray, np.ndarray, int, np.ndarray]:
    class_map = smooth_scores.argmax(dim=0).detach().cpu().numpy()
    minimum_area = max(48, int(class_map.size * 0.00035))
    selected = aggregated_selected_mask(
        class_map,
        road_ids=road_ids,
        sidewalk_ids=sidewalk_ids,
    )
    top_scores = torch.topk(smooth_scores, k=2, dim=0).values
    score_margin = (
        (top_scores[0] - top_scores[1]).detach().float().cpu().numpy()
    )
    return selected, score_margin, minimum_area, class_map


def postprocess_candidate(
    selected: np.ndarray,
    score_margin: np.ndarray,
    minimum_area: int,
    *,
    hysteresis_margin: float,
    candidate: CandidateState,
) -> np.ndarray:
    if candidate.previous_selected is not None:
        hold = (selected != candidate.previous_selected) & (
            score_margin < hysteresis_margin
        )
        selected = selected.copy()
        selected[hold] = candidate.previous_selected[hold]
    retained_road = remove_small_components(selected == 1, minimum_area)
    retained_sidewalk = remove_small_components(selected == 2, minimum_area)
    selected = np.zeros_like(selected)
    selected[retained_road] = 1
    selected[retained_sidewalk] = 2
    selected = refine_road_islands(
        selected,
        maximum_area=candidate.maximum_road_component_area,
        minimum_sidewalk_ring_ratio=candidate.minimum_sidewalk_ring_ratio,
        action=candidate.action,
    )
    candidate.previous_selected = selected.copy()
    return selected


def frame_metrics(candidate: np.ndarray, reference: np.ndarray) -> dict[str, float]:
    return {
        "selected_label_agreement": float(np.mean(candidate == reference)),
        "road_iou": binary_iou(candidate == 1, reference == 1),
        "sidewalk_iou": binary_iou(candidate == 2, reference == 2),
        "road_area_ratio": float(np.mean(candidate == 1)),
        "sidewalk_area_ratio": float(np.mean(candidate == 2)),
    }


def mean_metrics(rows: list[dict[str, float]]) -> dict[str, float]:
    return {key: float(np.mean([row[key] for row in rows])) for key in rows[0]}


def clip_starts(frame_count: int, clip_count: int, clip_length: int) -> list[int]:
    if frame_count <= clip_length:
        return [0]
    return sorted(
        {
            int(round(position))
            for position in np.linspace(0, frame_count - clip_length, clip_count)
        }
    )


def update_label_confusion(
    confusion: dict[int, np.ndarray],
    class_map: np.ndarray,
    reference: np.ndarray,
) -> None:
    for class_id in np.unique(class_map):
        predicted = class_map == class_id
        confusion[int(class_id)] += np.bincount(
            reference[predicted], minlength=3
        ).astype(np.int64)


def main() -> int:
    args = parse_args()
    if args.clips_per_video <= 0 or args.frames_per_clip <= 0:
        raise ValueError("clip and frame counts must be positive")
    videos = discover_videos(args.input_dir.expanduser().resolve())
    if not videos:
        raise RuntimeError("no input videos found")

    print("MODEL_LOAD_START profile=swin-l-best-so-far", flush=True)
    swin = BestSoFarSegmenter(
        BestSoFarConfig(profile=SWIN_L_PROFILE, device=args.device)
    )
    print("MODEL_LOAD_START profile=r50-fp16-640x360", flush=True)
    r50 = BestSoFarSegmenter(
        BestSoFarConfig(profile=R50_PROFILE, device=args.device)
    )
    candidates = candidate_states()
    tuned_road_labels = tuple(
        label for label in ROAD_LABELS if label not in {"Bike Lane", "Parking", "Service Lane"}
    )
    tuned_sidewalk_labels = SIDEWALK_LABELS + ("Bike Lane",)
    tuned_road_ids = resolve_ids(r50.model.config.id2label, tuned_road_labels)
    tuned_sidewalk_ids = resolve_ids(
        r50.model.config.id2label, tuned_sidewalk_labels
    )
    tuned_manhole_sidewalk_labels = tuned_sidewalk_labels + ("Manhole",)
    tuned_manhole_sidewalk_ids = resolve_ids(
        r50.model.config.id2label, tuned_manhole_sidewalk_labels
    )
    canonical_road_ids = resolve_ids(r50.model.config.id2label, ROAD_LABELS)
    canonical_sidewalk_ids = resolve_ids(r50.model.config.id2label, SIDEWALK_LABELS)
    rows: dict[str, list[dict[str, float]]] = defaultdict(list)
    label_confusion: dict[int, np.ndarray] = defaultdict(
        lambda: np.zeros(3, dtype=np.int64)
    )
    processed = 0
    skipped_clips: list[dict[str, Any]] = []
    r50_inference_seconds: list[float] = []
    started = time.perf_counter()

    for video in videos:
        probe = cv2.VideoCapture(str(video))
        if not probe.isOpened():
            skipped_clips.append({"video": str(video), "reason": "open-failed"})
            continue
        frame_count = int(probe.get(cv2.CAP_PROP_FRAME_COUNT))
        probe.release()
        for start in clip_starts(
            frame_count, args.clips_per_video, args.frames_per_clip
        ):
            capture = cv2.VideoCapture(str(video))
            if not capture.isOpened():
                skipped_clips.append(
                    {"video": str(video), "start_frame": start, "reason": "open-failed"}
                )
                continue
            capture.set(cv2.CAP_PROP_POS_FRAMES, start)
            swin.reset()
            r50.reset()
            reset_candidates(candidates)
            previous_scores: torch.Tensor | None = None
            decoded = 0
            try:
                for _ in range(args.frames_per_clip):
                    ok, frame = capture.read()
                    if not ok or frame is None:
                        break
                    reference = swin.segment(frame).selected_mask
                    inference_started = time.perf_counter()
                    scores = r50._semantic_scores(frame)
                    r50._synchronize()
                    r50_inference_seconds.append(time.perf_counter() - inference_started)
                    smooth_scores = (
                        scores
                        if previous_scores is None
                        else 0.62 * scores + 0.38 * previous_scores
                    )
                    previous_scores = smooth_scores
                    base_selected, score_margin, minimum_area, class_map = (
                        base_postprocess(
                            smooth_scores,
                            road_ids=canonical_road_ids,
                            sidewalk_ids=canonical_sidewalk_ids,
                        )
                    )
                    tuned_selected = aggregated_selected_mask(
                        class_map,
                        road_ids=tuned_road_ids,
                        sidewalk_ids=tuned_sidewalk_ids,
                    )
                    tuned_manhole_selected = aggregated_selected_mask(
                        class_map,
                        road_ids=tuned_road_ids,
                        sidewalk_ids=tuned_manhole_sidewalk_ids,
                    )
                    update_label_confusion(label_confusion, class_map, reference)
                    selected_by_mapping = {
                        "current": base_selected,
                        "swin-tuned": tuned_selected,
                        "swin-tuned-manhole": tuned_manhole_selected,
                    }
                    for candidate in candidates:
                        selected = postprocess_candidate(
                            selected_by_mapping[candidate.semantic_mapping],
                            score_margin,
                            minimum_area,
                            hysteresis_margin=candidate.hysteresis_margin,
                            candidate=candidate,
                        )
                        rows[candidate.name].append(frame_metrics(selected, reference))
                    processed += 1
                    decoded += 1
                    if processed % args.progress_every == 0:
                        rate = processed / (time.perf_counter() - started)
                        print(
                            f"DIRECT_SWEEP_PROGRESS frames={processed} rate={rate:.3f}fps",
                            flush=True,
                        )
            finally:
                capture.release()
            if decoded == 0:
                skipped_clips.append(
                    {"video": str(video), "start_frame": start, "reason": "decode-failed"}
                )

    if not rows:
        raise RuntimeError("no frames were processed")
    summaries: list[dict[str, Any]] = []
    for candidate in candidates:
        summary: dict[str, Any] = mean_metrics(rows[candidate.name])
        summary.update(
            {
                "name": candidate.name,
                "action": candidate.action,
                "semantic_mapping": candidate.semantic_mapping,
                "maximum_road_component_area": candidate.maximum_road_component_area,
                "minimum_sidewalk_ring_ratio": candidate.minimum_sidewalk_ring_ratio,
                "hysteresis_margin": candidate.hysteresis_margin,
                "mean_surface_iou": (
                    summary["road_iou"] + summary["sidewalk_iou"]
                )
                / 2.0,
            }
        )
        summaries.append(summary)
    summaries.sort(
        key=lambda item: (
            item["mean_surface_iou"], item["selected_label_agreement"]
        ),
        reverse=True,
    )

    id2label = {int(key): value for key, value in r50.model.config.id2label.items()}
    confusion_rows = []
    for class_id, counts in sorted(label_confusion.items()):
        total = int(counts.sum())
        confusion_rows.append(
            {
                "class_id": class_id,
                "label": id2label.get(class_id, f"class-{class_id}"),
                "pixels": total,
                "reference_background_ratio": float(counts[0] / total),
                "reference_road_ratio": float(counts[1] / total),
                "reference_sidewalk_ratio": float(counts[2] / total),
                "majority_reference": ("background", "road", "sidewalk")[
                    int(np.argmax(counts))
                ],
                "currently_road": class_id in r50.road_ids,
                "currently_sidewalk": class_id in r50.sidewalk_ids,
            }
        )
    payload = {
        "schema_version": 1,
        "status": "complete",
        "updated_at": utc_now(),
        "input_directory": str(args.input_dir.expanduser().resolve()),
        "video_count": len(videos),
        "sampling": {
            "clips_per_video": args.clips_per_video,
            "frames_per_clip": args.frames_per_clip,
            "processed_frames": processed,
            "skipped_clips": skipped_clips,
        },
        "runtimes": {"swin_l": swin.metadata(), "r50": r50.metadata()},
        "r50_inference_fps": (
            len(r50_inference_seconds) / sum(r50_inference_seconds)
        ),
        "candidates": summaries,
        "best_candidate": summaries[0],
        "semantic_tuning": {
            "road_labels": list(tuned_road_labels),
            "sidewalk_labels": list(tuned_manhole_sidewalk_labels),
            "excluded_road_labels": ["Parking", "Service Lane"],
            "remapped_road_to_sidewalk_labels": ["Bike Lane"],
            "added_sidewalk_labels": ["Manhole"],
        },
        "label_confusion": confusion_rows,
        "wall_elapsed_seconds": time.perf_counter() - started,
    }
    output = args.output_report.expanduser().resolve()
    atomic_write_json(output, payload)
    print(
        f"DIRECT_SWEEP_COMPLETE frames={processed} best={summaries[0]['name']}",
        flush=True,
    )
    print(f"REPORT_WRITTEN path={output}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
