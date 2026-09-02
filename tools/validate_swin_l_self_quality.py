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
"""Capture and compare a fixed Swin-L-only segmentation validation corpus."""

from __future__ import annotations

import argparse
import json
import os
import tempfile
import time
from itertools import pairwise
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from benchmark_best_so_far import DEFAULT_TOPIC, iter_mcap_packets
from best_so_far_runtime import SWIN_L_PROFILE, BestSoFarConfig, BestSoFarSegmenter


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, path)
    except BaseException:
        Path(temporary).unlink(missing_ok=True)
        raise


def _atomic_npz(path: Path, **arrays: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.stem}.", suffix=".npz", dir=path.parent
    )
    os.close(descriptor)
    try:
        np.savez_compressed(temporary, **arrays)
        os.replace(temporary, path)
    except BaseException:
        Path(temporary).unlink(missing_ok=True)
        raise


def _iou(left: np.ndarray, right: np.ndarray, label: int) -> float:
    left_mask = left == label
    right_mask = right == label
    union = np.count_nonzero(left_mask | right_mask)
    if union == 0:
        return 1.0
    return float(np.count_nonzero(left_mask & right_mask) / union)


def _components(mask: np.ndarray) -> int:
    count, _ = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
    return max(0, int(count) - 1)


def _run_metrics(masks: np.ndarray, segment_ids: np.ndarray) -> dict[str, float]:
    adjacent_road: list[float] = []
    adjacent_sidewalk: list[float] = []
    selected_change: list[float] = []
    for index, (previous, current) in enumerate(pairwise(masks), start=1):
        if segment_ids[index - 1] != segment_ids[index]:
            continue
        adjacent_road.append(_iou(previous, current, 1))
        adjacent_sidewalk.append(_iou(previous, current, 2))
        selected_change.append(float(np.mean(previous != current)))
    return {
        "mean_road_area_ratio": float(np.mean(masks == 1)),
        "mean_sidewalk_area_ratio": float(np.mean(masks == 2)),
        "mean_road_components": float(
            np.mean([_components(mask == 1) for mask in masks])
        ),
        "mean_sidewalk_components": float(
            np.mean([_components(mask == 2) for mask in masks])
        ),
        "mean_road_adjacent_iou": float(np.mean(adjacent_road)),
        "mean_sidewalk_adjacent_iou": float(np.mean(adjacent_sidewalk)),
        "mean_selected_label_change": float(np.mean(selected_change)),
    }


def _comparison(reference: np.ndarray, candidate: np.ndarray) -> dict[str, Any]:
    if reference.shape != candidate.shape:
        raise ValueError(
            f"mask shapes differ: reference={reference.shape}, candidate={candidate.shape}"
        )
    exact = np.all(reference == candidate, axis=(1, 2))
    return {
        "frame_count": int(reference.shape[0]),
        "exact_frame_count": int(np.count_nonzero(exact)),
        "exact_frame_ratio": float(np.mean(exact)),
        "selected_mask_agreement": float(np.mean(reference == candidate)),
        "road_iou": _iou(reference, candidate, 1),
        "sidewalk_iou": _iou(reference, candidate, 2),
        "changed_pixel_count": int(np.count_nonzero(reference != candidate)),
    }


def _capture(
    input_path: Path,
    topic: str,
    offsets: list[float],
    frames_per_segment: int,
) -> tuple[dict[str, np.ndarray], list[np.ndarray], dict[str, Any]]:
    segmenter = BestSoFarSegmenter(BestSoFarConfig(profile=SWIN_L_PROFILE))
    masks: list[np.ndarray] = []
    segment_ids: list[int] = []
    sequences: list[int] = []
    timestamps: list[int] = []
    preview_frames: list[np.ndarray] = []
    started = time.perf_counter()
    for segment_id, offset in enumerate(offsets):
        segmenter.reset()
        packets = iter_mcap_packets(
            input_path,
            topic,
            start_offset_seconds=offset,
            max_frames=frames_per_segment,
        )
        for packet in packets:
            result = segmenter.segment(packet.frame_bgr)
            masks.append(result.selected_mask.copy())
            segment_ids.append(segment_id)
            sequences.append(packet.sequence)
            timestamps.append(packet.source_timestamp_ns)
            if len(preview_frames) < 6:
                preview_frames.append(
                    segmenter.render_overlay(
                        packet.frame_bgr,
                        result.selected_mask,
                        frame_index=packet.sequence,
                        fps=20.0,
                    )
                )
    stacked = np.stack(masks)
    arrays = {
        "masks": stacked,
        "segment_ids": np.asarray(segment_ids, dtype=np.int16),
        "sequences": np.asarray(sequences, dtype=np.int32),
        "source_timestamps_ns": np.asarray(timestamps, dtype=np.int64),
        "offsets_seconds": np.asarray(offsets, dtype=np.float64),
    }
    metadata = {
        "profile": segmenter.metadata(),
        "frame_count": int(stacked.shape[0]),
        "wall_elapsed_seconds": time.perf_counter() - started,
        "metrics": _run_metrics(stacked, arrays["segment_ids"]),
    }
    return arrays, preview_frames, metadata


def _write_contact_sheet(path: Path, frames: list[np.ndarray]) -> None:
    if not frames:
        return
    width, height = 480, 270
    resized = [cv2.resize(frame, (width, height)) for frame in frames]
    while len(resized) % 3:
        resized.append(np.zeros_like(resized[0]))
    rows = [cv2.hconcat(resized[index : index + 3]) for index in range(0, len(resized), 3)]
    sheet = cv2.vconcat(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), sheet):
        raise RuntimeError(f"could not write contact sheet: {path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--offsets", type=float, nargs="+", default=[0.0, 145.0, 292.0])
    parser.add_argument("--frames-per-segment", type=int, default=12)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--reference-npz", type=Path)
    args = parser.parse_args()
    if args.frames_per_segment <= 1 or args.repeat <= 0:
        parser.error("--frames-per-segment must exceed one and --repeat must be positive")
    return args


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir.expanduser().resolve()
    reference_masks: np.ndarray | None = None
    if args.reference_npz is not None:
        with np.load(args.reference_npz.expanduser().resolve()) as reference:
            reference_masks = reference["masks"].copy()
    runs: list[dict[str, Any]] = []
    first_masks: np.ndarray | None = None
    for repeat_index in range(1, args.repeat + 1):
        arrays, preview_frames, metadata = _capture(
            args.input.expanduser().resolve(),
            args.topic,
            args.offsets,
            args.frames_per_segment,
        )
        npz_path = output_dir / f"{args.label}-run{repeat_index}.npz"
        _atomic_npz(npz_path, **arrays)
        if repeat_index == 1:
            _write_contact_sheet(
                output_dir / f"{args.label}-contact-sheet.jpg", preview_frames
            )
            first_masks = arrays["masks"].copy()
        comparison_target = reference_masks if reference_masks is not None else first_masks
        metadata.update(
            {
                "run": repeat_index,
                "mask_artifact": str(npz_path),
                "comparison": (
                    _comparison(comparison_target, arrays["masks"])
                    if comparison_target is not None
                    else None
                ),
            }
        )
        runs.append(metadata)
    report = {
        "schema_version": 1,
        "input": str(args.input.expanduser().resolve()),
        "topic": args.topic,
        "label": args.label,
        "offsets_seconds": args.offsets,
        "frames_per_segment": args.frames_per_segment,
        "reference_npz": (
            str(args.reference_npz.expanduser().resolve())
            if args.reference_npz is not None
            else None
        ),
        "runs": runs,
    }
    _atomic_json(output_dir / f"{args.label}-report.json", report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
