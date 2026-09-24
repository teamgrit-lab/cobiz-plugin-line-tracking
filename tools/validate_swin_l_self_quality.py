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
import torch
from benchmark_best_so_far import DEFAULT_TOPIC, iter_mcap_packets
from best_so_far_runtime import (
    INFERENCE_BACKENDS,
    PROFILE_NAMES,
    SWIN_L_ASPECT_PROFILE,
    BestSoFarConfig,
    BestSoFarSegmenter,
)
from local_path import (
    PATH_MASK_CLASSES,
    LocalPathConfig,
    LocalPathSmoother,
    extract_sidewalk_centerline,
    selected_path_region,
)


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
    profile: str,
    device: str,
    path_mask_class: int,
    seed: int,
    backend: str = "pytorch",
    tensorrt_engine: str | None = None,
    tensorrt_manifest: str | None = None,
) -> tuple[dict[str, np.ndarray], list[np.ndarray], dict[str, Any]]:
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    cuda_baseline_allocated: int | None = None
    cuda_baseline_reserved: int | None = None
    if torch.cuda.is_available() and device in {"auto", "cuda"}:
        torch.cuda.empty_cache()
        cuda_baseline_allocated = int(torch.cuda.memory_allocated())
        cuda_baseline_reserved = int(torch.cuda.memory_reserved())
    segmenter = BestSoFarSegmenter(
        BestSoFarConfig(
            profile=profile,
            device=device,
            backend=backend,
            tensorrt_engine_path=tensorrt_engine,
            tensorrt_manifest_path=tensorrt_manifest,
        )
    )
    model_parameters = (
        tuple(segmenter.model.parameters()) if segmenter.model is not None else ()
    )
    model_buffers = (
        tuple(segmenter.model.buffers()) if segmenter.model is not None else ()
    )
    parameter_bytes = int(
        sum(
            parameter.numel() * parameter.element_size()
            for parameter in model_parameters
        )
    )
    floating_parameter_bytes = int(
        sum(
            parameter.numel() * parameter.element_size()
            for parameter in model_parameters
            if parameter.is_floating_point()
        )
    )
    buffer_bytes = int(
        sum(buffer.numel() * buffer.element_size() for buffer in model_buffers)
    )
    dtype_parameter_bytes: dict[str, int] = {}
    for parameter in model_parameters:
        name = str(parameter.dtype)
        dtype_parameter_bytes[name] = dtype_parameter_bytes.get(name, 0) + int(
            parameter.numel() * parameter.element_size()
        )

    local_config = LocalPathConfig()
    smoother = LocalPathSmoother(local_config)
    point_count = local_config.path_points
    masks: list[np.ndarray] = []
    segment_ids: list[int] = []
    sequences: list[int] = []
    timestamps: list[int] = []
    path_valid: list[bool] = []
    path_points: list[np.ndarray] = []
    path_confidence: list[float] = []
    path_valid_ratio: list[float] = []
    path_mean_width: list[float] = []
    smoothed_path_valid: list[bool] = []
    smoothed_path_points: list[np.ndarray] = []
    smoothed_path_confidence: list[float] = []
    preview_frames: list[np.ndarray] = []

    cuda_after_model_allocated: int | None = None
    cuda_after_model_reserved: int | None = None
    cuda_steady_allocated: int | None = None
    cuda_steady_reserved: int | None = None
    if segmenter.device.type == "cuda":
        torch.cuda.synchronize(segmenter.device)
        cuda_after_model_allocated = int(torch.cuda.memory_allocated(segmenter.device))
        cuda_after_model_reserved = int(torch.cuda.memory_reserved(segmenter.device))
        warmup = next(
            iter_mcap_packets(
                input_path,
                topic,
                start_offset_seconds=offsets[0],
                max_frames=1,
            ),
            None,
        )
        if warmup is None:
            raise RuntimeError("no camera frame available for CUDA warm-up")
        segmenter.segment(warmup.frame_bgr)
        segmenter.reset()
        torch.cuda.synchronize(segmenter.device)
        torch.cuda.empty_cache()
        cuda_steady_allocated = int(torch.cuda.memory_allocated(segmenter.device))
        cuda_steady_reserved = int(torch.cuda.memory_reserved(segmenter.device))
        torch.cuda.reset_peak_memory_stats(segmenter.device)

    started = time.perf_counter()
    for segment_id, offset in enumerate(offsets):
        segmenter.reset()
        smoother.reset()
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
            estimate = extract_sidewalk_centerline(
                selected_path_region(result.selected_mask, path_mask_class),
                local_config,
            )
            smoothed = smoother.update(
                estimate,
                packet.source_timestamp_ns / 1_000_000_000,
            )
            missing_points = np.full((point_count, 2), np.nan, dtype=np.float32)
            path_valid.append(estimate is not None and estimate.is_valid)
            path_points.append(
                estimate.points_xy.copy()
                if estimate is not None
                else missing_points.copy()
            )
            path_confidence.append(
                float(estimate.confidence) if estimate is not None else float("nan")
            )
            path_valid_ratio.append(
                float(estimate.valid_ratio) if estimate is not None else float("nan")
            )
            path_mean_width.append(
                float(estimate.mean_sidewalk_width_m)
                if estimate is not None
                else float("nan")
            )
            smoothed_path_valid.append(smoothed is not None)
            smoothed_path_points.append(
                smoothed.points_xy.copy()
                if smoothed is not None
                else missing_points.copy()
            )
            smoothed_path_confidence.append(
                float(smoothed.confidence) if smoothed is not None else float("nan")
            )
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
        "path_valid": np.asarray(path_valid, dtype=bool),
        "path_points": np.stack(path_points),
        "path_confidence": np.asarray(path_confidence, dtype=np.float32),
        "path_valid_ratio": np.asarray(path_valid_ratio, dtype=np.float32),
        "path_mean_width_m": np.asarray(path_mean_width, dtype=np.float32),
        "smoothed_path_valid": np.asarray(smoothed_path_valid, dtype=bool),
        "smoothed_path_points": np.stack(smoothed_path_points),
        "smoothed_path_confidence": np.asarray(
            smoothed_path_confidence, dtype=np.float32
        ),
    }
    overall_peak_allocated: int | None = None
    overall_peak_reserved: int | None = None
    activation_peak_allocated: int | None = None
    if segmenter.device.type == "cuda":
        torch.cuda.synchronize(segmenter.device)
        overall_peak_allocated = int(torch.cuda.max_memory_allocated(segmenter.device))
        overall_peak_reserved = int(torch.cuda.max_memory_reserved(segmenter.device))
        activation_peak_allocated = max(
            0, overall_peak_allocated - int(cuda_steady_allocated or 0)
        )
    metadata = {
        "profile": segmenter.metadata(),
        "frame_count": int(stacked.shape[0]),
        "wall_elapsed_seconds": time.perf_counter() - started,
        "metrics": _run_metrics(stacked, arrays["segment_ids"]),
        "path_mask_class": path_mask_class,
        "memory": {
            "parameter_bytes": parameter_bytes,
            "floating_parameter_bytes": floating_parameter_bytes,
            "buffer_bytes": buffer_bytes,
            "dtype_parameter_bytes": dtype_parameter_bytes,
            "cuda_baseline_allocated_bytes": cuda_baseline_allocated,
            "cuda_baseline_reserved_bytes": cuda_baseline_reserved,
            "cuda_after_model_allocated_bytes": cuda_after_model_allocated,
            "cuda_after_model_reserved_bytes": cuda_after_model_reserved,
            "cuda_steady_allocated_bytes": cuda_steady_allocated,
            "cuda_steady_reserved_bytes": cuda_steady_reserved,
            "activation_peak_allocated_bytes": activation_peak_allocated,
            "overall_peak_allocated_bytes": overall_peak_allocated,
            "overall_peak_reserved_bytes": overall_peak_reserved,
        },
    }
    return arrays, preview_frames, metadata


def _write_contact_sheet(path: Path, frames: list[np.ndarray]) -> None:
    if not frames:
        return
    width, height = 480, 270
    resized = [cv2.resize(frame, (width, height)) for frame in frames]
    while len(resized) % 3:
        resized.append(np.zeros_like(resized[0]))
    rows = [
        cv2.hconcat(resized[index : index + 3]) for index in range(0, len(resized), 3)
    ]
    sheet = cv2.vconcat(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), sheet):
        raise RuntimeError(f"could not write contact sheet: {path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument(
        "--profile", choices=PROFILE_NAMES, default=SWIN_L_ASPECT_PROFILE
    )
    parser.add_argument("--device", default="auto")
    parser.add_argument("--backend", choices=INFERENCE_BACKENDS, default="pytorch")
    parser.add_argument("--tensorrt-engine")
    parser.add_argument("--tensorrt-manifest")
    parser.add_argument(
        "--path-mask-class", type=int, choices=tuple(PATH_MASK_CLASSES), default=2
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--offsets", type=float, nargs="+", default=[0.0, 145.0, 292.0])
    parser.add_argument("--frames-per-segment", type=int, default=12)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--reference-npz", type=Path)
    args = parser.parse_args()
    if args.frames_per_segment <= 1 or args.repeat <= 0:
        parser.error(
            "--frames-per-segment must exceed one and --repeat must be positive"
        )
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
            args.profile,
            args.device,
            args.path_mask_class,
            args.seed,
            args.backend,
            args.tensorrt_engine,
            args.tensorrt_manifest,
        )
        npz_path = output_dir / f"{args.label}-run{repeat_index}.npz"
        _atomic_npz(npz_path, **arrays)
        if repeat_index == 1:
            _write_contact_sheet(
                output_dir / f"{args.label}-contact-sheet.jpg", preview_frames
            )
            first_masks = arrays["masks"].copy()
        comparison_target = (
            reference_masks if reference_masks is not None else first_masks
        )
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
        "seed": args.seed,
        "runs": runs,
    }
    _atomic_json(output_dir / f"{args.label}-report.json", report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
