#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = ["numpy==2.5.2"]
# ///
"""Compare independently captured Swin-L FP32 and FP16 artifacts."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import tempfile
from typing import Any

import numpy as np


def _iou(left: np.ndarray, right: np.ndarray, label: int) -> float:
    left_class = left == label
    right_class = right == label
    union = int(np.count_nonzero(left_class | right_class))
    if union == 0:
        return 1.0
    return float(np.count_nonzero(left_class & right_class) / union)


def _summary(values: np.ndarray) -> dict[str, float | None]:
    finite = np.asarray(values, dtype=np.float64)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return {"mean": None, "p95": None, "max": None}
    return {
        "mean": float(np.mean(finite)),
        "p95": float(np.percentile(finite, 95)),
        "max": float(np.max(finite)),
    }


def compare_masks(fp32: np.ndarray, fp16: np.ndarray) -> dict[str, Any]:
    if fp32.shape != fp16.shape:
        raise ValueError(f"mask shapes differ: fp32={fp32.shape}, fp16={fp16.shape}")
    if fp32.ndim != 3:
        raise ValueError("mask artifacts must have shape [frames, height, width]")
    exact = np.all(fp32 == fp16, axis=(1, 2))
    per_frame_agreement = np.mean(fp32 == fp16, axis=(1, 2))
    per_frame_road_iou = np.asarray(
        [_iou(left, right, 1) for left, right in zip(fp32, fp16, strict=True)]
    )
    per_frame_sidewalk_iou = np.asarray(
        [_iou(left, right, 2) for left, right in zip(fp32, fp16, strict=True)]
    )
    return {
        "frame_count": int(fp32.shape[0]),
        "exact_frame_ratio": float(np.mean(exact)),
        "selected_mask_agreement": float(np.mean(fp32 == fp16)),
        "road_iou": _iou(fp32, fp16, 1),
        "sidewalk_iou": _iou(fp32, fp16, 2),
        "changed_pixel_count": int(np.count_nonzero(fp32 != fp16)),
        "per_frame": {
            "selected_mask_agreement": _summary(per_frame_agreement),
            "road_iou": _summary(per_frame_road_iou),
            "sidewalk_iou": _summary(per_frame_sidewalk_iou),
        },
    }


def compare_paths(
    fp32_valid: np.ndarray,
    fp32_points: np.ndarray,
    fp32_confidence: np.ndarray,
    fp16_valid: np.ndarray,
    fp16_points: np.ndarray,
    fp16_confidence: np.ndarray,
) -> dict[str, Any]:
    if fp32_valid.shape != fp16_valid.shape:
        raise ValueError("path validity arrays differ in shape")
    if fp32_points.shape != fp16_points.shape:
        raise ValueError("path point arrays differ in shape")
    if fp32_confidence.shape != fp16_confidence.shape:
        raise ValueError("path confidence arrays differ in shape")
    paired = np.asarray(fp32_valid, dtype=bool) & np.asarray(fp16_valid, dtype=bool)
    lateral_error = np.abs(fp32_points[paired, :, 1] - fp16_points[paired, :, 1])
    confidence_error = np.abs(fp32_confidence[paired] - fp16_confidence[paired])
    return {
        "validity_agreement": float(np.mean(fp32_valid == fp16_valid)),
        "validity_mismatch_count": int(np.count_nonzero(fp32_valid != fp16_valid)),
        "paired_valid_frames": int(np.count_nonzero(paired)),
        "lateral_error_m": _summary(lateral_error),
        "confidence_abs_error": _summary(confidence_error),
    }


def _ratio(candidate: Any, reference: Any) -> float | None:
    if candidate is None or reference in (None, 0):
        return None
    return float(candidate / reference)


def memory_ratios(
    fp32: dict[str, Any], fp16: dict[str, Any]
) -> dict[str, float | None]:
    return {
        "floating_parameter_ratio": _ratio(
            fp16.get("floating_parameter_bytes"),
            fp32.get("floating_parameter_bytes"),
        ),
        "activation_peak_allocated_ratio": _ratio(
            fp16.get("activation_peak_allocated_bytes"),
            fp32.get("activation_peak_allocated_bytes"),
        ),
        "overall_peak_allocated_ratio": _ratio(
            fp16.get("overall_peak_allocated_bytes"),
            fp32.get("overall_peak_allocated_bytes"),
        ),
    }


def _load_report(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    runs = payload.get("runs")
    if not isinstance(runs, list) or len(runs) != 1:
        raise ValueError(f"expected exactly one capture run in {path}")
    return runs[0]


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


def _passes_at_most(value: float | None, maximum: float) -> bool:
    return value is not None and value <= maximum


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fp32-npz", type=Path, required=True)
    parser.add_argument("--fp16-npz", type=Path, required=True)
    parser.add_argument("--fp32-report", type=Path, required=True)
    parser.add_argument("--fp16-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    with (
        np.load(args.fp32_npz.expanduser().resolve()) as fp32_data,
        np.load(args.fp16_npz.expanduser().resolve()) as fp16_data,
    ):
        for key in ("segment_ids", "sequences", "source_timestamps_ns"):
            if not np.array_equal(fp32_data[key], fp16_data[key]):
                raise ValueError(f"capture identity differs for {key}")
        masks = compare_masks(fp32_data["masks"], fp16_data["masks"])
        raw_paths = compare_paths(
            fp32_data["path_valid"],
            fp32_data["path_points"],
            fp32_data["path_confidence"],
            fp16_data["path_valid"],
            fp16_data["path_points"],
            fp16_data["path_confidence"],
        )
        smoothed_paths = compare_paths(
            fp32_data["smoothed_path_valid"],
            fp32_data["smoothed_path_points"],
            fp32_data["smoothed_path_confidence"],
            fp16_data["smoothed_path_valid"],
            fp16_data["smoothed_path_points"],
            fp16_data["smoothed_path_confidence"],
        )

    fp32_run = _load_report(args.fp32_report.expanduser().resolve())
    fp16_run = _load_report(args.fp16_report.expanduser().resolve())
    memory = memory_ratios(fp32_run["memory"], fp16_run["memory"])
    gates = {
        "selected_mask_agreement": masks["selected_mask_agreement"] >= 0.99,
        "road_iou": masks["road_iou"] >= 0.99,
        "sidewalk_iou": masks["sidewalk_iou"] >= 0.99,
        "smoothed_path_validity": smoothed_paths["validity_agreement"] == 1.0,
        "smoothed_path_lateral_p95": _passes_at_most(
            smoothed_paths["lateral_error_m"]["p95"], 0.10
        ),
        "floating_parameter_ratio": _passes_at_most(
            memory["floating_parameter_ratio"], 0.52
        ),
        "activation_peak_ratio": _passes_at_most(
            memory["activation_peak_allocated_ratio"], 0.75
        ),
    }
    report = {
        "schema_version": 1,
        "fp32_profile": fp32_run["profile"],
        "fp16_profile": fp16_run["profile"],
        "masks": masks,
        "raw_paths": raw_paths,
        "smoothed_paths": smoothed_paths,
        "memory_ratios": memory,
        "acceptance_gates": gates,
        "verdict": "PASS" if all(gates.values()) else "FAIL",
    }
    output = args.output.expanduser().resolve()
    _atomic_json(output, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    print(f"PRECISION_COMPARISON_WRITTEN path={output}")
    return 0 if all(gates.values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
