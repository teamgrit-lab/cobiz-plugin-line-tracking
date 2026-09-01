#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = [
#   "numpy==2.5.2",
#   "opencv-python-headless==5.0.0.93",
# ]
# ///
"""Sweep cheap spatial refinements on retained R50/Swin-L comparison clips.

The retained comparison videos contain a Swin-L overlay on the left and an
R50 overlay on the right.  This tool reconstructs their three-valued masks by
comparing each panel with the original frame, checks the reconstruction
against the retained JSON metrics, then tests whether small R50 Road islands
surrounded by Sidewalk should be dropped or reassigned.
"""

from __future__ import annotations

import argparse
import json
import os
import tempfile
from collections import defaultdict
from pathlib import Path
from typing import Any

import cv2
import numpy as np


ROAD_COLOR = np.array([0.0, 255.0, 0.0], dtype=np.float32)
SIDEWALK_COLOR = np.array([255.0, 0.0, 255.0], dtype=np.float32)
OVERLAY_ALPHA = 0.48


def parse_args() -> argparse.Namespace:
    repository = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--comparison-dir",
        type=Path,
        default=(
            repository
            / "rosbag-results"
            / "full-profile-comparison-20260831"
            / "sampled-validation"
            / "current-vs-swin"
        ),
    )
    parser.add_argument(
        "--output-report",
        type=Path,
        default=(
            repository
            / "rosbag-results"
            / "r50-swin-experiments-20260901"
            / "postprocess-mask-sweep-20260901.json"
        ),
    )
    parser.add_argument("--progress-every", type=int, default=100)
    return parser.parse_args()


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
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


def reconstruct_mask(frame: np.ndarray, overlay: np.ndarray) -> np.ndarray:
    source = frame.astype(np.float32)
    expected = np.stack(
        (
            source,
            source * (1.0 - OVERLAY_ALPHA) + ROAD_COLOR * OVERLAY_ALPHA,
            source * (1.0 - OVERLAY_ALPHA) + SIDEWALK_COLOR * OVERLAY_ALPHA,
        ),
        axis=0,
    )
    distances = np.mean(
        np.abs(expected - overlay.astype(np.float32)[None, ...]), axis=-1
    )
    selected = distances.argmin(axis=0).astype(np.uint8)

    # render_overlay adds a legend and frame/profile text.  Exclude those
    # pixels so the sweep measures only segmentation content.
    height, width = selected.shape
    panel_width = max(280, int(round(width * 0.43)))
    panel_height = max(105, int(round(height * 0.29)))
    selected[:panel_height, :panel_width] = 0
    selected[max(0, height - 34) :, : max(340, int(width * 0.55))] = 0
    return selected


def binary_iou(left: np.ndarray, right: np.ndarray) -> float:
    union = int(np.count_nonzero(left | right))
    return float(np.count_nonzero(left & right) / union) if union else 1.0


def refine_road_islands(
    selected: np.ndarray,
    *,
    maximum_area: int,
    minimum_sidewalk_ring_ratio: float,
    action: str,
) -> np.ndarray:
    refined = selected.copy()
    road = selected == 1
    sidewalk = selected == 2
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        road.astype(np.uint8), connectivity=8
    )
    ring_kernel = np.ones((5, 5), dtype=np.uint8)
    for component in range(1, count):
        area = int(stats[component, cv2.CC_STAT_AREA])
        if area > maximum_area:
            continue
        component_mask = labels == component
        ring = cv2.dilate(component_mask.astype(np.uint8), ring_kernel).astype(bool)
        ring &= ~component_mask
        ring_size = int(np.count_nonzero(ring))
        sidewalk_ratio = (
            float(np.count_nonzero(ring & sidewalk) / ring_size) if ring_size else 0.0
        )
        if sidewalk_ratio < minimum_sidewalk_ring_ratio:
            continue
        refined[component_mask] = 2 if action == "reassign-sidewalk" else 0
    return refined


def summarize(rows: list[dict[str, float]]) -> dict[str, float]:
    return {
        key: float(np.mean([row[key] for row in rows]))
        for key in rows[0]
    }


def metrics(candidate: np.ndarray, reference: np.ndarray) -> dict[str, float]:
    return {
        "selected_label_agreement": float(np.mean(candidate == reference)),
        "road_iou": binary_iou(candidate == 1, reference == 1),
        "sidewalk_iou": binary_iou(candidate == 2, reference == 2),
        "road_area_ratio": float(np.mean(candidate == 1)),
        "sidewalk_area_ratio": float(np.mean(candidate == 2)),
    }


def main() -> int:
    args = parse_args()
    comparison_dir = args.comparison_dir.expanduser().resolve()
    reports = sorted(comparison_dir.glob("*-comparison.json"))
    if not reports:
        raise RuntimeError(f"no comparison JSON files found in {comparison_dir}")
    repository = Path(__file__).resolve().parents[1]

    candidates = [("baseline", 0, 0.0, "none")]
    for action in ("drop", "reassign-sidewalk"):
        for maximum_area in (160, 640, 2560):
            for ring_ratio in (0.10, 0.50):
                candidates.append(
                    (
                        f"{action}-road-{maximum_area}-ring-{ring_ratio:.2f}",
                        maximum_area,
                        ring_ratio,
                        action,
                    )
                )

    rows: dict[str, list[dict[str, float]]] = defaultdict(list)
    reconstruction_rows: list[dict[str, float]] = []
    skipped_reports: list[str] = []
    processed = 0
    for report_path in reports:
        report = json.loads(report_path.read_text(encoding="utf-8"))
        input_path = Path(report["input"])
        if not input_path.is_file():
            fallback = repository / "rosbag-results" / "full" / input_path.name
            if fallback.is_file():
                input_path = fallback
        comparison_path = Path(report["output_video"])
        source = cv2.VideoCapture(str(input_path))
        comparison = cv2.VideoCapture(str(comparison_path))
        if not source.isOpened() or not comparison.isOpened():
            raise RuntimeError(f"could not open inputs for {report_path}")
        source.set(cv2.CAP_PROP_POS_FRAMES, int(report["start_frame"]))
        clip_metrics: list[dict[str, float]] = []
        try:
            for _ in range(int(report["processed_frames"])):
                source_ok, frame = source.read()
                comparison_ok, panels = comparison.read()
                if not source_ok or not comparison_ok:
                    break
                height, width = frame.shape[:2]
                if panels.shape[:2] != (height, width * 2):
                    raise RuntimeError(
                        f"unexpected comparison size {panels.shape[:2]} for "
                        f"source {(height, width)}"
                    )
                left = panels[:, :width]
                right = panels[:, width:]
                if (height, width) != (360, 640):
                    frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
                    left = cv2.resize(left, (640, 360), interpolation=cv2.INTER_AREA)
                    right = cv2.resize(right, (640, 360), interpolation=cv2.INTER_AREA)
                swin = reconstruct_mask(frame, left)
                r50 = reconstruct_mask(frame, right)
                clip_metrics.append(metrics(r50, swin))
                for name, maximum_area, ring_ratio, action in candidates:
                    candidate = (
                        r50
                        if action == "none"
                        else refine_road_islands(
                            r50,
                            maximum_area=maximum_area,
                            minimum_sidewalk_ring_ratio=ring_ratio,
                            action=action,
                        )
                    )
                    rows[name].append(metrics(candidate, swin))
                processed += 1
                if processed % args.progress_every == 0:
                    print(f"SWEEP_PROGRESS frames={processed}", flush=True)
        finally:
            source.release()
            comparison.release()
        if not clip_metrics:
            skipped_reports.append(str(report_path))
            continue
        reconstructed = summarize(clip_metrics)
        retained = report["cross_profile"]
        reconstruction_rows.append(
            {
                "agreement_error": abs(
                    reconstructed["selected_label_agreement"]
                    - retained["mean_selected_label_agreement"]
                ),
                "road_iou_error": abs(
                    reconstructed["road_iou"] - retained["mean_road_iou"]
                ),
                "sidewalk_iou_error": abs(
                    reconstructed["sidewalk_iou"] - retained["mean_sidewalk_iou"]
                ),
            }
        )

    summaries = []
    for name, maximum_area, ring_ratio, action in candidates:
        summary = summarize(rows[name])
        summary.update(
            {
                "name": name,
                "action": action,
                "maximum_road_component_area": maximum_area,
                "minimum_sidewalk_ring_ratio": ring_ratio,
                "mean_surface_iou": (
                    summary["road_iou"] + summary["sidewalk_iou"]
                )
                / 2.0,
            }
        )
        summaries.append(summary)
    summaries.sort(
        key=lambda item: (
            item["mean_surface_iou"],
            item["selected_label_agreement"],
        ),
        reverse=True,
    )
    payload = {
        "schema_version": 1,
        "status": "complete",
        "source_comparison_directory": str(comparison_dir),
        "processed_frames": processed,
        "skipped_reports": skipped_reports,
        "reconstruction_validation": summarize(reconstruction_rows),
        "candidates": summaries,
        "best_candidate": summaries[0],
    }
    output = args.output_report.expanduser().resolve()
    atomic_write_json(output, payload)
    print(f"SWEEP_COMPLETE frames={processed} best={summaries[0]['name']}", flush=True)
    print(f"REPORT_WRITTEN path={output}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
