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
"""Test a binary Road/Sidewalk aggregation of Mapillary surface subclasses."""

from __future__ import annotations

import argparse
import json
import platform
import signal
from dataclasses import asdict
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from evaluate_sidewalk_road_mapillary import (
    MODEL_ID,
    MODEL_REVISION,
    binary_iou,
    semantic_prediction,
)
from segment_sidewalk_road import (
    RunLock,
    atomic_write_json,
    choose_device,
    class_metrics,
    discover_videos,
    inspect_video,
    make_contact_sheet,
    package_versions,
    render_overlay,
    select_sample_frames,
    summarize_metrics,
    utc_now,
)
from transformers import AutoImageProcessor, Mask2FormerForUniversalSegmentation

ROAD_LABELS = (
    "Road",
    "Bike Lane",
    "Crosswalk - Plain",
    "Parking",
    "Service Lane",
    "Lane Marking - Crosswalk",
    "Lane Marking - General",
)
SIDEWALK_LABELS = ("Sidewalk", "Pedestrian Area", "Curb Cut")


def resolve_ids(labels: dict[int, str] | dict[str, str], names: tuple[str, ...]) -> list[int]:
    normalized = {str(label).strip().lower(): int(raw_id) for raw_id, label in labels.items()}
    missing = [name for name in names if name.lower() not in normalized]
    if missing:
        raise RuntimeError(f"model is missing aggregation labels: {missing}")
    return [normalized[name.lower()] for name in names]


def load_canonical_mask(
    output_dir: Path, video_stem: str, frame_index: int
) -> np.ndarray | None:
    path = (
        output_dir
        / "experiments"
        / "candidate-mask2former-mapillary-vistas"
        / "samples"
        / video_stem
        / f"frame-{frame_index:08d}-mask.png"
    )
    return cv2.imread(str(path), cv2.IMREAD_GRAYSCALE) if path.is_file() else None


def mean(values: list[float]) -> float | None:
    return float(np.mean(values)) if values else None


def write_report(
    path: Path,
    *,
    summary: dict[str, Any],
    comparison: dict[str, Any],
    label_coverage: dict[str, float],
    contact_sheet: Path,
) -> None:
    lines = [
        "# Mapillary surface-label aggregation",
        "",
        "Road combines Road, Bike Lane, plain Crosswalk, Parking, Service Lane, "
        "and lane markings. Sidewalk combines Sidewalk, Pedestrian Area, and "
        "Curb Cut. Curb itself remains excluded.",
        "",
        f"- Sample frames: {summary['sample_frame_count']}",
        f"- Road detected: {summary['road']['detected_frame_ratio']:.1%}",
        f"- Sidewalk detected: {summary['sidewalk']['detected_frame_ratio']:.1%}",
        "- Median added Road area over canonical labels: "
        f"{comparison['median_added_road_area_ratio']:.4f}",
        "- Mean added Road area over canonical labels: "
        f"{comparison['mean_added_road_area_ratio']:.4f}",
        "- Median added Sidewalk area over canonical labels: "
        f"{comparison['median_added_sidewalk_area_ratio']:.4f}",
        "- Mean added Sidewalk area over canonical labels: "
        f"{comparison['mean_added_sidewalk_area_ratio']:.4f}",
        f"- Mean Road IoU with canonical labels: {comparison['mean_road_iou']}",
        "- Mean Sidewalk IoU with canonical labels: "
        f"{comparison['mean_sidewalk_iou']}",
        f"- Contact sheet: `{contact_sheet}`",
        "",
        "## Mean source-label area",
        "",
    ]
    lines.extend(
        f"- {label}: {area:.5f}" for label, area in sorted(label_coverage.items())
    )
    lines.extend(
        [
            "",
            "This is a coverage experiment, not an automatic promotion. Visual "
            "review must reject additions that land on non-traversable objects.",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    repository = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-dir", type=Path, default=repository / "rosbag-results" / "full"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repository / "rosbag-results" / "sidewalk-road-results",
    )
    parser.add_argument(
        "--experiment-id",
        default="candidate-mask2former-mapillary-surface-aggregate",
    )
    parser.add_argument("--model-id", default=MODEL_ID)
    parser.add_argument("--model-revision", default=MODEL_REVISION)
    parser.add_argument("--samples-per-video", type=int, default=7)
    parser.add_argument("--device", default="auto")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.samples_per_video < 3:
        raise ValueError("--samples-per-video must be at least 3")
    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    experiment_dir = output_dir / "experiments" / args.experiment_id
    contact_sheet = experiment_dir / "contact-sheets" / "all-videos.jpg"
    state_path = output_dir / "state.json"
    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    with RunLock(output_dir / "run.lock"):
        videos = [inspect_video(path) for path in discover_videos(input_dir)]
        device = choose_device(args.device)
        processor = AutoImageProcessor.from_pretrained(
            args.model_id, revision=args.model_revision
        )
        model = Mask2FormerForUniversalSegmentation.from_pretrained(
            args.model_id, revision=args.model_revision
        ).to(device)
        model.eval()
        road_ids = resolve_ids(model.config.id2label, ROAD_LABELS)
        sidewalk_ids = resolve_ids(model.config.id2label, SIDEWALK_LABELS)
        all_group_labels = ROAD_LABELS + SIDEWALK_LABELS
        all_group_ids = road_ids + sidewalk_ids

        state = json.loads(state_path.read_text(encoding="utf-8"))
        state.update(
            {
                "status": "mapillary_label_aggregation_running",
                "updated_at": utc_now(),
                "active_experiment_id": args.experiment_id,
                "next_action": "Finish the current Mapillary aggregation checkpoint.",
            }
        )
        atomic_write_json(state_path, state)

        records: list[dict[str, Any]] = []
        comparison_rows: list[dict[str, float]] = []
        label_areas: dict[str, list[float]] = {label: [] for label in all_group_labels}
        overlay_paths: list[Path] = []
        for video_number, video in enumerate(videos, start=1):
            if stop_requested:
                break
            capture = cv2.VideoCapture(video.path)
            try:
                if not capture.isOpened():
                    raise RuntimeError(f"could not open video: {video.path}")
                samples = select_sample_frames(
                    capture, video.frame_count, video.fps, args.samples_per_video
                )
            finally:
                capture.release()

            video_overlays: list[Path] = []
            for frame_index, time_seconds, frame_bgr, scene_change in samples:
                if stop_requested:
                    break
                class_map, scores = semantic_prediction(
                    processor, model, frame_bgr, device
                )
                road_mask = np.isin(class_map, road_ids)
                sidewalk_mask = np.isin(class_map, sidewalk_ids)
                selected = np.zeros(class_map.shape, dtype=np.uint8)
                selected[road_mask] = 1
                selected[sidewalk_mask] = 2
                road_confidence = scores[road_ids].max(axis=0)
                sidewalk_confidence = scores[sidewalk_ids].max(axis=0)
                minimum_area = max(48, int(selected.size * 0.00035))
                contributions = {
                    label: float(np.mean(class_map == label_id))
                    for label, label_id in zip(all_group_labels, all_group_ids, strict=True)
                }
                for label, area in contributions.items():
                    label_areas[label].append(area)
                row: dict[str, Any] = {
                    "video": video.path,
                    "video_stem": video.stem,
                    "frame_index": frame_index,
                    "time_seconds": time_seconds,
                    "scene_change_score": scene_change,
                    "road": asdict(
                        class_metrics(
                            road_mask,
                            road_confidence,
                            small_component_area=minimum_area,
                        )
                    ),
                    "sidewalk": asdict(
                        class_metrics(
                            sidewalk_mask,
                            sidewalk_confidence,
                            small_component_area=minimum_area,
                        )
                    ),
                    "source_label_area_ratios": contributions,
                    "classes_are_disjoint": bool(not np.any(road_mask & sidewalk_mask)),
                }
                canonical = load_canonical_mask(output_dir, video.stem, frame_index)
                if canonical is not None:
                    if canonical.shape != selected.shape:
                        canonical = cv2.resize(
                            canonical,
                            (selected.shape[1], selected.shape[0]),
                            interpolation=cv2.INTER_NEAREST,
                        )
                    comparison = {
                        "road_iou": binary_iou(selected == 1, canonical == 1),
                        "sidewalk_iou": binary_iou(selected == 2, canonical == 2),
                        "added_road_area_ratio": float(
                            np.mean((selected == 1) & (canonical != 1))
                        ),
                        "added_sidewalk_area_ratio": float(
                            np.mean((selected == 2) & (canonical != 2))
                        ),
                    }
                    row["canonical_comparison"] = comparison
                    comparison_rows.append(comparison)
                records.append(row)

                video_dir = experiment_dir / "samples" / video.stem
                video_dir.mkdir(parents=True, exist_ok=True)
                mask_path = video_dir / f"frame-{frame_index:08d}-mask.png"
                overlay_path = video_dir / f"frame-{frame_index:08d}-overlay.jpg"
                if not cv2.imwrite(str(mask_path), selected):
                    raise RuntimeError(f"could not write {mask_path}")
                overlay = render_overlay(
                    frame_bgr, selected, frame_index=frame_index, fps=video.fps
                )
                if not cv2.imwrite(
                    str(overlay_path), overlay, [cv2.IMWRITE_JPEG_QUALITY, 91]
                ):
                    raise RuntimeError(f"could not write {overlay_path}")
                video_overlays.append(overlay_path)
                overlay_paths.append(overlay_path)

            make_contact_sheet(
                video_overlays,
                experiment_dir / "contact-sheets" / f"{video.stem}.jpg",
                columns=2,
                tile_width=480,
            )
            state.update(
                {
                    "updated_at": utc_now(),
                    "aggregation_completed_video_count": video_number,
                    "aggregation_sample_frame_count": len(records),
                }
            )
            atomic_write_json(state_path, state)

        summary = summarize_metrics(records)
        comparison = {
            "compared_frame_count": len(comparison_rows),
            "mean_road_iou": mean([row["road_iou"] for row in comparison_rows]),
            "mean_sidewalk_iou": mean(
                [row["sidewalk_iou"] for row in comparison_rows]
            ),
            "median_added_road_area_ratio": float(
                np.median([row["added_road_area_ratio"] for row in comparison_rows])
            ),
            "mean_added_road_area_ratio": mean(
                [row["added_road_area_ratio"] for row in comparison_rows]
            ),
            "median_added_sidewalk_area_ratio": float(
                np.median(
                    [row["added_sidewalk_area_ratio"] for row in comparison_rows]
                )
            ),
            "mean_added_sidewalk_area_ratio": mean(
                [row["added_sidewalk_area_ratio"] for row in comparison_rows]
            ),
        }
        label_coverage = {label: mean(areas) or 0.0 for label, areas in label_areas.items()}
        make_contact_sheet(overlay_paths, contact_sheet, columns=4, tile_width=360)
        report = {
            "schema_version": 1,
            "experiment_id": args.experiment_id,
            "created_at": utc_now(),
            "completed": not stop_requested,
            "model": {"id": args.model_id, "revision": args.model_revision},
            "aggregation": {
                "road_labels": list(ROAD_LABELS),
                "road_label_ids": road_ids,
                "sidewalk_labels": list(SIDEWALK_LABELS),
                "sidewalk_label_ids": sidewalk_ids,
            },
            "runtime": {
                "device": str(device),
                "python": platform.python_version(),
                "platform": platform.platform(),
                "packages": package_versions(),
            },
            "videos": [asdict(video) for video in videos],
            "summary": summary,
            "canonical_comparison": comparison,
            "mean_source_label_area_ratios": label_coverage,
            "frames": records,
        }
        atomic_write_json(experiment_dir / "metrics.json", report)
        write_report(
            experiment_dir / "REPORT.md",
            summary=summary,
            comparison=comparison,
            label_coverage=label_coverage,
            contact_sheet=contact_sheet,
        )
        state.update(
            {
                "status": (
                    "mapillary_label_aggregation_complete"
                    if not stop_requested
                    else "mapillary_label_aggregation_interrupted"
                ),
                "updated_at": utc_now(),
                "aggregation_metrics": str(experiment_dir / "metrics.json"),
                "aggregation_report": str(experiment_dir / "REPORT.md"),
                "aggregation_contact_sheet": str(contact_sheet),
                "next_action": (
                    "Visually review the aggregation contact sheet and run a "
                    "temporal comparison only if the added coverage is semantically sound."
                ),
                "success": False,
            }
        )
        atomic_write_json(state_path, state)
    return 130 if stop_requested else 0


if __name__ == "__main__":
    raise SystemExit(main())
