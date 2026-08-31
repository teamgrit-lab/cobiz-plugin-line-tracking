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
"""Evaluate a Mapillary-trained Road/Sidewalk model on representative frames."""

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
import torch
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
    resolve_label_id,
    select_sample_frames,
    summarize_metrics,
    utc_now,
)
from transformers import AutoImageProcessor, Mask2FormerForUniversalSegmentation

MODEL_ID = "facebook/mask2former-swin-large-mapillary-vistas-semantic"
MODEL_REVISION = "4772b6bf101d91f2534c106dc524d906aeb3c68a"


def binary_iou(left: np.ndarray, right: np.ndarray) -> float:
    union = int(np.count_nonzero(left | right))
    if union == 0:
        return 1.0
    return int(np.count_nonzero(left & right)) / union


def mean(values: list[float]) -> float | None:
    return float(np.mean(values)) if values else None


def semantic_prediction(
    processor: AutoImageProcessor,
    model: Mask2FormerForUniversalSegmentation,
    frame_bgr: np.ndarray,
    device: torch.device,
) -> tuple[np.ndarray, np.ndarray]:
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    inputs = processor(images=frame_rgb, return_tensors="pt")
    inputs = {name: value.to(device) for name, value in inputs.items()}
    with torch.inference_mode():
        outputs = model(**inputs)
        processed = processor.post_process_semantic_segmentation(
            outputs,
            target_sizes=[frame_bgr.shape[:2]],
            return_segmentation_scores=True,
        )[0]
    segmentation = processed["segmentation"].cpu().numpy()
    scores = processed["segmentation_scores"].cpu().numpy()
    return segmentation, scores


def load_cityscapes_reference(
    output_dir: Path, video_stem: str, frame_index: int
) -> np.ndarray | None:
    path = (
        output_dir
        / "experiments"
        / "candidate-segformer-b2-cityscapes"
        / "samples"
        / video_stem
        / f"frame-{frame_index:08d}-mask.png"
    )
    return cv2.imread(str(path), cv2.IMREAD_GRAYSCALE) if path.is_file() else None


def write_report(
    path: Path,
    *,
    summary: dict[str, Any],
    agreement: dict[str, Any],
    contact_sheet: Path,
) -> None:
    road = summary["road"]
    sidewalk = summary["sidewalk"]
    lines = [
        "# Mapillary Vistas complementary candidate",
        "",
        "Mask2Former trained on Mapillary Vistas was evaluated on the same "
        "representative frames as the retained Cityscapes B2 model.",
        "",
        f"- Sample frames: {summary['sample_frame_count']}",
        f"- Road detected: {road['detected_frame_ratio']:.1%}",
        f"- Sidewalk detected: {sidewalk['detected_frame_ratio']:.1%}",
        f"- Mean road score when present: {road['mean_confidence_when_present']}",
        "- Mean sidewalk score when present: "
        f"{sidewalk['mean_confidence_when_present']}",
        "- Mean Road IoU versus Cityscapes B2: "
        f"{agreement['mean_road_iou']}",
        "- Mean Sidewalk IoU versus Cityscapes B2: "
        f"{agreement['mean_sidewalk_iou']}",
        f"- Contact sheet: `{contact_sheet}`",
        "",
        "This candidate is an independent domain-shift check, not ground truth. "
        "It must be visually reviewed before it can replace the retained result.",
    ]
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
        "--experiment-id", default="candidate-mask2former-mapillary-vistas"
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
    samples_dir = experiment_dir / "samples"
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
        road_id = resolve_label_id(model.config.id2label, "road")
        sidewalk_id = resolve_label_id(model.config.id2label, "sidewalk")

        records: list[dict[str, Any]] = []
        agreement_rows: list[dict[str, float]] = []
        overlay_paths: list[Path] = []
        state = json.loads(state_path.read_text(encoding="utf-8"))
        state.update(
            {
                "status": "mapillary_candidate_running",
                "updated_at": utc_now(),
                "active_experiment_id": args.experiment_id,
                "active_model_id": args.model_id,
                "active_model_revision": args.model_revision,
            }
        )
        atomic_write_json(state_path, state)

        for video_number, video in enumerate(videos, start=1):
            if stop_requested:
                break
            capture = cv2.VideoCapture(video.path)
            try:
                if not capture.isOpened():
                    raise RuntimeError(f"could not open video: {video.path}")
                samples = select_sample_frames(
                    capture,
                    video.frame_count,
                    video.fps,
                    args.samples_per_video,
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
                road_mask = class_map == road_id
                sidewalk_mask = class_map == sidewalk_id
                selected = np.zeros(class_map.shape, dtype=np.uint8)
                selected[road_mask] = 1
                selected[sidewalk_mask] = 2
                minimum_area = max(48, int(selected.size * 0.00035))
                row: dict[str, Any] = {
                    "video": video.path,
                    "video_stem": video.stem,
                    "frame_index": frame_index,
                    "time_seconds": time_seconds,
                    "scene_change_score": scene_change,
                    "road": asdict(
                        class_metrics(
                            road_mask,
                            scores[road_id],
                            small_component_area=minimum_area,
                        )
                    ),
                    "sidewalk": asdict(
                        class_metrics(
                            sidewalk_mask,
                            scores[sidewalk_id],
                            small_component_area=minimum_area,
                        )
                    ),
                    "classes_are_disjoint": bool(not np.any(road_mask & sidewalk_mask)),
                }
                reference = load_cityscapes_reference(
                    output_dir, video.stem, frame_index
                )
                if reference is not None:
                    if reference.shape != selected.shape:
                        reference = cv2.resize(
                            reference,
                            (selected.shape[1], selected.shape[0]),
                            interpolation=cv2.INTER_NEAREST,
                        )
                    agreement = {
                        "road_iou": binary_iou(selected == 1, reference == 1),
                        "sidewalk_iou": binary_iou(selected == 2, reference == 2),
                    }
                    row["cityscapes_b2_agreement"] = agreement
                    agreement_rows.append(agreement)
                records.append(row)

                video_dir = samples_dir / video.stem
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
                    "mapillary_completed_video_count": video_number,
                    "mapillary_sample_frame_count": len(records),
                }
            )
            atomic_write_json(state_path, state)

        summary = summarize_metrics(records)
        agreement = {
            "compared_frame_count": len(agreement_rows),
            "mean_road_iou": mean([row["road_iou"] for row in agreement_rows]),
            "mean_sidewalk_iou": mean(
                [row["sidewalk_iou"] for row in agreement_rows]
            ),
        }
        make_contact_sheet(overlay_paths, contact_sheet, columns=4, tile_width=360)
        report = {
            "schema_version": 1,
            "experiment_id": args.experiment_id,
            "created_at": utc_now(),
            "completed": not stop_requested,
            "model": {
                "id": args.model_id,
                "revision": args.model_revision,
                "training_dataset": "Mapillary Vistas",
                "road_label_id": road_id,
                "sidewalk_label_id": sidewalk_id,
                "all_labels": {
                    str(key): value for key, value in model.config.id2label.items()
                },
            },
            "runtime": {
                "device": str(device),
                "python": platform.python_version(),
                "platform": platform.platform(),
                "packages": package_versions(),
            },
            "videos": [asdict(video) for video in videos],
            "summary": summary,
            "cityscapes_b2_agreement": agreement,
            "frames": records,
        }
        atomic_write_json(experiment_dir / "metrics.json", report)
        write_report(
            experiment_dir / "REPORT.md",
            summary=summary,
            agreement=agreement,
            contact_sheet=contact_sheet,
        )
        state.update(
            {
                "status": (
                    "mapillary_candidate_complete"
                    if not stop_requested
                    else "mapillary_candidate_interrupted"
                ),
                "updated_at": utc_now(),
                "mapillary_candidate_report": str(experiment_dir / "REPORT.md"),
                "mapillary_candidate_metrics": str(experiment_dir / "metrics.json"),
                "mapillary_candidate_contact_sheet": str(contact_sheet),
                "next_action": (
                    "Visually compare the Mapillary candidate with the retained "
                    "B2 contact sheet and keep only a genuine semantic improvement."
                ),
                "success": False,
            }
        )
        atomic_write_json(state_path, state)
    return 130 if stop_requested else 0


if __name__ == "__main__":
    raise SystemExit(main())
