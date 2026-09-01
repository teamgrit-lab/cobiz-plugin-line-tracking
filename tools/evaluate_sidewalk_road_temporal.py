#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = [
#   "numpy==2.5.2",
#   "opencv-python-headless==5.0.0.93",
#   "pillow==12.3.0",
#   "torch==2.13.0",
#   "torchvision==0.28.0",
#   "transformers==5.16.1",
# ]
# ///
"""Measure Road/Sidewalk segmentation stability on consecutive video frames."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
from dataclasses import asdict
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
from segment_sidewalk_road import (
    VIDEO_EXTENSIONS,
    RunLock,
    atomic_write_json,
    choose_device,
    class_metrics,
    discover_videos,
    inspect_video,
    make_contact_sheet,
    render_overlay,
    resolve_label_id,
    utc_now,
)
from torch.nn import functional
from transformers import AutoImageProcessor, SegformerForSemanticSegmentation

TEMPORAL_MODEL_ID = "nvidia/segformer-b2-finetuned-cityscapes-1024-1024"
TEMPORAL_MODEL_REVISION = "2416842a88764bd96f8dc5c7dbacd79b1aca2918"


def collect_bursts(
    path: Path,
    *,
    frame_count: int,
    fps: float,
    burst_count: int,
    burst_length: int,
) -> list[list[tuple[int, float, np.ndarray]]]:
    centers = np.linspace(0.25, 0.75, burst_count)
    starts = [
        max(
            0,
            min(
                frame_count - burst_length,
                round(frame_count * float(center) - burst_length / 2),
            ),
        )
        for center in centers
    ]
    wanted: dict[int, int] = {}
    for burst_id, start in enumerate(starts):
        for index in range(start, start + burst_length):
            wanted[index] = burst_id

    bursts: list[list[tuple[int, float, np.ndarray]]] = [[] for _ in range(burst_count)]
    capture = cv2.VideoCapture(str(path))
    try:
        if not capture.isOpened():
            raise RuntimeError(f"could not open video: {path}")
        index = 0
        final_wanted = max(wanted)
        while index <= final_wanted:
            ok, frame = capture.read()
            if not ok or frame is None:
                break
            burst_id = wanted.get(index)
            if burst_id is not None:
                timestamp = float(capture.get(cv2.CAP_PROP_POS_MSEC)) / 1000.0
                if timestamp < 0:
                    timestamp = index / fps
                bursts[burst_id].append((index, timestamp, frame.copy()))
            index += 1
    finally:
        capture.release()
    for burst_id, burst in enumerate(bursts):
        if len(burst) < max(3, burst_length // 2):
            raise RuntimeError(
                f"decoded only {len(burst)} frames for burst {burst_id} in {path}"
            )
    return bursts


def remove_small_components(mask: np.ndarray, minimum_area: int) -> np.ndarray:
    binary = mask.astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    retained = stats[:, cv2.CC_STAT_AREA] >= minimum_area
    retained[0] = False
    return retained[labels]


def selected_mask(
    class_map: np.ndarray,
    *,
    road_id: int,
    sidewalk_id: int,
    minimum_area: int | None = None,
) -> np.ndarray:
    road = class_map == road_id
    sidewalk = class_map == sidewalk_id
    if minimum_area is not None:
        road = remove_small_components(road, minimum_area)
        sidewalk = remove_small_components(sidewalk, minimum_area)
    selected = np.zeros(class_map.shape, dtype=np.uint8)
    selected[road] = 1
    selected[sidewalk] = 2
    return selected


def binary_iou(left: np.ndarray, right: np.ndarray) -> float:
    union = int(np.count_nonzero(left | right))
    if union == 0:
        return 1.0
    return int(np.count_nonzero(left & right)) / union


def pair_metrics(previous: np.ndarray, current: np.ndarray) -> dict[str, float]:
    return {
        "selected_label_change_ratio": float(np.mean(previous != current)),
        "road_change_ratio": float(np.mean((previous == 1) != (current == 1))),
        "sidewalk_change_ratio": float(np.mean((previous == 2) != (current == 2))),
        "road_iou": binary_iou(previous == 1, current == 1),
        "sidewalk_iou": binary_iou(previous == 2, current == 2),
        "road_area_jump": abs(float(np.mean(previous == 1) - np.mean(current == 1))),
        "sidewalk_area_jump": abs(
            float(np.mean(previous == 2) - np.mean(current == 2))
        ),
    }


def mean(values: list[float]) -> float:
    return float(np.mean(values)) if values else 0.0


def summarize_candidate(
    frame_rows: list[dict[str, Any]], pair_rows: list[dict[str, float]]
) -> dict[str, Any]:
    return {
        "frame_count": len(frame_rows),
        "pair_count": len(pair_rows),
        "mean_selected_label_change_ratio": mean(
            [row["selected_label_change_ratio"] for row in pair_rows]
        ),
        "mean_road_change_ratio": mean([row["road_change_ratio"] for row in pair_rows]),
        "mean_sidewalk_change_ratio": mean(
            [row["sidewalk_change_ratio"] for row in pair_rows]
        ),
        "mean_road_iou": mean([row["road_iou"] for row in pair_rows]),
        "mean_sidewalk_iou": mean([row["sidewalk_iou"] for row in pair_rows]),
        "max_road_area_jump": max(
            (row["road_area_jump"] for row in pair_rows), default=0.0
        ),
        "max_sidewalk_area_jump": max(
            (row["sidewalk_area_jump"] for row in pair_rows), default=0.0
        ),
        "mean_road_small_component_noise": mean(
            [row["road"]["small_component_pixel_ratio"] for row in frame_rows]
        ),
        "mean_sidewalk_small_component_noise": mean(
            [row["sidewalk"]["small_component_pixel_ratio"] for row in frame_rows]
        ),
    }


def validate_video(path: Path, expected_frames: int) -> dict[str, Any]:
    ffprobe = shutil.which("ffprobe")
    if ffprobe is None:
        raise RuntimeError("ffprobe is required to validate preview video")
    completed = subprocess.run(
        [
            ffprobe,
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=width,height,avg_frame_rate,nb_frames,duration",
            "-of",
            "json",
            str(path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    streams = json.loads(completed.stdout).get("streams", [])
    if not streams:
        raise RuntimeError(f"preview has no video stream: {path}")
    stream = streams[0]
    actual_frames = int(stream.get("nb_frames", 0))
    if actual_frames != expected_frames:
        raise RuntimeError(
            f"preview frame mismatch: expected {expected_frames}, got {actual_frames}"
        )
    capture = cv2.VideoCapture(str(path))
    try:
        ok, first = capture.read()
        if not ok or first is None:
            raise RuntimeError("preview first frame is not decodable")
        capture.set(cv2.CAP_PROP_POS_FRAMES, expected_frames - 1)
        ok, last = capture.read()
        if not ok or last is None:
            raise RuntimeError("preview last frame is not decodable")
    finally:
        capture.release()
    return stream


def write_report(
    path: Path,
    *,
    raw: dict[str, Any],
    filtered: dict[str, Any],
    selected: str,
    preview_path: Path,
) -> None:
    preservation = filtered["mean_raw_preservation_iou"]
    lines = [
        "# Consecutive-frame temporal validation",
        "",
        f"Selected preview candidate: `{selected}`.",
        "",
        "| Metric | Raw B2 | Temporal + spatial |",
        "|---|---:|---:|",
        (
            "| Selected-label change | "
            f"{raw['mean_selected_label_change_ratio']:.5f} | "
            f"{filtered['mean_selected_label_change_ratio']:.5f} |"
        ),
        (
            "| Road adjacent IoU | "
            f"{raw['mean_road_iou']:.5f} | {filtered['mean_road_iou']:.5f} |"
        ),
        (
            "| Sidewalk adjacent IoU | "
            f"{raw['mean_sidewalk_iou']:.5f} | "
            f"{filtered['mean_sidewalk_iou']:.5f} |"
        ),
        (
            "| Sidewalk small-component noise | "
            f"{raw['mean_sidewalk_small_component_noise']:.5f} | "
            f"{filtered['mean_sidewalk_small_component_noise']:.5f} |"
        ),
        "",
        f"Mean filtered-to-raw semantic preservation IoU: {preservation:.5f}.",
        f"Fixed preview video: `{preview_path}`.",
        "",
        "This experiment measures short-horizon stability, not semantic ground-truth "
        "accuracy. The retained candidate still needs broader domain-shift checks.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    repository = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=repository / "rosbag-results" / "full",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repository / "rosbag-results" / "sidewalk-road-results",
    )
    parser.add_argument("--model-id", default=TEMPORAL_MODEL_ID)
    parser.add_argument("--model-revision", default=TEMPORAL_MODEL_REVISION)
    parser.add_argument("--bursts-per-video", type=int, default=2)
    parser.add_argument("--burst-length", type=int, default=15)
    parser.add_argument("--temporal-alpha", type=float, default=0.72)
    parser.add_argument("--device", default="auto")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.bursts_per_video <= 0 or args.burst_length < 3:
        raise ValueError("burst counts must be positive and burst length at least 3")
    if not 0.5 <= args.temporal_alpha <= 1.0:
        raise ValueError("--temporal-alpha must be in [0.5, 1.0]")

    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    experiment_dir = output_dir / "temporal-validation"
    staging_dir = output_dir / ".staging"
    current_dir = output_dir / "current"
    staging_dir.mkdir(parents=True, exist_ok=True)
    current_dir.mkdir(parents=True, exist_ok=True)
    preview_staging = staging_dir / "best-temporal-preview.tmp.mp4"
    preview_final = current_dir / "best-temporal-preview.mp4"
    for path in staging_dir.iterdir():
        if path.is_file() and path.suffix.lower() in VIDEO_EXTENSIONS:
            path.unlink()

    with RunLock(output_dir / "run.lock"):
        videos = [inspect_video(path) for path in discover_videos(input_dir)]
        device = choose_device(args.device)
        processor = AutoImageProcessor.from_pretrained(
            args.model_id, revision=args.model_revision
        )
        model = SegformerForSemanticSegmentation.from_pretrained(
            args.model_id, revision=args.model_revision
        ).to(device)
        model.eval()
        road_id = resolve_label_id(model.config.id2label, "road")
        sidewalk_id = resolve_label_id(model.config.id2label, "sidewalk")

        raw_frames: list[dict[str, Any]] = []
        filtered_frames: list[dict[str, Any]] = []
        raw_pairs: list[dict[str, float]] = []
        filtered_pairs: list[dict[str, float]] = []
        preservation_ious: list[float] = []
        review_paths: list[Path] = []
        preview_frames: list[tuple[np.ndarray, np.ndarray, int, float, str]] = []

        for video in videos:
            bursts = collect_bursts(
                Path(video.path),
                frame_count=video.frame_count,
                fps=video.fps,
                burst_count=args.bursts_per_video,
                burst_length=args.burst_length,
            )
            for burst_id, burst in enumerate(bursts):
                previous_logits: torch.Tensor | None = None
                previous_raw: np.ndarray | None = None
                previous_filtered: np.ndarray | None = None
                for frame_position, (frame_index, time_seconds, frame_bgr) in enumerate(
                    burst
                ):
                    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                    inputs = processor(images=frame_rgb, return_tensors="pt")
                    inputs = {key: value.to(device) for key, value in inputs.items()}
                    with torch.inference_mode():
                        logits = model(**inputs).logits
                        logits = functional.interpolate(
                            logits,
                            size=frame_bgr.shape[:2],
                            mode="bilinear",
                            align_corners=False,
                        )[0]
                        raw_probabilities = logits.softmax(dim=0)
                        raw_map = raw_probabilities.argmax(dim=0).cpu().numpy()
                        if previous_logits is None:
                            smooth_logits = logits
                        else:
                            smooth_logits = (
                                args.temporal_alpha * logits
                                + (1.0 - args.temporal_alpha) * previous_logits
                            )
                        previous_logits = smooth_logits.detach()
                        smooth_probabilities = smooth_logits.softmax(dim=0)
                        smooth_map = smooth_probabilities.argmax(dim=0).cpu().numpy()
                        raw_road_confidence = raw_probabilities[road_id].cpu().numpy()
                        raw_sidewalk_confidence = (
                            raw_probabilities[sidewalk_id].cpu().numpy()
                        )
                        smooth_road_confidence = (
                            smooth_probabilities[road_id].cpu().numpy()
                        )
                        smooth_sidewalk_confidence = (
                            smooth_probabilities[sidewalk_id].cpu().numpy()
                        )

                    minimum_area = max(48, int(raw_map.size * 0.00035))
                    raw_selected = selected_mask(
                        raw_map, road_id=road_id, sidewalk_id=sidewalk_id
                    )
                    filtered_selected = selected_mask(
                        smooth_map,
                        road_id=road_id,
                        sidewalk_id=sidewalk_id,
                        minimum_area=minimum_area,
                    )
                    raw_road = raw_selected == 1
                    raw_sidewalk = raw_selected == 2
                    filtered_road = filtered_selected == 1
                    filtered_sidewalk = filtered_selected == 2

                    common = {
                        "video": video.path,
                        "video_stem": video.stem,
                        "burst_id": burst_id,
                        "frame_index": frame_index,
                        "time_seconds": time_seconds,
                    }
                    raw_frames.append(
                        {
                            **common,
                            "road": asdict(
                                class_metrics(
                                    raw_road,
                                    raw_road_confidence,
                                    small_component_area=minimum_area,
                                )
                            ),
                            "sidewalk": asdict(
                                class_metrics(
                                    raw_sidewalk,
                                    raw_sidewalk_confidence,
                                    small_component_area=minimum_area,
                                )
                            ),
                        }
                    )
                    filtered_frames.append(
                        {
                            **common,
                            "road": asdict(
                                class_metrics(
                                    filtered_road,
                                    smooth_road_confidence,
                                    small_component_area=minimum_area,
                                )
                            ),
                            "sidewalk": asdict(
                                class_metrics(
                                    filtered_sidewalk,
                                    smooth_sidewalk_confidence,
                                    small_component_area=minimum_area,
                                )
                            ),
                        }
                    )
                    preservation_ious.extend(
                        [
                            binary_iou(raw_road, filtered_road),
                            binary_iou(raw_sidewalk, filtered_sidewalk),
                        ]
                    )
                    if previous_raw is not None and previous_filtered is not None:
                        raw_pairs.append(pair_metrics(previous_raw, raw_selected))
                        filtered_pairs.append(
                            pair_metrics(previous_filtered, filtered_selected)
                        )
                    previous_raw = raw_selected
                    previous_filtered = filtered_selected
                    preview_frames.append(
                        (
                            frame_bgr,
                            filtered_selected,
                            frame_index,
                            video.fps,
                            video.stem,
                        )
                    )

                    if frame_position in {0, len(burst) // 2, len(burst) - 1}:
                        raw_overlay = render_overlay(
                            frame_bgr,
                            raw_selected,
                            frame_index=frame_index,
                            fps=video.fps,
                        )
                        filtered_overlay = render_overlay(
                            frame_bgr,
                            filtered_selected,
                            frame_index=frame_index,
                            fps=video.fps,
                        )
                        comparison = np.concatenate(
                            (raw_overlay, filtered_overlay), axis=1
                        )
                        review_dir = experiment_dir / "review-frames"
                        review_dir.mkdir(parents=True, exist_ok=True)
                        review_path = review_dir / (
                            f"{video.stem}-burst-{burst_id}-frame-{frame_index:08d}.jpg"
                        )
                        if not cv2.imwrite(
                            str(review_path),
                            comparison,
                            [cv2.IMWRITE_JPEG_QUALITY, 90],
                        ):
                            raise RuntimeError(f"could not write {review_path}")
                        review_paths.append(review_path)

        raw_summary = summarize_candidate(raw_frames, raw_pairs)
        filtered_summary = summarize_candidate(filtered_frames, filtered_pairs)
        filtered_summary["mean_raw_preservation_iou"] = mean(preservation_ious)
        change_improved = (
            filtered_summary["mean_selected_label_change_ratio"]
            <= raw_summary["mean_selected_label_change_ratio"] * 0.98
        )
        noise_improved = (
            filtered_summary["mean_sidewalk_small_component_noise"]
            <= raw_summary["mean_sidewalk_small_component_noise"]
        )
        preservation_ok = filtered_summary["mean_raw_preservation_iou"] >= 0.85
        selected = (
            "temporal_spatial"
            if change_improved and noise_improved and preservation_ok
            else "raw"
        )

        writer = cv2.VideoWriter(
            str(preview_staging),
            cv2.VideoWriter_fourcc(*"mp4v"),
            10.0,
            (960, 540),
        )
        if not writer.isOpened():
            raise RuntimeError(f"could not create preview: {preview_staging}")
        try:
            for (
                frame_bgr,
                filtered_selected,
                frame_index,
                fps,
                video_stem,
            ) in preview_frames:
                overlay = render_overlay(
                    frame_bgr,
                    filtered_selected,
                    frame_index=frame_index,
                    fps=fps,
                )
                overlay = cv2.resize(overlay, (960, 540), interpolation=cv2.INTER_AREA)
                cv2.putText(
                    overlay,
                    video_stem[:64],
                    (18, 525),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                writer.write(overlay)
        finally:
            writer.release()
        preview_validation = validate_video(preview_staging, len(preview_frames))
        os.replace(preview_staging, preview_final)
        for path in current_dir.iterdir():
            if (
                path.is_file()
                and path.suffix.lower() in VIDEO_EXTENSIONS
                and path != preview_final
            ):
                path.unlink()

        make_contact_sheet(
            review_paths,
            experiment_dir / "temporal-comparison-contact-sheet.jpg",
            columns=2,
            tile_width=720,
        )
        report = {
            "schema_version": 1,
            "created_at": utc_now(),
            "model": {"id": args.model_id, "revision": args.model_revision},
            "settings": {
                "bursts_per_video": args.bursts_per_video,
                "burst_length": args.burst_length,
                "temporal_alpha": args.temporal_alpha,
            },
            "raw": {"summary": raw_summary, "frames": raw_frames, "pairs": raw_pairs},
            "temporal_spatial": {
                "summary": filtered_summary,
                "frames": filtered_frames,
                "pairs": filtered_pairs,
            },
            "selection": {
                "selected": selected,
                "change_improved": change_improved,
                "noise_improved": noise_improved,
                "preservation_ok": preservation_ok,
                "final_acceptance": False,
                "reason": (
                    "Short-horizon validation completed; semantic domain-shift "
                    "review remains before full-video acceptance."
                ),
            },
            "preview": {
                "path": str(preview_final),
                "validation": preview_validation,
            },
        }
        atomic_write_json(experiment_dir / "metrics.json", report)
        write_report(
            experiment_dir / "REPORT.md",
            raw=raw_summary,
            filtered=filtered_summary,
            selected=selected,
            preview_path=preview_final,
        )

        state_path = output_dir / "state.json"
        state = (
            json.loads(state_path.read_text(encoding="utf-8"))
            if state_path.is_file()
            else {}
        )
        state.update(
            {
                "status": "temporal_validation_complete",
                "updated_at": utc_now(),
                "current_best_preview_video": str(preview_final),
                "temporal_validation_metrics": str(experiment_dir / "metrics.json"),
                "temporal_validation_report": str(experiment_dir / "REPORT.md"),
                "temporal_candidate_selected": selected,
                "next_action": (
                    "Visually inspect the temporal comparison and preview, then "
                    "test a complementary segmentation model or labeled anchors "
                    "for domain-shift validation."
                ),
                "success": False,
            }
        )
        atomic_write_json(state_path, state)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
