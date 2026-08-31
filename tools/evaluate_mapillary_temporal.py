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
"""Validate Mapillary Mask2Former stability on consecutive frame bursts."""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
from dataclasses import asdict
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
import torch.nn.functional as torch_functional
from evaluate_mapillary_label_aggregation import ROAD_LABELS, SIDEWALK_LABELS, resolve_ids
from evaluate_sidewalk_road_temporal import (
    binary_iou,
    collect_bursts,
    mean,
    pair_metrics,
    remove_small_components,
    selected_mask,
    summarize_candidate,
    validate_video,
)
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
from transformers import AutoImageProcessor, Mask2FormerForUniversalSegmentation

MODEL_ID = "facebook/mask2former-swin-large-mapillary-vistas-semantic"
MODEL_REVISION = "4772b6bf101d91f2534c106dc524d906aeb3c68a"
EVALUATION_SIZE = (360, 640)


def aggregated_selected_mask(
    class_map: np.ndarray,
    *,
    road_ids: list[int],
    sidewalk_ids: list[int],
    road_confidence: np.ndarray | None = None,
    sidewalk_confidence: np.ndarray | None = None,
    minimum_confidence: float = 0.0,
    minimum_area: int | None = None,
    morph_close_kernel: int = 0,
) -> np.ndarray:
    road = np.isin(class_map, road_ids)
    sidewalk = np.isin(class_map, sidewalk_ids)
    if minimum_confidence > 0.0:
        if road_confidence is None or sidewalk_confidence is None:
            raise ValueError("confidence maps are required when thresholding")
        road &= road_confidence >= minimum_confidence
        sidewalk &= sidewalk_confidence >= minimum_confidence
    if minimum_area is not None:
        road = remove_small_components(road, minimum_area)
        sidewalk = remove_small_components(sidewalk, minimum_area)
    if morph_close_kernel:
        kernel = np.ones((morph_close_kernel, morph_close_kernel), dtype=np.uint8)
        road = cv2.morphologyEx(
            road.astype(np.uint8), cv2.MORPH_CLOSE, kernel
        ).astype(bool)
        sidewalk = cv2.morphologyEx(
            sidewalk.astype(np.uint8), cv2.MORPH_CLOSE, kernel
        ).astype(bool)
        overlap = road & sidewalk
        if np.any(overlap):
            if road_confidence is None or sidewalk_confidence is None:
                raise ValueError("confidence maps are required to resolve morphology overlap")
            road_wins = road_confidence >= sidewalk_confidence
            road[overlap] = road_wins[overlap]
            sidewalk[overlap] = ~road_wins[overlap]
    selected = np.zeros(class_map.shape, dtype=np.uint8)
    selected[road] = 1
    selected[sidewalk] = 2
    return selected


def semantic_scores(
    processor: AutoImageProcessor,
    model: Mask2FormerForUniversalSegmentation,
    frame_bgr: np.ndarray,
    device: torch.device,
    target_size: tuple[int, int],
) -> torch.Tensor:
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    inputs = processor(images=frame_rgb, return_tensors="pt")
    inputs = {name: value.to(device) for name, value in inputs.items()}
    with torch.inference_mode():
        outputs = model(**inputs)
        processed = processor.post_process_semantic_segmentation(
            outputs,
            target_sizes=[target_size],
            return_segmentation_scores=True,
        )[0]
    return processed["segmentation_scores"].detach().float().cpu()


def motion_compensated_history(
    previous_scores: torch.Tensor,
    previous_gray: np.ndarray,
    current_gray: np.ndarray,
) -> tuple[torch.Tensor, float]:
    """Warp the previous score field into the current frame using backward flow."""
    flow_to_previous = cv2.calcOpticalFlowFarneback(
        current_gray,
        previous_gray,
        None,
        pyr_scale=0.5,
        levels=3,
        winsize=15,
        iterations=3,
        poly_n=5,
        poly_sigma=1.2,
        flags=0,
    )
    height, width = current_gray.shape
    grid_x, grid_y = np.meshgrid(
        np.arange(width, dtype=np.float32),
        np.arange(height, dtype=np.float32),
    )
    source_x = grid_x + flow_to_previous[..., 0]
    source_y = grid_y + flow_to_previous[..., 1]
    normalized_x = 2.0 * source_x / max(width - 1, 1) - 1.0
    normalized_y = 2.0 * source_y / max(height - 1, 1) - 1.0
    sampling_grid = torch.from_numpy(
        np.stack((normalized_x, normalized_y), axis=-1)
    ).unsqueeze(0)
    warped_scores = torch_functional.grid_sample(
        previous_scores.unsqueeze(0),
        sampling_grid,
        mode="bilinear",
        padding_mode="border",
        align_corners=True,
    ).squeeze(0)
    mean_displacement = float(np.linalg.norm(flow_to_previous, axis=2).mean())
    return warped_scores, mean_displacement


def upscale_mask(mask: np.ndarray, frame_bgr: np.ndarray) -> np.ndarray:
    if mask.shape == frame_bgr.shape[:2]:
        return mask
    return cv2.resize(
        mask,
        (frame_bgr.shape[1], frame_bgr.shape[0]),
        interpolation=cv2.INTER_NEAREST,
    )


def open_writer(path: Path) -> cv2.VideoWriter:
    writer = cv2.VideoWriter(
        str(path), cv2.VideoWriter_fourcc(*"mp4v"), 10.0, (960, 540)
    )
    if not writer.isOpened():
        raise RuntimeError(f"could not create preview: {path}")
    return writer


def preview_frame(
    frame_bgr: np.ndarray,
    mask: np.ndarray,
    *,
    frame_index: int,
    fps: float,
    video_stem: str,
) -> np.ndarray:
    overlay = render_overlay(
        frame_bgr,
        upscale_mask(mask, frame_bgr),
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
    return overlay


def write_report(
    path: Path,
    *,
    raw: dict[str, Any],
    filtered: dict[str, Any],
    selected: str,
    candidate_video: Path,
    profile: str,
) -> None:
    lines = [
        f"# Mapillary consecutive-frame validation — {profile}",
        "",
        f"Selected candidate: `{selected}`.",
        "",
        "| Metric | Raw | Temporal + spatial |",
        "|---|---:|---:|",
        "| Selected-label change | "
        f"{raw['mean_selected_label_change_ratio']:.5f} | "
        f"{filtered['mean_selected_label_change_ratio']:.5f} |",
        "| Road adjacent IoU | "
        f"{raw['mean_road_iou']:.5f} | {filtered['mean_road_iou']:.5f} |",
        "| Sidewalk adjacent IoU | "
        f"{raw['mean_sidewalk_iou']:.5f} | {filtered['mean_sidewalk_iou']:.5f} |",
        "| Sidewalk small-component noise | "
        f"{raw['mean_sidewalk_small_component_noise']:.5f} | "
        f"{filtered['mean_sidewalk_small_component_noise']:.5f} |",
        "",
        "Mean filtered-to-raw preservation IoU: "
        f"{filtered['mean_raw_preservation_iou']:.5f}.",
        f"Staged candidate video: `{candidate_video}`.",
        "",
        "The staged video is not promoted until visual review confirms that the "
        "semantic improvement is retained across consecutive frames.",
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
    parser.add_argument("--model-id", default=MODEL_ID)
    parser.add_argument("--model-revision", default=MODEL_REVISION)
    parser.add_argument("--bursts-per-video", type=int, default=2)
    parser.add_argument("--burst-length", type=int, default=15)
    parser.add_argument("--temporal-alpha", type=float, default=0.72)
    parser.add_argument(
        "--evaluation-size",
        type=int,
        nargs=2,
        metavar=("HEIGHT", "WIDTH"),
        default=EVALUATION_SIZE,
    )
    parser.add_argument("--device", default="auto")
    parser.add_argument(
        "--experiment-id",
        help="Optional isolated output directory and staging filename prefix.",
    )
    parser.add_argument(
        "--surface-aggregate",
        action="store_true",
        help="Merge Mapillary pedestrian and road-surface subclasses.",
    )
    parser.add_argument(
        "--surface-confidence-threshold",
        type=float,
        default=0.0,
        help="Drop aggregate-profile Road/Sidewalk pixels below this score.",
    )
    parser.add_argument(
        "--morph-close-kernel",
        type=int,
        default=0,
        help="Optional odd kernel size for conservative post-filter mask closing.",
    )
    parser.add_argument(
        "--temporal-reset-change-threshold",
        type=float,
        default=0.0,
        help="Reset score history when raw selected-label change exceeds this ratio.",
    )
    parser.add_argument(
        "--temporal-confidence-ceiling",
        type=float,
        default=0.0,
        help=(
            "Apply temporal score blending only where the current maximum class "
            "confidence is below this value; 0 keeps global blending."
        ),
    )
    parser.add_argument(
        "--motion-compensate",
        action="store_true",
        help="Warp the previous score field with dense optical flow before blending.",
    )
    parser.add_argument(
        "--temporal-hysteresis-margin",
        type=float,
        default=0.0,
        help=(
            "Keep the previous selected label when it changes and the current "
            "top-two class-score margin is below this value."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.bursts_per_video <= 0 or args.burst_length < 3:
        raise ValueError("burst counts must be positive and burst length at least 3")
    if not 0.5 <= args.temporal_alpha <= 1.0:
        raise ValueError("--temporal-alpha must be in [0.5, 1.0]")
    if any(size < 128 or size > 2048 for size in args.evaluation_size):
        raise ValueError("--evaluation-size dimensions must be in [128, 2048]")
    if args.experiment_id and not re.fullmatch(r"[a-z0-9][a-z0-9._-]*", args.experiment_id):
        raise ValueError("--experiment-id must be a safe lowercase filename component")
    if not 0.0 <= args.surface_confidence_threshold <= 1.0:
        raise ValueError("--surface-confidence-threshold must be in [0, 1]")
    if args.surface_confidence_threshold and not args.surface_aggregate:
        raise ValueError("--surface-confidence-threshold requires --surface-aggregate")
    if args.morph_close_kernel not in {0, 3, 5, 7, 9}:
        raise ValueError("--morph-close-kernel must be 0 or an odd size from 3 to 9")
    if args.morph_close_kernel and not args.surface_aggregate:
        raise ValueError("--morph-close-kernel requires --surface-aggregate")
    if not 0.0 <= args.temporal_reset_change_threshold <= 1.0:
        raise ValueError("--temporal-reset-change-threshold must be in [0, 1]")
    if not 0.0 <= args.temporal_confidence_ceiling <= 1.0:
        raise ValueError("--temporal-confidence-ceiling must be in [0, 1]")
    if not 0.0 <= args.temporal_hysteresis_margin <= 1.0:
        raise ValueError("--temporal-hysteresis-margin must be in [0, 1]")

    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    profile = "surface-aggregate" if args.surface_aggregate else "canonical"
    state_prefix = (
        "mapillary_aggregation_temporal"
        if args.surface_aggregate
        else "mapillary_temporal"
    )
    default_experiment_id = (
        "mapillary-aggregation-temporal-validation"
        if args.surface_aggregate
        else "mapillary-temporal-validation"
    )
    experiment_id = args.experiment_id or default_experiment_id
    experiment_dir = output_dir / experiment_id
    staging_dir = output_dir / ".staging"
    staging_dir.mkdir(parents=True, exist_ok=True)
    filename_prefix = args.experiment_id or (
        "mapillary-aggregation" if args.surface_aggregate else "mapillary"
    )
    raw_video = staging_dir / f"{filename_prefix}-raw.tmp.mp4"
    filtered_video = staging_dir / f"{filename_prefix}-temporal.tmp.mp4"
    candidate_video = staging_dir / f"{filename_prefix}-temporal-candidate.mp4"
    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    with RunLock(output_dir / "run.lock"):
        for path in staging_dir.iterdir():
            if path.is_file() and path.suffix.lower() in VIDEO_EXTENSIONS:
                path.unlink()

        state_path = output_dir / "state.json"
        state = json.loads(state_path.read_text(encoding="utf-8"))
        state.update(
            {
                "status": f"{state_prefix}_validation_running",
                "updated_at": utc_now(),
                "next_action": f"Finish the current Mapillary {profile} temporal checkpoint.",
            }
        )
        atomic_write_json(state_path, state)

        videos = [inspect_video(path) for path in discover_videos(input_dir)]
        device = choose_device(args.device)
        processor = AutoImageProcessor.from_pretrained(
            args.model_id, revision=args.model_revision
        )
        model = Mask2FormerForUniversalSegmentation.from_pretrained(
            args.model_id, revision=args.model_revision
        ).to(device)
        model.eval()
        road_ids = (
            resolve_ids(model.config.id2label, ROAD_LABELS)
            if args.surface_aggregate
            else [resolve_label_id(model.config.id2label, "road")]
        )
        sidewalk_ids = (
            resolve_ids(model.config.id2label, SIDEWALK_LABELS)
            if args.surface_aggregate
            else [resolve_label_id(model.config.id2label, "sidewalk")]
        )

        raw_rows: list[dict[str, Any]] = []
        filtered_rows: list[dict[str, Any]] = []
        raw_pairs: list[dict[str, float]] = []
        filtered_pairs: list[dict[str, float]] = []
        preservation: list[float] = []
        review_paths: list[Path] = []
        raw_writer = open_writer(raw_video)
        filtered_writer = open_writer(filtered_video)
        preview_count = 0
        temporal_reset_count = 0
        temporal_blended_pixel_fractions: list[float] = []
        motion_displacements: list[float] = []
        temporal_hysteresis_hold_fractions: list[float] = []
        try:
            for video_number, video in enumerate(videos, start=1):
                if stop_requested:
                    break
                bursts = collect_bursts(
                    Path(video.path),
                    frame_count=video.frame_count,
                    fps=video.fps,
                    burst_count=args.bursts_per_video,
                    burst_length=args.burst_length,
                )
                for burst_id, burst in enumerate(bursts):
                    previous_scores: torch.Tensor | None = None
                    previous_gray: np.ndarray | None = None
                    previous_raw: np.ndarray | None = None
                    previous_filtered: np.ndarray | None = None
                    for position, (frame_index, time_seconds, frame_bgr) in enumerate(
                        burst
                    ):
                        if stop_requested:
                            break
                        scores = semantic_scores(
                            processor,
                            model,
                            frame_bgr,
                            device,
                            tuple(args.evaluation_size),
                        )
                        raw_map = scores.argmax(dim=0).numpy()
                        minimum_area = max(48, int(raw_map.size * 0.00035))
                        raw_road_confidence = scores[road_ids].max(dim=0).values.numpy()
                        raw_sidewalk_confidence = (
                            scores[sidewalk_ids].max(dim=0).values.numpy()
                        )
                        if args.surface_aggregate:
                            raw_selected = aggregated_selected_mask(
                                raw_map,
                                road_ids=road_ids,
                                sidewalk_ids=sidewalk_ids,
                                road_confidence=raw_road_confidence,
                                sidewalk_confidence=raw_sidewalk_confidence,
                                minimum_confidence=args.surface_confidence_threshold,
                            )
                        else:
                            raw_selected = selected_mask(
                                raw_map,
                                road_id=road_ids[0],
                                sidewalk_id=sidewalk_ids[0],
                            )
                        raw_change_from_previous = (
                            None
                            if previous_raw is None
                            else float(np.mean(previous_raw != raw_selected))
                        )
                        temporal_reset = bool(
                            previous_scores is not None
                            and raw_change_from_previous is not None
                            and args.temporal_reset_change_threshold > 0.0
                            and raw_change_from_previous
                            > args.temporal_reset_change_threshold
                        )
                        if temporal_reset:
                            temporal_reset_count += 1
                        current_gray = None
                        if args.motion_compensate:
                            evaluation_frame = cv2.resize(
                                frame_bgr,
                                (args.evaluation_size[1], args.evaluation_size[0]),
                                interpolation=cv2.INTER_AREA,
                            )
                            current_gray = cv2.cvtColor(
                                evaluation_frame, cv2.COLOR_BGR2GRAY
                            )
                        if previous_scores is None or temporal_reset:
                            smooth_scores = scores
                            temporal_blended_pixel_fractions.append(0.0)
                        else:
                            history_scores = previous_scores
                            if args.motion_compensate:
                                if previous_gray is None or current_gray is None:
                                    raise RuntimeError(
                                        "motion-compensation grayscale history missing"
                                    )
                                history_scores, displacement = (
                                    motion_compensated_history(
                                        previous_scores, previous_gray, current_gray
                                    )
                                )
                                motion_displacements.append(displacement)
                            blended_scores = (
                                args.temporal_alpha * scores
                                + (1.0 - args.temporal_alpha) * history_scores
                            )
                            if args.temporal_confidence_ceiling > 0.0:
                                blend_mask = (
                                    scores.max(dim=0).values
                                    < args.temporal_confidence_ceiling
                                )
                                smooth_scores = torch.where(
                                    blend_mask.unsqueeze(0), blended_scores, scores
                                )
                                temporal_blended_pixel_fractions.append(
                                    float(blend_mask.float().mean().item())
                                )
                            else:
                                smooth_scores = blended_scores
                                temporal_blended_pixel_fractions.append(1.0)
                        previous_scores = smooth_scores
                        previous_gray = current_gray
                        smooth_map = smooth_scores.argmax(dim=0).numpy()
                        filtered_road_confidence = (
                            smooth_scores[road_ids].max(dim=0).values.numpy()
                        )
                        filtered_sidewalk_confidence = (
                            smooth_scores[sidewalk_ids].max(dim=0).values.numpy()
                        )
                        if args.surface_aggregate:
                            filtered_selected = aggregated_selected_mask(
                                smooth_map,
                                road_ids=road_ids,
                                sidewalk_ids=sidewalk_ids,
                                road_confidence=filtered_road_confidence,
                                sidewalk_confidence=filtered_sidewalk_confidence,
                                minimum_confidence=args.surface_confidence_threshold,
                                minimum_area=minimum_area,
                                morph_close_kernel=args.morph_close_kernel,
                            )
                        else:
                            filtered_selected = selected_mask(
                                smooth_map,
                                road_id=road_ids[0],
                                sidewalk_id=sidewalk_ids[0],
                                minimum_area=minimum_area,
                            )
                        if (
                            args.temporal_hysteresis_margin > 0.0
                            and previous_filtered is not None
                        ):
                            top_scores = torch.topk(
                                smooth_scores, k=2, dim=0
                            ).values.numpy()
                            score_margin = top_scores[0] - top_scores[1]
                            hold_mask = (
                                (filtered_selected != previous_filtered)
                                & (score_margin < args.temporal_hysteresis_margin)
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
                            temporal_hysteresis_hold_fractions.append(
                                float(np.mean(hold_mask))
                            )
                        else:
                            temporal_hysteresis_hold_fractions.append(0.0)
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
                            "raw_change_from_previous": raw_change_from_previous,
                            "temporal_reset": temporal_reset,
                        }
                        raw_rows.append(
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
                        filtered_rows.append(
                            {
                                **common,
                                "road": asdict(
                                    class_metrics(
                                        filtered_road,
                                        filtered_road_confidence,
                                        small_component_area=minimum_area,
                                    )
                                ),
                                "sidewalk": asdict(
                                    class_metrics(
                                        filtered_sidewalk,
                                        filtered_sidewalk_confidence,
                                        small_component_area=minimum_area,
                                    )
                                ),
                            }
                        )
                        preservation.extend(
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

                        raw_writer.write(
                            preview_frame(
                                frame_bgr,
                                raw_selected,
                                frame_index=frame_index,
                                fps=video.fps,
                                video_stem=video.stem,
                            )
                        )
                        filtered_writer.write(
                            preview_frame(
                                frame_bgr,
                                filtered_selected,
                                frame_index=frame_index,
                                fps=video.fps,
                                video_stem=video.stem,
                            )
                        )
                        preview_count += 1

                        if position in {0, len(burst) // 2, len(burst) - 1}:
                            comparison = np.concatenate(
                                (
                                    render_overlay(
                                        frame_bgr,
                                        upscale_mask(raw_selected, frame_bgr),
                                        frame_index=frame_index,
                                        fps=video.fps,
                                    ),
                                    render_overlay(
                                        frame_bgr,
                                        upscale_mask(filtered_selected, frame_bgr),
                                        frame_index=frame_index,
                                        fps=video.fps,
                                    ),
                                ),
                                axis=1,
                            )
                            review_dir = experiment_dir / "review-frames"
                            review_dir.mkdir(parents=True, exist_ok=True)
                            review_path = review_dir / (
                                f"{video.stem}-burst-{burst_id}-"
                                f"frame-{frame_index:08d}.jpg"
                            )
                            if not cv2.imwrite(
                                str(review_path),
                                comparison,
                                [cv2.IMWRITE_JPEG_QUALITY, 90],
                            ):
                                raise RuntimeError(f"could not write {review_path}")
                            review_paths.append(review_path)
                state.update(
                    {
                        "updated_at": utc_now(),
                        f"{state_prefix}_completed_video_count": video_number,
                        f"{state_prefix}_frame_count": preview_count,
                    }
                )
                atomic_write_json(state_path, state)
        finally:
            raw_writer.release()
            filtered_writer.release()

        if stop_requested:
            raw_video.unlink(missing_ok=True)
            filtered_video.unlink(missing_ok=True)
            state.update(
                {
                    "status": f"{state_prefix}_validation_interrupted",
                    "updated_at": utc_now(),
                }
            )
            atomic_write_json(state_path, state)
            return 130

        raw_summary = summarize_candidate(raw_rows, raw_pairs)
        filtered_summary = summarize_candidate(filtered_rows, filtered_pairs)
        filtered_summary["mean_raw_preservation_iou"] = mean(preservation)
        filtered_summary["temporal_reset_count"] = temporal_reset_count
        filtered_summary["mean_temporal_blended_pixel_fraction"] = mean(
            temporal_blended_pixel_fractions
        )
        filtered_summary["mean_motion_displacement_pixels"] = mean(
            motion_displacements
        )
        filtered_summary["mean_temporal_hysteresis_hold_fraction"] = mean(
            temporal_hysteresis_hold_fractions
        )
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
        selected_video = filtered_video if selected == "temporal_spatial" else raw_video
        rejected_video = raw_video if selected == "temporal_spatial" else filtered_video
        validation = validate_video(selected_video, preview_count)
        os.replace(selected_video, candidate_video)
        rejected_video.unlink(missing_ok=True)

        contact_sheet = experiment_dir / "temporal-comparison-contact-sheet.jpg"
        make_contact_sheet(review_paths, contact_sheet, columns=2, tile_width=720)
        report = {
            "schema_version": 1,
            "created_at": utc_now(),
            "model": {"id": args.model_id, "revision": args.model_revision},
            "settings": {
                "bursts_per_video": args.bursts_per_video,
                "burst_length": args.burst_length,
                "temporal_alpha": args.temporal_alpha,
                "surface_confidence_threshold": args.surface_confidence_threshold,
                "morph_close_kernel": args.morph_close_kernel,
                "temporal_reset_change_threshold": (
                    args.temporal_reset_change_threshold
                ),
                "temporal_confidence_ceiling": args.temporal_confidence_ceiling,
                "motion_compensate": args.motion_compensate,
                "temporal_hysteresis_margin": args.temporal_hysteresis_margin,
                "evaluation_size": list(args.evaluation_size),
                "profile": profile,
                "road_labels": list(ROAD_LABELS) if args.surface_aggregate else ["Road"],
                "road_label_ids": road_ids,
                "sidewalk_labels": (
                    list(SIDEWALK_LABELS)
                    if args.surface_aggregate
                    else ["Sidewalk"]
                ),
                "sidewalk_label_ids": sidewalk_ids,
            },
            "raw": {"summary": raw_summary, "frames": raw_rows, "pairs": raw_pairs},
            "temporal_spatial": {
                "summary": filtered_summary,
                "frames": filtered_rows,
                "pairs": filtered_pairs,
            },
            "selection": {
                "selected": selected,
                "change_improved": change_improved,
                "noise_improved": noise_improved,
                "preservation_ok": preservation_ok,
                "final_acceptance": False,
                "reason": "Quantitative gate passed; visual promotion review remains.",
            },
            "candidate_video": {"path": str(candidate_video), "validation": validation},
        }
        atomic_write_json(experiment_dir / "metrics.json", report)
        write_report(
            experiment_dir / "REPORT.md",
            raw=raw_summary,
            filtered=filtered_summary,
            selected=selected,
            candidate_video=candidate_video,
            profile=profile,
        )
        state.update(
            {
                "status": f"{state_prefix}_candidate_complete",
                "updated_at": utc_now(),
                f"{state_prefix}_metrics": str(experiment_dir / "metrics.json"),
                f"{state_prefix}_report": str(experiment_dir / "REPORT.md"),
                f"{state_prefix}_contact_sheet": str(contact_sheet),
                f"{state_prefix}_candidate_video": str(candidate_video),
                f"{state_prefix}_selected": selected,
                "next_action": (
                    f"Visually review the staged Mapillary {profile} temporal candidate; "
                    "promote it only if it preserves the semantic sample improvement."
                ),
                "success": False,
            }
        )
        atomic_write_json(state_path, state)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
