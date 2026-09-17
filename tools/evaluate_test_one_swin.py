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
"""Compare Swin-L with the two source/painted-overlay pairs in test-one.

The painted ADE20K B5 videos are reference outputs, not independent pixel
ground truth. Comparison uses only confident paint and unpainted pixels, and
ignores the on-video legend, dark camera occlusion, and ambiguous paint edges.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from best_so_far_runtime import (
    SWIN_L_ASPECT_PROFILE,
    SWIN_L_ASPECT_QUALITY_PROFILE,
    SWIN_L_PROFILE,
    BestSoFarConfig,
    BestSoFarSegmenter,
)


PAIRS = (
    (
        "거의정답지",
        "[거의정답지원본].mp4",
        "[거의 정답지].mp4",
        (5, 20, 35, 50, 65),
    ),
    (
        "정답으로수정",
        "[정답으로수정원본].mp4",
        "[정답으로수정].mp4",
        (5, 20, 35, 50, 65, 80),
    ),
)
EVALUATION_SIZE = (640, 360)
SECOND_PAIR_MANUAL_PROBES = {
    "sidewalk_tile_interior": (slice(260, 330), slice(170, 340), 2),
    "asphalt_interior": (slice(210, 250), slice(470, 520), 1),
}


def extract_reference(
    source_bgr: np.ndarray, overlay_bgr: np.ndarray
) -> tuple[np.ndarray, np.ndarray, float]:
    """Decode green/magenta paint from a paired frame at 640x360.

    Green paint raises G relative to B/R; magenta paint lowers it. The
    chromatic residual is insensitive to the source scene's own color.
    """

    if source_bgr.shape != overlay_bgr.shape:
        raise ValueError("source and reference overlay dimensions differ")
    width, height = EVALUATION_SIZE
    source = cv2.resize(source_bgr, (width, height), interpolation=cv2.INTER_AREA)
    overlay = cv2.resize(overlay_bgr, (width, height), interpolation=cv2.INTER_AREA)
    difference = overlay.astype(np.float32) - source.astype(np.float32)
    chroma = difference[:, :, 1] - 0.5 * (
        difference[:, :, 0] + difference[:, :, 2]
    )
    reference = np.zeros((height, width), np.uint8)
    reference[chroma >= 30] = 1
    reference[chroma <= -30] = 2
    valid = (np.abs(chroma) >= 30) | (np.abs(chroma) <= 12)
    valid[:120, :] = False  # excludes the legend and distant, tiny regions
    valid &= np.mean(source, axis=2) >= 28  # excludes the camera housing
    unpainted = valid & (reference == 0)
    background_error = float(
        np.median(np.abs(difference[unpainted])) if np.any(unpainted) else 0.0
    )
    return reference, valid, background_error


def confusion_matrix(
    predicted: np.ndarray, reference: np.ndarray, valid: np.ndarray
) -> np.ndarray:
    return np.bincount(
        (reference[valid].astype(np.int32) * 3 + predicted[valid]).ravel(),
        minlength=9,
    ).reshape(3, 3)


def summarize(matrix: np.ndarray) -> dict[str, float | int | None]:
    out: dict[str, float | int | None] = {"evaluated_pixels": int(matrix.sum())}
    ious: list[float] = []
    for label, name in ((1, "road"), (2, "sidewalk")):
        intersection = int(matrix[label, label])
        union = int(matrix[label, :].sum() + matrix[:, label].sum() - intersection)
        iou = intersection / union if union else None
        out[f"{name}_iou"] = iou
        out[f"{name}_reference_pixels"] = int(matrix[label, :].sum())
        out[f"{name}_predicted_pixels"] = int(matrix[:, label].sum())
        if iou is not None:
            ious.append(iou)
    out["mean_road_sidewalk_iou"] = float(np.mean(ious)) if ious else None
    return out


def read_frame(capture: cv2.VideoCapture, index: int) -> np.ndarray:
    capture.set(cv2.CAP_PROP_POS_FRAMES, index)
    ok, frame = capture.read()
    if not ok:
        raise RuntimeError(f"could not decode frame {index}")
    return frame


def comparison_preview(
    source: np.ndarray,
    reference: np.ndarray,
    selected: np.ndarray,
    valid: np.ndarray,
) -> np.ndarray:
    width, height = EVALUATION_SIZE
    source = cv2.resize(source, (width, height), interpolation=cv2.INTER_AREA)
    colors = np.asarray(((0, 0, 0), (0, 170, 0), (170, 0, 170)), np.uint8)
    panes = [source]
    for mask in (reference, selected):
        painted = cv2.addWeighted(source, 0.55, colors[mask], 0.45, 0)
        painted[~valid] = source[~valid] // 2
        panes.append(painted)
    wrong = valid & (reference != selected)
    error = source.copy()
    error[wrong] = (0, 0, 255)
    panes.append(error)
    return cv2.hconcat(panes)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-dir", type=Path, default=Path("rosbag-results/test-one")
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--single-pair", choices=tuple(pair[0] for pair in PAIRS))
    parser.add_argument("--start-seconds", type=float)
    parser.add_argument("--preview-every-frames", type=int, default=0)
    parser.add_argument(
        "--profile",
        choices=(SWIN_L_PROFILE, SWIN_L_ASPECT_PROFILE, SWIN_L_ASPECT_QUALITY_PROFILE),
        default=SWIN_L_PROFILE,
    )
    parser.add_argument("--window-frames", type=int, default=8)
    parser.add_argument(
        "--time-offset", type=float, default=0.0,
        help="shift all sampling windows by this many seconds",
    )
    parser.add_argument("--temporal-alpha", type=float)
    parser.add_argument("--temporal-hysteresis-margin", type=float)
    parser.add_argument("--evaluation-size", nargs=2, type=int, default=(360, 640))
    parser.add_argument("--model-input-size", nargs=2, type=int)
    parser.add_argument("--pedestrian-area-road-expansion", type=int, default=0)
    parser.add_argument("--device", default="auto")
    parser.add_argument(
        "--clahe-clip", type=float, default=0.0,
        help="optional LAB luminance CLAHE clip limit before inference",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.window_frames <= 0:
        raise ValueError("window-frames must be positive")
    if args.time_offset < 0:
        raise ValueError("time-offset must be non-negative")
    if args.start_seconds is not None and args.start_seconds < 0:
        raise ValueError("start-seconds must be non-negative")
    if args.preview_every_frames < 0:
        raise ValueError("preview-every-frames must be non-negative")
    if args.clahe_clip < 0:
        raise ValueError("clahe-clip must be non-negative")
    directory = args.input_dir.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    segmenter = BestSoFarSegmenter(
        BestSoFarConfig(
            profile=args.profile,
            temporal_alpha=args.temporal_alpha,
            temporal_hysteresis_margin=args.temporal_hysteresis_margin,
            evaluation_height=args.evaluation_size[0],
            evaluation_width=args.evaluation_size[1],
            pedestrian_area_road_expansion=args.pedestrian_area_road_expansion,
            device=args.device,
        )
    )
    clahe = (
        cv2.createCLAHE(clipLimit=args.clahe_clip, tileGridSize=(8, 8))
        if args.clahe_clip > 0
        else None
    )
    if args.model_input_size is not None:
        height, width = args.model_input_size
        if height <= 0 or width <= 0:
            raise ValueError("model input dimensions must be positive")
        segmenter.processor.size = {"height": height, "width": width}
    pair_reports = []
    for pair_name, source_name, overlay_name, times in PAIRS:
        if args.single_pair is not None and pair_name != args.single_pair:
            continue
        if args.start_seconds is not None:
            times = (args.start_seconds,)
        source_path, overlay_path = directory / source_name, directory / overlay_name
        source_capture = cv2.VideoCapture(str(source_path))
        overlay_capture = cv2.VideoCapture(str(overlay_path))
        if not source_capture.isOpened() or not overlay_capture.isOpened():
            raise FileNotFoundError(f"could not open both videos for {pair_name}")
        source_count = int(source_capture.get(cv2.CAP_PROP_FRAME_COUNT))
        overlay_count = int(overlay_capture.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = float(overlay_capture.get(cv2.CAP_PROP_FPS))
        matrix = np.zeros((3, 3), np.int64)
        frame_reports = []
        previews = []
        runtimes = []
        try:
            for seconds in times:
                first = round((seconds + args.time_offset) * fps)
                if first + args.window_frames > min(source_count, overlay_count):
                    continue
                segmenter.reset()
                for offset in range(args.window_frames):
                    index = first + offset
                    source = read_frame(source_capture, index)
                    overlay = read_frame(overlay_capture, index)
                    reference, valid, background_error = extract_reference(
                        source, overlay
                    )
                    if background_error > 20:
                        raise RuntimeError(
                            f"source/overlay misalignment at {pair_name} frame {index}: "
                            f"median background error={background_error:.1f}"
                        )
                    model_input = source
                    if clahe is not None:
                        lab = cv2.cvtColor(source, cv2.COLOR_BGR2LAB)
                        lab[:, :, 0] = clahe.apply(lab[:, :, 0])
                        model_input = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
                    result = segmenter.segment(model_input)
                    prediction = result.selected_mask
                    runtimes.append(result.total_seconds)
                    if prediction.shape != reference.shape:
                        prediction = cv2.resize(
                            prediction,
                            EVALUATION_SIZE,
                            interpolation=cv2.INTER_NEAREST,
                        )
                    frame_matrix = confusion_matrix(prediction, reference, valid)
                    matrix += frame_matrix
                    manual_probes = {}
                    if pair_name == "정답으로수정":
                        for probe_name, (rows, columns, expected_label) in (
                            SECOND_PAIR_MANUAL_PROBES.items()
                        ):
                            manual_probes[probe_name] = {
                                "candidate_expected_label_fraction": float(
                                    np.mean(
                                        prediction[rows, columns] == expected_label
                                    )
                                ),
                                "reference_expected_label_fraction": float(
                                    np.mean(reference[rows, columns] == expected_label)
                                ),
                            }
                    frame_reports.append(
                        {
                            "frame": index,
                            "seconds": index / fps,
                            "background_error": background_error,
                            "processing_seconds": result.total_seconds,
                            "manual_semantic_probes": manual_probes,
                            **summarize(frame_matrix),
                        }
                    )
                    if offset == 0 or (
                        args.preview_every_frames > 0
                        and offset % args.preview_every_frames == 0
                    ):
                        previews.append(
                            comparison_preview(source, reference, prediction, valid)
                        )
                print(f"PAIR_PROGRESS pair={pair_name} seconds={seconds}", flush=True)
        finally:
            source_capture.release()
            overlay_capture.release()
        sheet_path = output / f"{args.label}-{pair_name}-contact-sheet.jpg"
        if previews and not cv2.imwrite(str(sheet_path), cv2.vconcat(previews)):
            raise RuntimeError(f"could not write preview {sheet_path}")
        pair_reports.append(
            {
                "name": pair_name,
                "source": str(source_path),
                "overlay": str(overlay_path),
                "source_frames": source_count,
                "overlay_frames": overlay_count,
                "fps": fps,
                "confusion_matrix": matrix.tolist(),
                "metrics": summarize(matrix),
                "mean_processing_seconds": float(np.mean(runtimes)),
                "frames": frame_reports,
                "contact_sheet": str(sheet_path),
            }
        )
    combined = sum(
        (np.asarray(pair["confusion_matrix"], np.int64) for pair in pair_reports),
        np.zeros((3, 3), np.int64),
    )
    report = {
        "schema_version": 1,
        "reference_note": "ADE20K B5 painted overlays are proxy labels, not pixel ground truth",
        "runtime": segmenter.metadata(),
        "actual_model_input_size": segmenter.processor.size,
        "window_frames": args.window_frames,
        "time_offset": args.time_offset,
        "start_seconds": args.start_seconds,
        "single_pair": args.single_pair,
        "clahe_clip": args.clahe_clip,
        "pairs": pair_reports,
        "combined": summarize(combined),
    }
    path = output / f"{args.label}-report.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(f"REPORT={path}")
    print(json.dumps(report["combined"], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
