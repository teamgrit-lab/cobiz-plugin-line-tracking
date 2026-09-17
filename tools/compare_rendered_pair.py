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
"""Compare one rendered Road/Sidewalk MP4 with its source and painted reference.

The reference overlay is another model's output, not human ground truth.
All streams are paired by decoded frame number and evaluated at 640x360.
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

from evaluate_test_one_swin import (
    comparison_preview,
    confusion_matrix,
    extract_reference,
    summarize,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--label", default="rendered-comparison")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument(
        "--preview-seconds",
        type=float,
        nargs="*",
        default=(0, 5, 15, 25, 35, 45, 55, 65),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.max_frames < 0:
        raise ValueError("--max-frames must be non-negative")
    if any(second < 0 for second in args.preview_seconds):
        raise ValueError("--preview-seconds must be non-negative")
    paths = {
        key: getattr(args, key).expanduser().resolve()
        for key in ("source", "reference", "candidate")
    }
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / f"{args.label}-report.json"
    sheet_path = output_dir / f"{args.label}-contact-sheet.jpg"
    for output_path in (report_path, sheet_path):
        if output_path.exists():
            raise FileExistsError(f"refusing to overwrite {output_path}")

    captures = {key: cv2.VideoCapture(str(path)) for key, path in paths.items()}
    if not all(capture.isOpened() for capture in captures.values()):
        missing = [key for key, capture in captures.items() if not capture.isOpened()]
        for capture in captures.values():
            capture.release()
        raise FileNotFoundError(f"could not open video streams: {missing}")
    counts = {
        key: int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        for key, capture in captures.items()
    }
    fps = float(captures["source"].get(cv2.CAP_PROP_FPS))
    end = min(counts.values())
    if args.max_frames:
        end = min(end, args.max_frames)
    if end <= 0 or not np.isfinite(fps) or fps <= 0:
        raise ValueError("videos must have frames and a valid source frame rate")
    preview_indices = {
        round(second * fps)
        for second in args.preview_seconds
        if round(second * fps) < end
    }
    overall = np.zeros((3, 3), np.int64)
    by_second: dict[int, np.ndarray] = defaultdict(lambda: np.zeros((3, 3), np.int64))
    frames = []
    previews = []
    try:
        for index in range(end):
            decoded = {}
            for key, capture in captures.items():
                ok, frame = capture.read()
                if not ok:
                    raise RuntimeError(f"could not decode {key} frame {index}")
                decoded[key] = frame
            reference, reference_valid, reference_error = extract_reference(
                decoded["source"], decoded["reference"]
            )
            candidate, candidate_valid, candidate_error = extract_reference(
                decoded["source"], decoded["candidate"]
            )
            if max(reference_error, candidate_error) > 20:
                raise RuntimeError(
                    f"large source/overlay residual at frame {index}: "
                    f"reference={reference_error:.1f}, candidate={candidate_error:.1f}"
                )
            valid = reference_valid & candidate_valid
            matrix = confusion_matrix(candidate, reference, valid)
            overall += matrix
            by_second[int(index / fps)] += matrix
            frames.append(
                {
                    "frame": index,
                    "seconds": index / fps,
                    "reference_background_error": reference_error,
                    "candidate_background_error": candidate_error,
                    **summarize(matrix),
                }
            )
            if index in preview_indices:
                previews.append(
                    comparison_preview(decoded["source"], reference, candidate, valid)
                )
            if (index + 1) % 200 == 0:
                print(f"COMPARE_PROGRESS frames={index + 1}/{end}", flush=True)
    finally:
        for capture in captures.values():
            capture.release()

    if previews and not cv2.imwrite(str(sheet_path), cv2.vconcat(previews)):
        raise RuntimeError(f"could not write {sheet_path}")
    report = {
        "schema_version": 1,
        "reference_note": "Painted reference is model output, not human pixel ground truth",
        "paths": {key: str(path) for key, path in paths.items()},
        "frame_counts": counts,
        "compared_frames": end,
        "fps": fps,
        "combined": summarize(overall),
        "per_second": [
            {"second": second, **summarize(by_second[second])}
            for second in sorted(by_second)
        ],
        "frames": frames,
        "contact_sheet": str(sheet_path),
    }
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(f"REPORT={report_path}")
    print(json.dumps(report["combined"], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
