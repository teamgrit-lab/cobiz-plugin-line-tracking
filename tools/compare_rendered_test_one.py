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
"""Compare the already-rendered test-one Swin-L videos in full-run context.

The ADE20K B5 reference is a proxy, not a human-labeled ground truth. All
four videos are aligned by frame number. Only pixels confidently decoded in
both candidates and the reference contribute to either candidate's metrics.
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

from evaluate_test_one_swin import PAIRS, confusion_matrix, extract_reference, summarize


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, default=Path("rosbag-results/test-one"))
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=Path("rosbag-results/test-one-swin-validation-20260916"),
    )
    parser.add_argument("--pair", choices=tuple(pair[0] for pair in PAIRS), required=True)
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def paired_paths(args: argparse.Namespace) -> dict[str, Path]:
    name, source_name, reference_name, _ = next(
        pair for pair in PAIRS if pair[0] == args.pair
    )
    stem = Path(source_name).stem
    ordinal = "first" if name == PAIRS[0][0] else "second"
    return {
        "source": args.input_dir / source_name,
        "reference": args.input_dir / reference_name,
        "fast": args.results_dir
        / f"full-aspect-224x384{'' if ordinal == 'first' else '-second'}"
        / f"{stem}-swin-l-aspect-224x384-segmented-full.mp4",
        "quality": args.results_dir
        / f"full-aspect-448x768-{ordinal}"
        / f"{stem}-swin-l-aspect-448x768-segmented-full.mp4",
    }


def main() -> int:
    args = parse_args()
    if args.start_frame < 0 or args.max_frames < 0:
        raise ValueError("start-frame and max-frames must be non-negative")
    paths = paired_paths(args)
    captures = {name: cv2.VideoCapture(str(path)) for name, path in paths.items()}
    if not all(capture.isOpened() for capture in captures.values()):
        missing = [name for name, capture in captures.items() if not capture.isOpened()]
        raise FileNotFoundError(f"could not open videos: {missing}")
    counts = {
        name: int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        for name, capture in captures.items()
    }
    fps = float(captures["source"].get(cv2.CAP_PROP_FPS))
    end = min(counts.values())
    if args.max_frames:
        end = min(end, args.start_frame + args.max_frames)
    if args.start_frame >= end:
        raise ValueError("no common frames in selected range")
    for capture in captures.values():
        capture.set(cv2.CAP_PROP_POS_FRAMES, args.start_frame)

    totals = {"fast": np.zeros((3, 3), np.int64), "quality": np.zeros((3, 3), np.int64)}
    per_second: dict[int, dict[str, np.ndarray]] = defaultdict(
        lambda: {"fast": np.zeros((3, 3), np.int64), "quality": np.zeros((3, 3), np.int64)}
    )
    frames = []
    try:
        for index in range(args.start_frame, end):
            decoded = {}
            for name, capture in captures.items():
                ok, frame = capture.read()
                if not ok:
                    raise RuntimeError(f"could not decode {name} frame {index}")
                decoded[name] = frame
            reference, reference_valid, reference_error = extract_reference(
                decoded["source"], decoded["reference"]
            )
            fast, fast_valid, fast_error = extract_reference(
                decoded["source"], decoded["fast"]
            )
            quality, quality_valid, quality_error = extract_reference(
                decoded["source"], decoded["quality"]
            )
            if max(reference_error, fast_error, quality_error) > 20:
                raise RuntimeError(
                    f"large background residual at frame {index}: "
                    f"{reference_error:.1f}, {fast_error:.1f}, {quality_error:.1f}"
                )
            common_valid = reference_valid & fast_valid & quality_valid
            second = int(index / fps)
            item = {
                "frame": index,
                "seconds": index / fps,
                "valid_pixels": int(np.count_nonzero(common_valid)),
                "background_residual": {
                    "reference": reference_error,
                    "fast": fast_error,
                    "quality": quality_error,
                },
            }
            for name, candidate in (("fast", fast), ("quality", quality)):
                matrix = confusion_matrix(candidate, reference, common_valid)
                totals[name] += matrix
                per_second[second][name] += matrix
                item[name] = summarize(matrix)
            frames.append(item)
            if (index - args.start_frame + 1) % 200 == 0:
                print(f"PROGRESS pair={args.pair} frames={index - args.start_frame + 1}/{end - args.start_frame}", flush=True)
    finally:
        for capture in captures.values():
            capture.release()

    report = {
        "schema_version": 1,
        "reference_note": "ADE20K B5 painted overlay is a model output, not pixel ground truth",
        "pair": args.pair,
        "paths": {name: str(path.resolve()) for name, path in paths.items()},
        "video_frame_counts": counts,
        "fps": fps,
        "start_frame": args.start_frame,
        "end_frame_exclusive": end,
        "combined": {name: summarize(matrix) for name, matrix in totals.items()},
        "per_second": [
            {
                "second": second,
                **{name: summarize(matrix) for name, matrix in per_second[second].items()},
            }
            for second in sorted(per_second)
        ],
        "frames": frames,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(f"REPORT={args.output.resolve()}")
    print(json.dumps(report["combined"], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
