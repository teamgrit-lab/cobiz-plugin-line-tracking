#!/usr/bin/env python3
"""Generate a YOLOP road/line segmentation overlay video.

The output keeps the input video's frame size and frame rate.  It does not
copy the input audio stream because OpenCV's ``VideoWriter`` only writes video
frames.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Optional, Tuple

import cv2
import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = REPOSITORY_ROOT / "ros_ws" / "src" / "line_tracking"
if str(PACKAGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE))

from line_tracking.segmentation import (  # noqa: E402
    SegmentationResult,
    YolopConfig,
    YolopSegmenter,
)


PROFILE_INPUTS = {
    # YOLOP's skip connections require dimensions divisible by 32. The
    # camera frames remain 640x360 or 1280x720; the extra rows are letterbox
    # padding removed again when masks are restored to the camera frame.
    "360p": (640, 384),
    "720p": (1280, 736),
}


def select_profile(
    profile: str, frame_width: int, frame_height: int
) -> Tuple[str, int, int]:
    """Resolve a named or automatic camera profile to model dimensions."""

    if profile == "auto":
        profile = "720p" if frame_height >= 540 else "360p"
    if profile not in PROFILE_INPUTS:
        raise ValueError(f"unsupported profile: {profile}")
    input_width, input_height = PROFILE_INPUTS[profile]
    return profile, input_width, input_height


def _paint_mask(
    image_bgr: np.ndarray,
    mask: Optional[np.ndarray],
    color_bgr: Tuple[int, int, int],
    alpha: float,
) -> np.ndarray:
    """Blend one binary mask into an image without changing its dimensions."""

    if mask is None:
        return image_bgr
    selected = mask > 0
    if not np.any(selected):
        return image_bgr
    color = np.asarray(color_bgr, dtype=np.float32)
    image_bgr[selected] = (
        image_bgr[selected].astype(np.float32) * (1.0 - alpha)
        + color * alpha
    ).astype(np.uint8)
    return image_bgr


def render_segmentation_overlay(
    frame_bgr: np.ndarray,
    result: SegmentationResult,
    *,
    show_legend: bool = True,
) -> np.ndarray:
    """Render road, raw line and road-gated line masks over a video frame.

    Colors are BGR because the input and output are handled by OpenCV:

    * green: drivable-road segmentation
    * red: raw YOLOP lane-line segmentation
    * yellow: line pixels remaining after road gating
    """

    output = frame_bgr.copy()
    _paint_mask(output, result.road_mask, (0, 180, 0), 0.28)
    _paint_mask(output, result.raw_line_mask, (0, 0, 255), 0.55)
    _paint_mask(output, result.line_mask, (0, 255, 255), 0.75)

    if show_legend:
        legend = [
            ("ROAD", (0, 180, 0)),
            ("RAW LINE", (0, 0, 255)),
            ("ROAD-GATED LINE", (0, 255, 255)),
        ]
        cv2.rectangle(output, (12, 12), (300, 116), (0, 0, 0), -1)
        cv2.rectangle(output, (12, 12), (300, 116), (220, 220, 220), 1)
        for index, (label, color) in enumerate(legend):
            y = 38 + index * 25
            cv2.line(output, (25, y - 5), (50, y - 5), color, 6)
            cv2.putText(
                output,
                label,
                (62, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
    return output


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run YOLOP road/lane segmentation over a video and save an overlay."
    )
    parser.add_argument("--input", required=True, type=Path, help="input video path")
    parser.add_argument(
        "--output", required=True, type=Path, help="output overlay video path"
    )
    parser.add_argument(
        "--model",
        "--model-path",
        dest="model_path",
        required=True,
        type=Path,
        help="YOLOP ONNX model path",
    )
    parser.add_argument(
        "--profile",
        choices=("auto", "360p", "720p"),
        default="auto",
        help="model input profile; auto selects from input video height",
    )
    parser.add_argument(
        "--road-threshold", type=float, default=0.50, help="road mask threshold"
    )
    parser.add_argument(
        "--line-threshold", type=float, default=0.50, help="line mask threshold"
    )
    parser.add_argument(
        "--road-gate-kernel",
        type=int,
        default=21,
        help="odd dilation kernel used for road-gating the line mask",
    )
    parser.add_argument(
        "--codec",
        default="mp4v",
        help="four-character OpenCV VideoWriter codec, default: mp4v",
    )
    parser.add_argument(
        "--no-legend",
        action="store_true",
        help="do not draw the color legend on output frames",
    )
    return parser


def run(args: argparse.Namespace) -> int:
    input_path = args.input.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    model_path = args.model_path.expanduser().resolve()

    if not input_path.is_file():
        raise FileNotFoundError(f"input video does not exist: {input_path}")
    if not model_path.is_file():
        raise FileNotFoundError(f"YOLOP model does not exist: {model_path}")
    if input_path == output_path:
        raise ValueError("--output must be different from --input")
    if len(args.codec) != 4:
        raise ValueError("--codec must contain exactly four characters")

    capture = cv2.VideoCapture(str(input_path))
    if not capture.isOpened():
        raise RuntimeError(f"could not open input video: {input_path}")

    frame_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    if frame_width <= 0 or frame_height <= 0:
        capture.release()
        raise RuntimeError("input video has no readable frame dimensions")
    if not np.isfinite(fps) or fps <= 0.0:
        fps = 30.0

    profile, input_width, input_height = select_profile(
        args.profile, frame_width, frame_height
    )
    segmenter = YolopSegmenter(
        YolopConfig(
            model_path=str(model_path),
            input_width=input_width,
            input_height=input_height,
            road_threshold=args.road_threshold,
            line_threshold=args.line_threshold,
            road_gate_kernel=args.road_gate_kernel,
        )
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*args.codec),
        fps,
        (frame_width, frame_height),
    )
    if not writer.isOpened():
        capture.release()
        raise RuntimeError(
            f"could not open output writer: {output_path}; try --codec avc1 or mp4v"
        )

    frame_count = 0
    try:
        print(
            f"profile={profile} model_input={input_width}x{input_height} "
            f"video={frame_width}x{frame_height} fps={fps:.3f}"
        )
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            result = segmenter.segment(frame)
            writer.write(
                render_segmentation_overlay(
                    frame, result, show_legend=not args.no_legend
                )
            )
            frame_count += 1
            if frame_count % 100 == 0:
                print(f"processed_frames={frame_count}")
    finally:
        capture.release()
        writer.release()

    if frame_count == 0:
        raise RuntimeError("input video contained no readable frames")
    print(f"wrote={output_path} frames={frame_count}")
    return 0


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    try:
        return run(args)
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        parser.error(str(error))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
