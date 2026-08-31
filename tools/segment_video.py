#!/usr/bin/env python3
"""Generate a YOLOP, OpenCV, or mixed line-segmentation overlay video.

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
from line_tracking.vision import (  # noqa: E402
    LaneLineResult,
    RoadLaneResult,
    VisionConfig,
    VisionResult,
    YellowLineVision,
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


def render_opencv_overlay(
    frame_bgr: np.ndarray,
    result: VisionResult,
    *,
    show_legend: bool = True,
) -> np.ndarray:
    """Render the OpenCV-only yellow-color mask and configured ROI."""

    output = frame_bgr.copy()
    _paint_mask(output, result.mask, (0, 255, 255), 0.75)
    roi_polygon = result.line_roi_polygon_px
    if roi_polygon is None:
        roi_polygon = result.roi_polygon_px
    if roi_polygon.size:
        polygon = roi_polygon.reshape(-1, 1, 2).astype(np.int32)
        cv2.polylines(output, [polygon], isClosed=True, color=(255, 255, 255), thickness=2)

    if show_legend:
        cv2.rectangle(output, (12, 12), (330, 92), (0, 0, 0), -1)
        cv2.rectangle(output, (12, 12), (330, 92), (220, 220, 220), 1)
        cv2.line(output, (25, 38), (50, 38), (0, 255, 255), 6)
        cv2.putText(
            output,
            "OPENCV YELLOW MASK",
            (62, 43),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.line(output, (25, 68), (50, 68), (255, 255, 255), 2)
        cv2.putText(
            output,
            "ROI",
            (62, 73),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return output


def render_line_first_overlay(
    frame_bgr: np.ndarray,
    result: VisionResult,
    *,
    show_legend: bool = True,
) -> np.ndarray:
    """Render Hough line corridors and their yellow-color intersection."""

    output = frame_bgr.copy()
    _paint_mask(output, result.line_feature_mask, (255, 255, 0), 0.42)
    _paint_mask(output, result.mask, (0, 255, 255), 0.90)

    roi_polygon = result.line_roi_polygon_px
    if roi_polygon is None:
        roi_polygon = result.roi_polygon_px
    if roi_polygon.size:
        polygon = roi_polygon.reshape(-1, 1, 2).astype(np.int32)
        cv2.polylines(
            output,
            [polygon],
            isClosed=True,
            color=(255, 255, 255),
            thickness=2,
        )

    if show_legend:
        legend = [
            ("HOUGH LINE CANDIDATES", (255, 255, 0), 6),
            ("YELLOW LINE FINAL", (0, 255, 255), 6),
            ("ROI", (255, 255, 255), 2),
        ]
        cv2.rectangle(output, (12, 12), (360, 112), (0, 0, 0), -1)
        cv2.rectangle(output, (12, 12), (360, 112), (220, 220, 220), 1)
        for index, (label, color, thickness) in enumerate(legend):
            y = 38 + index * 30
            cv2.line(output, (25, y - 5), (50, y - 5), color, thickness)
            cv2.putText(
                output,
                label,
                (62, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
    return output


def render_road_lines_overlay(
    frame_bgr: np.ndarray,
    road_mask: np.ndarray,
    road_line_mask: np.ndarray,
    *,
    show_legend: bool = True,
) -> np.ndarray:
    """Render YOLOP's full road mask and OpenCV lines inside that mask."""

    output = frame_bgr.copy()
    _paint_mask(output, road_mask, (0, 180, 0), 0.28)
    _paint_mask(output, road_line_mask, (255, 255, 0), 0.90)

    if show_legend:
        legend = [
            ("YOLOP ROAD MASK", (0, 180, 0), 6),
            ("OPENCV ROAD LINES", (255, 255, 0), 6),
        ]
        cv2.rectangle(output, (12, 12), (360, 92), (0, 0, 0), -1)
        cv2.rectangle(output, (12, 12), (360, 92), (220, 220, 220), 1)
        for index, (label, color, thickness) in enumerate(legend):
            y = 38 + index * 30
            cv2.line(output, (25, y - 5), (50, y - 5), color, thickness)
            cv2.putText(
                output,
                label,
                (62, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
    return output


def render_gray_road_lines_overlay(
    frame_bgr: np.ndarray,
    gray_road_mask: np.ndarray,
    road_line_mask: np.ndarray,
    roi_polygon_px: Optional[np.ndarray] = None,
    *,
    show_legend: bool = True,
) -> np.ndarray:
    """Render the OpenCV gray-road ROI mask and lines found inside it."""

    output = frame_bgr.copy()
    _paint_mask(output, gray_road_mask, (0, 180, 0), 0.28)
    _paint_mask(output, road_line_mask, (255, 255, 0), 0.90)

    if roi_polygon_px is not None and roi_polygon_px.size:
        polygon = roi_polygon_px.reshape(-1, 1, 2).astype(np.int32)
        cv2.polylines(
            output,
            [polygon],
            isClosed=True,
            color=(255, 255, 255),
            thickness=2,
        )

    if show_legend:
        legend = [
            ("OPENCV GRAY ROAD", (0, 180, 0), 6),
            ("OPENCV ROAD LINES", (255, 255, 0), 6),
            ("ROI", (255, 255, 255), 2),
        ]
        cv2.rectangle(output, (12, 12), (360, 122), (0, 0, 0), -1)
        cv2.rectangle(output, (12, 12), (360, 122), (220, 220, 220), 1)
        for index, (label, color, thickness) in enumerate(legend):
            y = 38 + index * 30
            cv2.line(output, (25, y - 5), (50, y - 5), color, thickness)
            cv2.putText(
                output,
                label,
                (62, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
    return output


def render_lane_only_overlay(
    frame_bgr: np.ndarray,
    result: LaneLineResult,
    *,
    lane_color: str = "both",
    road_mask: Optional[np.ndarray] = None,
    show_legend: bool = True,
) -> np.ndarray:
    """Render color-supported Hough lanes, optionally over a road mask."""

    output = frame_bgr.copy()
    if road_mask is not None:
        _paint_mask(output, road_mask, (0, 180, 0), 0.28)
    yellow_mask = result.yellow_line_mask > 0
    white_mask = result.white_line_mask > 0
    if lane_color == "both":
        overlap_mask = (yellow_mask & white_mask).astype(np.uint8) * 255
        yellow_only_mask = (yellow_mask & ~white_mask).astype(np.uint8) * 255
        white_only_mask = (white_mask & ~yellow_mask).astype(np.uint8) * 255
        _paint_mask(output, yellow_only_mask, (0, 255, 255), 0.95)
        _paint_mask(output, white_only_mask, (255, 255, 255), 0.95)
        _paint_mask(output, overlap_mask, (0, 0, 255), 0.95)
    elif lane_color == "yellow":
        _paint_mask(output, result.yellow_line_mask, (0, 255, 255), 0.95)
    elif lane_color == "white":
        _paint_mask(output, result.white_line_mask, (255, 255, 255), 0.95)

    if result.roi_polygon_px.size:
        polygon = result.roi_polygon_px.reshape(-1, 1, 2).astype(np.int32)
        cv2.polylines(
            output,
            [polygon],
            isClosed=True,
            color=(180, 180, 180),
            thickness=2,
        )

    if show_legend:
        legend = []
        if road_mask is not None:
            legend.append(("OPENCV GRAY ROAD", (0, 180, 0), 6))
        if lane_color in ("both", "yellow"):
            legend.append(("YELLOW LANE LINES", (0, 255, 255), 5))
        if lane_color in ("both", "white"):
            legend.append(("WHITE LANE LINES", (255, 255, 255), 5))
        if lane_color == "both":
            legend.append(("OVERLAP", (0, 0, 255), 5))
        legend.append(("ROI", (180, 180, 180), 2))
        legend_bottom = 12 + 20 + len(legend) * 30
        cv2.rectangle(output, (12, 12), (360, legend_bottom), (0, 0, 0), -1)
        cv2.rectangle(
            output,
            (12, 12),
            (360, legend_bottom),
            (220, 220, 220),
            1,
        )
        for index, (label, color, thickness) in enumerate(legend):
            y = 38 + index * 30
            cv2.line(output, (25, y - 5), (50, y - 5), color, thickness)
            cv2.putText(
                output,
                label,
                (62, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
    return output


def render_advanced_lane_overlay(
    frame_bgr: np.ndarray,
    result: RoadLaneResult,
    *,
    show_legend: bool = True,
) -> np.ndarray:
    """Render a fitted left/right lane pair and the corridor between them."""

    output = frame_bgr.copy()
    if result.detected:
        lane_mask = np.zeros(frame_bgr.shape[:2], dtype=np.uint8)
        polygon = np.rint(result.lane_polygon_px).astype(np.int32)
        cv2.fillPoly(lane_mask, [polygon], 255)
        _paint_mask(output, lane_mask, (0, 180, 0), 0.34)
        cv2.polylines(
            output,
            [np.rint(result.left_curve_px).astype(np.int32)],
            isClosed=False,
            color=(0, 255, 255),
            thickness=5,
            lineType=cv2.LINE_AA,
        )
        cv2.polylines(
            output,
            [np.rint(result.right_curve_px).astype(np.int32)],
            isClosed=False,
            color=(255, 255, 255),
            thickness=5,
            lineType=cv2.LINE_AA,
        )
        cv2.polylines(
            output,
            [np.rint(result.centerline_points_px).astype(np.int32)],
            isClosed=False,
            color=(255, 255, 0),
            thickness=3,
            lineType=cv2.LINE_AA,
        )

    if show_legend:
        panel_width = min(max(frame_bgr.shape[1] - 24, 240), 430)
        cv2.rectangle(output, (12, 12), (panel_width, 128), (0, 0, 0), -1)
        cv2.rectangle(
            output,
            (12, 12),
            (panel_width, 128),
            (220, 220, 220),
            1,
        )
        status = "LANE PAIR DETECTED" if result.detected else "LANE PAIR NOT DETECTED"
        status_color = (80, 255, 80) if result.detected else (80, 80, 255)
        cv2.putText(
            output,
            status,
            (24, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            status_color,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            output,
            f"confidence={result.confidence:.2f}",
            (24, 66),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        curvature = (
            f"{result.curvature_m:.1f} m"
            if result.curvature_m is not None
            else "n/a"
        )
        cv2.putText(
            output,
            f"curvature={curvature}",
            (24, 91),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        offset = (
            f"{result.center_offset_m:+.2f} m"
            if result.center_offset_m is not None
            else "n/a"
        )
        cv2.putText(
            output,
            f"vehicle_offset={offset} (right +)",
            (24, 116),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return output


def render_mix_overlay(
    frame_bgr: np.ndarray,
    model_result: SegmentationResult,
    mixed_result: VisionResult,
    *,
    show_legend: bool = True,
) -> np.ndarray:
    """Render every YOLOP-to-OpenCV stage used by the mixed backend."""

    output = frame_bgr.copy()
    _paint_mask(output, model_result.road_mask, (0, 180, 0), 0.22)
    _paint_mask(output, model_result.raw_line_mask, (0, 0, 255), 0.42)
    _paint_mask(output, model_result.line_mask, (255, 255, 0), 0.58)
    _paint_mask(output, mixed_result.mask, (0, 255, 255), 0.90)

    roi_polygon = mixed_result.line_roi_polygon_px
    if roi_polygon is None:
        roi_polygon = mixed_result.roi_polygon_px
    if roi_polygon.size:
        polygon = roi_polygon.reshape(-1, 1, 2).astype(np.int32)
        cv2.polylines(
            output,
            [polygon],
            isClosed=True,
            color=(255, 255, 255),
            thickness=2,
        )

    if show_legend:
        legend = [
            ("YOLOP ROAD", (0, 180, 0), 6),
            ("YOLOP RAW LINE", (0, 0, 255), 6),
            ("YOLOP ROAD-GATED LINE", (255, 255, 0), 6),
            ("MIX FINAL LINE", (0, 255, 255), 6),
            ("ROI", (255, 255, 255), 2),
        ]
        cv2.rectangle(output, (12, 12), (360, 162), (0, 0, 0), -1)
        cv2.rectangle(output, (12, 12), (360, 162), (220, 220, 220), 1)
        for index, (label, color, thickness) in enumerate(legend):
            y = 38 + index * 27
            cv2.line(output, (25, y - 5), (50, y - 5), color, thickness)
            cv2.putText(
                output,
                label,
                (62, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
    return output


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run YOLOP, OpenCV, or mixed line segmentation over a video."
    )
    parser.add_argument("--input", required=True, type=Path, help="input video path")
    parser.add_argument(
        "--output", required=True, type=Path, help="output overlay video path"
    )
    parser.add_argument(
        "--model",
        "--model-path",
        dest="model_path",
        type=Path,
        help="YOLOP ONNX model path; required for --backend yolop, road-lines, or mix",
    )
    parser.add_argument(
        "--backend",
        choices=(
            "yolop",
            "opencv",
            "line-first",
            "lane-only",
            "advanced-lane",
            "road-lines",
            "gray-road-lines",
            "gray-road-lane-only",
            "mix",
        ),
        default="yolop",
        help=(
            "segmentation backend; mix applies OpenCV color/shape detection "
            "inside YOLOP's road-gated lane mask; line-first runs Hough "
            "before yellow-color segmentation; road-lines runs Hough only "
            "inside YOLOP's road mask; gray-road-lines builds an adaptive "
            "LAB gray-road mask before Hough; gray-road-lane-only gates "
            "lane-only by that OpenCV road mask; lane-only filters "
            "yellow/white paint before Canny/Hough; advanced-lane fits a "
            "bird's-eye quadratic left/right lane pair and road corridor"
        ),
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
    parser.add_argument(
        "--hsv-lower",
        nargs=3,
        type=int,
        metavar=("H", "S", "V"),
        default=(0, 25, 35),
        help="OpenCV lower HSV threshold",
    )
    parser.add_argument(
        "--hsv-upper",
        nargs=3,
        type=int,
        metavar=("H", "S", "V"),
        default=(42, 255, 255),
        help="OpenCV upper HSV threshold",
    )
    parser.add_argument(
        "--lab-b-min",
        type=int,
        default=110,
        help="OpenCV minimum LAB b-channel threshold",
    )
    parser.add_argument(
        "--adaptive-lab-percentile",
        type=float,
        default=0.0,
        help="OpenCV adaptive LAB percentile; use 0 to disable it",
    )
    parser.add_argument(
        "--red-blue-min",
        type=int,
        default=-8,
        help="minimum signed R-B difference for warm-camera yellow detection",
    )
    parser.add_argument(
        "--red-green-min",
        type=int,
        default=25,
        help="minimum signed R-G difference for warm-camera yellow detection",
    )
    parser.add_argument(
        "--warm-luminance-min",
        type=int,
        default=145,
        help="minimum LAB luminance for warm-camera yellow detection",
    )
    parser.add_argument(
        "--no-warm-camera-preference",
        action="store_true",
        help="disable the warm-camera mask preference and use HSV+LAB fallback",
    )
    parser.add_argument(
        "--line-close-kernel",
        type=int,
        default=31,
        help="close kernel used to join broken centerline pixels",
    )
    parser.add_argument(
        "--line-min-area-px",
        type=int,
        default=250,
        help="minimum dominant line component area at 1280px width",
    )
    parser.add_argument(
        "--line-min-span-px",
        type=int,
        default=40,
        help="minimum dominant line component span at 1280px width",
    )
    parser.add_argument(
        "--line-min-elongation",
        type=float,
        default=2.0,
        help="minimum PCA major/minor axis ratio for a line component",
    )
    parser.add_argument(
        "--no-line-feature",
        action="store_true",
        help="disable the PCA line-shape filter and use color components only",
    )
    parser.add_argument(
        "--line-first-canny-low",
        type=int,
        default=40,
        help="lower Canny threshold for the line-first backend",
    )
    parser.add_argument(
        "--line-first-canny-high",
        type=int,
        default=120,
        help="upper Canny threshold for the line-first backend",
    )
    parser.add_argument(
        "--line-first-hough-threshold",
        type=int,
        default=24,
        help="minimum Hough votes for a line-first segment",
    )
    parser.add_argument(
        "--line-first-min-length-px",
        type=int,
        default=35,
        help="minimum Hough line length at 1280px width",
    )
    parser.add_argument(
        "--line-first-max-gap-px",
        type=int,
        default=24,
        help="maximum gap joined by Hough at 1280px width",
    )
    parser.add_argument(
        "--line-first-corridor-width-px",
        type=int,
        default=31,
        help="width around Hough segments searched for yellow pixels",
    )
    parser.add_argument(
        "--line-first-recovery-width-px",
        type=int,
        default=81,
        help="width used to recover the full yellow region around line seeds",
    )
    parser.add_argument(
        "--line-first-band-close-kernel-px",
        type=int,
        default=61,
        help="closing kernel used to join both edges of a thick painted line",
    )
    parser.add_argument(
        "--road-line-canny-low",
        type=int,
        default=40,
        help="lower Canny threshold for road-lines and gray-road-lines backends",
    )
    parser.add_argument(
        "--road-line-canny-high",
        type=int,
        default=120,
        help="upper Canny threshold for road-lines and gray-road-lines backends",
    )
    parser.add_argument(
        "--road-line-hough-threshold",
        type=int,
        default=35,
        help="minimum Hough votes for road-lines and gray-road-lines backends",
    )
    parser.add_argument(
        "--road-line-min-length-px",
        type=int,
        default=45,
        help="minimum road-line Hough segment length at 1280px width",
    )
    parser.add_argument(
        "--road-line-max-gap-px",
        type=int,
        default=18,
        help="maximum gap joined by road-line Hough at 1280px width",
    )
    parser.add_argument(
        "--road-line-corridor-width-px",
        type=int,
        default=7,
        help="draw width for OpenCV road-line candidates",
    )
    parser.add_argument(
        "--gray-road-lab-tolerance",
        type=float,
        default=24.0,
        help="adaptive LAB chroma distance allowed for gray-road pixels",
    )
    parser.add_argument(
        "--gray-road-min-luminance",
        type=int,
        default=35,
        help="minimum LAB luminance for gray-road pixels",
    )
    parser.add_argument(
        "--gray-road-max-luminance",
        type=int,
        default=230,
        help="maximum LAB luminance for gray-road pixels",
    )
    parser.add_argument(
        "--gray-road-top-y",
        type=float,
        default=0.58,
        help="normalized top y of the conservative gray-road gate",
    )
    parser.add_argument(
        "--gray-road-open-kernel",
        type=int,
        default=3,
        help="odd morphology opening kernel for gray-road mask",
    )
    parser.add_argument(
        "--gray-road-close-kernel",
        type=int,
        default=21,
        help="odd morphology closing kernel for gray-road mask",
    )
    parser.add_argument(
        "--gray-road-min-component-area-px",
        type=int,
        default=1000,
        help="minimum bottom-connected gray-road component area",
    )
    parser.add_argument(
        "--lane-yellow-hsv-lower",
        nargs=3,
        type=int,
        metavar=("H", "S", "V"),
        default=(15, 70, 80),
        help="lower HSV threshold for balanced-camera yellow lane paint",
    )
    parser.add_argument(
        "--lane-yellow-hsv-upper",
        nargs=3,
        type=int,
        metavar=("H", "S", "V"),
        default=(42, 255, 255),
        help="upper HSV threshold for balanced-camera yellow lane paint",
    )
    parser.add_argument(
        "--lane-cast-yellow-hue-margin",
        type=int,
        default=12,
        help="hue margin around 0/179 for warm-camera yellow paint",
    )
    parser.add_argument(
        "--lane-cast-yellow-saturation-min",
        type=int,
        default=45,
        help="minimum saturation for warm-camera yellow paint",
    )
    parser.add_argument(
        "--lane-cast-yellow-value-min",
        type=int,
        default=120,
        help="minimum value for warm-camera yellow paint",
    )
    parser.add_argument(
        "--lane-yellow-red-blue-min",
        type=int,
        default=20,
        help="minimum R-B difference for warm-camera yellow paint",
    )
    parser.add_argument(
        "--lane-yellow-red-green-min",
        type=int,
        default=25,
        help="minimum R-G difference for warm-camera yellow paint",
    )
    parser.add_argument(
        "--lane-white-saturation-max",
        type=int,
        default=40,
        help="maximum saturation for white lane paint",
    )
    parser.add_argument(
        "--lane-white-value-min",
        type=int,
        default=200,
        help="minimum value for white lane paint",
    )
    parser.add_argument(
        "--lane-color",
        choices=("both", "yellow", "white"),
        default="both",
        help="lane-only overlay color to render",
    )
    parser.add_argument(
        "--lane-color-close-kernel",
        type=int,
        default=5,
        help="odd closing kernel for color lane candidates",
    )
    parser.add_argument(
        "--lane-canny-low",
        type=int,
        default=40,
        help="lower Canny threshold for lane-only",
    )
    parser.add_argument(
        "--lane-canny-high",
        type=int,
        default=120,
        help="upper Canny threshold for lane-only",
    )
    parser.add_argument(
        "--lane-hough-threshold",
        type=int,
        default=30,
        help="minimum Hough votes for lane-only",
    )
    parser.add_argument(
        "--lane-min-length-px",
        type=int,
        default=60,
        help="minimum lane Hough segment length at 1280px width",
    )
    parser.add_argument(
        "--lane-max-gap-px",
        type=int,
        default=20,
        help="maximum gap joined by Hough for lane-only",
    )
    parser.add_argument(
        "--lane-draw-width-px",
        type=int,
        default=5,
        help="render width of accepted lane segments",
    )
    parser.add_argument(
        "--lane-min-vertical-ratio",
        type=float,
        default=0.15,
        help="minimum absolute vertical component / segment length",
    )
    parser.add_argument(
        "--lane-min-color-support-ratio",
        type=float,
        default=0.60,
        help="minimum fraction of a Hough segment supported by lane color",
    )
    parser.add_argument(
        "--advanced-lane-windows",
        type=int,
        default=9,
        help="number of bird's-eye sliding windows",
    )
    parser.add_argument(
        "--advanced-lane-margin-px",
        type=int,
        default=45,
        help="half-width of each lane search window in bird's-eye pixels",
    )
    parser.add_argument(
        "--advanced-lane-min-pixels",
        type=int,
        default=25,
        help="pixels needed to recenter one sliding window",
    )
    parser.add_argument(
        "--advanced-lane-min-points",
        type=int,
        default=120,
        help="minimum pixels needed to fit each lane curve",
    )
    parser.add_argument(
        "--advanced-lane-min-width-ratio",
        type=float,
        default=0.35,
        help="minimum lane width divided by bird's-eye image width",
    )
    parser.add_argument(
        "--advanced-lane-max-width-ratio",
        type=float,
        default=1.05,
        help="maximum lane width divided by bird's-eye image width",
    )
    parser.add_argument(
        "--advanced-lane-max-width-std-ratio",
        type=float,
        default=0.20,
        help="maximum lane-width standard deviation divided by median width",
    )
    parser.add_argument(
        "--advanced-lane-width-m",
        type=float,
        default=3.70,
        help="real lane width used for curvature and offset metrics",
    )
    parser.add_argument(
        "--advanced-lane-visible-distance-m",
        type=float,
        default=30.0,
        help="road distance represented by the bird's-eye image",
    )
    parser.add_argument(
        "--advanced-lane-smoothing-alpha",
        type=float,
        default=0.35,
        help="new-frame weight for temporal polynomial smoothing",
    )
    return parser


def run(args: argparse.Namespace) -> int:
    input_path = args.input.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    model_path = args.model_path.expanduser().resolve() if args.model_path else None

    if not input_path.is_file():
        raise FileNotFoundError(f"input video does not exist: {input_path}")
    model_required = args.backend in ("yolop", "road-lines", "mix")
    if model_required and model_path is None:
        raise ValueError(f"--model is required when --backend is {args.backend}")
    if model_required and not model_path.is_file():
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

    segmenter = None
    vision = None
    if model_required:
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
    if args.backend in (
        "opencv",
        "line-first",
        "lane-only",
        "advanced-lane",
        "road-lines",
        "gray-road-lines",
        "gray-road-lane-only",
        "mix",
    ):
        vision = YellowLineVision(
            VisionConfig(
                hsv_lower=tuple(args.hsv_lower),
                hsv_upper=tuple(args.hsv_upper),
                lab_b_min=args.lab_b_min,
                adaptive_lab_percentile=args.adaptive_lab_percentile,
                red_blue_min=args.red_blue_min,
                red_green_min=args.red_green_min,
                warm_luminance_min=args.warm_luminance_min,
                prefer_warm_camera_mask=not args.no_warm_camera_preference,
                line_close_kernel=args.line_close_kernel,
                line_min_area_px=args.line_min_area_px,
                line_min_span_px=args.line_min_span_px,
                line_feature_enabled=not args.no_line_feature,
                line_min_elongation=args.line_min_elongation,
                line_first_canny_low=args.line_first_canny_low,
                line_first_canny_high=args.line_first_canny_high,
                line_first_hough_threshold=args.line_first_hough_threshold,
                line_first_min_length_px=args.line_first_min_length_px,
                line_first_max_gap_px=args.line_first_max_gap_px,
                line_first_corridor_width_px=args.line_first_corridor_width_px,
                line_first_recovery_width_px=args.line_first_recovery_width_px,
                line_first_band_close_kernel_px=(
                    args.line_first_band_close_kernel_px
                ),
                road_line_canny_low=args.road_line_canny_low,
                road_line_canny_high=args.road_line_canny_high,
                road_line_hough_threshold=args.road_line_hough_threshold,
                road_line_min_length_px=args.road_line_min_length_px,
                road_line_max_gap_px=args.road_line_max_gap_px,
                road_line_corridor_width_px=args.road_line_corridor_width_px,
                gray_road_lab_tolerance=args.gray_road_lab_tolerance,
                gray_road_min_luminance=args.gray_road_min_luminance,
                gray_road_max_luminance=args.gray_road_max_luminance,
                gray_road_top_y=args.gray_road_top_y,
                gray_road_open_kernel=args.gray_road_open_kernel,
                gray_road_close_kernel=args.gray_road_close_kernel,
                gray_road_min_component_area_px=(
                    args.gray_road_min_component_area_px
                ),
                lane_yellow_hsv_lower=tuple(args.lane_yellow_hsv_lower),
                lane_yellow_hsv_upper=tuple(args.lane_yellow_hsv_upper),
                lane_cast_yellow_hue_margin=args.lane_cast_yellow_hue_margin,
                lane_cast_yellow_saturation_min=(
                    args.lane_cast_yellow_saturation_min
                ),
                lane_cast_yellow_value_min=args.lane_cast_yellow_value_min,
                lane_yellow_red_blue_min=args.lane_yellow_red_blue_min,
                lane_yellow_red_green_min=args.lane_yellow_red_green_min,
                lane_white_saturation_max=args.lane_white_saturation_max,
                lane_white_value_min=args.lane_white_value_min,
                lane_color_close_kernel=args.lane_color_close_kernel,
                lane_canny_low=args.lane_canny_low,
                lane_canny_high=args.lane_canny_high,
                lane_hough_threshold=args.lane_hough_threshold,
                lane_min_length_px=args.lane_min_length_px,
                lane_max_gap_px=args.lane_max_gap_px,
                lane_draw_width_px=args.lane_draw_width_px,
                lane_min_vertical_ratio=args.lane_min_vertical_ratio,
                lane_min_color_support_ratio=args.lane_min_color_support_ratio,
                advanced_lane_windows=args.advanced_lane_windows,
                advanced_lane_margin_px=args.advanced_lane_margin_px,
                advanced_lane_min_pixels=args.advanced_lane_min_pixels,
                advanced_lane_min_points=args.advanced_lane_min_points,
                advanced_lane_min_width_ratio=(
                    args.advanced_lane_min_width_ratio
                ),
                advanced_lane_max_width_ratio=(
                    args.advanced_lane_max_width_ratio
                ),
                advanced_lane_max_width_std_ratio=(
                    args.advanced_lane_max_width_std_ratio
                ),
                advanced_lane_width_m=args.advanced_lane_width_m,
                advanced_lane_visible_distance_m=(
                    args.advanced_lane_visible_distance_m
                ),
                advanced_lane_smoothing_alpha=(
                    args.advanced_lane_smoothing_alpha
                ),
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
        if args.backend == "opencv":
            backend_details = "opencv_color=warm-camera BGR+LAB "
        elif args.backend == "line-first":
            backend_details = "opencv_order=Canny-Hough-then-yellow "
        elif args.backend == "lane-only":
            backend_details = "opencv_order=color-Canny-Hough-yellow+white "
        elif args.backend == "advanced-lane":
            backend_details = (
                "opencv_order=color-perspective-sliding-window-quadratic "
            )
        elif args.backend == "road-lines":
            backend_details = (
                f"profile={profile} model_input={input_width}x{input_height} "
                "opencv_gate=YOLOP-road "
            )
        elif args.backend == "gray-road-lines":
            backend_details = "opencv_gate=adaptive-LAB-gray-road "
        elif args.backend == "gray-road-lane-only":
            backend_details = (
                "opencv_order=adaptive-LAB-road-mask-then-color-Canny-Hough "
            )
        elif args.backend == "mix":
            backend_details = (
                f"profile={profile} model_input={input_width}x{input_height} "
                "opencv_gate=YOLOP-road-line "
            )
        else:
            backend_details = (
                f"profile={profile} model_input={input_width}x{input_height} "
            )
        print(
            f"backend={args.backend} {backend_details}"
            f"video={frame_width}x{frame_height} fps={fps:.3f}"
        )
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if args.backend == "yolop":
                result = segmenter.segment(frame)
                overlay = render_segmentation_overlay(
                    frame, result, show_legend=not args.no_legend
                )
            elif args.backend == "mix":
                model_result = segmenter.segment(frame)
                mixed_result = vision.process(
                    frame,
                    candidate_gate_mask=model_result.line_mask,
                )
                overlay = render_mix_overlay(
                    frame,
                    model_result,
                    mixed_result,
                    show_legend=not args.no_legend,
                )
            elif args.backend == "line-first":
                result = vision.process(frame, line_first=True)
                overlay = render_line_first_overlay(
                    frame,
                    result,
                    show_legend=not args.no_legend,
                )
            elif args.backend == "lane-only":
                lane_result = vision.detect_lane_lines(frame)
                overlay = render_lane_only_overlay(
                    frame,
                    lane_result,
                    lane_color=args.lane_color,
                    show_legend=not args.no_legend,
                )
            elif args.backend == "advanced-lane":
                road_lane_result = vision.detect_advanced_lanes(frame)
                overlay = render_advanced_lane_overlay(
                    frame,
                    road_lane_result,
                    show_legend=not args.no_legend,
                )
            elif args.backend == "road-lines":
                model_result = segmenter.segment(frame)
                road_line_mask = vision.detect_lines_in_road_mask(
                    frame,
                    model_result.road_mask,
                )
                overlay = render_road_lines_overlay(
                    frame,
                    model_result.road_mask,
                    road_line_mask,
                    show_legend=not args.no_legend,
                )
            elif args.backend == "gray-road-lines":
                gray_road_mask = vision.detect_gray_road_mask(frame)
                road_line_mask = vision.detect_lines_in_mask(
                    frame,
                    gray_road_mask,
                )
                roi_polygon = vision._normalized_points(
                    vision.config.line_roi_polygon,
                    frame.shape[1],
                    frame.shape[0],
                )
                overlay = render_gray_road_lines_overlay(
                    frame,
                    gray_road_mask,
                    road_line_mask,
                    roi_polygon,
                    show_legend=not args.no_legend,
                )
            elif args.backend == "gray-road-lane-only":
                gray_road_mask = vision.detect_gray_road_mask(frame)
                lane_result = vision.detect_lane_lines(
                    frame,
                    road_mask=gray_road_mask,
                )
                overlay = render_lane_only_overlay(
                    frame,
                    lane_result,
                    lane_color=args.lane_color,
                    road_mask=gray_road_mask,
                    show_legend=not args.no_legend,
                )
            else:
                result = vision.process(frame)
                overlay = render_opencv_overlay(
                    frame, result, show_legend=not args.no_legend
                )
            writer.write(overlay)
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
