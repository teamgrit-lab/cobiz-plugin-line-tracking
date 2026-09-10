#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = [
#   "mcap==1.4.0",
#   "mcap-ros2-support==0.5.7",
#   "numpy==2.5.2",
#   "opencv-python-headless==5.0.0.93",
#   "pillow==12.3.0",
#   "scipy==1.18.1",
#   "torch==2.13.0",
#   "torchvision==0.28.0",
#   "transformers==5.16.1",
# ]
# ///
"""One-command MCAP overlay tests with the retained Swin-L quality baseline.

sidewalk: infer every camera frame and overlay Road/Sidewalk segmentation.
local-path: reuse the existing path smoothing and LiDAR debug pipeline.
Both modes write an MP4 and a JSON report, optionally opening the video.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
from collections.abc import Sequence
from datetime import datetime, timezone
from pathlib import Path

DEFAULT_IMAGE_TOPIC = "/a2/front_camera/res_360p/image_raw"
DEFAULT_LIDAR_TOPIC = "/unitree/slam_lidar/points2"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("sidewalk", "local-path"))
    parser.add_argument(
        "--input", type=Path, required=True, help="one ROS 2 .mcap file"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="default: a new folder in rosbag-results/swin-l-tests",
    )
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--lidar-topic", default=DEFAULT_LIDAR_TOPIC)
    parser.add_argument("--device", default="auto", help="auto, mps, cuda or cpu")
    parser.add_argument(
        "--start-offset", type=float, default=0.0, help="seconds from bag start"
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=200,
        help="camera frames; 0 processes the full bag",
    )
    parser.add_argument(
        "--output-fps", type=float, default=20.0, help="MP4 playback FPS"
    )
    parser.add_argument(
        "--inference-hz",
        type=float,
        default=4.0,
        help="local-path only; sidewalk always infers every frame",
    )
    parser.add_argument(
        "--open",
        action="store_true",
        help="open the finished MP4 in the system video player",
    )
    args = parser.parse_args(argv)
    args.input = args.input.expanduser().resolve()
    if not args.input.is_file() or args.input.suffix.lower() != ".mcap":
        parser.error(f"input must be an existing .mcap file: {args.input}")
    if args.start_offset < 0 or args.max_frames < 0:
        parser.error("start-offset and max-frames must be non-negative")
    if args.output_fps <= 0 or args.inference_hz <= 0:
        parser.error("output-fps and inference-hz must be positive")
    return args


def prepare_outputs(args: argparse.Namespace) -> tuple[Path, Path]:
    if args.output_dir is None:
        root = Path(__file__).resolve().parents[1] / "rosbag-results" / "swin-l-tests"
        root.mkdir(parents=True, exist_ok=True)
        # Jetson's ROS Humble image can still use Python 3.10 (no datetime.UTC).
        now = datetime.now(timezone.utc).astimezone()
        prefix = f"{now:%Y%m%d-%H%M%S}-{args.mode}-"
        output_dir = Path(tempfile.mkdtemp(prefix=prefix, dir=root))
    else:
        output_dir = args.output_dir.expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
    video = output_dir / f"{args.mode}-overlay.mp4"
    report = output_dir / f"{args.mode}-report.json"
    for path in (video, report):
        if path.exists():
            raise FileExistsError(
                f"result already exists; choose another --output-dir: {path}"
            )
    return video, report


def build_debug_arguments(
    args: argparse.Namespace, video: Path, report: Path
) -> list[str]:
    from best_so_far_runtime import SWIN_L_PROFILE, resolve_profile

    profile = resolve_profile(SWIN_L_PROFILE)
    # Pin the model even when a local .env selects R50 or another checkpoint.
    # Geometry settings continue to use the existing local-path configuration.
    return [
        "mcap",
        "--input",
        str(args.input),
        "--output",
        str(video),
        "--report",
        str(report),
        "--overlay-mode",
        args.mode,
        "--profile",
        SWIN_L_PROFILE,
        "--model-id",
        profile.model_id,
        "--model-revision",
        profile.model_revision,
        "--evaluation-size",
        "360",
        "640",
        "--device",
        args.device,
        "--image-topic",
        args.image_topic,
        "--lidar-topic",
        args.lidar_topic,
        "--start-offset",
        str(args.start_offset),
        "--max-frames",
        str(args.max_frames),
        "--output-fps",
        str(args.output_fps),
        "--inference-hz",
        str(args.inference_hz),
    ]


def open_video(path: Path) -> None:
    try:
        if sys.platform == "win32":
            os.startfile(str(path))
        else:
            command = "open" if sys.platform == "darwin" else "xdg-open"
            subprocess.run([command, str(path)], check=True)
    except (OSError, subprocess.CalledProcessError) as error:
        print(
            f"Video saved, but the player could not open it: {error}\n{path}",
            file=sys.stderr,
        )


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    video, report = prepare_outputs(args)
    import swin_l_local_path_debug as debug

    debug_args = debug.parse_args(build_debug_arguments(args, video, report))
    print(
        f"SWIN_L_TEST mode={args.mode} profile={debug_args.profile}\nVIDEO={video}\nREPORT={report}",
        flush=True,
    )
    status = debug.run_mcap(debug_args)
    if status == 0 and args.open:
        open_video(video)
    return status


if __name__ == "__main__":
    raise SystemExit(main())
