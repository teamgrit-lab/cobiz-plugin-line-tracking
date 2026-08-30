#!/usr/bin/env python3
"""Convert one ROS 2 camera topic in MCAP bag files to an MP4 video.

The input MCAP file or rosbag directory is only opened for reading. ROS 2 is
not required; ``mcap`` and ``mcap-ros2-support`` decode the CDR messages and
FFmpeg receives the image frames through stdin.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from itertools import chain, islice
import os
from pathlib import Path
import shutil
import statistics
import subprocess
import tempfile
from typing import Any, Iterable, Iterator, Optional, Sequence


RAW_IMAGE_TYPE = "sensor_msgs/msg/Image"
COMPRESSED_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage"
SUPPORTED_MESSAGE_TYPES = {RAW_IMAGE_TYPE, COMPRESSED_IMAGE_TYPE}
RAW_ENCODINGS = {
    "rgb8": ("rgb24", 3),
    "bgr8": ("bgr24", 3),
    "mono8": ("gray", 1),
    "rgba8": ("rgba", 4),
    "bgra8": ("bgra", 4),
}


@dataclass(frozen=True)
class VideoFrame:
    """One FFmpeg-ready frame and its MCAP timestamp."""

    payload: bytes
    timestamp_ns: int
    message_type: str
    input_format: str
    width: int = 0
    height: int = 0
    encoding: str = ""


def resolve_mcap_files(input_path: Path) -> list[Path]:
    """Resolve a single MCAP file or every split file in a bag directory."""

    path = input_path.expanduser().resolve()
    if path.is_file():
        if path.suffix.lower() != ".mcap":
            raise ValueError(f"input file is not MCAP: {path}")
        return [path]
    if not path.is_dir():
        raise FileNotFoundError(f"MCAP file or rosbag directory not found: {path}")
    files = sorted(item for item in path.iterdir() if item.suffix.lower() == ".mcap")
    if not files:
        raise ValueError(f"rosbag directory contains no .mcap files: {path}")
    return files


def _contiguous_raw_payload(message: Any, bytes_per_pixel: int) -> bytes:
    """Remove ROS Image row padding while validating dimensions and size."""

    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    if width <= 0 or height <= 0:
        raise ValueError(f"invalid image dimensions: {width}x{height}")
    row_bytes = width * bytes_per_pixel
    if step < row_bytes:
        raise ValueError(
            f"image step {step} is smaller than the required row size {row_bytes}"
        )
    data = bytes(message.data)
    required_size = step * height
    if len(data) < required_size:
        raise ValueError(
            f"image data has {len(data)} bytes but {required_size} are required"
        )
    if step == row_bytes:
        return data[:required_size]
    return b"".join(
        data[row * step : row * step + row_bytes] for row in range(height)
    )


def decode_video_frame(
    decoded_message: Any,
    message_type: str,
    timestamp_ns: int,
) -> VideoFrame:
    """Convert a dynamically decoded ROS message to an FFmpeg input frame."""

    if message_type == RAW_IMAGE_TYPE:
        encoding = str(decoded_message.encoding).lower()
        if encoding not in RAW_ENCODINGS:
            raise ValueError(
                f"unsupported raw image encoding '{encoding}'; "
                f"supported encodings: {', '.join(sorted(RAW_ENCODINGS))}"
            )
        input_format, bytes_per_pixel = RAW_ENCODINGS[encoding]
        return VideoFrame(
            payload=_contiguous_raw_payload(decoded_message, bytes_per_pixel),
            timestamp_ns=timestamp_ns,
            message_type=message_type,
            input_format=input_format,
            width=int(decoded_message.width),
            height=int(decoded_message.height),
            encoding=encoding,
        )

    if message_type == COMPRESSED_IMAGE_TYPE:
        compression = str(decoded_message.format).lower()
        if "jpeg" in compression or "jpg" in compression:
            encoding = "jpeg"
        elif "png" in compression:
            encoding = "png"
        else:
            raise ValueError(
                f"unsupported compressed image format '{decoded_message.format}'; "
                "expected JPEG or PNG"
            )
        payload = bytes(decoded_message.data)
        if not payload:
            raise ValueError("compressed image message has no data")
        return VideoFrame(
            payload=payload,
            timestamp_ns=timestamp_ns,
            message_type=message_type,
            input_format="image2pipe",
            encoding=encoding,
        )

    raise ValueError(
        f"topic type '{message_type}' is not a supported camera message; "
        f"expected one of {sorted(SUPPORTED_MESSAGE_TYPES)}"
    )


def iter_camera_frames(
    mcap_files: Sequence[Path],
    topic: str,
    frame_step: int,
) -> Iterator[VideoFrame]:
    """Yield every Nth decoded image from one or more split MCAP files."""

    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError as error:
        raise RuntimeError(
            "MCAP dependencies are unavailable; install 'mcap' and "
            "'mcap-ros2-support' or use the documented uv command"
        ) from error

    source_index = 0
    found_topic = False
    for mcap_file in mcap_files:
        with mcap_file.open("rb") as stream:
            reader = make_reader(stream, decoder_factories=[DecoderFactory()])
            for schema, channel, message, decoded in reader.iter_decoded_messages(
                topics=[topic]
            ):
                found_topic = True
                message_type = schema.name
                if message_type not in SUPPORTED_MESSAGE_TYPES:
                    raise ValueError(
                        f"topic '{channel.topic}' has unsupported type "
                        f"'{message_type}'"
                    )
                current_index = source_index
                source_index += 1
                if current_index % frame_step != 0:
                    continue
                yield decode_video_frame(decoded, message_type, int(message.log_time))
    if not found_topic:
        raise ValueError(f"camera topic '{topic}' was not found in the MCAP input")


def estimate_fps(frames: Sequence[VideoFrame], fallback: float = 20.0) -> float:
    """Estimate constant output FPS from the median positive timestamp delta."""

    deltas = [
        current.timestamp_ns - previous.timestamp_ns
        for previous, current in zip(frames, frames[1:])
        if current.timestamp_ns > previous.timestamp_ns
    ]
    if not deltas:
        return fallback
    fps = 1_000_000_000.0 / statistics.median(deltas)
    if not 0.1 <= fps <= 240.0:
        return fallback
    return fps


def _validate_matching_frame(first: VideoFrame, current: VideoFrame) -> None:
    if current.message_type != first.message_type:
        raise ValueError("camera topic changed message type within the bag")
    if current.input_format != first.input_format:
        raise ValueError("camera topic changed image encoding within the bag")
    if first.message_type == RAW_IMAGE_TYPE and (
        current.width != first.width or current.height != first.height
    ):
        raise ValueError(
            "camera resolution changed from "
            f"{first.width}x{first.height} to {current.width}x{current.height}"
        )


def build_ffmpeg_command(
    *,
    ffmpeg: str,
    first_frame: VideoFrame,
    output_path: Path,
    fps: float,
    codec: str,
    crf: int,
) -> list[str]:
    """Build a constant-frame-rate MP4 encoding command."""

    command = [ffmpeg, "-hide_banner", "-loglevel", "error", "-y"]
    if first_frame.message_type == RAW_IMAGE_TYPE:
        command.extend(
            [
                "-f",
                "rawvideo",
                "-pixel_format",
                first_frame.input_format,
                "-video_size",
                f"{first_frame.width}x{first_frame.height}",
                "-framerate",
                f"{fps:.8f}",
                "-i",
                "pipe:0",
            ]
        )
    else:
        command.extend(
            [
                "-f",
                "image2pipe",
                "-framerate",
                f"{fps:.8f}",
                "-i",
                "pipe:0",
            ]
        )
    command.extend(
        [
            "-an",
            "-c:v",
            codec,
            "-preset",
            "medium",
            "-crf",
            str(crf),
            "-vf",
            "pad=ceil(iw/2)*2:ceil(ih/2)*2",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(output_path),
        ]
    )
    return command


def _take_initial_frames(
    frames: Iterator[VideoFrame], count: int
) -> tuple[list[VideoFrame], Iterator[VideoFrame]]:
    buffered: list[VideoFrame] = []
    for _ in range(count):
        try:
            buffered.append(next(frames))
        except StopIteration:
            break
    return buffered, frames


def encode_frames(
    frames: Iterable[VideoFrame],
    output_path: Path,
    *,
    fps: float,
    ffmpeg: str,
    codec: str,
    crf: int,
    max_frames: int,
) -> int:
    """Stream frames to FFmpeg and atomically publish the finished MP4."""

    output = output_path.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.suffix.lower() != ".mp4":
        raise ValueError("output path must end in .mp4")
    frame_iterator = iter(frames)
    buffered, frame_iterator = _take_initial_frames(
        frame_iterator,
        min(60, max_frames) if max_frames else 60,
    )
    if not buffered:
        raise ValueError("camera topic contained no readable image messages")
    first = buffered[0]
    output_fps = fps if fps > 0.0 else estimate_fps(buffered)

    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.stem}.", suffix=".mp4", dir=output.parent
    )
    os.close(file_descriptor)
    temporary_path = Path(temporary_name)
    process: Optional[subprocess.Popen[bytes]] = None
    frame_count = 0
    try:
        command = build_ffmpeg_command(
            ffmpeg=ffmpeg,
            first_frame=first,
            output_path=temporary_path,
            fps=output_fps,
            codec=codec,
            crf=crf,
        )
        process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        assert process.stdin is not None
        output_frames: Iterable[VideoFrame] = chain(buffered, frame_iterator)
        if max_frames:
            output_frames = islice(output_frames, max_frames)
        for frame in output_frames:
            _validate_matching_frame(first, frame)
            process.stdin.write(frame.payload)
            frame_count += 1
            if frame_count % 200 == 0:
                print(f"encoded_frames={frame_count}", flush=True)
        process.stdin.close()
        assert process.stderr is not None
        stderr = process.stderr.read().decode("utf-8", errors="replace").strip()
        return_code = process.wait()
        if return_code != 0:
            raise RuntimeError(f"FFmpeg failed with code {return_code}: {stderr}")
        if (
            frame_count == 0
            or not temporary_path.is_file()
            or temporary_path.stat().st_size == 0
        ):
            raise RuntimeError("FFmpeg produced no MP4 output")
        os.replace(temporary_path, output)
    except BrokenPipeError as error:
        stderr = ""
        if process is not None and process.stderr is not None:
            stderr = process.stderr.read().decode("utf-8", errors="replace").strip()
        raise RuntimeError(
            f"FFmpeg stopped while receiving frames: {stderr}"
        ) from error
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait()
        if temporary_path.exists():
            temporary_path.unlink()

    print(
        f"wrote={output} frames={frame_count} fps={output_fps:.5f} "
        f"encoding={first.encoding or first.input_format}"
    )
    return frame_count


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert a ROS 2 Image or CompressedImage MCAP topic to MP4."
    )
    parser.add_argument(
        "--input",
        required=True,
        type=Path,
        help="MCAP file or rosbag directory containing one or more MCAP files",
    )
    parser.add_argument("--topic", required=True, help="camera image topic name")
    parser.add_argument("--output", required=True, type=Path, help="output .mp4 path")
    parser.add_argument(
        "--fps",
        type=float,
        default=0.0,
        help="output FPS; 0 estimates it from MCAP timestamps",
    )
    parser.add_argument(
        "--frame-step",
        type=int,
        default=1,
        help="encode every Nth camera message, default: 1",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="maximum encoded frames; 0 encodes the complete topic",
    )
    parser.add_argument(
        "--ffmpeg",
        default="ffmpeg",
        help="FFmpeg executable name or path, default: ffmpeg",
    )
    parser.add_argument("--codec", default="libx264", help="FFmpeg video codec")
    parser.add_argument("--crf", type=int, default=18, help="FFmpeg CRF quality value")
    parser.add_argument(
        "--overwrite", action="store_true", help="replace an existing output file"
    )
    return parser


def run(args: argparse.Namespace) -> int:
    if args.frame_step <= 0:
        raise ValueError("--frame-step must be positive")
    if args.max_frames < 0:
        raise ValueError("--max-frames must be zero or positive")
    if args.fps < 0.0:
        raise ValueError("--fps must be zero or positive")
    if not 0 <= args.crf <= 63:
        raise ValueError("--crf must be in [0, 63]")
    ffmpeg = shutil.which(args.ffmpeg)
    if ffmpeg is None:
        raise FileNotFoundError(f"FFmpeg executable not found: {args.ffmpeg}")

    output_path = args.output.expanduser().resolve()
    if output_path.exists() and not args.overwrite:
        raise FileExistsError(
            f"output already exists: {output_path}; pass --overwrite to replace it"
        )
    mcap_files = resolve_mcap_files(args.input)
    frames = iter_camera_frames(mcap_files, args.topic, args.frame_step)
    encode_frames(
        frames,
        output_path,
        fps=args.fps,
        ffmpeg=ffmpeg,
        codec=args.codec,
        crf=args.crf,
        max_frames=args.max_frames,
    )
    return 0


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return run(args)
    except (FileExistsError, FileNotFoundError, RuntimeError, ValueError) as error:
        parser.error(str(error))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
