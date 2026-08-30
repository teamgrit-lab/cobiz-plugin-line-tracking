from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.rosbag_mcap_to_mp4 import (
    COMPRESSED_IMAGE_TYPE,
    RAW_IMAGE_TYPE,
    VideoFrame,
    decode_video_frame,
    estimate_fps,
    resolve_mcap_files,
)


def test_resolves_sorted_split_mcap_files(tmp_path):
    (tmp_path / "bag_1.mcap").touch()
    (tmp_path / "bag_0.mcap").touch()
    (tmp_path / "metadata.yaml").touch()

    assert resolve_mcap_files(tmp_path) == [
        Path(tmp_path / "bag_0.mcap"),
        Path(tmp_path / "bag_1.mcap"),
    ]


def test_raw_rgb_frame_removes_row_padding():
    message = SimpleNamespace(
        width=2,
        height=2,
        encoding="rgb8",
        step=8,
        data=b"abcdefXXghijklYY",
    )

    frame = decode_video_frame(message, RAW_IMAGE_TYPE, 123)

    assert frame.payload == b"abcdefghijkl"
    assert frame.input_format == "rgb24"
    assert frame.width == 2
    assert frame.height == 2


def test_decodes_jpeg_compressed_image():
    message = SimpleNamespace(format="jpeg", data=b"jpeg-data")

    frame = decode_video_frame(message, COMPRESSED_IMAGE_TYPE, 123)

    assert frame.payload == b"jpeg-data"
    assert frame.input_format == "image2pipe"
    assert frame.encoding == "jpeg"


def test_estimates_effective_fps_from_selected_frame_timestamps():
    frames = [
        VideoFrame(b"", timestamp, RAW_IMAGE_TYPE, "rgb24", 640, 360, "rgb8")
        for timestamp in (0, 100_000_000, 200_000_000)
    ]

    assert estimate_fps(frames) == pytest.approx(10.0)


def test_rejects_unsupported_raw_encoding():
    message = SimpleNamespace(
        width=2,
        height=2,
        encoding="16UC1",
        step=4,
        data=b"12345678",
    )

    with pytest.raises(ValueError, match="unsupported raw image encoding"):
        decode_video_frame(message, RAW_IMAGE_TYPE, 123)
