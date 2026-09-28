import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from swin_l_local_path_debug import (  # noqa: E402
    FramePacket,
    LatestFrameQueue,
    camera_image_rgb,
    validate_camera_image,
)


def packet(sequence: int) -> FramePacket:
    return FramePacket(
        image_message=object(),
        sequence=sequence,
    )


def test_rate_limit_wait_keeps_replacing_with_the_freshest_frame():
    queue = LatestFrameQueue()
    queue.put(packet(1))
    ready_at = time.monotonic() + 0.05

    def replace_pending_frame() -> None:
        time.sleep(0.01)
        queue.put(packet(2))

    producer = threading.Thread(target=replace_pending_frame)
    producer.start()
    selected = queue.get_latest_at(ready_at)
    producer.join()

    assert selected is not None
    assert selected.sequence == 2
    assert queue.overwritten == 1


def test_close_interrupts_rate_limit_wait_without_processing_pending_frame():
    queue = LatestFrameQueue()
    queue.put(packet(1))
    selected = []

    consumer = threading.Thread(
        target=lambda: selected.append(queue.get_latest_at(time.monotonic() + 5.0))
    )
    consumer.start()
    time.sleep(0.01)
    queue.close()
    consumer.join(timeout=0.5)

    assert not consumer.is_alive()
    assert selected == [None]


def test_native_rgb_preserves_channels_and_padded_rows_without_bridge_conversion():
    pixels = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
    padded = np.full((2, 12), 255, np.uint8)
    padded[:, :9] = pixels.reshape(2, 9)
    message = SimpleNamespace(
        encoding="rgb8", width=3, height=2, step=12, data=padded.tobytes()
    )
    bridge = SimpleNamespace(encoding_to_dtype_with_channels=lambda _: ("uint8", 3))
    validate_camera_image(message, bridge)

    result = camera_image_rgb(message, bridge)

    np.testing.assert_array_equal(result, pixels)
    assert result.flags.c_contiguous


def test_packed_native_rgb_shares_the_ros_message_buffer():
    data = np.arange(18, dtype=np.uint8)
    message = SimpleNamespace(
        encoding="rgb8", width=3, height=2, step=9, data=data
    )
    result = camera_image_rgb(message, object())
    assert np.shares_memory(result, data)
    np.testing.assert_array_equal(result, data.reshape(2, 3, 3))


@pytest.mark.parametrize(
    "overrides", [{"width": 0}, {"height": 0}, {"step": 8}, {"data": bytes(17)}]
)
def test_metadata_validation_rejects_malformed_images_without_decoding(overrides):
    message = SimpleNamespace(
        encoding="rgb8", width=3, height=2, step=9, data=bytes(18)
    )
    message.__dict__.update(overrides)
    bridge = SimpleNamespace(encoding_to_dtype_with_channels=lambda _: ("uint8", 3))
    with pytest.raises(ValueError, match="sensor_msgs/Image"):
        validate_camera_image(message, bridge)


def test_non_rgb_camera_encoding_is_converted_directly_to_rgb():
    message = SimpleNamespace(encoding="bgr8")
    expected = np.array([[[10, 20, 30]]], dtype=np.uint8)

    def convert(actual_message, *, desired_encoding):
        assert actual_message is message
        assert desired_encoding == "rgb8"
        return expected

    result = camera_image_rgb(message, SimpleNamespace(imgmsg_to_cv2=convert))
    assert result is expected
