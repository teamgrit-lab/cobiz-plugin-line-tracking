import sys
import threading
import time
from pathlib import Path

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from swin_l_local_path_debug import FramePacket, LatestFrameQueue  # noqa: E402


def packet(sequence: int) -> FramePacket:
    return FramePacket(
        frame_bgr=np.zeros((1, 1, 3), dtype=np.uint8),
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
