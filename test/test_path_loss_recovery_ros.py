"""Camera-only recovery remains available when the avoidance overlay is disabled."""

import json
import time
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_apriltag_task_stop_ros import Message, RosHarness, SPORT, ZERO, debug


@pytest.mark.parametrize("recover_at", [2, 3, None])
def test_completed_inferences_recover_straight_stop_on_third_failure_and_reset(
    monkeypatch, recover_at
):
    ros = RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV, "LINE_TRACKING_PATH_LOSS_RECOVERY_ENABLED", "true")
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(
            debug.ENV, "LINE_TRACKING_STOP_ON_" + check.upper(), "false"
        )
    good = np.zeros((360, 640), np.uint8)
    good[:, :320] = 2
    detection = NS(mask=good.copy())
    monkeypatch.setattr(
        debug,
        "BestSoFarSegmenter",
        lambda _: NS(
            device=NS(type="cuda"),
            reset=lambda: None,
            segment=lambda _frame, **kwargs: NS(
                selected_mask=detection.mask.copy(), inference_seconds=0.01
            ),
        ),
    )

    def scenario(node):
        node.drive_config = replace(node.drive_config, heading_gain=2.0)
        ros.start()

        def feed():
            ros.now += 0.05
            node.publish_state()
            previous = ros.metrics()["inference_count"]
            image = Message()
            image.header.stamp = ros.stamp()
            node.on_image(image)
            deadline = time.perf_counter() + 5
            while True:
                node.publish_state()
                if ros.metrics()["inference_count"] > previous:
                    return ros.metrics()
                assert time.perf_counter() < deadline, ros.errors
                time.sleep(0.001)

        detection.mask[:] = 0
        assert feed()["drive_reason"] == "waiting_for_path"
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        detection.mask = good.copy()
        assert feed()["path_tracked"]
        saved = json.loads(ros.published[SPORT][-1].parameter)
        assert saved["x"] > 0 and abs(saved["z"]) > 0
        for count in range(1, 4):
            detection.mask = good.copy() if count == recover_at else np.zeros_like(good)
            result = feed()
            if count == recover_at:
                assert result["path_tracked"]
                assert result["path_recovery"]["failed_inferences"] == 0
                assert json.loads(ros.published[SPORT][-1].parameter)["z"] != 0
                break
            expected = (
                "tracking_path_recovery" if count < 3 else "path_recovery_exhausted"
            )
            assert result["drive_reason"] == expected
            assert result["path_recovery"]["failed_inferences"] == count
            command = json.loads(ros.published[SPORT][-1].parameter)
            assert command == (
                {"x": saved["x"], "y": 0.0, "z": 0.0} if count < 3 else ZERO
            )
            for _ in range(10):
                node.publish_state()
            assert ros.metrics()["path_recovery"]["failed_inferences"] == count
        if recover_at is None:
            detection.mask = good.copy()
            assert feed()["path_tracked"]
        detection.mask[:] = 0
        assert feed()["drive_reason"] == "tracking_path_recovery"
        assert ros.metrics()["path_recovery"]["failed_inferences"] == 1
        node.on_task_event(
            Message(json.dumps({"type": "TASK_ABORTED", "task_id": "tag-stop-1"}))
        )
        ros.now += 1.1
        node.publish_state()
        ros.start(task_id="new-window")
        node.publish_state()
        assert ros.metrics()["path_recovery"]["failed_inferences"] == 0
        assert node.last_valid_forward_mps is None
        assert feed()["drive_reason"] == "waiting_for_path"

    ros.run(scenario, "--branch-preference", "center")
