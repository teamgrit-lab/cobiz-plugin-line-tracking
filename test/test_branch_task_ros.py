"""Run the real inference publication/drive boundary with synthetic masks."""

import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import local_path
import swin_l_local_path_debug as debug
from test_apriltag_task_stop_ros import RosHarness, Message, SPORT
from test_branch_path import X, Y, corridor, direction, fork


@pytest.mark.parametrize("preference", ["left", "right"])
@pytest.mark.parametrize("recovery_enabled", [False, True])
def test_branch_guard_and_task_restart_with_ordinary_stops_disabled(
    monkeypatch, preference, recovery_enabled
):
    ros = RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV,"LINE_TRACKING_PATH_LOSS_RECOVERY_ENABLED",str(recovery_enabled))
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(
            debug.ENV, "LINE_TRACKING_STOP_ON_" + check.upper(), "false"
        )
    monkeypatch.setattr(
        local_path, "_birdseye_sidewalk", lambda mask, _config: (mask, X, Y)
    )
    scene = {"mask": fork()}
    monkeypatch.setattr(
        debug,
        "BestSoFarSegmenter",
        lambda _config: SimpleNamespace(
            device=SimpleNamespace(type="cuda"),
            reset=lambda: None,
            segment=lambda _frame, **_kwargs: SimpleNamespace(
                selected_mask=np.where(scene["mask"], 2, 0).astype(np.uint8),
                inference_seconds=0.01,
            ),
        ),
    )

    def scenario(node):
        def feed(mask):
            scene["mask"] = mask
            node.publish_state()
            target = ros.metrics()["inference_count"] + 1
            ros.now += 1 / 3
            message = Message()
            message.header.stamp = ros.stamp()
            node.on_image(message)
            deadline = time.perf_counter() + 5
            while True:
                node.publish_state()
                if ros.metrics()["inference_count"] >= target:
                    return ros.metrics()
                assert time.perf_counter() < deadline, ros.errors
                time.sleep(0.001)

        ros.start()
        pending = feed(fork())
        assert pending["drive_reason"] == "branch_confirming"
        assert pending["path_tracked"] is False
        for _ in range(5):
            node.publish_state()
            assert ros.metrics()["branch_selection"]["confirmation_hits"] == 1
        selected = feed(fork())
        assert selected["branch_selection"]["reason"] == f"{preference}_branch_selected"
        assert selected["branch_selection"]["preference"] == preference
        path = ros.published["/line_tracking/swin_l/local_path"][-1]
        assert direction(preference) * path.poses[-1].pose.position.y > 1.5
        assert json.loads(ros.published[SPORT][-1].parameter)["x"] > 0
        lost = feed(corridor(-direction(preference) * np.maximum(X - 4, 0) * 0.55))
        assert lost["drive_reason"] == "branch_path_lost"
        assert json.loads(ros.published[SPORT][-1].parameter) == {
            "x": 0,
            "y": 0,
            "z": 0,
        }
        assert node.last_valid_yaw_rate is None
        assert node.last_valid_forward_mps is None
        assert not ros.published["/line_tracking/swin_l/local_path"][-1].poses

        node.on_task_event(
            Message(json.dumps({"type": "TASK_ABORTED", "task_id": "tag-stop-1"}))
        )
        ros.now += 1.1
        node.publish_state()  # Release the previous command publisher.
        ros.start(task_id="new-fork-task")
        assert feed(fork())["drive_reason"] == "branch_confirming"
        assert (
            feed(fork())["branch_selection"]["reason"]
            == f"{preference}_branch_selected"
        )

    ros.run(scenario, "--branch-preference", preference, "--unrestricted-path-mode")
