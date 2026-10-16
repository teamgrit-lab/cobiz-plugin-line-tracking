"""Sidewalk tracking stays centered even with legacy vegetation settings."""

import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_apriltag_task_stop_ros import RosHarness, SPORT, debug
from test_lidar_obstacles_ros import complete_inference


@pytest.mark.parametrize("mask_class", [0, 1, 2])
@pytest.mark.parametrize("sign", [1, -1])
def test_task_path_is_unbiased_with_or_without_vegetation(monkeypatch, mask_class, sign):
    ros = RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV, "SWIN_L_SIDEWALK_VEGETATION_BIAS_M", "0.20")
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_" + check.upper(), "false")
    selected = np.zeros((360, 640), np.uint8)
    # Keep the opposite half as context; visual left has positive robot y.
    region = np.s_[:, :320] if sign == -1 else np.s_[:, 320:]
    selected[region] = 1 if mask_class == 1 else 2
    scene = NS(vegetation=(selected == 0))
    requests = []

    def segment(_frame, **kwargs):
        requests.append(kwargs)
        return NS(selected_mask=selected.copy(), vegetation_mask=scene.vegetation.copy(),
                  inference_seconds=0.01)

    monkeypatch.setattr(debug, "BestSoFarSegmenter", lambda _: NS(
        device=NS(type="cuda"), reset=lambda: None, segment=segment))

    def scenario(node):
        ros.start()
        ros.now = 2.1
        assert complete_inference(node, ros)["drive_reason"] in ("tracking", "tracking_slow_turn")
        first_path = ros.published["/line_tracking/swin_l/local_path"][-1]
        first = np.array([pose.pose.position.y for pose in first_path.poses])
        first_yaw = json.loads(ros.published[SPORT][-1].parameter)["z"]
        first_move = json.loads(ros.published[SPORT][-1].parameter)
        assert first_move["x"] > 0
        assert not requests[-1].get("detect_vegetation", False)
        scene.vegetation[:] = False
        ros.now += 0.3
        assert complete_inference(node, ros)["drive_reason"] in ("tracking", "tracking_slow_turn")
        centered_path = ros.published["/line_tracking/swin_l/local_path"][-1]
        centered = np.array([pose.pose.position.y for pose in centered_path.poses])
        delta = first - centered
        np.testing.assert_allclose(delta, 0., atol=1e-6)
        assert ros.metrics()["vegetation_bias_control"]["active"] is False
        if mask_class != 1:
            assert first_yaw != 0  # Existing heading tracking, no forced yaw=0.
            assert first_move["y"] == 0

    ros.run(scenario, "--path-mask-class", str(mask_class), "--branch-preference", "center",
            "--unrestricted-path-mode", "--ground-half-width-m", "1.6",
            "--search-half-width-m", "1.4")


@pytest.mark.parametrize("sign", [1, -1])
def test_centered_sidewalk_does_not_move_toward_vegetation(monkeypatch, sign):
    import local_path
    import sidewalk_vegetation
    from test_sidewalk_vegetation import X, Y, corridor

    ros = RosHarness(monkeypatch)
    monkeypatch.setitem(debug.ENV, "SWIN_L_SIDEWALK_VEGETATION_BIAS_M", "0.20")
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV, "LINE_TRACKING_STOP_ON_" + check.upper(), "false")
    project = lambda mask, _config: (mask.copy(), X, Y)
    monkeypatch.setattr(local_path, "_birdseye_sidewalk", project)
    monkeypatch.setattr(sidewalk_vegetation, "_birdseye_sidewalk", project)
    scene = NS(center=0.0)

    def segment(_frame, **_kwargs):
        return NS(selected_mask=2 * corridor(scene.center),
                  vegetation_mask=corridor(scene.center + sign * 1.4, 0.6),
                  inference_seconds=0.01)

    monkeypatch.setattr(debug, "BestSoFarSegmenter", lambda _: NS(
        device=NS(type="cuda"), reset=lambda: None, segment=segment))

    def scenario(node):
        ros.start()
        ros.now = 2.1
        complete_inference(node, ros)
        move = json.loads(ros.published[SPORT][-1].parameter)
        assert move["x"] > 0 and move["z"] == 0
        assert move["y"] == 0
        assert ros.metrics()["vegetation_bias_control"]["active"] is False
        # A real center offset still uses the original heading controller.
        scene.center = -sign * 0.14
        ros.now += 0.3
        complete_inference(node, ros)
        near = json.loads(ros.published[SPORT][-1].parameter)
        assert near["x"] > 0 and near["z"] * -sign > 0
        assert near["y"] == 0
        scene.center = 0.
        ros.now += 0.3
        complete_inference(node, ros)
        reached = json.loads(ros.published[SPORT][-1].parameter)
        assert reached["x"] > 0 and reached["y"] == reached["z"] == 0
        assert ros.metrics()["vegetation_bias_control"]["active"] is False

    ros.run(scenario, "--path-mask-class", "2", "--branch-preference", "center",
            "--unrestricted-path-mode")
