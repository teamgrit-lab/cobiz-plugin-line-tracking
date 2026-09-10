import json
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import swin_l_local_path_debug as debug
import swin_l_rosbag_overlay as cli
from best_so_far_runtime import SWIN_L_PROFILE, resolve_profile


@pytest.mark.parametrize("mode,updates", [("sidewalk", 12), ("local-path", 4)])
def test_overlay_modes_keep_the_pinned_model_and_frame_policy(
    tmp_path, monkeypatch, mode, updates
):
    source = tmp_path / "camera.mcap"
    source.touch()
    output = tmp_path / "results"
    configurations = []
    calls = []

    class Segmenter:
        def __init__(self, config):
            configurations.append(config)

        def segment(self, frame):
            # The MCAP camera is RGB; the runtime must receive unmodified BGR.
            assert frame[0, 0].tolist() == [3, 2, 1]
            calls.append(frame)
            return SimpleNamespace(
                selected_mask=np.full((360, 640), 2, np.uint8), total_seconds=0.01
            )

        def render_overlay(self, frame, mask, **kwargs):
            return frame

        def metadata(self):
            return {"profile": SWIN_L_PROFILE}

    def events(path, topics, start_time_ns=0):
        assert path == source
        expected_topics = (cli.DEFAULT_IMAGE_TOPIC,)
        if mode == "local-path":
            expected_topics += (cli.DEFAULT_LIDAR_TOPIC,)
        assert topics == expected_topics
        camera = SimpleNamespace(
            encoding="rgb8",
            width=640,
            height=360,
            step=1920,
            data=bytes([1, 2, 3]) * 640 * 360,
        )
        for index in range(12):
            yield (
                SimpleNamespace(name="sensor_msgs/msg/Image"),
                SimpleNamespace(topic=cli.DEFAULT_IMAGE_TOPIC),
                SimpleNamespace(log_time=(index + 1) * 100_000_000),
                camera,
            )

    monkeypatch.setattr(debug, "BestSoFarSegmenter", Segmenter)
    monkeypatch.setattr(debug, "_iter_mcap_events", events)
    monkeypatch.setitem(debug.ENV, "SWIN_L_PROFILE", "r50-fp16-640x360")
    monkeypatch.setitem(debug.ENV, "SWIN_L_MODEL_ID", "different/checkpoint")
    monkeypatch.setitem(debug.ENV, "SWIN_L_MODEL_REVISION", "different-revision")
    if mode == "sidewalk":

        def no_path_config(_):
            pytest.fail("segmentation-only mode must not initialize path or LiDAR")

        monkeypatch.setattr(debug, "_local_path_config_from_args", no_path_config)
        monkeypatch.setattr(debug, "_lidar_config_from_args", no_path_config)

    assert (
        cli.main(
            [
                mode,
                "--input",
                str(source),
                "--output-dir",
                str(output),
                "--max-frames",
                "12",
            ]
        )
        == 0
    )
    profile = resolve_profile(SWIN_L_PROFILE)
    config = configurations[0]
    assert (config.profile, config.model_id, config.model_revision) == (
        SWIN_L_PROFILE,
        profile.model_id,
        profile.model_revision,
    )
    assert (config.evaluation_height, config.evaluation_width) == (360, 640)
    assert len(calls) == updates
    report = json.loads((output / f"{mode}-report.json").read_text())
    assert report["frames_written"] == 12
    assert report["swin_l_updates"] == updates
    assert (report["local_path"] is not None) == (mode == "local-path")
    assert report["inference_policy"] == (
        "every_frame" if mode == "sidewalk" else "bag_time_rate"
    )
    capture = cv2.VideoCapture(str(output / f"{mode}-overlay.mp4"))
    decoded = 0
    while capture.read()[0]:
        decoded += 1
    capture.release()
    assert decoded == 12


def test_existing_results_are_not_overwritten(tmp_path):
    video = tmp_path / "sidewalk-overlay.mp4"
    video.write_bytes(b"previous result")
    args = SimpleNamespace(output_dir=tmp_path, mode="sidewalk")
    with pytest.raises(FileExistsError, match="result already exists"):
        cli.prepare_outputs(args)
    assert video.read_bytes() == b"previous result"


def test_default_outputs_create_distinct_host_mounted_directories(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(cli, "__file__", str(tmp_path / "tools" / "overlay.py"))
    args = SimpleNamespace(output_dir=None, mode="sidewalk")
    first_video, first_report = cli.prepare_outputs(args)
    second_video, _ = cli.prepare_outputs(args)

    assert first_video.parent != second_video.parent
    assert first_video.parent == first_report.parent
    assert first_video.parent.parent == tmp_path / "rosbag-results" / "swin-l-tests"
    assert first_video.parent.is_dir()
    assert second_video.parent.is_dir()
