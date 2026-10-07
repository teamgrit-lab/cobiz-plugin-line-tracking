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
from best_so_far_runtime import SWIN_L_ASPECT_FP16_PROFILE, resolve_profile


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
            return {"profile": SWIN_L_ASPECT_FP16_PROFILE}

    def events(path, topics, start_time_ns=0):
        assert path == source
        expected_topics = (cli.DEFAULT_IMAGE_TOPIC,)
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
    # This case verifies rate-limited replay, independently of the deployment
    # default that intentionally processes every frame in unrestricted mode.
    monkeypatch.setitem(debug.ENV, "SWIN_L_UNRESTRICTED_PATH_MODE", "false")
    if mode == "sidewalk":

        def no_path_config(_):
            pytest.fail("segmentation-only mode must not initialize path")

        monkeypatch.setattr(debug, "_local_path_config_from_args", no_path_config)

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
                "--no-avoidance-enabled",
            ]
        )
        == 0
    )
    profile = resolve_profile(SWIN_L_ASPECT_FP16_PROFILE)
    config = configurations[0]
    assert (config.profile, config.model_id, config.model_revision) == (
        SWIN_L_ASPECT_FP16_PROFILE,
        profile.model_id,
        profile.model_revision,
    )
    assert (config.evaluation_height, config.evaluation_width) == (360, 640)
    assert len(calls) == updates
    report = json.loads((output / f"{mode}-report.json").read_text())
    assert "lidar_topic" not in report
    assert "lidar_safety" not in report
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


def test_replay_arguments_forward_obstacle_calibration_and_explicit_camera_only_mode(tmp_path):
    source = tmp_path / "camera.mcap"
    source.touch()
    args = cli.parse_args(["local-path", "--input", str(source),
                           "--no-avoidance-enabled", "--lidar-topic", "/custom/points",
                           "--base-to-camera-transform", "measured-matrix",
                           "--obstacle-imu-accel-unit", "g",
                           "--obstacle-height-reference", "base_link",
                           "--obstacle-min-height-m", "-0.2", "--obstacle-max-height-m", "0.2",
                           "--obstacle-horizontal-range-m", "1.5", "--obstacle-fov-deg", "190"])
    forwarded = cli.build_debug_arguments(
        args, tmp_path / "overlay.mp4", tmp_path / "report.json"
    )
    assert "--no-avoidance-enabled" in forwarded
    assert forwarded[forwarded.index("--lidar-topic")+1] == "/custom/points"
    assert forwarded[forwarded.index("--base-to-camera-transform")+1] == "measured-matrix"
    assert forwarded[forwarded.index("--obstacle-imu-accel-unit")+1] == "g"
    assert forwarded[forwarded.index("--obstacle-height-reference")+1] == "base_link"
    assert forwarded[forwarded.index("--obstacle-min-height-m")+1] == "-0.2"
    assert forwarded[forwarded.index("--obstacle-max-height-m")+1] == "0.2"
    assert forwarded[forwarded.index("--obstacle-horizontal-range-m")+1] == "1.5"
    assert forwarded[forwarded.index("--obstacle-fov-deg")+1] == "190"


def test_active_parsers_default_to_livox_obstacle_avoidance():
    for argv in (["ros2"], ["task-drive"]):
        args = debug.parse_args(argv)
        assert args.avoidance_enabled
        assert args.lidar_topic == "/livox/lidar"
        assert args.lidar_imu_topic == "/livox/imu"
        assert args.lidar_frame_id == "livox_frame"
        assert args.obstacle_imu_accel_unit == "auto"
        assert args.obstacle_height_reference == "base_link"
        assert args.obstacle_min_height_m == -.2
        assert args.obstacle_max_height_m == .2
        assert args.obstacle_horizontal_range_m == 1.5
        assert args.obstacle_fov_deg == 190
        assert not hasattr(args, "clearance_topic")


def test_overlay_renders_optional_status_without_safety_object():
    frame = np.zeros((360, 640, 3), np.uint8)
    mask = np.zeros((360, 640), np.uint8)
    kwargs = {"frame_index": 1, "inference_count": 0, "inference_hz": 0.0}
    plain = debug.render_local_path_overlay(
        frame, mask, None, None, debug.LocalPathConfig(), **kwargs
    )
    status = debug.render_local_path_overlay(
        frame,
        mask,
        None,
        None,
        debug.LocalPathConfig(),
        status_text="AprilTag detections stale",
        **kwargs,
    )
    assert plain.shape == status.shape == frame.shape
    assert np.any(plain != status)

def test_mcap_updates_obstacles_between_rate_limited_camera_inferences(tmp_path, monkeypatch):
    from lidar_common import a2_livox_front_transforms
    from sensor_fixtures import camera_info, cloud, header, imu

    source = tmp_path / "sensors.mcap"
    source.touch()
    report_path = tmp_path / "report.json"
    base, _ = a2_livox_front_transforms("livox_frame", "base_link", "camera_optical_frame")
    calls = []
    monkeypatch.setattr(debug, "BestSoFarSegmenter", lambda _: SimpleNamespace(
        segment=lambda frame: (calls.append(frame) or SimpleNamespace(
            selected_mask=np.full((360, 640), 2, np.uint8), total_seconds=.01)),
        metadata=lambda: {}))
    frames = []
    monkeypatch.setattr(debug, "_build_writer", lambda *_: SimpleNamespace(write=frames.append, release=lambda: None))

    def events(path, topics, start_time_ns=0):
        assert not any("odom" in topic for topic in topics)
        for tick, distance in enumerate([2., 1.5, .7]):
            stamp = 100. + tick * .1
            camera = SimpleNamespace(header=header(stamp, "camera_optical_frame"), encoding="rgb8",
                width=640, height=360, step=1920, data=bytes(640 * 360 * 3))
            raw = (np.array([[distance, 0., base[2, 3]]]) - base[:3, 3]) @ base[:3, :3]
            packets = [("/livox/imu", imu(stamp, up=base[:3, :3].T @ [0, 0, 1])),
                ("/a2/front_camera/res_360p/camera_info", camera_info(stamp)),
                ("/livox/lidar", cloud(raw, stamp)), (debug.DEFAULT_IMAGE_TOPIC, camera)]
            for topic, packet in packets:
                yield (SimpleNamespace(name="sensor_msgs/msg/Image"), SimpleNamespace(topic=topic),
                       SimpleNamespace(log_time=round(stamp * 1e9)), packet)

    monkeypatch.setattr(debug, "_iter_mcap_events", events)
    args = debug.parse_args(["mcap", "--input", str(source), "--output", str(tmp_path / "overlay.mp4"),
        "--report", str(report_path), "--inference-hz", "1", "--no-unrestricted-path-mode", "--branch-preference", "center"])
    assert debug.run_mcap(args) == 0
    report = json.loads(report_path.read_text())
    assert len(calls) == 1 and len(frames) == 3
    assert report["lidar_obstacles"]["scan_stamp_ns"] == 100_200_000_000
    assert report["lidar_obstacles"]["nearest_obstacle_m"] == pytest.approx(.7)
    assert report["lidar_obstacles"]["reason"].startswith("obstacle_")
