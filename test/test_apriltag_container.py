"""Camera pairing and child lifecycle checks without starting ROS or motion."""
from pathlib import Path
import os
import signal
import subprocess
import sys
import time
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from apriltag_camera_relay import CameraRelayState, camera_info_for_image
from line_tracking_container import child_commands, supervise


def test_camera_info_is_paired_to_image_without_changing_source():
    image = NS(width=1280, height=720, header=NS(stamp=123, frame_id="camera_optical"))
    info = NS(width=1280, height=720, header=NS(stamp=100, frame_id="original"),
              k=[1.0] * 9, d=[0.2] * 8)
    paired = camera_info_for_image(image, info)
    assert paired.header.stamp == 123 and paired.header.frame_id == "camera_optical"
    assert info.header.stamp == 100 and info.header.frame_id == "original"
    assert paired.k == info.k and paired.d == info.d
    paired.k[0] = 7
    assert info.k[0] == 1
    assert camera_info_for_image(image, None) is None
    assert camera_info_for_image(NS(width=640, height=360), info) is None


def test_detector_uses_same_camera_as_tracking_and_is_independent_of_lidar():
    env = {"SWIN_L_IMAGE_TOPIC": "/a2/front_camera/res_720p/image_raw",
           "LINE_TRACKING_AVOIDANCE_ENABLED": "false"}
    relay, detector, tracking = child_commands("task-drive", env)
    assert "input_image_topic:=/a2/front_camera/res_720p/image_raw" in relay[1]
    assert "input_camera_info_topic:=/a2/front_camera/res_720p/camera_info" in relay[1]
    assert "detections:=/detections" in detector[1]
    assert detector[1][:4] == ["ros2", "run", "apriltag_ros", "apriltag_node"]
    assert tracking[1][-1] == "task-drive"
    env.update(SWIN_L_APRILTAG_CAMERA_INFO_TOPIC="/calibrated/info",
               SWIN_L_APRILTAG_IMAGE_TOPIC="/other/image_raw",
               SWIN_L_APRILTAG_DETECTIONS_TOPIC="/other/detections")
    commands = child_commands("ros2", env)
    assert "input_camera_info_topic:=/calibrated/info" in commands[0][1]
    assert "input_image_topic:=/other/image_raw" in commands[0][1]
    assert "detections:=/other/detections" in commands[1][1]
    env["SWIN_L_APRILTAG_DETECTOR_ENABLED"] = "false"
    assert len(child_commands("task-drive", env)) == 1


def test_shared_input_preserves_frames_and_calibration_before_tag_throttle():
    raw, infos, tags = [], [], []
    relay = CameraRelayState(1 / 6, lambda image, info: tags.append((image, info)),
                             raw.append, infos.append)
    frames = [NS(width=640, height=360, encoding="rgb8", data=bytes([i]),
                 header=NS(stamp=i, frame_id="camera")) for i in range(20)]
    # Missing calibration must not interrupt the road inference input.
    assert relay.on_image(frames[0], 0) is False
    info = NS(width=640, height=360, header=NS(stamp=-1, frame_id="calibration"),
              k=list(range(9)), d=[.1, .2, .3, .4])
    relay.on_info(info)
    for i, frame in enumerate(frames[1:], 1):
        assert relay.on_image(frame, i / 20)
    assert all(a is b for a, b in zip(raw, frames)) and len(raw) == 20
    assert infos == [info] and infos[0] is info
    assert 4 <= len(tags) <= 6
    for image, paired in tags:
        assert image is frames[image.header.stamp]
        assert paired.header == image.header
        assert paired.k == info.k and paired.d == info.d
    assert info.header.stamp == -1


def test_shared_input_continues_when_calibration_size_does_not_match():
    raw, tags = [], []
    relay = CameraRelayState(.1, lambda *args: tags.append(args), raw.append)
    relay.on_info(NS(width=1280, height=720))
    frame = NS(width=640, height=360)
    assert relay.on_image(frame, 1) is False
    assert raw == [frame] and tags == []


def test_shared_camera_wiring_keeps_detector_on_and_remote_input_separate():
    env = {"SWIN_L_APRILTAG_DETECTOR_ENABLED": "true",
           "SWIN_L_APRILTAG_IMAGE_TOPIC": "/a2/front_camera/res_360p/image_raw",
           "SWIN_L_SHARED_CAMERA_IMAGE_TOPIC": "/line_tracking/camera/image_raw",
           "SWIN_L_SHARED_CAMERA_INFO_TOPIC": "/line_tracking/camera/camera_info",
           "SWIN_L_IMAGE_TOPIC": "/line_tracking/camera/image_raw"}
    relay, detector, tracking = child_commands("task-drive", env)
    assert "input_image_topic:=/a2/front_camera/res_360p/image_raw" in relay[1]
    assert "shared_image_topic:=/line_tracking/camera/image_raw" in relay[1]
    assert "shared_camera_info_topic:=/line_tracking/camera/camera_info" in relay[1]
    assert "detections:=/detections" in detector[1]
    assert tracking[1][-1] == "task-drive"
    with pytest.raises(ValueError, match="outputs must differ"):
        child_commands("task-drive", {**env, "SWIN_L_APRILTAG_IMAGE_TOPIC": env["SWIN_L_IMAGE_TOPIC"]})
    with pytest.raises(ValueError, match="requires the AprilTag"):
        child_commands("task-drive", {**env, "SWIN_L_APRILTAG_DETECTOR_ENABLED": "false"})


@pytest.mark.parametrize("env", [
    {"SWIN_L_APRILTAG_THREADS": "0"}, {"SWIN_L_APRILTAG_DECIMATE": "nan"},
    {"SWIN_L_APRILTAG_MAX_HZ": "0"}, {"SWIN_L_APRILTAG_MAX_HAMMING": "-1"},
    {"SWIN_L_APRILTAG_DETECTOR_ENABLED": "typo"},
])
def test_bad_detector_settings_do_not_start_children(env):
    with pytest.raises(ValueError):
        child_commands("task-drive", env)


def child_script(tmp_path):
    script = tmp_path / "child.py"
    script.write_text('''
from pathlib import Path
import signal, sys, time
root=Path(sys.argv[1]);name=sys.argv[2]
def stop(*_):
    (root/(name+'.stopped')).touch()
    sys.exit(0)
signal.signal(signal.SIGTERM,stop)
(root/(name+'.ready')).touch()
if name=='failing':
    deadline=time.monotonic()+5
    while not all((root/(n+'.ready')).exists() for n in ('relay','tracking')):
        if time.monotonic()>deadline:sys.exit(9)
        time.sleep(.01)
    sys.exit(7)
while True:time.sleep(.01)
''')
    return script


def test_detector_failure_terminates_tracking_and_relay(tmp_path):
    script = child_script(tmp_path)
    commands = [(name, [sys.executable, str(script), str(tmp_path), name])
                for name in ("relay", "failing", "tracking")]
    assert supervise(commands, shutdown_sec=2) == 7
    assert (tmp_path / "relay.stopped").exists()
    assert (tmp_path / "tracking.stopped").exists()


def test_container_sigterm_reaches_all_children(tmp_path):
    script = child_script(tmp_path)
    commands = [(name, [sys.executable, str(script), str(tmp_path), name])
                for name in ("relay", "tracking")]
    program = "from line_tracking_container import supervise; raise SystemExit(supervise(" + repr(commands) + ",shutdown_sec=2))"
    env = dict(os.environ, PYTHONPATH=str(ROOT / "tools"))
    parent = subprocess.Popen([sys.executable, "-c", program], env=env)
    try:
        deadline = time.monotonic() + 5
        while not all((tmp_path / (n + ".ready")).exists() for n in ("relay", "tracking")):
            assert parent.poll() is None
            assert time.monotonic() < deadline
            time.sleep(.01)
        parent.send_signal(signal.SIGTERM)
        assert parent.wait(timeout=5) == 0
        assert (tmp_path / "relay.stopped").exists()
        assert (tmp_path / "tracking.stopped").exists()
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait()


def test_detector_configuration_and_compose_use_one_matching_interface():
    import yaml
    dockerfile = (ROOT / "Dockerfile.swin-l-debug").read_text()
    assert "ros-humble-apriltag-ros" in dockerfile
    assert "COPY third_party/apriltag_msgs" not in dockerfile
    config = yaml.safe_load((ROOT / "docker/apriltag.yaml").read_text())["/**"]["ros__parameters"]
    assert config["qos_profile"] == "sensor_data"
    assert config["pose_estimation_method"] == ""
    assert "tag" not in config
    services = yaml.safe_load((ROOT / "docker-compose.yml").read_text())["services"]
    assert services["actual-activate"]["environment"]["SWIN_L_APRILTAG_DETECTOR_ENABLED"].endswith(":-true}")
    assert services["debugging-swin-l"]["environment"]["SWIN_L_APRILTAG_DETECTOR_ENABLED"].endswith(":-false}")
    for name in ("actual-activate", "debugging-swin-l"):
        assert services[name]["environment"]["SWIN_L_APRILTAG_DETECTIONS_TOPIC"].endswith(":-/detections}")
        assert services[name]["stop_grace_period"] == "15s"
