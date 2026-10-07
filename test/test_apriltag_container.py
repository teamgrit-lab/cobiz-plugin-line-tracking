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
from apriltag_camera_relay import camera_info_for_image
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
