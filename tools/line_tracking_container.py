#!/usr/bin/env python3
"""Supervise the camera relay, apriltag_ros, and line tracking in one container."""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
IMAGE = "/line_tracking/apriltag/image_rect"
INFO = "/line_tracking/apriltag/camera_info"


def detector_enabled(env):
    value = env.get("SWIN_L_APRILTAG_DETECTOR_ENABLED", "true").strip().lower()
    if value not in ("true", "false", "1", "0", "yes", "no", "on", "off"):
        raise ValueError("SWIN_L_APRILTAG_DETECTOR_ENABLED must be true or false")
    return value in ("true", "1", "yes", "on")


def child_commands(mode, env):
    tracking = ("line tracking", [sys.executable, str(ROOT / "tools/swin_l_local_path_debug.py"), mode])
    if not detector_enabled(env):
        return [tracking]
    image = (env.get("SWIN_L_APRILTAG_IMAGE_TOPIC", "").strip()
             or env.get("SWIN_L_IMAGE_TOPIC", "").strip()
             or "/a2/front_camera/res_720p/image_raw")
    info = (env.get("SWIN_L_APRILTAG_CAMERA_INFO_TOPIC", "").strip()
            or image.rsplit("/", 1)[0] + "/camera_info")
    topic = env.get("SWIN_L_APRILTAG_DETECTIONS_TOPIC", "/detections").strip()
    family = env.get("SWIN_L_APRILTAG_FAMILY", "36h11").strip()
    threads = int(env.get("SWIN_L_APRILTAG_THREADS", "2"))
    decimate = float(env.get("SWIN_L_APRILTAG_DECIMATE", "1.0"))
    max_hz = float(env.get("SWIN_L_APRILTAG_MAX_HZ", "10.0"))
    hamming = int(env.get("SWIN_L_APRILTAG_MAX_HAMMING", "0"))
    if (not topic or not family or threads < 1 or hamming < 0
            or not math.isfinite(decimate) or decimate < 1
            or not math.isfinite(max_hz) or max_hz <= 0):
        raise ValueError("invalid AprilTag detector settings")
    relay = ("AprilTag camera relay", [sys.executable, str(ROOT / "tools/apriltag_camera_relay.py"),
        "--ros-args", "-p", "input_image_topic:=" + image,
        "-p", "input_camera_info_topic:=" + info,
        "-p", "max_hz:=" + str(max_hz),
        "-r", "image_rect:=" + IMAGE, "-r", "camera_info:=" + INFO])
    detector = ("AprilTag detector", ["ros2", "run", "apriltag_ros", "apriltag_node",
        "--ros-args", "--params-file", str(ROOT / "docker/apriltag.yaml"),
        "-r", "__node:=line_tracking_apriltag",
        "-r", "image_rect:=" + IMAGE, "-r", "camera_info:=" + INFO,
        "-r", "detections:=" + topic,
        "-p", "family:=" + family, "-p", "detector.threads:=" + str(threads),
        "-p", "detector.decimate:=" + str(decimate), "-p", "max_hamming:=" + str(hamming)])
    return [relay, detector, tracking]


def supervise(commands, shutdown_sec=10.0):
    children = []
    stopping = False
    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True
    previous = {s: signal.signal(s, stop) for s in (signal.SIGTERM, signal.SIGINT)}
    result = 0
    try:
        for name, command in commands:
            if stopping:
                break
            print(f"[line-tracking] starting {name}", flush=True)
            children.append((name, subprocess.Popen(command, start_new_session=True)))
        while not stopping:
            for name, child in children:
                status = child.poll()
                if status is not None:
                    print(f"[line-tracking] {name} exited ({status}); shutting down container", file=sys.stderr)
                    result = status if status > 0 else 1
                    stopping = True
                    break
            if not stopping:
                time.sleep(0.1)
    except OSError as error:
        print(f"[line-tracking] child startup failed: {error}", file=sys.stderr)
        result = 1
    finally:
        # The tracking node handles SIGTERM by sending its existing shutdown stop.
        # Signal entire groups so ros2 run cannot leave a detector orphan behind.
        for _, child in reversed(children):
            try:
                os.killpg(child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + shutdown_sec
        for _, child in reversed(children):
            try:
                child.wait(timeout=max(0.0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                child.wait()
        for signum, handler in previous.items():
            signal.signal(signum, handler)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("ros2", "task-drive"))
    args = parser.parse_args()
    try:
        commands = child_commands(args.mode, os.environ)
    except (ValueError, TypeError) as error:
        parser.error(str(error))
    if len(commands) == 1:
        os.execv(commands[0][1][0], commands[0][1])
    return supervise(commands)


if __name__ == "__main__":
    raise SystemExit(main())
