from pathlib import Path
import os
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def test_r50_rejects_tensorrt_before_ros_setup_or_engine_build():
    environment = dict(os.environ)
    environment.pop("ROS_DISTRO", None)
    environment.update(
        SWIN_L_PROFILE="r50-fp16-640x360",
        SWIN_L_BACKEND="tensorrt",
        SWIN_L_TRT_AUTO_BUILD="true",
    )
    result = subprocess.run(
        ["bash", str(ROOT / "docker" / "swin_l_debug_entrypoint.sh")],
        env=environment, capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 1
    assert "R50 requires SWIN_L_BACKEND=pytorch" in result.stderr
    assert result.stdout == ""


def test_entrypoint_validates_and_rebuilds_invalid_tensorrt_artifacts():
    entrypoint = (ROOT / "docker" / "swin_l_debug_entrypoint.sh").read_text()

    assert "validate_swin_l_tensorrt.py" in entrypoint
    assert "existing TensorRT artifact is invalid; rebuilding" in entrypoint
    assert '--manifest-output "${manifest_path}"' in entrypoint
