from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_entrypoint_validates_and_rebuilds_invalid_tensorrt_artifacts():
    entrypoint = (ROOT / "docker" / "swin_l_debug_entrypoint.sh").read_text()

    assert "validate_swin_l_tensorrt.py" in entrypoint
    assert "existing TensorRT artifact is invalid; rebuilding" in entrypoint
    assert '--manifest-output "${manifest_path}"' in entrypoint
