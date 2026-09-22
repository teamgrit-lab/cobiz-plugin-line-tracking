import json
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

from build_swin_l_tensorrt import _write_artifacts_atomically  # noqa: E402


def test_validated_engine_and_manifest_replace_existing_artifacts(tmp_path):
    engine = tmp_path / "model.plan"
    manifest = tmp_path / "model.plan.json"
    engine.write_bytes(b"old-engine")
    manifest.write_text('{"old": true}\n', encoding="utf-8")

    _write_artifacts_atomically(
        b"new-engine",
        {"engine_sha256": "digest"},
        engine,
        manifest,
    )

    assert engine.read_bytes() == b"new-engine"
    assert json.loads(manifest.read_text(encoding="utf-8")) == {
        "engine_sha256": "digest"
    }
    assert not list(tmp_path.glob("*.tmp"))
