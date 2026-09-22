import json
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

from tensorrt_backend import (
    ENGINE_MANIFEST_SCHEMA_VERSION,
    HYBRID_ARTIFACT_FORMAT,
    load_engine_manifest,
    normalize_cuda_device,
    sha256_file,
    validate_engine_manifest,
)


def _manifest():
    return {
        "schema_version": ENGINE_MANIFEST_SCHEMA_VERSION,
        "artifact_format": HYBRID_ARTIFACT_FORMAT,
        "profile": "swin-l-aspect-224x384-fp16",
        "model": {
            "revision": "revision",
            "id2label": {str(index): f"class-{index}" for index in range(65)},
        },
        "checkpoint": {
            "path": "/models/checkpoint",
            "manifest_file": "checkpoint-manifest.json",
            "safetensors_file": "model.safetensors",
            "safetensors_sha256": "b" * 64,
        },
        "input": {
            "name": "pixel_values",
            "shape": [1, 3, 224, 384],
            "dtype": "float16",
        },
        "output": {
            "name": "semantic_scores",
            "shape": [65, 360, 640],
            "dtype": "float16",
        },
        "engine_sha256": "a" * 64,
        "partitioning": {
            "pytorch_patch_embedding": True,
            "pytorch_mask2former_decoders": True,
            "tensorrt_partition_count": 4,
            "tensorrt_swin_stage_count": 4,
            "stages": [
                {
                    "index": index,
                    "file": f"stage_{index}.ep",
                    "sha256": f"{index + 1:x}" * 64,
                    "tensor_input_count": 1,
                    "has_downsample": index < 3,
                    "partition_count": 1,
                }
                for index in range(4)
            ],
        },
    }


def _validate(manifest):
    validate_engine_manifest(
        manifest,
        profile="swin-l-aspect-224x384-fp16",
        model_revision="revision",
        input_shape=(1, 3, 224, 384),
        output_shape=(65, 360, 640),
    )


def test_fixed_fp16_hybrid_manifest_is_accepted():
    _validate(_manifest())


@pytest.mark.parametrize(
    ("path", "value", "message"),
    [
        (("profile",), "other", "profile"),
        (("model", "revision"), "other", "revision"),
        (("input", "shape"), [1, 3, 384, 384], "input shape"),
        (("output", "dtype"), "float32", "output must use float16"),
        (("artifact_format",), "raw_plan", "hybrid program"),
        (
            ("partitioning", "pytorch_mask2former_decoders"),
            False,
            "decoders must remain in PyTorch",
        ),
        (
            ("partitioning", "tensorrt_swin_stage_count"),
            3,
            "stage metadata count",
        ),
        (
            ("partitioning", "stages", 0, "file"),
            "../stage_0.ep",
            "stage file",
        ),
    ],
)
def test_manifest_contract_mismatch_is_rejected(path, value, message):
    manifest = _manifest()
    target = manifest
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    with pytest.raises(RuntimeError, match=message):
        _validate(manifest)


def test_manifest_loader_and_engine_digest(tmp_path):
    engine = tmp_path / "model.plan"
    engine.write_bytes(b"trusted-engine")
    manifest_path = tmp_path / "model.plan.json"
    manifest_path.write_text(json.dumps(_manifest()), encoding="utf-8")

    assert load_engine_manifest(manifest_path)["profile"].startswith("swin-l")
    assert sha256_file(engine) == (
        "c702bd7d2498cd2b803d474a452bfe509194a0b5ce8a0b929b45572fb1043403"
    )


def test_indexless_cuda_device_is_normalized(monkeypatch):
    import torch

    monkeypatch.setattr(torch.cuda, "current_device", lambda: 2)

    assert normalize_cuda_device(torch.device("cuda")) == torch.device("cuda:2")
    assert normalize_cuda_device(torch.device("cuda:1")) == torch.device("cuda:1")
