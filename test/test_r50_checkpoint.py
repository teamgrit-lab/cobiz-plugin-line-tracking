import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))


@pytest.fixture
def checkpoint(tmp_path):
    from r50_checkpoint import MAPILLARY_ID2LABEL

    config = {
        "model_type": "maskformer",
        "id2label": MAPILLARY_ID2LABEL,
        "backbone_config": {
            "model_type": "resnet",
            "depths": [3, 4, 6, 3],
            "layer_type": "bottleneck",
            "num_channels": 3,
            "hidden_sizes": [256, 512, 1024, 2048],
        },
    }
    processor = {
        "image_processor_type": "MaskFormerImageProcessor",
        "size": {"height": 360, "width": 640},
        "size_divisor": 32,
        "do_resize": True,
        "do_normalize": True,
        "do_rescale": True,
        "rescale_factor": 1 / 255,
        "image_mean": [0.485, 0.456, 0.406],
        "image_std": [0.229, 0.224, 0.225],
        "ignore_index": 255,
        "do_reduce_labels": False,
    }
    (tmp_path / "config.json").write_text(json.dumps(config))
    (tmp_path / "preprocessor_config.json").write_text(json.dumps(processor))
    # Manifest integrity tests do not construct or load model tensors.
    (tmp_path / "model.safetensors").write_bytes(b"fixture weights")
    return tmp_path


def write_manifest(path):
    from r50_checkpoint import write_checkpoint_manifest

    return write_checkpoint_manifest(
        path,
        dataset_manifest_sha256="a" * 64,
        training={"step": 5},
        evaluation={"val_loss": 1.2},
    )


def test_portable_checkpoint_pin_survives_directory_copy(checkpoint, tmp_path_factory):
    import shutil

    from r50_checkpoint import checkpoint_manifest_sha256, validate_checkpoint

    write_manifest(checkpoint)
    pin = checkpoint_manifest_sha256(checkpoint)
    copied = tmp_path_factory.mktemp("jetson") / "best"
    shutil.copytree(checkpoint, copied)
    report = validate_checkpoint(copied, pin, require_pin=True)
    assert report["training"]["step"] == 5
    assert report["label_policy"] == "operational-road-includes-manhole"


def test_changed_weights_are_rejected_even_with_matching_manifest_pin(checkpoint):
    from r50_checkpoint import checkpoint_manifest_sha256, validate_checkpoint

    write_manifest(checkpoint)
    pin = checkpoint_manifest_sha256(checkpoint)
    (checkpoint / "model.safetensors").write_bytes(b"changed weights")
    with pytest.raises(ValueError, match="SHA-256"):
        validate_checkpoint(checkpoint, pin, require_pin=True)


def test_drive_validation_requires_explicit_matching_pin(checkpoint):
    from r50_checkpoint import validate_checkpoint

    write_manifest(checkpoint)
    with pytest.raises(ValueError, match="required"):
        validate_checkpoint(checkpoint, require_pin=True)
    with pytest.raises(ValueError, match="manifest SHA-256"):
        validate_checkpoint(checkpoint, "b" * 64, require_pin=True)


@pytest.mark.parametrize(
    "damage", ["labels", "backbone", "resize", "ignore", "normalization"]
)
def test_wrong_model_or_preprocessing_cannot_be_exported(checkpoint, damage):
    name = (
        "config.json"
        if damage in ("labels", "backbone")
        else "preprocessor_config.json"
    )
    path = checkpoint / name
    doc = json.loads(path.read_text())
    if damage == "labels":
        doc["id2label"]["15"] = "Road"
    if damage == "backbone":
        doc["backbone_config"]["depths"] = [2, 2, 2, 2]
    if damage == "resize":
        doc["size"]["height"] = 720
    if damage == "ignore":
        doc["ignore_index"] = 65
    if damage == "normalization":
        doc["image_mean"] = [0, 0, 0]
    path.write_text(json.dumps(doc))
    with pytest.raises(ValueError):
        write_manifest(checkpoint)


def test_validation_rechecks_contract_after_manifest_was_rehashed(checkpoint):
    import hashlib

    from r50_checkpoint import validate_checkpoint

    doc = write_manifest(checkpoint)
    path = checkpoint / "config.json"
    config = json.loads(path.read_text())
    config["id2label"]["15"] = "Road"
    path.write_text(json.dumps(config))
    doc["files"]["config.json"] = hashlib.sha256(path.read_bytes()).hexdigest()
    (checkpoint / "checkpoint-manifest.json").write_text(json.dumps(doc))
    with pytest.raises(ValueError, match="label"):
        validate_checkpoint(checkpoint)


def test_manifest_cannot_reference_files_outside_checkpoint(checkpoint):
    from r50_checkpoint import validate_checkpoint

    doc = write_manifest(checkpoint)
    doc["files"]["../outside"] = "a" * 64
    (checkpoint / "checkpoint-manifest.json").write_text(json.dumps(doc))
    with pytest.raises(ValueError, match="files"):
        validate_checkpoint(checkpoint)
