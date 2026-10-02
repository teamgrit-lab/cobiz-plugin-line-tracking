"""Portable, hash-pinned deployment contract for locally fine-tuned R50 models.

This verifies artifact integrity and the inference interface, not driving
quality. It deliberately has no torch/ROS dependencies so exports can be
checked before loading the GPU runtime.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any

BASE_MODEL_ID = "facebook/maskformer-resnet50-vistas"
BASE_MODEL_REVISION = "ae4b8c2590c0a090fc32d5c217d78738a2dd4b19"
FINETUNED_PROFILE = "r50-finetuned-fp16-640x360"
MANIFEST_NAME = "checkpoint-manifest.json"
REQUIRED_FILES = ("config.json", "preprocessor_config.json", "model.safetensors")
LABEL_POLICY = "operational-road-includes-manhole"
MAPILLARY_LABELS = (
    "Bird",
    "Ground Animal",
    "Curb",
    "Fence",
    "Guard Rail",
    "Barrier",
    "Wall",
    "Bike Lane",
    "Crosswalk - Plain",
    "Curb Cut",
    "Parking",
    "Pedestrian Area",
    "Rail Track",
    "Road",
    "Service Lane",
    "Sidewalk",
    "Bridge",
    "Building",
    "Tunnel",
    "Person",
    "Bicyclist",
    "Motorcyclist",
    "Other Rider",
    "Lane Marking - Crosswalk",
    "Lane Marking - General",
    "Mountain",
    "Sand",
    "Sky",
    "Snow",
    "Terrain",
    "Vegetation",
    "Water",
    "Banner",
    "Bench",
    "Bike Rack",
    "Billboard",
    "Catch Basin",
    "CCTV Camera",
    "Fire Hydrant",
    "Junction Box",
    "Mailbox",
    "Manhole",
    "Phone Booth",
    "Pothole",
    "Street Light",
    "Pole",
    "Traffic Sign Frame",
    "Utility Pole",
    "Traffic Light",
    "Traffic Sign (Back)",
    "Traffic Sign (Front)",
    "Trash Can",
    "Bicycle",
    "Boat",
    "Bus",
    "Car",
    "Caravan",
    "Motorcycle",
    "On Rails",
    "Other Vehicle",
    "Trailer",
    "Truck",
    "Wheeled Slow",
    "Car Mount",
    "Ego Vehicle",
)
MAPILLARY_ID2LABEL = {str(i): name for i, name in enumerate(MAPILLARY_LABELS)}


def file_sha256(path: Path | str) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise ValueError(f"Cannot read checkpoint JSON: {path.name}") from error
    if not isinstance(value, dict):
        raise ValueError(f"Checkpoint JSON must be an object: {path.name}")  # noqa: TRY004 - invalid file contents
    return value


def _digest(value: Any, description: str) -> str:
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-fA-F]{64}", value) is None:
        raise ValueError(f"{description} must be a 64-character SHA-256")
    return value.lower()


def _local_file(root: Path, name: str) -> Path:
    path = root / name
    if not path.is_file() or not path.resolve().is_relative_to(root):
        raise ValueError(
            f"Checkpoint files must exist inside the checkpoint directory: {name}"
        )
    return path


def _validate_model_contract(root: Path) -> None:
    config = _json(_local_file(root, "config.json"))
    if config.get("model_type") != "maskformer":
        raise ValueError("Expected a MaskFormer checkpoint")
    labels = config.get("id2label")
    if labels != MAPILLARY_ID2LABEL or config.get("num_labels", 65) != 65:
        raise ValueError("Checkpoint must preserve the exact 65 Mapillary class labels")
    if "label2id" in config:
        expected = {name: int(key) for key, name in MAPILLARY_ID2LABEL.items()}
        if config["label2id"] != expected:
            raise ValueError("Checkpoint label2id does not match id2label")
    backbone = config.get("backbone_config", {})
    required = {
        "model_type": "resnet",
        "depths": [3, 4, 6, 3],
        "layer_type": "bottleneck",
        "num_channels": 3,
        "hidden_sizes": [256, 512, 1024, 2048],
    }
    if not isinstance(backbone, dict) or any(
        backbone.get(k) != v for k, v in required.items()
    ):
        raise ValueError("Expected the original ResNet-50 backbone configuration")
    processor = _json(_local_file(root, "preprocessor_config.json"))
    if processor.get("image_processor_type") not in (
        "MaskFormerImageProcessor",
        "MaskFormerImageProcessorFast",
    ):
        raise ValueError("Expected a MaskFormer image processor")
    if (
        processor.get("size") != {"height": 360, "width": 640}
        or processor.get("size_divisor") != 32
    ):
        raise ValueError(
            "Processor must use the deployed 360x640 resize target and divisor 32"
        )
    if processor.get("pad_size") is not None:
        raise ValueError("Fixed custom processor padding is not supported")
    if (
        processor.get("ignore_index") != 255
        or processor.get("do_reduce_labels", False)
        or processor.get("reduce_labels", False)
    ):
        raise ValueError(
            "Processor must use ignore_index=255 without reducing label IDs"
        )
    for key in ("do_resize", "do_rescale", "do_normalize"):
        if processor.get(key) is not True:
            raise ValueError(f"Processor {key} must be enabled")
    expected_vectors = {
        "image_mean": [0.485, 0.456, 0.406],
        "image_std": [0.229, 0.224, 0.225],
    }
    for key, expected in expected_vectors.items():
        value = processor.get(key)
        if (
            not isinstance(value, list)
            or len(value) != 3
            or any(
                not isinstance(a, (int, float)) or not math.isclose(a, b, rel_tol=1e-6)
                for a, b in zip(value, expected)
            )
        ):
            raise ValueError(f"Processor {key} differs from deployment normalization")
    scale = processor.get("rescale_factor")
    if not isinstance(scale, (int, float)) or not math.isclose(
        scale, 1 / 255, rel_tol=1e-6
    ):
        raise ValueError("Processor must rescale uint8 RGB by 1/255")
    if processor.get("resample", 2) != 2:
        raise ValueError("Processor must use bilinear image resizing")


def checkpoint_manifest_sha256(checkpoint_dir: Path | str) -> str:
    root = Path(checkpoint_dir).expanduser().resolve()
    return file_sha256(_local_file(root, MANIFEST_NAME))


def write_checkpoint_manifest(
    checkpoint_dir: Path | str,
    *,
    dataset_manifest_sha256: str,
    training: dict,
    evaluation: dict,
) -> dict:
    root = Path(checkpoint_dir).expanduser().resolve()
    _validate_model_contract(root)
    manifest = {
        "schema_version": 1,
        "profile": FINETUNED_PROFILE,
        "model_family": "maskformer",
        "base_model_id": BASE_MODEL_ID,
        "base_model_revision": BASE_MODEL_REVISION,
        "num_labels": 65,
        "preprocessing": {
            "resize_height": 360,
            "resize_width": 640,
            "size_divisor": 32,
            "color_order": "rgb",
        },
        "label_policy": LABEL_POLICY,
        "dataset_manifest_sha256": _digest(
            dataset_manifest_sha256, "Dataset manifest hash"
        ),
        "files": {
            name: file_sha256(_local_file(root, name)) for name in REQUIRED_FILES
        },
        "training": training,
        "evaluation": evaluation,
        "note": "Integrity and interface validation do not certify driving quality.",
    }
    temporary = root / (MANIFEST_NAME + ".tmp")
    temporary.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    )
    temporary.replace(root / MANIFEST_NAME)
    return manifest


def validate_checkpoint(
    checkpoint_dir: Path | str,
    expected_manifest_sha256: str | None = None,
    *,
    require_pin: bool = False,
) -> dict:
    root = Path(checkpoint_dir).expanduser().resolve()
    if not root.is_dir():
        raise ValueError(f"Local checkpoint directory does not exist: {root}")
    if require_pin and not expected_manifest_sha256:
        raise ValueError(
            "An explicit checkpoint manifest SHA-256 is required for task-drive"
        )
    actual = checkpoint_manifest_sha256(root)
    if expected_manifest_sha256:
        expected = _digest(expected_manifest_sha256, "Expected manifest SHA-256")
        if actual != expected:
            raise ValueError(
                "Checkpoint manifest SHA-256 does not match the selected deployment"
            )
    manifest = _json(root / MANIFEST_NAME)
    expected_fields = {
        "schema_version": 1,
        "profile": FINETUNED_PROFILE,
        "model_family": "maskformer",
        "base_model_id": BASE_MODEL_ID,
        "base_model_revision": BASE_MODEL_REVISION,
        "num_labels": 65,
        "label_policy": LABEL_POLICY,
    }
    for key, value in expected_fields.items():
        if manifest.get(key) != value:
            raise ValueError(f"Checkpoint manifest has incompatible {key}")
    if manifest.get("preprocessing") != {
        "resize_height": 360,
        "resize_width": 640,
        "size_divisor": 32,
        "color_order": "rgb",
    }:
        raise ValueError(
            "Checkpoint manifest preprocessing contract does not match deployment"
        )
    files = manifest.get("files", {})
    if not isinstance(files, dict) or set(files) != set(REQUIRED_FILES):
        raise ValueError(
            "Checkpoint manifest files must list exactly the three deployment artifacts"
        )
    for name in REQUIRED_FILES:
        expected = _digest(files[name], f"{name} hash")
        if file_sha256(_local_file(root, name)) != expected:
            raise ValueError(f"Checkpoint file SHA-256 mismatch: {name}")
    _digest(manifest.get("dataset_manifest_sha256"), "Dataset manifest hash")
    _validate_model_contract(root)
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--expected-sha256")
    args = parser.parse_args()
    report = validate_checkpoint(args.checkpoint, args.expected_sha256)
    print(
        json.dumps(
            {
                "status": "passed",
                "profile": report["profile"],
                "manifest_sha256": checkpoint_manifest_sha256(args.checkpoint),
                "training": report["training"],
                "evaluation": report["evaluation"],
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
