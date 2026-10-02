"""Offline deployment checks: no network downloads, ROS, or motion commands."""

import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import best_so_far_runtime as runtime
import swin_l_local_path_debug as debug

PROFILE = "r50-finetuned-fp16-640x360"


@pytest.fixture(autouse=True)
def isolated_environment(monkeypatch):
    monkeypatch.setattr(debug, "ENV", {})


@pytest.fixture
def checkpoint(tmp_path):
    from r50_checkpoint import MAPILLARY_ID2LABEL, write_checkpoint_manifest

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
    # Only HF construction is replaced; manifest validation uses real files.
    (tmp_path / "model.safetensors").write_bytes(b"local fixture weights")
    manifest = write_checkpoint_manifest(
        tmp_path,
        dataset_manifest_sha256="a" * 64,
        training={"step": 5},
        evaluation={"val_loss": 1.2},
    )
    pin = hashlib.sha256(
        (tmp_path / "checkpoint-manifest.json").read_bytes()
    ).hexdigest()
    return SimpleNamespace(path=tmp_path, pin=pin, manifest=manifest)


def _drive_args(checkpoint, *extra):
    return debug.parse_args(
        [
            "task-drive",
            "--profile",
            PROFILE,
            "--backend",
            "pytorch",
            "--model-id",
            str(checkpoint.path),
            "--checkpoint-sha256",
            checkpoint.pin,
            *extra,
        ]
    )


def _fake_hf_loaders(monkeypatch, checkpoint, loading_info=None):
    """Replace model deserialization/GPU allocation, leaving runtime logic real."""
    labels = json.loads((checkpoint.path / "config.json").read_text())["id2label"]
    model = torch.nn.Identity()
    model.config = SimpleNamespace(id2label={int(k): v for k, v in labels.items()})
    calls = []

    def load_processor(path, **kwargs):
        calls.append(("processor", path, kwargs))
        return SimpleNamespace(size={"height": 360, "width": 640})

    def load_model(path, **kwargs):
        calls.append(("model", path, kwargs))
        if kwargs.get("output_loading_info"):
            return model, loading_info or {
                "missing_keys": [],
                "unexpected_keys": [],
                "mismatched_keys": [],
                "error_msgs": [],
            }
        return model

    monkeypatch.setattr(runtime.AutoImageProcessor, "from_pretrained", load_processor)
    monkeypatch.setattr(
        runtime.MaskFormerForInstanceSegmentation, "from_pretrained", load_model
    )
    return calls


def test_local_profile_resolves_default_without_hub_revision():
    profile = runtime.resolve_profile(PROFILE)
    assert profile.model_id == "/models/r50-surface-v6/best"
    assert profile.model_revision is None
    assert profile.model_family == "maskformer"
    assert (profile.input_height, profile.input_width, profile.precision) == (
        360,
        640,
        "fp16",
    )


def test_environment_and_cli_pin_reach_runtime(monkeypatch):
    monkeypatch.setattr(
        debug,
        "ENV",
        {
            "SWIN_L_PROFILE": PROFILE,
            "SWIN_L_MODEL_ID": "/models/custom/best",
            "SWIN_L_MODEL_REVISION": "",
            "SWIN_L_CHECKPOINT_SHA256": "a" * 64,
            "SWIN_L_BACKEND": "pytorch",
        },
    )
    config = debug._runtime_config(debug.parse_args(["task-drive"]))
    assert config.profile == PROFILE
    assert config.model_id == "/models/custom/best"
    assert config.model_revision is None
    assert config.checkpoint_manifest_sha256 == "a" * 64
    override = debug._runtime_config(
        debug.parse_args(
            [
                "ros2",
                "--checkpoint-sha256",
                "b" * 64,
            ]
        )
    )
    assert override.checkpoint_manifest_sha256 == "b" * 64


def test_drive_accepts_valid_local_checkpoint_with_matching_pin(checkpoint):
    args = _drive_args(checkpoint)
    debug._validate_task_drive_preflight(args)
    debug._runtime_config(args).validate()


@pytest.mark.parametrize("pin", [None, "", "b" * 64])
def test_drive_rejects_missing_or_wrong_manifest_pin(checkpoint, pin):
    args = _drive_args(checkpoint)
    args.checkpoint_manifest_sha256 = pin
    with pytest.raises(ValueError, match="required|SHA-256"):
        debug._validate_task_drive_preflight(args)


@pytest.mark.parametrize(
    "field,value,message",
    [
        ("model_revision", "main", "revision"),
        ("backend", "tensorrt", "R50.*pytorch"),
        ("evaluation_size", [720, 1280], "360x640"),
        ("path_frame_id", "camera", "base_link"),
        ("output_hz", 5, "at least 10 Hz"),
        ("allow_backend_fallback", True, "prohibits"),
    ],
)
def test_local_drive_preserves_preflight_requirements(
    checkpoint, field, value, message
):
    args = _drive_args(checkpoint)
    setattr(args, field, value)
    with pytest.raises(ValueError, match=message):
        debug._validate_task_drive_preflight(args)


def test_drive_rejects_tampered_weights_with_matching_manifest_pin(checkpoint):
    (checkpoint.path / "model.safetensors").write_bytes(b"changed weights")
    with pytest.raises(ValueError, match="SHA-256"):
        debug._validate_task_drive_preflight(_drive_args(checkpoint))


@pytest.mark.parametrize("pin_required", [False, True])
def test_local_load_is_offline_strict_and_records_verified_identity(
    monkeypatch,
    checkpoint,
    pin_required,
):
    calls = _fake_hf_loaders(monkeypatch, checkpoint)
    config = runtime.BestSoFarConfig(
        profile=PROFILE,
        model_id=str(checkpoint.path),
        device="cpu",
        checkpoint_manifest_sha256=checkpoint.pin if pin_required else None,
    )
    segmenter = runtime.BestSoFarSegmenter(config)
    assert calls == [
        ("processor", str(checkpoint.path), {"local_files_only": True}),
        (
            "model",
            str(checkpoint.path),
            {
                "local_files_only": True,
                "use_safetensors": True,
                "output_loading_info": True,
            },
        ),
    ]
    metadata = segmenter.metadata()
    assert metadata["checkpoint_manifest_sha256"] == checkpoint.pin
    assert metadata["checkpoint"]["files"] == checkpoint.manifest["files"]
    assert (
        metadata["checkpoint"]["base_model_revision"]
        == "ae4b8c2590c0a090fc32d5c217d78738a2dd4b19"
    )
    assert metadata["checkpoint"]["label_policy"] == "operational-road-includes-manhole"
    # Live metrics must not repeat training logs or a 65x65 confusion matrix.
    assert "evaluation" not in metadata["checkpoint"]
    assert "training" not in metadata["checkpoint"]
    assert metadata["model"]["id"] == str(checkpoint.path)
    assert metadata["model"]["revision"] is None
    assert segmenter.maximum_road_island_area == 2560
    assert segmenter.minimum_sidewalk_ring_ratio == pytest.approx(0.10)
    assert segmenter.temporal_alpha == pytest.approx(0.62)
    assert segmenter.temporal_hysteresis_margin == 0


@pytest.mark.parametrize("damage", ["manifest", "weights", "pin", "remote"])
def test_bad_checkpoint_is_rejected_before_either_hf_loader(
    monkeypatch, checkpoint, damage
):
    calls = _fake_hf_loaders(monkeypatch, checkpoint)
    model_id, pin = str(checkpoint.path), checkpoint.pin
    if damage == "manifest":
        (checkpoint.path / "checkpoint-manifest.json").unlink()
    elif damage == "weights":
        (checkpoint.path / "model.safetensors").write_bytes(b"tampered")
    elif damage == "pin":
        pin = "b" * 64
    else:
        model_id = "facebook/maskformer-resnet50-vistas"
    with pytest.raises((ValueError, FileNotFoundError)):
        runtime.BestSoFarSegmenter(
            runtime.BestSoFarConfig(
                profile=PROFILE,
                model_id=model_id,
                device="cpu",
                checkpoint_manifest_sha256=pin,
            )
        )
    assert calls == []


@pytest.mark.parametrize(
    "problem",
    [
        "missing_keys",
        "unexpected_keys",
        "mismatched_keys",
        "error_msgs",
    ],
)
def test_local_load_rejects_incomplete_or_incompatible_weights(
    monkeypatch, checkpoint, problem
):
    _fake_hf_loaders(monkeypatch, checkpoint, {problem: ["bad_tensor"]})
    with pytest.raises(ValueError, match=problem):
        runtime.BestSoFarSegmenter(
            runtime.BestSoFarConfig(
                profile=PROFILE,
                model_id=str(checkpoint.path),
                device="cpu",
            )
        )


@pytest.mark.parametrize(
    "profile,manhole_surface",
    [
        (PROFILE, 1),
        (runtime.R50_PROFILE, 2),
    ],
)
def test_manhole_mapping_and_other_r50_groups(
    monkeypatch, checkpoint, profile, manhole_surface
):
    _fake_hf_loaders(monkeypatch, checkpoint)
    segmenter = runtime.BestSoFarSegmenter(
        runtime.BestSoFarConfig(
            profile=profile,
            model_id=str(checkpoint.path),
            device="cpu",
        )
    )
    labels = segmenter.model.config.id2label
    names = ["Manhole", "Road", "Sidewalk", "Bike Lane", "Parking", "Service Lane"]
    scores = torch.zeros((65, 1, len(names)))
    for column, name in enumerate(names):
        scores[
            next(key for key, value in labels.items() if value == name), 0, column
        ] = 1
    expected = np.array([[manhole_surface, 1, 2, 2, 0, 0]], dtype=np.uint8)
    np.testing.assert_array_equal(segmenter._cpu_selected_mask(scores)[0], expected)
    np.testing.assert_array_equal(
        segmenter._accelerator_selected_mask(scores)[0], expected
    )


@pytest.mark.parametrize("profile", [PROFILE, runtime.R50_PROFILE])
def test_shell_rejects_wrong_r50_backend_before_ros_setup(profile):
    result = subprocess.run(
        ["bash", str(ROOT / "docker/swin_l_debug_entrypoint.sh")],
        env={
            "PATH": os.environ["PATH"],
            "SWIN_L_PROFILE": profile,
            "SWIN_L_BACKEND": "tensorrt",
        },
        text=True,
        capture_output=True,
        timeout=5,
        check=False,
    )
    assert result.returncode != 0
    assert "R50 requires SWIN_L_BACKEND=pytorch" in result.stderr
    assert "ROS_DISTRO" not in result.stderr


@pytest.mark.parametrize("service", ["actual-activate", "debugging-swin-l"])
def test_live_services_forward_local_checkpoint_identity(service):
    environment = yaml.safe_load((ROOT / "docker-compose.yml").read_text())["services"][
        service
    ]["environment"]
    for key in ("SWIN_L_MODEL_ID", "SWIN_L_MODEL_REVISION", "SWIN_L_CHECKPOINT_SHA256"):
        assert environment.get(key) == "${" + key + ":-}"
