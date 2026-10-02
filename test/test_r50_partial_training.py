"""Behavioral tests for partial-label training; no pretrained weights/downloads."""

import importlib
import json
import math
import random
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import torch
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))


@pytest.fixture
def training():
    assert (ROOT / "tools/r50_partial_training.py").exists(), (
        "partial trainer is not implemented"
    )
    return importlib.import_module("r50_partial_training")


@pytest.fixture
def dataset_root(tmp_path):
    root = tmp_path / "dataset"
    (root / "splits").mkdir(parents=True)
    rows = []
    for index, status in enumerate(
        ["assistant_visual_partial", "legacy_propagated_road"]
    ):
        row = {
            "sample_id": str(index),
            "split": "train",
            "training_eligible": True,
            "supervised_pixels": 4,
            "annotation_review_status": status,
            "width": 4,
            "height": 2,
        }
        arrays = {
            "image": np.arange(24, dtype=np.uint8).reshape(2, 4, 3),
            "partial_label": np.array([[13, 13, 2, 2], [255, 255, 15, 15]], np.uint8),
            "valid_mask": np.array([[255, 255, 255, 255], [0, 0, 255, 255]], np.uint8),
            "annotation_origin_map": np.array([[2, 2, 2, 2], [0, 0, 2, 2]], np.uint8),
        }
        for key, array in arrays.items():
            row[key] = f"{index}-{key}.png"
            Image.fromarray(array).save(root / row[key])
        rows.append(row)
    (root / "splits/train.jsonl").write_text("\n".join(map(json.dumps, rows)) + "\n")
    val = dict(
        rows[0],
        sample_id="val-0",
        split="val",
        training_eligible=False,
        image="val-image.png",
    )
    Image.fromarray(np.full((2, 4, 3), 120, np.uint8)).save(root / val["image"])
    (root / "splits/val.jsonl").write_text(json.dumps(val) + "\n")
    (root / "manifest.jsonl").write_text(
        "\n".join(map(json.dumps, rows + [val])) + "\n"
    )
    # Invalid JSON catches any attempt to use test data, even for smoke limits.
    (root / "splits/test.jsonl").write_text("TEST MUST NEVER BE READ")
    return root


def test_nll_ignores_unknown_and_invalid_pixels_in_backward(training):
    scores = torch.ones(1, 65, 1, 3, requires_grad=True)
    labels = torch.tensor([[[13, 255, 2]]])
    valid = torch.tensor([[[True, True, False]]])
    loss = training.masked_partial_nll(scores, labels, valid)
    assert loss.dtype == torch.float32
    assert loss.item() == pytest.approx(math.log(65))
    loss.backward()
    assert scores.grad[0, 13, 0, 0] < 0
    assert scores.grad[..., 1:].count_nonzero() == 0


def test_loss_balances_image_classes_and_keeps_legacy_reliability(training):
    scores = torch.ones(1, 65, 1, 101)
    scores[:, 13, :, :100] = 64  # Road probability = 1/2.
    scores[:, 2, :, 100] = 16  # Curb probability = 1/5.
    labels = torch.full((1, 1, 101), 13)
    labels[..., 100] = 2
    origin = torch.ones_like(labels)
    origin[..., 100] = 2
    loss = training.masked_partial_nll(
        scores, labels, torch.ones_like(labels, dtype=torch.bool), origin
    )
    assert loss.item() == pytest.approx((0.5 * math.log(2) + 2 * math.log(5)) / 3)


def test_all_unknown_batch_has_finite_zero_loss_and_zero_gradient(training):
    scores = torch.rand(2, 65, 2, 3, requires_grad=True)
    loss = training.masked_partial_nll(
        scores, torch.full((2, 2, 3), 255), torch.zeros(2, 2, 3, dtype=torch.bool)
    )
    assert loss.item() == 0
    loss.backward()
    assert scores.grad.count_nonzero() == 0


def test_score_projection_drops_null_class_and_stays_differentiable(training):
    class_logits = torch.zeros(1, 2, 66, requires_grad=True)
    mask_logits = torch.zeros(1, 2, 2, 3, requires_grad=True)
    scores = training.semantic_scores(class_logits, mask_logits, (4, 6))
    assert scores.shape == (1, 65, 4, 6)
    assert scores[0, 0, 0, 0].item() == pytest.approx(1 / 66)
    training.masked_partial_nll(
        scores, torch.full((1, 4, 6), 13), torch.ones(1, 4, 6, dtype=torch.bool)
    ).backward()
    assert class_logits.grad is not None and class_logits.grad.abs().sum() > 0
    assert mask_logits.grad is not None


def test_split_filters_ineligible_unsupervised_and_proposals_without_test_leakage(
    training, dataset_root
):
    path = dataset_root / "splits/train.jsonl"
    base = json.loads(path.read_text().splitlines()[0])
    rejected = [
        dict(base, sample_id="ineligible", training_eligible=False),
        dict(base, sample_id="unlabeled", supervised_pixels=0),
        dict(
            base, sample_id="proposal", annotation_review_status="optical_flow_proposal"
        ),
        dict(base, sample_id="review", split="review"),
    ]
    with path.open("a") as handle:
        handle.write("\n".join(map(json.dumps, rejected)) + "\n")
    assert [row["sample_id"] for row in training.load_split(dataset_root, "train")] == [
        "0",
        "1",
    ]
    assert len(training.load_split(dataset_root, "val", limit=1)) == 1
    with pytest.raises(ValueError, match="train.*val"):
        training.load_split(dataset_root, "test", limit=1)


def test_native_targets_resize_directly_and_origin_alignment_is_checked(
    training, dataset_root
):
    rows = training.load_split(dataset_root, "train")
    data = training.PartialDataset(
        dataset_root, rows, output_size=(2, 2), augment=False
    )
    sample = data[0]
    assert sample["image"].shape == (2, 4, 3)  # Original RGB goes to processor.
    assert sample["labels"].tolist() == [[13, 2], [255, 15]]
    assert sample["valid_mask"].tolist() == [[True, True], [False, True]]
    Image.fromarray(np.zeros((1, 4), np.uint8)).save(
        dataset_root / rows[0]["annotation_origin_map"]
    )
    with pytest.raises(ValueError, match="alignment"):
        data[0]


def test_augmentation_is_repeatable_photometric_and_does_not_change_targets(
    training, dataset_root
):
    rows = training.load_split(dataset_root, "train")
    original = training.PartialDataset(
        dataset_root, rows, output_size=(2, 4), augment=False
    )[0]
    data = training.PartialDataset(dataset_root, rows, output_size=(2, 4), augment=True)
    a, b = data[(0, 481)], data[(0, 481)]
    np.testing.assert_array_equal(a["image"], b["image"])
    assert a["image"].shape == original["image"].shape
    for key in ["labels", "valid_mask", "origins"]:
        assert torch.equal(a[key], original[key])


def test_sampler_balances_groups_and_resumes_exactly(training):
    rows = [{"annotation_review_status": "assistant_visual_partial"}] + [
        {"annotation_review_status": "legacy_propagated_road"}
    ] * 30
    sampler = training.BalancedSampler(rows, seed=29)
    draws = sampler.draw(2000)
    assert 850 < sum(index == 0 for index, seed in draws) < 1150
    state = sampler.state_dict()
    expected = sampler.draw(100)
    resumed = training.BalancedSampler(rows, seed=999)
    resumed.load_state_dict(state)
    assert resumed.draw(100) == expected
    assert all(
        index == 0
        for index, seed in training.BalancedSampler(rows[:1], seed=1).draw(10)
    )


@pytest.fixture
def tiny_model():
    from transformers import (
        DetrConfig,
        MaskFormerConfig,
        MaskFormerForInstanceSegmentation,
        ResNetConfig,
    )

    torch.set_num_threads(2)
    config = MaskFormerConfig(
        backbone_config=ResNetConfig(
            embedding_size=8,
            hidden_sizes=[16, 32, 64, 128],
            depths=[1, 1, 1, 1],
            out_features=["stage1", "stage2", "stage3", "stage4"],
        ),
        decoder_config=DetrConfig(
            d_model=32,
            decoder_layers=1,
            decoder_attention_heads=4,
            decoder_ffn_dim=64,
            encoder_layers=1,
            encoder_attention_heads=4,
            encoder_ffn_dim=64,
            num_queries=4,
        ),
        fpn_feature_size=32,
        mask_feature_size=32,
        num_labels=65,
        use_auxiliary_loss=False,
    )
    return MaskFormerForInstanceSegmentation(config)


def test_tiny_maskformer_trains_head_while_backbone_and_bn_stay_frozen(
    training, tiny_model
):
    groups = training.configure_trainable(tiny_model, lr=1e-3, backbone_lr=1e-4)
    training.set_training_mode(tiny_model)
    backbone = tiny_model.model.pixel_level_module.encoder
    before = {name: value.clone() for name, value in backbone.state_dict().items()}
    optimizer = torch.optim.AdamW(groups)
    head_before = tiny_model.class_predictor.weight.clone()
    outputs = tiny_model(pixel_values=torch.rand(2, 3, 64, 64))
    assert outputs.loss is None
    scores = training.semantic_scores(
        outputs.class_queries_logits, outputs.masks_queries_logits, (16, 16)
    )
    loss = training.masked_partial_nll(
        scores, torch.full((2, 16, 16), 13), torch.ones(2, 16, 16, dtype=torch.bool)
    )
    loss.backward()
    optimizer.step()
    assert not torch.equal(head_before, tiny_model.class_predictor.weight)
    assert tiny_model.class_predictor.out_features == 66
    assert all(
        torch.equal(before[name], value)
        for name, value in backbone.state_dict().items()
    )
    assert all(param.grad is None for param in backbone.parameters())


def test_unfreeze_only_last_resnet_stage_preserves_frozen_bn(training, tiny_model):
    training.configure_trainable(
        tiny_model, lr=1e-3, backbone_lr=1e-4, unfreeze_last_stage=True
    )
    training.set_training_mode(tiny_model)
    backbone = tiny_model.model.pixel_level_module.encoder
    assert all(not p.requires_grad for p in backbone.encoder.stages[0].parameters())
    assert all(p.requires_grad for p in backbone.encoder.stages[-1].parameters())
    assert all(
        not layer.training
        for layer in backbone.modules()
        if isinstance(layer, torch.nn.modules.batchnorm._BatchNorm)
    )
    assert tiny_model.model.transformer_module.training


def test_processor_matches_deployment_on_original_rgb_and_saves_policy(
    training, tmp_path
):
    from transformers import MaskFormerImageProcessor

    processor = MaskFormerImageProcessor(
        size={"height": 360, "width": 640}, ignore_index=65, size_divisor=32
    )
    training.configure_processor(processor)
    image = np.zeros((720, 1280, 3), np.uint8)
    image[:, :640, 0] = 255
    inputs = training.prepare_inputs(processor, [image], torch.device("cpu"))
    reference = processor(images=image, return_tensors="pt")
    assert torch.equal(inputs["pixel_values"], reference["pixel_values"])
    assert tuple(inputs["pixel_values"].shape) == (1, 3, 384, 640)
    processor.save_pretrained(tmp_path)
    saved = json.loads((tmp_path / "preprocessor_config.json").read_text())
    assert saved["ignore_index"] == 255
    assert not saved.get("do_reduce_labels", saved.get("reduce_labels"))
    assert saved["size"] == {"height": 360, "width": 640}


def test_metrics_report_only_labeled_recall_and_absent_sidewalk(training):
    metrics = training.LabeledMetrics()
    scores = torch.zeros(1, 65, 1, 3)
    scores[:, 13, :, 0] = 1
    scores[:, 2, :, 1:] = 1
    metrics.update(
        scores, torch.tensor([[[13, 13, 255]]]), torch.ones(1, 1, 3, dtype=torch.bool)
    )
    result = metrics.compute()
    assert result["per_class"]["13"]["recall"] == 0.5
    assert result["per_class"]["15"]["recall"] is None
    assert result["sidewalk_present"] is False
    assert result["confusion"][13][2] == 1
    assert result["labeled_pixels"] == 2
    assert "iou" not in result


def test_resume_state_restores_optimizer_rng_scaler_and_sampler(training, tmp_path):
    model = torch.nn.Linear(3, 1)
    optimizer = torch.optim.AdamW(model.parameters(), lr=0.01)
    scaler = torch.amp.GradScaler("cuda", enabled=False)
    model(torch.ones(2, 3)).sum().backward()
    optimizer.step()
    sampler = training.BalancedSampler(
        [{"annotation_review_status": "legacy_propagated_road"}], seed=7
    )
    sampler.draw(3)
    path = tmp_path / "trainer-state.pt"
    training.save_training_state(
        path,
        optimizer,
        scaler,
        sampler,
        step=1,
        best_loss=2.5,
        contract={"data": "hash"},
    )
    expected = (random.random(), np.random.rand(), torch.rand(3), sampler.draw(2))
    fresh_optimizer = torch.optim.AdamW(model.parameters(), lr=0.9)
    resumed_sampler = training.BalancedSampler(
        [{"annotation_review_status": "legacy_propagated_road"}], seed=0
    )
    state = training.restore_training_state(
        path, fresh_optimizer, scaler, resumed_sampler, contract={"data": "hash"}
    )
    actual = (random.random(), np.random.rand(), torch.rand(3), resumed_sampler.draw(2))
    assert state["step"] == 1 and state["best_loss"] == 2.5
    assert (
        actual[:2] == expected[:2]
        and torch.equal(actual[2], expected[2])
        and actual[3] == expected[3]
    )
    assert fresh_optimizer.param_groups[0]["lr"] == 0.01
    assert len(fresh_optimizer.state) == len(optimizer.state) > 0
    with pytest.raises(ValueError, match="contract"):
        training.restore_training_state(
            path, fresh_optimizer, scaler, resumed_sampler, contract={"data": "changed"}
        )


def test_cli_help_and_dry_run_need_no_weights_and_leave_test_unused(
    training, dataset_root, tmp_path
):
    command = [sys.executable, str(ROOT / "tools/train_r50_partial.py")]
    help_run = subprocess.run(
        command + ["--help"], capture_output=True, text=True, check=False
    )
    assert help_run.returncode == 0, help_run.stderr
    output = tmp_path / "model"
    run = subprocess.run(
        command
        + [
            "--dataset",
            str(dataset_root),
            "--output",
            str(output),
            "--dry-run",
            "--max-train-samples",
            "1",
            "--max-val-samples",
            "1",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert run.returncode == 0, run.stderr
    result = json.loads(run.stdout)
    assert result["train_samples"] == 1 and result["val_samples"] == 1
    assert result["test_used"] is False
    assert result["limits"]["max_train_samples"] == 1
    assert not output.exists()
    output.mkdir()
    (output / "keep.txt").write_text("existing work")
    run = subprocess.run(
        command
        + ["--dataset", str(dataset_root), "--output", str(output), "--dry-run"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert run.returncode != 0 and "nonempty" in run.stderr.lower()


def test_strict_load_rejects_missing_actual_transformer_weights(
    training, tiny_model, tmp_path
):
    from safetensors.torch import load_file, save_file

    tiny_model.save_pretrained(tmp_path, safe_serialization=True)
    weights = load_file(tmp_path / "model.safetensors")
    del weights["class_predictor.weight"]
    save_file(weights, tmp_path / "model.safetensors", metadata={"format": "pt"})
    with pytest.raises(ValueError, match="Incomplete.*missing_keys"):
        training.strict_load_model(
            str(tmp_path), local_files_only=True, use_safetensors=True
        )


def test_cli_output_dir_alias_matches_documented_command(training):
    from train_r50_partial import build_parser

    args = build_parser().parse_args(["--output-dir", "/tmp/documented-r50-output"])
    assert args.output == Path("/tmp/documented-r50-output")


def test_amp_overflow_backs_off_without_updating_weights_then_recovers(training):
    parameter = torch.nn.Parameter(torch.tensor([2.0]))
    optimizer = torch.optim.AdamW([parameter], lr=0.01)
    scaler = torch.amp.GradScaler("cpu", init_scale=65536.0)
    scaler.scale((parameter * float("inf")).sum()).backward()
    updated, norm = training.finish_optimizer_step(
        optimizer, scaler, supervised_images=1
    )
    assert not updated and norm is None
    assert parameter.item() == 2.0 and not optimizer.state
    assert scaler.get_scale() == 32768.0
    optimizer.zero_grad(set_to_none=True)
    scaler.scale(parameter.square().sum()).backward()
    updated, norm = training.finish_optimizer_step(
        optimizer, scaler, supervised_images=1
    )
    assert updated and norm == pytest.approx(4.0)
    assert parameter.item() < 2.0 and optimizer.state


def test_optimizer_step_averages_accumulated_supervised_images(training):
    parameter = torch.nn.Parameter(torch.tensor([1.0]))
    optimizer = torch.optim.SGD([parameter], lr=0.1)
    scaler = torch.amp.GradScaler("cpu", init_scale=8.0)
    scaler.scale(parameter.sum() * 2).backward()
    updated, norm = training.finish_optimizer_step(
        optimizer, scaler, supervised_images=2
    )
    assert updated and norm == pytest.approx(1.0)
    assert parameter.item() == pytest.approx(0.9)


def test_non_amp_nonfinite_gradient_still_fails_without_update(training):
    parameter = torch.nn.Parameter(torch.tensor([2.0]))
    optimizer = torch.optim.SGD([parameter], lr=0.1)
    scaler = torch.amp.GradScaler("cpu", enabled=False)
    (parameter * float("inf")).sum().backward()
    with pytest.raises(FloatingPointError, match="gradient"):
        training.finish_optimizer_step(optimizer, scaler, supervised_images=1)
    assert parameter.item() == 2.0


def test_failed_best_export_preserves_previous_valid_files(training, tmp_path):
    directory = tmp_path / "best"
    directory.mkdir()
    (directory / "model.safetensors").write_bytes(b"previous valid weights")
    (directory / "checkpoint-manifest.json").write_text("previous valid manifest")

    class FailingModel:
        def save_pretrained(self, destination, **kwargs):
            (Path(destination) / "model.safetensors").write_bytes(b"incomplete write")
            raise OSError("simulated disk failure")

    with pytest.raises(OSError, match="disk failure"):
        training.save_model_artifacts(
            FailingModel(),
            None,
            directory,
            dataset_hash="a" * 64,
            training={},
            evaluation={},
        )
    assert (directory / "model.safetensors").read_bytes() == b"previous valid weights"
    assert (
        directory / "checkpoint-manifest.json"
    ).read_text() == "previous valid manifest"
    assert list(tmp_path.iterdir()) == [directory]
