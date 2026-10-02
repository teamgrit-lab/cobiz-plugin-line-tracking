"""Partial-region supervision for the unchanged 65-class MaskFormer R50 head.

Unknown pixels are never background. Loss is a mean of per-image, per-present-
class NLLs; origin reliability scales each pixel's contribution before that mean.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import random
import shutil
import tempfile
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image
from torch.utils.data import DataLoader, Dataset

OUTPUT_SIZE = (360, 640)
NUM_CLASSES = 65
IGNORE_INDEX = 255
NEW_STATUS = "assistant_visual_partial"
LEGACY_STATUS = "legacy_propagated_road"


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_split(root: Path, split: str, limit: int | None = None) -> list[dict]:
    """Read only supervised train/val manifests; proposals and review never enter."""
    if split not in ("train", "val"):
        raise ValueError("Only train and val splits are permitted in the trainer")
    if limit is not None and limit < 1:
        raise ValueError("Sample limits must be positive")
    rows = []
    with (Path(root) / "splits" / f"{split}.jsonl").open() as handle:
        for line in handle:
            if not line.strip():
                continue
            row = json.loads(line)
            if row.get("split") != split:
                continue
            if row.get("annotation_review_status") not in (NEW_STATUS, LEGACY_STATUS):
                continue
            if row.get("supervised_pixels", 0) <= 0:
                continue
            if split == "train" and row.get("training_eligible") is not True:
                continue
            paths = [
                row.get(key, "")
                for key in (
                    "image",
                    "partial_label",
                    "valid_mask",
                    "annotation_origin_map",
                )
            ]
            if any(
                {"proposals", "review", "test"}.intersection(Path(p).parts)
                for p in paths
            ):
                continue
            rows.append(row)
    ids = [row["sample_id"] for row in rows]
    if len(ids) != len(set(ids)):
        raise ValueError(f"Duplicate sample IDs in {split}")
    if not rows:
        raise ValueError(f"No eligible supervised samples in {split}")
    return rows[:limit] if limit else rows


def photometric_augmentation(image: np.ndarray, seed: int) -> np.ndarray:
    """Mild brightness/gamma/soft shadow; coordinates and labels never change."""
    rng = np.random.default_rng(seed)
    value = image.astype(np.float32) / 255.0
    value = np.power(value, rng.uniform(0.9, 1.1)) * rng.uniform(0.9, 1.1)
    if rng.random() < 0.5:
        height, width = image.shape[:2]
        y, x = np.ogrid[:height, :width]
        cx, cy = rng.uniform(0, width), rng.uniform(0, height)
        sx, sy = rng.uniform(0.15, 0.45) * width, rng.uniform(0.15, 0.45) * height
        shadow = np.exp(-0.5 * (((x - cx) / sx) ** 2 + ((y - cy) / sy) ** 2))
        value *= 1.0 - rng.uniform(0.05, 0.2) * shadow[..., None]
    return np.clip(value * 255.0, 0, 255).astype(np.uint8)


class PartialDataset(Dataset):
    def __init__(
        self, root: Path, rows: list[dict], *, output_size=OUTPUT_SIZE, augment=False
    ):
        self.root = Path(root).resolve()
        self.rows = rows
        self.output_size = tuple(output_size)
        self.augment = augment

    def __len__(self):
        return len(self.rows)

    def _path(self, row: dict, key: str) -> Path:
        path = (self.root / row[key]).resolve()
        if not path.is_relative_to(self.root) or not path.is_file():
            raise ValueError(f"Dataset file missing or outside dataset: {row[key]}")
        return path

    def read_native(self, index: int, *, verify_hashes=False) -> dict:
        row = self.rows[index]
        values = {}
        for key in ("image", "partial_label", "valid_mask", "annotation_origin_map"):
            path = self._path(row, key)
            expected_hash = row.get("sha256", {}).get(key)
            if verify_hashes and expected_hash and file_sha256(path) != expected_hash:
                raise ValueError(f"Dataset hash mismatch: {path}")
            with Image.open(path) as image:
                if key != "image" and image.mode != "L":
                    raise ValueError(f"Expected grayscale class/mask PNG: {path}")
                values[key] = np.array(
                    image.convert("RGB") if key == "image" else image, copy=True
                )
        shape = values["image"].shape[:2]
        expected = (row.get("height", shape[0]), row.get("width", shape[1]))
        if shape != expected or any(
            values[k].shape != shape
            for k in ("partial_label", "valid_mask", "annotation_origin_map")
        ):
            raise ValueError(
                f"Native image/label/valid/origin alignment mismatch: {row['sample_id']}"
            )
        labels, valid, origins = (
            values[k] for k in ("partial_label", "valid_mask", "annotation_origin_map")
        )
        if not np.all((labels < NUM_CLASSES) | (labels == IGNORE_INDEX)):
            raise ValueError(f"Class IDs must be 0..64 or 255: {row['sample_id']}")
        if not np.isin(valid, [0, 255]).all() or not np.isin(origins, [0, 1, 2]).all():
            raise ValueError(
                f"Invalid valid-mask or origin-map values: {row['sample_id']}"
            )
        supervised = (labels != IGNORE_INDEX) & (valid != 0)
        if np.any(supervised & (origins == 0)):
            raise ValueError(
                f"Supervised pixels have missing annotation origins: {row['sample_id']}"
            )
        return values

    def __getitem__(self, key):
        index, seed = key if isinstance(key, tuple) else (key, key)
        native = self.read_native(index)
        image = native["image"]
        if self.augment:
            image = photometric_augmentation(image, seed)
        height, width = self.output_size

        def resize(name):
            # Resize the native annotation directly to the deployed score map.
            # The image processor's divisor resize must NOT affect label geometry.
            array = np.array(
                Image.fromarray(native[name]).resize(
                    (width, height), Image.Resampling.NEAREST
                ),
                copy=True,
            )
            return torch.from_numpy(array)

        return {
            "image": image,
            "labels": resize("partial_label").long(),
            "valid_mask": resize("valid_mask").bool(),
            "origins": resize("annotation_origin_map").long(),
            "sample_id": self.rows[index]["sample_id"],
            "native_shape": list(image.shape[:2]),
        }


def collate_samples(samples):
    return {
        "images": [sample["image"] for sample in samples],
        **{
            key: torch.stack([sample[key] for sample in samples])
            for key in ("labels", "valid_mask", "origins")
        },
        "sample_ids": [sample["sample_id"] for sample in samples],
    }


class BalancedSampler:
    """Replacement sampling, 50/50 when both origins exist, with replayable seeds.

    Draws are made in the main process for one optimizer step at a time. Workers
    receive explicit augmentation seeds; prefetch cannot advance saved state.
    """

    def __init__(self, rows: list[dict], seed: int):
        self.groups = [
            [
                i
                for i, row in enumerate(rows)
                if row["annotation_review_status"] == status
            ]
            for status in (NEW_STATUS, LEGACY_STATUS)
        ]
        self.groups = [group for group in self.groups if group]
        if not self.groups:
            raise ValueError("Sampler needs at least one eligible group")
        self.generator = torch.Generator().manual_seed(seed)
        self.draws = 0

    def draw(self, count: int) -> list[tuple[int, int]]:
        result = []
        for _ in range(count):
            group = self.groups[
                int(torch.randint(len(self.groups), (), generator=self.generator))
            ]
            index = group[int(torch.randint(len(group), (), generator=self.generator))]
            seed = int(torch.randint(2**31, (), generator=self.generator))
            result.append((index, seed))
        self.draws += count
        return result

    def state_dict(self):
        return {
            "generator": self.generator.get_state().clone(),
            "draws": self.draws,
            "groups": self.groups,
        }

    def load_state_dict(self, state):
        if self.groups != state["groups"]:
            raise ValueError("Sampler groups changed on resume")
        self.generator.set_state(state["generator"])
        self.draws = state["draws"]


def semantic_scores(class_logits, mask_logits, output_size=OUTPUT_SIZE):
    if class_logits.shape[-1] != NUM_CLASSES + 1:
        raise ValueError("MaskFormer must retain 65 classes plus its null query class")
    # Autocast must also be disabled for einsum, not merely its inputs.
    with torch.autocast(device_type=class_logits.device.type, enabled=False):
        probabilities = class_logits.float().softmax(dim=-1)[..., :-1]
        masks = mask_logits.float().sigmoid()
        scores = torch.einsum("bqc,bqhw->bchw", probabilities, masks)
        return F.interpolate(
            scores, size=output_size, mode="bilinear", align_corners=False
        )


def class_weights(*, curb=2.0, sidewalk=2.0, road=1.0, negative=1.0, car_mount=0.5):
    weights = torch.full((NUM_CLASSES,), float(negative), dtype=torch.float32)
    weights[2], weights[15], weights[13], weights[63] = curb, sidewalk, road, car_mount
    return weights


def masked_partial_nll(
    scores,
    labels,
    valid_mask,
    origins=None,
    *,
    weights=None,
    legacy_reliability=0.5,
    new_reliability=1.0,
):
    """Class-balanced per-image NLL; only label!=255 AND valid pixels contribute.

    Reliability divides by pixel count (not sum of reliability), so an entirely
    legacy image has half the contribution of an equivalent new annotation.
    """
    if (
        scores.shape[1] != NUM_CLASSES
        or labels.shape != valid_mask.shape
        or labels.shape != (scores.shape[0], *scores.shape[-2:])
    ):
        raise ValueError("Score and target geometry/classes do not match")
    with torch.autocast(device_type=scores.device.type, enabled=False):
        scores = scores.float()
        probabilities = scores / scores.sum(dim=1, keepdim=True).clamp_min(1e-12)
        weights = (class_weights() if weights is None else weights).to(
            scores.device, dtype=torch.float32
        )
        image_losses = []
        for index in range(scores.shape[0]):
            valid = valid_mask[index].bool() & labels[index].ne(IGNORE_INDEX)
            if not valid.any():
                continue
            target = labels[index][valid]
            if ((target < 0) | (target >= NUM_CLASSES)).any():
                raise ValueError("Supervised class IDs must be in 0..64")
            pixel_prob = (
                probabilities[index]
                .permute(1, 2, 0)[valid]
                .gather(1, target[:, None])
                .squeeze(1)
            )
            nll = -pixel_prob.clamp_min(1e-12).log()
            if origins is not None:
                source = origins[index][valid]
                if not ((source == 1) | (source == 2)).all():
                    raise ValueError("Every supervised pixel needs an origin of 1 or 2")
                reliability = torch.where(
                    source == 1, legacy_reliability, new_reliability
                )
                nll = nll * reliability
            counts = torch.bincount(target, minlength=NUM_CLASSES)
            totals = torch.zeros(NUM_CLASSES, device=scores.device).scatter_add(
                0, target, nll
            )
            present = counts > 0
            image_losses.append(
                ((totals[present] / counts[present]) * weights[present]).sum()
                / weights[present].sum()
            )
        return torch.stack(image_losses).mean() if image_losses else scores.sum() * 0.0


def configure_trainable(model, *, lr, backbone_lr, unfreeze_last_stage=False):
    for parameter in model.parameters():
        parameter.requires_grad_(True)
    backbone = model.model.pixel_level_module.encoder
    for parameter in backbone.parameters():
        parameter.requires_grad_(False)
    if unfreeze_last_stage:
        for parameter in backbone.encoder.stages[-1].parameters():
            parameter.requires_grad_(True)
    backbone_ids = {id(parameter) for parameter in backbone.parameters()}
    groups = [
        {
            "params": [
                p
                for p in model.parameters()
                if p.requires_grad and id(p) not in backbone_ids
            ],
            "lr": lr,
        }
    ]
    unfrozen = [p for p in backbone.parameters() if p.requires_grad]
    if unfrozen:
        groups.append({"params": unfrozen, "lr": backbone_lr})
    return groups


def set_training_mode(model):
    model.train()
    # Even when stage4 weights are trainable, keep inherited BN running statistics.
    model.model.pixel_level_module.encoder.eval()


def finish_optimizer_step(optimizer, scaler, *, supervised_images):
    """Average gradients and let AMP back off on overflow without counting a step."""
    scaler.unscale_(optimizer)
    parameters = [
        parameter
        for group in optimizer.param_groups
        for parameter in group["params"]
        if parameter.grad is not None
    ]
    for parameter in parameters:
        parameter.grad.div_(supervised_images)
    norm = torch.linalg.vector_norm(
        torch.stack([torch.linalg.vector_norm(p.grad) for p in parameters])
    )
    if not torch.isfinite(norm):
        # unscale_ already recorded these overflows. step skips the optimizer,
        # and update lowers the scale so the next batch can recover.
        if scaler.is_enabled() and any(
            not torch.isfinite(p.grad).all() for p in parameters
        ):
            scaler.step(optimizer)
            scaler.update()
            return False, None
        raise FloatingPointError(
            "Nonfinite gradient norm outside recoverable AMP overflow"
        )
    torch.nn.utils.clip_grad_norm_(parameters, 1.0, error_if_nonfinite=True)
    scaler.step(optimizer)
    scaler.update()
    return True, float(norm)


def configure_processor(processor):
    processor.size = {"height": OUTPUT_SIZE[0], "width": OUTPUT_SIZE[1]}
    processor.ignore_index = IGNORE_INDEX
    processor.do_reduce_labels = False
    if hasattr(processor, "reduce_labels"):
        processor.reduce_labels = False
    return processor


def prepare_inputs(processor, images, device):
    inputs = processor(images=images, return_tensors="pt")
    values = inputs["pixel_values"]
    # MaskFormer rounds the requested size UP using size_divisor, rather than
    # letterboxing. Derive this from the actual saved processor configuration.
    divisor = int(processor.size_divisor)
    expected = tuple(math.ceil(size / divisor) * divisor for size in OUTPUT_SIZE)
    if (
        tuple(values.shape) != (len(images), 3, *expected)
        or not torch.isfinite(values).all()
    ):
        raise ValueError(
            f"Unexpected processor tensor shape/values: {tuple(values.shape)}, expected {(len(images), 3, *expected)}"
        )
    return {name: value.to(device) for name, value in inputs.items()}


def strict_load_model(path_or_id, **kwargs):
    from transformers import MaskFormerForInstanceSegmentation

    model, info = MaskFormerForInstanceSegmentation.from_pretrained(
        path_or_id, output_loading_info=True, **kwargs
    )
    problems = {
        key: info.get(key)
        for key in ("missing_keys", "mismatched_keys", "unexpected_keys", "error_msgs")
        if info.get(key)
    }
    if problems:
        raise ValueError(f"Incomplete MaskFormer checkpoint load rejected: {problems}")
    from r50_checkpoint import MAPILLARY_ID2LABEL

    if {str(k): v for k, v in model.config.id2label.items()} != MAPILLARY_ID2LABEL:
        raise ValueError("Model does not preserve the 65 Mapillary labels")
    backbone = model.config.backbone_config
    if (
        backbone.model_type != "resnet"
        or list(backbone.depths) != [3, 4, 6, 3]
        or list(backbone.hidden_sizes) != [256, 512, 1024, 2048]
        or backbone.layer_type != "bottleneck"
        or backbone.num_channels != 3
    ):
        raise ValueError("Model must preserve the original ResNet-50 backbone")
    return model


class LabeledMetrics:
    def __init__(self):
        self.confusion = torch.zeros(NUM_CLASSES, NUM_CLASSES, dtype=torch.int64)

    def update(self, scores, labels, valid_mask):
        valid = valid_mask.bool() & labels.ne(IGNORE_INDEX)
        target = labels[valid].cpu()
        predicted = scores.argmax(dim=1)[valid].cpu()
        self.confusion += torch.bincount(
            target * NUM_CLASSES + predicted, minlength=NUM_CLASSES**2
        ).reshape(NUM_CLASSES, NUM_CLASSES)

    def compute(self):
        from r50_checkpoint import MAPILLARY_ID2LABEL

        support = self.confusion.sum(dim=1)
        per_class = {
            str(index): {
                "name": MAPILLARY_ID2LABEL[str(index)],
                "labeled_pixels": int(support[index]),
                "recall": float(self.confusion[index, index] / support[index])
                if support[index]
                else None,
            }
            for index in range(NUM_CLASSES)
        }
        return {
            "scope": "Recall and confusion only on labeled partial regions; not full-image IoU or FPR",
            "split": "val",
            "test_used": False,
            "labeled_pixels": int(support.sum()),
            "per_class": per_class,
            "confusion": self.confusion.tolist(),
            "confusion_axes": "rows=true, columns=predicted; original Mapillary IDs 0..64",
            "sidewalk_present": bool(support[15]),
            "sidewalk_note": "Labeled sidewalk recall is available"
            if support[15]
            else "Sidewalk absent from labeled validation pixels; recall unavailable",
        }


def make_loader(dataset, *, batch_size, workers, keys=None, seed=0):
    return DataLoader(
        dataset,
        batch_size=batch_size,
        sampler=keys,
        shuffle=False,
        num_workers=workers,
        collate_fn=collate_samples,
        generator=torch.Generator().manual_seed(seed),
    )


def evaluate(model, processor, dataset, *, device, batch_size, workers, loss_options):
    model.eval()
    metrics = LabeledMetrics()
    total, images = 0.0, 0
    input_shape = None
    with torch.no_grad():
        for batch in make_loader(dataset, batch_size=batch_size, workers=workers):
            inputs = prepare_inputs(processor, batch["images"], device)
            input_shape = list(inputs["pixel_values"].shape[1:])
            with torch.autocast(
                device_type=device.type,
                dtype=torch.float16,
                enabled=device.type == "cuda",
            ):
                outputs = model(**inputs)
            scores = semantic_scores(
                outputs.class_queries_logits, outputs.masks_queries_logits
            )
            labels, valid, origins = (
                batch[key].to(device) for key in ("labels", "valid_mask", "origins")
            )
            loss = masked_partial_nll(scores, labels, valid, origins, **loss_options)
            if not torch.isfinite(loss) or not torch.isfinite(scores).all():
                raise FloatingPointError("Nonfinite validation scores/loss")
            count = int((valid & labels.ne(IGNORE_INDEX)).flatten(1).any(1).sum())
            total += float(loss) * count
            images += count
            metrics.update(scores, labels, valid)
    result = metrics.compute()
    result.update(
        masked_nll=total / images if images else None,
        supervised_images=images,
        samples=len(dataset),
        runtime_input_shape=input_shape,
    )
    return result


def save_training_state(path, optimizer, scaler, sampler, *, step, best_loss, contract):
    state = {
        "version": 1,
        "step": step,
        "best_loss": best_loss,
        "contract": contract,
        "optimizer": optimizer.state_dict(),
        "scaler": scaler.state_dict(),
        "sampler": sampler.state_dict(),
        "python_rng": random.getstate(),
        "numpy_rng": np.random.get_state(),
        "torch_rng": torch.get_rng_state(),
        "cuda_rng": torch.cuda.get_rng_state_all()
        if torch.cuda.is_available()
        else None,
    }
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    torch.save(state, temporary)
    os.replace(temporary, path)


def restore_training_state(path, optimizer, scaler, sampler, *, contract):
    # This is our explicitly selected local training state, containing Python and
    # NumPy RNG tuples as well as tensors; it is not a downloadable model pickle.
    state = torch.load(path, map_location="cpu", weights_only=False)
    if state.get("version") != 1 or state.get("contract") != contract:
        raise ValueError(
            "Resume contract differs: dataset, sampling, loss or training configuration changed"
        )
    optimizer.load_state_dict(state["optimizer"])
    scaler.load_state_dict(state["scaler"])
    sampler.load_state_dict(state["sampler"])
    random.setstate(state["python_rng"])
    np.random.set_state(state["numpy_rng"])
    torch.set_rng_state(state["torch_rng"])
    if state["cuda_rng"] is not None and torch.cuda.is_available():
        torch.cuda.set_rng_state_all(state["cuda_rng"])
    return state


def save_model_artifacts(
    model, processor, directory, *, dataset_hash, training, evaluation
):
    from r50_checkpoint import validate_checkpoint, write_checkpoint_manifest

    directory = Path(directory)
    directory.parent.mkdir(parents=True, exist_ok=True)
    previous = directory.with_name(f".{directory.name}-previous")
    # Recover a process interruption between the two directory renames.
    if not directory.exists() and previous.exists():
        os.replace(previous, directory)
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{directory.name}-export-", dir=directory.parent)
    )
    try:
        model.save_pretrained(temporary, safe_serialization=True)
        processor.save_pretrained(temporary)
        manifest = write_checkpoint_manifest(
            temporary,
            dataset_manifest_sha256=dataset_hash,
            training=training,
            evaluation=evaluation,
        )
        validate_checkpoint(temporary)
        if previous.exists():
            shutil.rmtree(previous)
        if directory.exists():
            os.replace(directory, previous)
        try:
            os.replace(temporary, directory)
        except OSError:
            if previous.exists() and not directory.exists():
                os.replace(previous, directory)
            raise
        if previous.exists():
            shutil.rmtree(previous)
        return manifest
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
