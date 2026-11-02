#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12,<3.13"
# dependencies = [
#   "numpy==2.5.4", "onnx==1.23.2", "onnxruntime==1.31.1",
#   "pillow==13.0.0", "torch==2.14.1", "torchvision==0.29.1",
# ]
# ///
"""Fine-tune the 512x256 Cityscapes FCN on the reviewed Dangjin 360 camera views."""

import argparse
import hashlib
import json
import math
import os
import random
import tempfile
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch
from onnx import numpy_helper
from PIL import Image
from torch import nn
from torch.nn import functional as F
from torch.utils.data import DataLoader, Dataset
from torchvision.models import resnet18

BASE_SHA256 = "f9632c857366cf0115c036a35642a72703205cd8f0666851bdd604732788f43b"
ROAD, IGNORE = 3, 255
HEIGHT, WIDTH = 256, 512
CLASSES = ("void ego_vehicle ground road sidewalk building wall fence pole traffic_light "
           "traffic_sign vegetation terrain sky person car truck bus train motorcycle bicycle").split()


def digest(path):
    with path.open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def write_json(path, data):
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    temporary.replace(path)


def arguments():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    # A copy beside the base model on a removable drive also works after a mount-point change.
    here = Path(__file__).resolve().parent
    volume = here.parent.parent if here.name == "fcn512-training" else Path("/Volumes/MY_HDD")
    parser.add_argument("--volume", type=Path, default=volume, help="Drive containing connectum_data and connectum_models")
    parser.add_argument("--dataset", type=Path, help="Override the dataset directory")
    parser.add_argument("--base-onnx", type=Path, help="Override path to the original FCN-ResNet18 512x256 ONNX")
    parser.add_argument("--output-root", type=Path, help="Parent of a unique directory for each training run")
    parser.add_argument("--device", choices=["auto", "cpu", "mps", "cuda"], default="auto")
    parser.add_argument("--epochs", type=int, default=8)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--lr", type=float, default=0.0003, help="Classifier learning rate")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if args.epochs < 1 or args.batch_size < 1 or not math.isfinite(args.lr) or args.lr <= 0 or args.seed < 0:
        parser.error("epochs, batch-size and lr must be positive; seed must be nonnegative")
    models = args.volume / "connectum_models"
    args.dataset = (args.dataset or args.volume / "connectum_data/real360/fcn512-road-dataset-v1").expanduser().resolve()
    args.base_onnx = (args.base_onnx or models / "fcn512-training/base/fcn_resnet18.onnx").expanduser().resolve()
    args.output_root = (args.output_root or models / "fcn512-road-runs").expanduser().resolve()
    return args


class CityscapesFCN(nn.Module):
    """Match the named backbone/classifier tensors in the original Jetson FCN ONNX."""

    def __init__(self):
        super().__init__()
        self.backbone = resnet18(weights=None)
        del self.backbone.fc
        self.classifier = nn.Sequential(
            nn.Conv2d(512, 128, 3, padding=1, bias=False), nn.BatchNorm2d(128),
            nn.ReLU(), nn.Dropout(0.1), nn.Conv2d(128, len(CLASSES), 1),
        )

    def forward(self, image):
        net = self.backbone
        x = net.maxpool(net.relu(net.bn1(net.conv1(image))))
        x = net.layer4(net.layer3(net.layer2(net.layer1(x))))
        return self.classifier(x)


def restore_base(path):
    if not path.is_file():
        raise FileNotFoundError(f"Missing FCN base: {path}. Supply the original model with --base-onnx.")
    if digest(path) != BASE_SHA256:
        raise ValueError("Base ONNX does not match the expected 512x256 Cityscapes weights")
    graph = onnx.load(path)
    model = CityscapesFCN()
    state = {t.name: torch.from_numpy(numpy_helper.to_array(t).copy()) for t in graph.graph.initializer}
    model.load_state_dict(state, strict=True)
    return model


def check_onnx_parity(model, path, samples):
    options = ort.SessionOptions()
    options.intra_op_num_threads = 3
    options.log_severity_level = 3
    session = ort.InferenceSession(str(path), sess_options=options, providers=["CPUExecutionProvider"])
    model.cpu().eval()
    max_error = 0.0
    with torch.inference_mode():
        for sample in samples:
            sample = sample.unsqueeze(0)
            native = session.run(["output_0"], {"input_0": sample.numpy()})[0]
            restored = model(sample).numpy()
            if native.shape != (1, 21, 8, 16) or not np.isfinite(restored).all():
                raise ValueError(f"Unexpected FCN output: {native.shape}")
            np.testing.assert_allclose(restored, native, rtol=2e-4, atol=2e-4)
            max_error = max(max_error, float(np.max(np.abs(restored - native))))
    return max_error


def reviewed_rows(root, split):
    paired = {}
    lists = (("", "semantic_label", "labels_semantic"), ("-binary", "binary_road_label", "labels_road_binary"))
    for suffix, label_key, label_dir in lists:
        path = root / "splits" / f"{split}{suffix}.jsonl"
        seen = set()
        for line in path.read_text().splitlines():
            row = json.loads(line)
            key = row["id"]
            if key in seen or row["split"] != split or row["annotation_status"] != "visual_partial":
                raise ValueError(f"{path}: duplicate or unreviewed/wrong-split row {key}")
            seen.add(key)
            stamp = row["pts_time_sec"]
            is_id482 = "ID482" in (row["clip"], row["recording_group"]) or row["source_key"] in ("id264", "id265")
            if not math.isfinite(stamp) or stamp < 0 or (is_id482 and stamp >= 1500):
                raise ValueError(f"{key}: invalid timestamp or excluded ID482 footage")
            for field, directory in (("image", "images"), (label_key, label_dir)):
                entry = Path(row[field])
                if entry.is_absolute() or ".." in entry.parts or entry.parts[:1] != (directory,):
                    raise ValueError(f"{key}: invalid {field} path {entry}")
            if key not in paired:
                paired[key] = {field: row[field] for field in ("id", "image", "recording_group", "group_id", "pts_time_sec", "view")}
            item = paired[key]
            if any(row[field] != item[field] for field in ("image", "recording_group", "group_id", "pts_time_sec", "view")):
                raise ValueError(f"{key}: the binary and semantic lists reference different views")
            item[label_key] = row[label_key]
    if not paired:
        raise ValueError(f"No reviewed {split} samples")
    return list(paired.values())


class ReviewedViews(Dataset):
    """Pair both annotation types with one image. Missing supervision is ignored."""
    def __init__(self, root, split):
        self.root = root
        self.rows = reviewed_rows(root, split)

    def __len__(self):
        return len(self.rows)

    def mask(self, row, key, highest_class):
        if key not in row:
            return torch.full((HEIGHT, WIDTH), IGNORE, dtype=torch.uint8)
        with Image.open(self.root / row[key]) as mask:
            values = np.array(mask)
        if (values.shape != (HEIGHT, WIDTH) or values.dtype != np.uint8
                or np.any((values != IGNORE) & (values > highest_class)) or np.all(values == IGNORE)):
            raise ValueError(f"Invalid target: {row[key]}")
        return torch.from_numpy(values)

    def __getitem__(self, index):
        row = self.rows[index]
        with Image.open(self.root / row["image"]) as img:
            rgb = np.array(img.convert("RGB"))
        if rgb.shape != (HEIGHT, WIDTH, 3):
            raise ValueError(f"Invalid input shape for {row['id']}: {rgb.shape}")
        return (torch.from_numpy(rgb).permute(2, 0, 1), self.mask(row, "semantic_label", 20),
                self.mask(row, "binary_road_label", 1))


def normalize(rgb):
    mean = rgb.new_tensor([0.485, 0.456, 0.406], dtype=torch.float32).view(3, 1, 1)
    std = rgb.new_tensor([0.229, 0.224, 0.225], dtype=torch.float32).view(3, 1, 1)
    return (rgb.float() / 255.0 - mean) / std


def read_dataset(root):
    schema = json.loads((root / "schema.json").read_text())
    if not (schema["classes"] == {str(i): c for i, c in enumerate(CLASSES)}
            and schema["road_id"] == ROAD and schema["ignore_index"] == IGNORE
            and schema["shape"] == [HEIGHT, WIDTH]
            and schema["dense_binary"] == {"nonroad": 0, "road": 1, "ignore": 255}
            and schema["input"]["normalization"] == {"mean": [.485, .456, .406], "std": [.229, .224, .225]}):
        raise ValueError("Dataset does not match the Dangjin FCN label/input contract")
    train = ReviewedViews(root, "train")
    valid = ReviewedViews(root, "validation")
    for field in ("id", "image", "group_id", "recording_group"):
        if {r[field] for r in train.rows} & {r[field] for r in valid.rows}:
            raise ValueError(f"Training and validation overlap on {field}")
    return train, valid, schema


def partial_loss(logits, target, weights=None):
    # A batch can contain no binary annotations. Sum and normalize only over valid pixels.
    valid = target != IGNORE
    if weights is None:
        denominator = valid.sum()
    else:
        denominator = (weights[target.masked_fill(~valid, 0)] * valid).sum()
    total = F.cross_entropy(logits, target, weight=weights, ignore_index=IGNORE, reduction="sum")
    return total / denominator.clamp_min(1)


def train_epoch(model, loader, optimizer, device, epoch):
    # The two camera recordings are too small to refit backbone BN statistics.
    model.train()
    for module in model.modules():
        if isinstance(module, nn.BatchNorm2d):
            module.eval()
    model.backbone.layer4.requires_grad_(epoch > 2)
    weights = torch.ones(len(CLASSES), device=device)
    weights[1], weights[4], weights[11] = 3, 3, 2
    total = 0.0
    for image, semantic, binary in loader:
        rgb = image.to(device).float()
        # Vary exposure without moving the camera mast or label polygons.
        rgb = (rgb * random.uniform(.9, 1.1) + random.uniform(-6, 6)).clamp(0, 255)
        scores = model(normalize(rgb))
        scores = F.interpolate(scores, size=(HEIGHT, WIDTH), mode="bilinear", align_corners=False)
        loss = partial_loss(scores, semantic.to(device, dtype=torch.long), weights)
        if bool((binary != IGNORE).any()):
            nonroad = torch.cat((scores[:, 1:ROAD], scores[:, ROAD + 1:]), dim=1).logsumexp(dim=1)
            binary_scores = torch.stack((nonroad, scores[:, ROAD]), dim=1)
            loss = loss + 0.5 * partial_loss(binary_scores, binary.to(device, dtype=torch.long))
        if not torch.isfinite(loss).item():
            raise FloatingPointError("Non-finite training loss")
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        nn.utils.clip_grad_norm_(model.parameters(), 1.0, error_if_nonfinite=True)
        optimizer.step()
        total += float(loss.detach()) * len(image)
    return total / len(loader.dataset)


def ratio(numerator, denominator):
    return float(numerator / denominator) if denominator else None


@torch.inference_mode()
def evaluate(model, loader, device):
    model.eval()
    sem_counts = np.zeros((len(CLASSES), 2), dtype=np.int64)
    bin_counts = np.zeros((2, 2), dtype=np.int64)
    for image, semantic, binary in loader:
        native = model(normalize(image.to(device)))
        if not torch.isfinite(native).all().item():
            raise FloatingPointError("Non-finite validation output")
        # Match the earlier FCN video evaluation: ignore void, nearest-cell upsampling.
        road = (native[:, 1:].argmax(dim=1) + 1 == ROAD).cpu().numpy()
        road = road.repeat(HEIGHT // native.size(2), axis=1).repeat(WIDTH // native.size(3), axis=2)
        for target, counts in ((semantic, sem_counts), (binary, bin_counts)):
            target = target.numpy()
            keep = target != IGNORE
            indices = 2 * target[keep].astype(np.intp) + road[keep]
            counts += np.bincount(indices, minlength=counts.size).reshape(counts.shape)
    tn, fp, fn, tp = map(int, bin_counts.ravel())
    road_hits = int(sem_counts[ROAD, 1])
    semantic_union = int(sem_counts[ROAD].sum() + sem_counts[:, 1].sum() - road_hits)
    return {
        "binary_road_iou": ratio(tp, tp + fp + fn), "binary_road_recall": ratio(tp, tp + fn),
        "binary_road_precision": ratio(tp, tp + fp),
        "binary_pixel_counts": {"tn": tn, "fp": fp, "fn": fn, "tp": tp},
        "semantic_road_iou": ratio(road_hits, semantic_union),
        "false_road_rate": {name: ratio(sem_counts[i, 1], sem_counts[i].sum())
                            for i, name in enumerate(CLASSES) if i in (1, 4, 5, 11, 14)},
        "semantic_pixel_counts": {CLASSES[i]: int(n) for i, n in enumerate(sem_counts.sum(axis=1))},
    }


def export_best(run, base_path, samples):
    model = CityscapesFCN()
    checkpoint = torch.load(run / "best.pt", weights_only=True, map_location="cpu")
    model.load_state_dict(checkpoint["state_dict"], strict=True)
    # Preserve the Jetson ONNX graph, I/O names, opset and fixed input dimensions.
    graph = onnx.load(base_path)
    state = model.state_dict()
    for tensor in graph.graph.initializer:
        tensor.CopyFrom(numpy_helper.from_array(state[tensor.name].numpy(), name=tensor.name))
    onnx.checker.check_model(graph)
    temporary = run / "fcn_resnet18.tmp.onnx"
    onnx.save(graph, temporary)
    error = check_onnx_parity(model, temporary, samples)
    temporary.replace(run / "fcn_resnet18.onnx")
    return error


def choose_device(requested):
    if requested != "auto":
        return torch.device(requested)
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("mps" if torch.backends.mps.is_available() else "cpu")


def selection_score(metrics):
    iou = metrics["binary_road_iou"]
    secondary = metrics["semantic_road_iou"]
    if iou is None or secondary is None:
        raise ValueError("Reviewed validation needs visible road in both the binary and the semantic targets")
    return iou, secondary


def main():
    args = arguments()
    random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.set_num_threads(min(4, os.cpu_count() or 1))
    device = choose_device(args.device)
    train, valid, schema = read_dataset(args.dataset)
    loader = DataLoader(train, batch_size=args.batch_size, shuffle=True,
                        generator=torch.Generator().manual_seed(args.seed))
    val_loader = DataLoader(valid, batch_size=args.batch_size, shuffle=False)
    examples = [normalize(valid[index][0]) for index in np.linspace(0, len(valid) - 1, min(4, len(valid)), dtype=int)]
    base = restore_base(args.base_onnx)
    base_error = check_onnx_parity(base, args.base_onnx, examples)
    model = base.to(device)
    model.backbone.requires_grad_(False)
    optimizer = torch.optim.AdamW([
        {"params": model.classifier.parameters(), "lr": args.lr},
        {"params": model.backbone.layer4.parameters(), "lr": args.lr / 10},
    ], weight_decay=1e-4)

    args.output_root.mkdir(parents=True, exist_ok=True)
    # A run has its own directory even when started twice in the same second.
    run = Path(tempfile.mkdtemp(prefix=datetime.now().strftime("%Y%m%d-%H%M%S-"), dir=args.output_root))
    split_files = ["train.jsonl", "train-binary.jsonl", "validation.jsonl", "validation-binary.jsonl"]
    config = {
        "architecture": "FCN-ResNet18-Cityscapes-512x256", "dataset": str(args.dataset),
        "dataset_contract": schema, "base_onnx_sha256": BASE_SHA256, "base_onnx": str(args.base_onnx),
        "device": str(device), "epochs": args.epochs, "batch_size": args.batch_size, "seed": args.seed,
        "lr_classifier": args.lr, "lr_layer4": args.lr / 10, "head_only_epochs": min(args.epochs, 2),
        "optimizer": "AdamW", "binary_loss_weight": .5, "semantic_weights": {"ego_vehicle": 3, "sidewalk": 3, "vegetation": 2, "default": 1},
        "selection": "highest binary road IoU, ties use semantic road IoU, among updated epochs",
        "runtime": "input_0: RGB ImageNet float32 [1,3,256,512]; output_0: logits [1,21,8,16]; argmax over classes 1..20 then nearest upsample",
        "supervised": {s: {"views": len(ds), "binary_views": sum("binary_road_label" in r for r in ds.rows)}
                       for s, ds in (("train", train), ("validation", valid))},
        "split_sha256": {name: digest(args.dataset / "splits" / name) for name in split_files},
    }
    write_json(run / "config.json", config)
    print(f"Device {device} | train {len(train)} | validation {len(valid)} | run {run}", flush=True)
    baseline = evaluate(model, val_loader, device)
    print(f"Baseline binary road IoU {selection_score(baseline)[0]:.4f}", flush=True)
    report = {"baseline": baseline, "epochs": [], "base_onnx_max_abs_diff": base_error}
    write_json(run / "metrics.json", report)
    best_score = (-1.0, -1.0)
    for epoch in range(1, args.epochs + 1):
        loss = train_epoch(model, loader, optimizer, device, epoch)
        metrics = evaluate(model, val_loader, device)
        score = selection_score(metrics)
        report["epochs"].append({"epoch": epoch, "train_loss": loss, "validation": metrics})
        if score > best_score:
            best_score = score
            weights = {name: tensor.detach().cpu() for name, tensor in model.state_dict().items()}
            torch.save({"architecture": config["architecture"], "epoch": epoch, "state_dict": weights},
                       run / "best.tmp.pt")
            (run / "best.tmp.pt").replace(run / "best.pt")
            report["best_epoch"] = epoch
        write_json(run / "metrics.json", report)
        print(f"Epoch {epoch}/{args.epochs} | loss {loss:.4f} | binary road IoU {score[0]:.4f} | "
              f"partial road IoU {score[1]:.4f} | best epoch {report['best_epoch']}", flush=True)

    export_error = export_best(run, args.base_onnx, examples)
    (run / "classes.txt").write_text("\n".join(CLASSES) + "\n")
    # Written last; its presence means both the checkpoint and the exported graph passed reload/parity.
    write_json(run / "complete.json", {
        "finished_utc": datetime.now(timezone.utc).isoformat(), "best_epoch": report["best_epoch"],
        "baseline_binary_road_iou": baseline["binary_road_iou"], "best_binary_road_iou": best_score[0],
        "onnx_vs_reloaded_pt_max_abs_diff": export_error, "parity_views": len(examples),
        "sha256": {name: digest(run / name) for name in ("best.pt", "fcn_resnet18.onnx", "config.json", "metrics.json", "classes.txt")},
    })
    print(f"Finished | best epoch {report['best_epoch']} | ONNX and PyTorch saved in {run}", flush=True)


if __name__ == "__main__":
    main()
