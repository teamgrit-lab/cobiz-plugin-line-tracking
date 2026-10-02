#!/usr/bin/env python3
"""Fine-tune pinned MaskFormer R50 on supervised partial labels, never test data.

output/best contains the best *updated* model and its deployment manifest.
checkpoints/last contains the latest evaluated model plus complete trainer state.
--max-steps counts optimizer updates (not batches); on resume it is the total
desired step count. --dry-run checks selected data without loading/downloading a
model or creating output. Run --help for smoke limits and weighting controls.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import shutil
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def build_parser():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--dataset",
        type=Path,
        default=ROOT / "rosbag-results/dataset/r50-surface-v6-20260929",
    )
    parser.add_argument(
        "--output",
        "--output-dir",
        dest="output",
        type=Path,
        default=ROOT / "rosbag-results/models/r50-surface-v6",
        help="Run directory; best export goes in best/, resumable state in checkpoints/last",
    )
    parser.add_argument(
        "--resume",
        type=Path,
        help="Local checkpoints/last directory; relative to output if absent in cwd",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=1000,
        help="Total successful optimizer updates; 0 evaluates baseline only",
    )
    parser.add_argument("--eval-every", type=int, default=100)
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--gradient-accumulation", type=int, default=4)
    parser.add_argument("--lr", type=float, default=1e-5)
    parser.add_argument("--backbone-lr", type=float, default=1e-6)
    parser.add_argument("--unfreeze-last-stage", action="store_true")
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--no-augmentation", action="store_true")
    parser.add_argument("--local-files-only", action="store_true")
    parser.add_argument(
        "--max-train-samples",
        type=int,
        help="Smoke limit, recorded in every manifest; no test data",
    )
    parser.add_argument(
        "--max-val-samples",
        type=int,
        help="Smoke limit, recorded in every manifest; no test data",
    )
    parser.add_argument("--legacy-reliability", type=float, default=0.5)
    parser.add_argument("--new-reliability", type=float, default=1.0)
    parser.add_argument("--curb-weight", type=float, default=2.0)
    parser.add_argument("--sidewalk-weight", type=float, default=2.0)
    parser.add_argument("--road-weight", type=float, default=1.0)
    parser.add_argument(
        "--negative-weight",
        type=float,
        default=1.0,
        help="Weight for other labeled classes",
    )
    parser.add_argument("--car-mount-weight", type=float, default=0.5)
    parser.add_argument("--dry-run", action="store_true")
    return parser


def check_arguments(args):
    for name in ("eval_every", "batch_size", "gradient_accumulation"):
        if getattr(args, name) <= 0:
            raise ValueError(f"--{name.replace('_', '-')} must be positive")
    if args.max_steps < 0 or args.workers < 0 or args.seed < 0:
        raise ValueError("max-steps, workers and seed must be nonnegative")
    for name in (
        "lr",
        "backbone_lr",
        "legacy_reliability",
        "new_reliability",
        "curb_weight",
        "sidewalk_weight",
        "road_weight",
        "negative_weight",
        "car_mount_weight",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"--{name.replace('_', '-')} must be finite and positive")
    args.dataset = args.dataset.expanduser().resolve()
    args.output = args.output.expanduser().resolve()
    if args.output == args.dataset or args.output.is_relative_to(args.dataset):
        raise ValueError("Output must not modify the immutable dataset directory")
    if args.resume:
        resume = args.resume.expanduser()
        if not resume.is_absolute() and not resume.exists():
            resume = args.output / resume
        args.resume = resume.resolve()
        if args.resume != args.output / "checkpoints/last":
            raise ValueError(
                "--resume must select this output's checkpoints/last directory"
            )
        if not (args.resume / "trainer-state.pt").is_file():
            raise ValueError("Resume checkpoint lacks trainer-state.pt")
    elif args.output.exists() and (
        not args.output.is_dir() or any(args.output.iterdir())
    ):
        raise ValueError(
            f"Refusing nonempty output directory: {args.output}; use --resume checkpoints/last"
        )


def inspect_dataset(args):
    import r50_partial_training as training

    train_rows = training.load_split(args.dataset, "train", args.max_train_samples)
    val_rows = training.load_split(args.dataset, "val", args.max_val_samples)
    # Check all eligible IDs before limits, so a smoke selection cannot conceal a
    # train/val split collision. Test manifests are never opened.
    all_train = training.load_split(args.dataset, "train")
    all_val = training.load_split(args.dataset, "val")
    if {row["sample_id"] for row in all_train} & {row["sample_id"] for row in all_val}:
        raise ValueError("Train and val contain overlapping sample IDs")
    if {row["image"] for row in all_train} & {row["image"] for row in all_val}:
        raise ValueError("Train and val contain overlapping image paths")
    report = {
        "dry_run": args.dry_run,
        "test_used": False,
        "dataset_manifest_sha256": training.file_sha256(
            args.dataset / "manifest.jsonl"
        ),
        "split_sha256": {
            split: training.file_sha256(args.dataset / "splits" / f"{split}.jsonl")
            for split in ("train", "val")
        },
        "train_samples": len(train_rows),
        "val_samples": len(val_rows),
        "available_train_samples": len(all_train),
        "available_val_samples": len(all_val),
        "limits": {
            "max_train_samples": args.max_train_samples,
            "max_val_samples": args.max_val_samples,
        },
        "target_shape": list(training.OUTPUT_SIZE),
        "geometry": "native labels -> nearest 360x640, RGB -> deployed processor",
        "groups": {
            status: sum(row["annotation_review_status"] == status for row in train_rows)
            for status in (training.NEW_STATUS, training.LEGACY_STATUS)
        },
    }
    shapes = set()
    for rows in (train_rows, val_rows):
        dataset = training.PartialDataset(args.dataset, rows)
        for index in range(len(dataset)):
            sample = dataset.read_native(index, verify_hashes=True)
            shapes.add(tuple(sample["image"].shape))
    report["native_rgb_shapes"] = sorted(map(list, shapes))
    return train_rows, val_rows, report


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def run(args):
    import numpy as np
    import r50_partial_training as training
    import torch
    import transformers
    from r50_checkpoint import BASE_MODEL_ID, BASE_MODEL_REVISION, validate_checkpoint

    check_arguments(args)
    train_rows, val_rows, data_report = inspect_dataset(args)
    if args.dry_run:
        print(json.dumps(data_report, indent=2, allow_nan=False))
        return data_report
    device = torch.device(
        "cuda"
        if args.device == "auto" and torch.cuda.is_available()
        else "cpu"
        if args.device == "auto"
        else args.device
    )
    if device.type == "cuda" and not torch.cuda.is_available():
        raise ValueError("CUDA requested but unavailable")
    random.seed(args.seed)
    np.random.seed(args.seed % 2**32)
    torch.manual_seed(args.seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(args.seed)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True

    weights = training.class_weights(
        curb=args.curb_weight,
        sidewalk=args.sidewalk_weight,
        road=args.road_weight,
        negative=args.negative_weight,
        car_mount=args.car_mount_weight,
    )
    loss_options = {
        "weights": weights.to(device),
        "legacy_reliability": args.legacy_reliability,
        "new_reliability": args.new_reliability,
    }
    contract = {
        "dataset_manifest_sha256": data_report["dataset_manifest_sha256"],
        "split_sha256": data_report["split_sha256"],
        "limits": data_report["limits"],
        "seed": args.seed,
        "batch_size": args.batch_size,
        "gradient_accumulation": args.gradient_accumulation,
        "lr": args.lr,
        "backbone_lr": args.backbone_lr,
        "unfreeze_last_stage": args.unfreeze_last_stage,
        "augmentation": not args.no_augmentation,
        "class_weights": weights.tolist(),
        "legacy_reliability": args.legacy_reliability,
        "new_reliability": args.new_reliability,
        "device_type": device.type,
        "torch_version": torch.__version__,
        "transformers_version": transformers.__version__,
        "base_model_id": BASE_MODEL_ID,
        "base_model_revision": BASE_MODEL_REVISION,
    }
    if args.resume:
        validate_checkpoint(args.resume)
        source = str(args.resume)
        load_options = {"local_files_only": True, "use_safetensors": True}
        processor_options = {"local_files_only": True}
    else:
        source = BASE_MODEL_ID
        load_options = {
            "revision": BASE_MODEL_REVISION,
            "local_files_only": args.local_files_only,
        }
        processor_options = {
            "revision": BASE_MODEL_REVISION,
            "local_files_only": args.local_files_only,
        }
    processor = training.configure_processor(
        transformers.AutoImageProcessor.from_pretrained(source, **processor_options)
    )
    model = training.strict_load_model(source, **load_options).to(device)
    groups = training.configure_trainable(
        model,
        lr=args.lr,
        backbone_lr=args.backbone_lr,
        unfreeze_last_stage=args.unfreeze_last_stage,
    )
    optimizer = torch.optim.AdamW(groups, weight_decay=0.01)
    scaler = torch.amp.GradScaler("cuda", enabled=device.type == "cuda")
    sampler = training.BalancedSampler(train_rows, args.seed)
    train_data = training.PartialDataset(
        args.dataset, train_rows, augment=not args.no_augmentation
    )
    val_data = training.PartialDataset(args.dataset, val_rows, augment=False)
    step, best_loss = 0, float("inf")
    if args.resume:
        state = training.restore_training_state(
            args.resume / "trainer-state.pt",
            optimizer,
            scaler,
            sampler,
            contract=contract,
        )
        step, best_loss = state["step"], state["best_loss"]
        if args.max_steps < step:
            raise ValueError(
                f"--max-steps {args.max_steps} precedes resumed step {step}"
            )
    args.output.mkdir(parents=True, exist_ok=True)

    def log(event, **values):
        record = {"event": event, "step": step, **values}
        line = json.dumps(record, allow_nan=False)
        with (args.output / "training.jsonl").open("a") as handle:
            handle.write(line + "\n")
        # Confusion matrices and all 65 recalls stay in JSONL/artifacts; terminal
        # progress includes only the interpretable summary.
        summary = {key: value for key, value in record.items() if key != "evaluation"}
        if "evaluation" in values:
            summary.update(
                masked_nll=values["evaluation"]["masked_nll"],
                sidewalk_present=values["evaluation"]["sidewalk_present"],
            )
        print(json.dumps(summary, allow_nan=False), flush=True)

    def evaluate():
        return training.evaluate(
            model,
            processor,
            val_data,
            device=device,
            batch_size=args.batch_size,
            workers=args.workers,
            loss_options=loss_options,
        )

    probe = training.prepare_inputs(processor, [train_data[0]["image"]], device)
    runtime_input_shape = list(probe["pixel_values"].shape[1:])
    del probe
    metadata = {
        **data_report,
        **contract,
        "dry_run": False,
        "runtime_input_shape": runtime_input_shape,
        "max_steps": args.max_steps,
        "eval_every": args.eval_every,
        "workers": args.workers,
        "loss": "per-image per-present-class weighted masked NLL; reliability scales before class averaging",
        "sampler": "replacement, equal new/legacy group probability when both exist",
        "trainable_parameters": sum(
            p.numel() for p in model.parameters() if p.requires_grad
        ),
        "backbone_batchnorm": "frozen running statistics, including optional last stage",
        "amp": "cuda float16 forward, float32 score projection and loss"
        if device.type == "cuda"
        else "float32",
        "gradient_clip_norm": 1.0,
        "resume_from": str(args.resume) if args.resume else None,
    }
    write_json(args.output / "run.json", metadata)
    log(
        "resume" if args.resume else "start",
        runtime_input_shape=runtime_input_shape,
        limits=data_report["limits"],
    )
    if not args.resume:
        baseline = evaluate()
        write_json(
            args.output / "baseline-evaluation.json",
            {"step": 0, "trained": False, **baseline},
        )
        log("baseline", evaluation=baseline, trained=False)
    # Preserve the original zero-step baseline on resume; no baseline model is
    # exported or eligible to win the best-trained-checkpoint selection.

    def save_checkpoints(evaluation):
        nonlocal best_loss
        loss = evaluation["masked_nll"]
        if loss is not None and loss < best_loss:
            training.save_model_artifacts(
                model,
                processor,
                args.output / "best",
                dataset_hash=data_report["dataset_manifest_sha256"],
                training={
                    **metadata,
                    "step": step,
                    "optimizer_steps": step,
                    "trained": True,
                    "checkpoint_kind": "best",
                },
                evaluation=evaluation,
            )
            best_loss = loss
            log("best", masked_nll=loss)
        checkpoints = args.output / "checkpoints"
        checkpoints.mkdir(exist_ok=True)
        temporary = Path(tempfile.mkdtemp(prefix=".last-", dir=checkpoints))
        try:
            training.save_model_artifacts(
                model,
                processor,
                temporary,
                dataset_hash=data_report["dataset_manifest_sha256"],
                training={
                    **metadata,
                    "step": step,
                    "optimizer_steps": step,
                    "trained": True,
                    "checkpoint_kind": "last",
                },
                evaluation=evaluation,
            )
            training.save_training_state(
                temporary / "trainer-state.pt",
                optimizer,
                scaler,
                sampler,
                step=step,
                best_loss=best_loss,
                contract=contract,
            )
            last, previous = checkpoints / "last", checkpoints / ".last-previous"
            if previous.exists():
                shutil.rmtree(previous)
            if last.exists():
                os.replace(last, previous)
            os.replace(temporary, last)
            if previous.exists():
                shutil.rmtree(previous)
        finally:
            if temporary.exists():
                shutil.rmtree(temporary)
        log("checkpoint", path=str(checkpoints / "last"))

    empty_attempts = 0
    overflow_attempts = 0
    while step < args.max_steps:
        training.set_training_mode(model)
        optimizer.zero_grad(set_to_none=True)
        keys = sampler.draw(args.batch_size * args.gradient_accumulation)
        loader = training.make_loader(
            train_data,
            batch_size=args.batch_size,
            workers=args.workers,
            keys=keys,
            seed=args.seed,
        )
        supervised_images, total_loss = 0, 0.0
        for batch in loader:
            labels, valid, origins = (
                batch[key].to(device) for key in ("labels", "valid_mask", "origins")
            )
            count = int(
                (valid & labels.ne(training.IGNORE_INDEX)).flatten(1).any(1).sum()
            )
            if count == 0:
                continue
            inputs = training.prepare_inputs(processor, batch["images"], device)
            with torch.autocast(
                device_type=device.type,
                dtype=torch.float16,
                enabled=device.type == "cuda",
            ):
                outputs = model(**inputs)
            scores = training.semantic_scores(
                outputs.class_queries_logits, outputs.masks_queries_logits
            )
            loss = training.masked_partial_nll(
                scores, labels, valid, origins, **loss_options
            )
            if not torch.isfinite(loss) or not torch.isfinite(scores).all():
                raise FloatingPointError(
                    "Nonfinite training scores/loss; last saved checkpoint is unchanged"
                )
            scaler.scale(loss * count).backward()
            supervised_images += count
            total_loss += float(loss.item()) * count
            del scores, outputs, loss, inputs
        if supervised_images == 0:
            empty_attempts += 1
            log("empty_batch_skipped", sampler_draws=sampler.draws)
            if empty_attempts >= 20:
                raise ValueError(
                    "20 successive sampled batches had no labeled valid pixels after resizing"
                )
            continue
        empty_attempts = 0
        updated, norm = training.finish_optimizer_step(
            optimizer, scaler, supervised_images=supervised_images
        )
        if not updated:
            overflow_attempts += 1
            log(
                "amp_overflow_skipped",
                scale=scaler.get_scale(),
                sampler_draws=sampler.draws,
            )
            if overflow_attempts >= 20:
                raise FloatingPointError(
                    "AMP gradients overflowed on 20 consecutive attempts"
                )
            continue
        overflow_attempts = 0
        step += 1
        log(
            "train",
            masked_nll=total_loss / supervised_images,
            gradient_norm=float(norm),
            supervised_images=supervised_images,
            sampler_draws=sampler.draws,
        )
        if step % args.eval_every == 0 or step == args.max_steps:
            evaluation = evaluate()
            log("validation", evaluation=evaluation, trained=True)
            save_checkpoints(evaluation)
    log(
        "complete",
        trained=step > 0,
        best_masked_nll=best_loss if math.isfinite(best_loss) else None,
    )
    return {"step": step, "output": str(args.output)}


def main():
    parser = build_parser()
    args = parser.parse_args()
    try:
        run(args)
    except (ValueError, OSError, FloatingPointError) as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    main()
