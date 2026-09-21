"""Freeze the pinned Hub checkpoint into one reproducible safetensors artifact."""

from __future__ import annotations

import argparse
import json
import random
from collections.abc import Sequence
from pathlib import Path

import numpy as np
import torch
from best_so_far_runtime import SWIN_L_ASPECT_FP16_PROFILE, resolve_profile
from tensorrt_backend import sha256_file
from transformers import AutoImageProcessor, Mask2FormerForUniversalSegmentation


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    pinned = resolve_profile(SWIN_L_ASPECT_FP16_PROFILE)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--model-id", default=pinned.model_id)
    parser.add_argument("--revision", default=pinned.model_revision)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--allow-initialized-weights",
        action="store_true",
        help="permit missing checkpoint keys after recording them in the manifest",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(args.seed)

    processor = AutoImageProcessor.from_pretrained(
        args.model_id,
        revision=args.revision,
    )
    model, loading_info = Mask2FormerForUniversalSegmentation.from_pretrained(
        args.model_id,
        revision=args.revision,
        use_safetensors=True,
        output_loading_info=True,
    )
    missing = sorted(loading_info.get("missing_keys", ()))
    unexpected = sorted(loading_info.get("unexpected_keys", ()))
    mismatched = sorted(str(item) for item in loading_info.get("mismatched_keys", ()))
    if (missing or mismatched) and not args.allow_initialized_weights:
        raise RuntimeError(
            "checkpoint requires initialized or mismatched weights; inspect the loading "
            "report and rerun with --allow-initialized-weights only after approval"
        )

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    model.save_pretrained(output_dir, safe_serialization=True)
    processor.save_pretrained(output_dir)
    safetensors_path = output_dir / "model.safetensors"
    if not safetensors_path.is_file():
        raise RuntimeError("expected a single model.safetensors output")
    manifest = {
        "schema_version": 1,
        "source_model_id": args.model_id,
        "source_revision": args.revision,
        "seed": args.seed,
        "safetensors_file": safetensors_path.name,
        "safetensors_sha256": sha256_file(safetensors_path),
        "loading_info": {
            "missing_keys": missing,
            "unexpected_keys": unexpected,
            "mismatched_keys": mismatched,
        },
    }
    manifest_path = output_dir / "checkpoint-manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(
        f"SWIN_L_CHECKPOINT_READY path={output_dir} sha256={manifest['safetensors_sha256']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
