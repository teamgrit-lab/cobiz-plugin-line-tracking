"""Validate and warm up the pinned Swin-L hybrid deployment artifact."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path

import torch
from best_so_far_runtime import (
    DEFAULT_EVALUATION_SIZE,
    SWIN_L_ASPECT_FP16_PROFILE,
    resolve_profile,
)
from tensorrt_backend import TensorRTSemanticBackend


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    profile = resolve_profile(SWIN_L_ASPECT_FP16_PROFILE)
    backend = TensorRTSemanticBackend(
        args.engine.expanduser().resolve(),
        args.manifest.expanduser().resolve(),
        profile=profile.name,
        model_revision=profile.model_revision,
        input_shape=(1, 3, profile.input_height, profile.input_width),
        output_shape=(65, *DEFAULT_EVALUATION_SIZE),
        device=torch.device("cuda"),
    )
    metadata = backend.metadata()
    print(
        "SWIN_L_TENSORRT_VALID "
        f"artifact={metadata['engine']} sha256={metadata['engine_sha256']} "
        f"backend={metadata['backend']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
