"""Build the fixed-shape FP16 Swin-L TensorRT plan on its target Jetson."""

from __future__ import annotations

import argparse
import json
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import torch
from best_so_far_runtime import (
    DEFAULT_EVALUATION_SIZE,
    SWIN_L_ASPECT_FP16_PROFILE,
    resolve_profile,
)
from swin_l_tensorrt_model import SwinLSemanticScores
from tensorrt_backend import ENGINE_MANIFEST_SCHEMA_VERSION, sha256_file
from transformers import Mask2FormerForUniversalSegmentation


def _read_checkpoint_manifest(checkpoint: Path) -> dict[str, Any]:
    path = checkpoint / "checkpoint-manifest.json"
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"invalid prepared checkpoint manifest: {path}") from error
    safetensors_path = checkpoint / manifest.get("safetensors_file", "")
    if not safetensors_path.is_file():
        raise RuntimeError("prepared checkpoint safetensors file is missing")
    if sha256_file(safetensors_path) != manifest.get("safetensors_sha256"):
        raise RuntimeError("prepared checkpoint safetensors SHA-256 mismatch")
    return manifest


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--optimization-level", type=int, choices=range(6), default=3)
    parser.add_argument("--workspace-mib", type=int, default=1024)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not torch.cuda.is_available():
        raise RuntimeError("TensorRT engine build requires the target CUDA device")
    try:
        import tensorrt as trt
        import torch_tensorrt
    except ImportError as error:
        raise RuntimeError(
            "engine build requires JetPack-compatible tensorrt and torch_tensorrt"
        ) from error

    profile = resolve_profile(SWIN_L_ASPECT_FP16_PROFILE)
    checkpoint = args.checkpoint.expanduser().resolve()
    checkpoint_manifest = _read_checkpoint_manifest(checkpoint)
    if checkpoint_manifest.get("source_revision") != profile.model_revision:
        raise RuntimeError(
            "prepared checkpoint revision does not match the pinned profile"
        )

    model = (
        Mask2FormerForUniversalSegmentation.from_pretrained(
            checkpoint,
            use_safetensors=True,
            local_files_only=True,
            dtype=torch.float16,
        )
        .eval()
        .cuda()
    )
    wrapper = SwinLSemanticScores(
        model,
        evaluation_height=DEFAULT_EVALUATION_SIZE[0],
        evaluation_width=DEFAULT_EVALUATION_SIZE[1],
    ).eval()
    sample = torch.zeros(
        (1, 3, profile.input_height, profile.input_width),
        dtype=torch.float16,
        device="cuda",
    )
    with torch.inference_mode():
        reference = wrapper(sample)
    expected_output_shape = (
        int(model.config.num_labels),
        DEFAULT_EVALUATION_SIZE[0],
        DEFAULT_EVALUATION_SIZE[1],
    )
    if tuple(reference.shape) != expected_output_shape:
        raise RuntimeError(
            f"unexpected semantic output shape: {tuple(reference.shape)}"
        )

    exported = torch.export.export(wrapper, (sample,))
    engine_bytes = (
        torch_tensorrt.dynamo.convert_exported_program_to_serialized_trt_engine(
            exported,
            arg_inputs=[sample],
            enabled_precisions={torch.float16},
            workspace_size=args.workspace_mib * 1024 * 1024,
            optimization_level=args.optimization_level,
            immutable_weights=True,
        )
    )
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(engine_bytes)

    runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
    engine = runtime.deserialize_cuda_engine(engine_bytes)
    if engine is None or engine.num_io_tensors != 2:
        raise RuntimeError("built engine must expose exactly one input and one output")
    input_names: list[str] = []
    output_names: list[str] = []
    for index in range(engine.num_io_tensors):
        name = engine.get_tensor_name(index)
        if engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
            input_names.append(name)
        else:
            output_names.append(name)
    if len(input_names) != 1 or len(output_names) != 1:
        raise RuntimeError("built engine has an unsupported binding layout")
    input_name, output_name = input_names[0], output_names[0]
    input_shape = tuple(engine.get_tensor_shape(input_name))
    output_shape = tuple(engine.get_tensor_shape(output_name))
    if input_shape != tuple(sample.shape) or output_shape != expected_output_shape:
        raise RuntimeError(
            "built engine bindings do not match the fixed runtime contract"
        )
    if engine.get_tensor_dtype(input_name) != trt.float16:
        raise RuntimeError("built engine input is not FP16")
    if engine.get_tensor_dtype(output_name) != trt.float16:
        raise RuntimeError("built engine output is not FP16")

    manifest = {
        "schema_version": ENGINE_MANIFEST_SCHEMA_VERSION,
        "profile": profile.name,
        "model": {
            "id": profile.model_id,
            "revision": profile.model_revision,
            "safetensors_sha256": checkpoint_manifest["safetensors_sha256"],
            "id2label": {
                str(key): value for key, value in model.config.id2label.items()
            },
        },
        "input": {"name": input_name, "shape": list(input_shape), "dtype": "float16"},
        "output": {
            "name": output_name,
            "shape": list(output_shape),
            "dtype": "float16",
        },
        "runtime": {
            "tensorrt_version": trt.__version__,
            "torch_version": torch.__version__,
            "torch_tensorrt_version": getattr(torch_tensorrt, "__version__", None),
            "cuda_version": torch.version.cuda,
            "compute_capability": list(torch.cuda.get_device_capability()),
        },
        "build": {
            "optimization_level": args.optimization_level,
            "workspace_mib": args.workspace_mib,
        },
        "engine_sha256": sha256_file(output),
    }
    manifest_path = output.with_suffix(output.suffix + ".json")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"SWIN_L_TENSORRT_READY engine={output} manifest={manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
