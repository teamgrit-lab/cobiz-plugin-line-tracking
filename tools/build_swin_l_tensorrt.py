"""Build the fixed-shape Swin-L PyTorch/TensorRT hybrid on its target Jetson."""

from __future__ import annotations

import argparse
import fcntl
import gc
import json
import os
from collections.abc import Sequence
from pathlib import Path
from tempfile import NamedTemporaryFile, TemporaryDirectory
from typing import Any
from zipfile import ZIP_STORED, ZipFile

import torch
from best_so_far_runtime import (
    DEFAULT_EVALUATION_SIZE,
    SWIN_L_ASPECT_FP16_PROFILE,
    resolve_profile,
)
from swin_l_tensorrt_model import (
    SwinLSemanticScores,
    restore_swin_stage_outputs,
    tensor_only_swin_stage_outputs,
)
from tensorrt_backend import (
    ENGINE_MANIFEST_SCHEMA_VERSION,
    HYBRID_ARTIFACT_FORMAT,
    sha256_file,
)
from transformers import Mask2FormerForUniversalSegmentation

# These rank-changing operators caused the TensorRT Myelin/rank failure in the
# monolithic Mask2Former engine. Keeping them in CUDA PyTorch creates explicit
# boundaries around large Swin attention/MLP TensorRT partitions.
PYTORCH_SHAPE_OPS = (
    torch.ops.aten._reshape_copy.default,
    torch.ops.aten.expand.default,
    torch.ops.aten.repeat.default,
    torch.ops.aten.unsqueeze.default,
)


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
    parser.add_argument(
        "--manifest-output",
        type=Path,
        help="manifest destination (defaults to <output>.json)",
    )
    # TensorRT 10.3 on Jetson can fail Myelin tactic selection when graphs are
    # aggressively fused. Keep the least aggressive builder level by default.
    parser.add_argument("--optimization-level", type=int, choices=range(6), default=0)
    parser.add_argument("--workspace-mib", type=int, default=2048)
    parser.add_argument(
        "--min-block-size",
        type=int,
        default=3,
        help="minimum supported ops in a TensorRT partition",
    )
    args = parser.parse_args(argv)
    if args.min_block_size < 2:
        parser.error("--min-block-size must be at least 2")
    return args


def _write_artifacts_atomically(
    engine_bytes: bytes,
    manifest: dict[str, Any],
    output: Path,
    manifest_path: Path,
) -> None:
    """Replace an artifact and manifest without exposing partially written files."""

    output.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    artifact_temporary: Path | None = None
    manifest_temporary: Path | None = None
    try:
        with NamedTemporaryFile(
            mode="wb",
            prefix=f".{output.name}.",
            suffix=".tmp",
            dir=output.parent,
            delete=False,
        ) as stream:
            artifact_temporary = Path(stream.name)
            stream.write(engine_bytes)
            stream.flush()
            os.fsync(stream.fileno())
        manifest_temporary = _write_manifest_temporary(manifest, manifest_path)
        artifact_temporary.replace(output)
        artifact_temporary = None
        manifest_temporary.replace(manifest_path)
        manifest_temporary = None
    finally:
        if artifact_temporary is not None:
            artifact_temporary.unlink(missing_ok=True)
        if manifest_temporary is not None:
            manifest_temporary.unlink(missing_ok=True)


def _write_manifest_temporary(
    manifest: dict[str, Any], manifest_path: Path
) -> Path:
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    with NamedTemporaryFile(
        mode="w",
        prefix=f".{manifest_path.name}.",
        suffix=".tmp",
        dir=manifest_path.parent,
        encoding="utf-8",
        delete=False,
    ) as stream:
        json.dump(manifest, stream, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
        return Path(stream.name)


def _publish_saved_artifact(
    artifact_temporary: Path,
    manifest: dict[str, Any],
    output: Path,
    manifest_path: Path,
) -> None:
    """Atomically publish a large artifact already serialized on disk."""

    output.parent.mkdir(parents=True, exist_ok=True)
    with artifact_temporary.open("rb") as stream:
        os.fsync(stream.fileno())
    manifest_temporary = _write_manifest_temporary(manifest, manifest_path)
    try:
        artifact_temporary.replace(output)
        manifest_temporary.replace(manifest_path)
    finally:
        manifest_temporary.unlink(missing_ok=True)


def _node_tensor_dtype(node: torch.fx.Node) -> torch.dtype | None:
    value = node.meta.get("val")
    return getattr(value, "dtype", None)


def _rewrite_tensorrt_incompatible_ops(
    exported: torch.export.ExportedProgram,
) -> dict[str, int]:
    """Rewrite inference-equivalent ops unsupported by Torch-TensorRT 2.8."""

    graph_module = exported.graph_module
    graph = graph_module.graph
    rewritten = {"boolean_mul": 0, "baddbmm": 0}

    for node in list(graph.nodes):
        if (
            node.op == "call_function"
            and node.target == torch.ops.aten.mul.Tensor
            and _node_tensor_dtype(node) == torch.bool
        ):
            with graph.inserting_before(node):
                replacement = graph.call_function(
                    torch.ops.aten.bitwise_and.Tensor,
                    args=node.args,
                    kwargs=node.kwargs,
                )
            replacement.meta = dict(node.meta)
            node.replace_all_uses_with(replacement)
            graph.erase_node(node)
            rewritten["boolean_mul"] += 1
            continue

        if node.op != "call_function" or node.target != torch.ops.aten.baddbmm.default:
            continue
        beta = node.kwargs.get("beta", node.args[3] if len(node.args) > 3 else 1)
        alpha = node.kwargs.get("alpha", node.args[4] if len(node.args) > 4 else 1)
        if beta != 1 or alpha != 1:
            raise RuntimeError(
                "cannot rewrite aten.baddbmm with non-default alpha or beta"
            )
        if len(node.args) < 3:
            raise RuntimeError("invalid aten.baddbmm node in exported graph")
        bias, batch1, batch2 = node.args[:3]
        with graph.inserting_before(node):
            product = graph.call_function(
                torch.ops.aten.bmm.default,
                args=(batch1, batch2),
            )
            replacement = graph.call_function(
                torch.ops.aten.add.Tensor,
                args=(bias, product),
            )
        product.meta = dict(node.meta)
        replacement.meta = dict(node.meta)
        node.replace_all_uses_with(replacement)
        graph.erase_node(node)
        rewritten["baddbmm"] += 1

    graph.lint()
    graph_module.recompile()
    return rewritten


class _FixedStageCall(torch.nn.Module):
    """Expose only tensor leaves while retaining a captured Swin stage call."""

    def __init__(
        self,
        stage: torch.nn.Module,
        args: tuple[Any, ...],
        kwargs: dict[str, Any],
    ) -> None:
        super().__init__()
        from torch.utils._pytree import tree_flatten

        self.stage = stage
        flat, self._spec = tree_flatten((args, kwargs))
        self._tensor_positions = tuple(
            index for index, value in enumerate(flat) if isinstance(value, torch.Tensor)
        )
        self._template = tuple(
            None if index in self._tensor_positions else value
            for index, value in enumerate(flat)
        )

    def forward(self, *tensor_inputs: torch.Tensor) -> Any:
        from torch.utils._pytree import tree_unflatten

        if len(tensor_inputs) != len(self._tensor_positions):
            raise ValueError("captured Swin stage tensor input count changed")
        flat = list(self._template)
        for position, tensor in zip(self._tensor_positions, tensor_inputs):
            flat[position] = tensor
        args, kwargs = tree_unflatten(flat, self._spec)
        return tensor_only_swin_stage_outputs(self.stage(*args, **kwargs))


class _CompiledStageProxy(torch.nn.Module):
    """Preserve the Transformers stage call while dispatching to compiled TRT."""

    def __init__(
        self,
        compiled: torch.nn.Module,
        tensor_input_count: int,
        *,
        example_inputs: tuple[torch.Tensor, ...],
        has_downsample: bool,
        partition_count: int,
    ) -> None:
        super().__init__()
        if len(example_inputs) != tensor_input_count:
            raise ValueError("compiled Swin stage example input count changed")
        self.compiled = compiled
        self.tensor_input_count = tensor_input_count
        self.partition_count = partition_count
        self._serialization_input_specs = tuple(
            (
                tuple(value.shape),
                tuple(value.stride()),
                value.dtype,
                value.device,
            )
            for value in example_inputs
        )
        # SwinEncoder uses this attribute after each stage to update the next
        # stage's static spatial dimensions. It only checks for None and does
        # not call the object, so retaining the original module would waste
        # memory and duplicate its parameters in the final artifact.
        self.downsample = True if has_downsample else None

    def serialization_inputs(self) -> tuple[torch.Tensor, ...]:
        return tuple(
            torch.empty_strided(
                shape,
                stride,
                dtype=dtype,
                device=device,
            ).zero_()
            for shape, stride, dtype, device in self._serialization_input_specs
        )

    def forward(self, *args: Any, **kwargs: Any) -> Any:
        from torch.utils._pytree import tree_flatten

        flat, _ = tree_flatten((args, kwargs))
        tensor_inputs = tuple(value for value in flat if isinstance(value, torch.Tensor))
        if len(tensor_inputs) != self.tensor_input_count:
            raise ValueError("Swin stage call no longer matches its compiled profile")
        return restore_swin_stage_outputs(self.compiled(*tensor_inputs))


def _swin_stages(model: torch.nn.Module) -> torch.nn.ModuleList:
    try:
        stages = model.model.pixel_level_module.encoder.swin.encoder.layers
    except AttributeError as error:
        raise RuntimeError(
            "unsupported Mask2Former layout: Swin encoder stages were not found"
        ) from error
    if not isinstance(stages, torch.nn.ModuleList) or not stages:
        raise RuntimeError("Swin encoder stages must be a non-empty ModuleList")
    return stages


def _capture_stage_calls(
    stages: torch.nn.ModuleList,
    wrapper: torch.nn.Module,
    sample: torch.Tensor,
) -> tuple[list[tuple[tuple[Any, ...], dict[str, Any]]], torch.Tensor]:
    calls: list[tuple[tuple[Any, ...], dict[str, Any]] | None] = [None] * len(stages)
    handles = []

    def capture(
        index: int,
        _module: torch.nn.Module,
        args: tuple[Any, ...],
        kwargs: dict[str, Any],
    ) -> None:
        if calls[index] is not None:
            raise RuntimeError(f"Swin stage {index} executed more than once")
        calls[index] = (args, dict(kwargs))

    for index, stage in enumerate(stages):
        handles.append(
            stage.register_forward_pre_hook(
                lambda module, args, kwargs, index=index: capture(
                    index, module, args, kwargs
                ),
                with_kwargs=True,
            )
        )
    try:
        with torch.no_grad():
            reference = wrapper(sample)
    finally:
        for handle in handles:
            handle.remove()
    if any(call is None for call in calls):
        raise RuntimeError("not every Swin stage executed during profile capture")
    return [call for call in calls if call is not None], reference


def _is_tensorrt_partition_node(node: torch.fx.Node) -> bool:
    """Recognize TRT partitions both before and after export serialization."""

    return (
        node.op == "call_module" and "_run_on_acc" in str(node.target)
    ) or (
        node.op == "call_function"
        and "tensorrt.execute_engine" in str(node.target)
    )


def _count_tensorrt_partitions(module: torch.nn.Module) -> int:
    count = 0
    for child in module.modules():
        if not isinstance(child, torch.fx.GraphModule):
            continue
        count += sum(_is_tensorrt_partition_node(node) for node in child.graph.nodes)
    return count


def _compile_swin_stages(
    stages: torch.nn.ModuleList,
    calls: list[tuple[tuple[Any, ...], dict[str, Any]]],
    *,
    torch_tensorrt: Any,
    optimization_level: int,
    workspace_size: int,
    min_block_size: int,
) -> tuple[int, dict[str, int]]:
    partition_count = 0
    rewrite_totals = {"boolean_mul": 0, "baddbmm": 0}
    for index, (stage, (args, kwargs)) in enumerate(zip(list(stages), calls)):
        fixed_call = _FixedStageCall(stage, args, kwargs).eval()
        tensor_inputs = tuple(
            value
            for value in torch.utils._pytree.tree_leaves((args, kwargs))
            if isinstance(value, torch.Tensor)
        )
        if not tensor_inputs:
            raise RuntimeError(f"Swin stage {index} has no tensor inputs")
        exported = torch.export.export(fixed_call, tensor_inputs, strict=False)
        rewrites = _rewrite_tensorrt_incompatible_ops(exported)
        for name, value in rewrites.items():
            rewrite_totals[name] += value
        compiled = torch_tensorrt.dynamo.compile(
            exported,
            arg_inputs=list(tensor_inputs),
            enabled_precisions={torch.float16},
            workspace_size=workspace_size,
            optimization_level=optimization_level,
            immutable_weights=True,
            require_full_compilation=False,
            min_block_size=min_block_size,
            torch_executed_ops=set(PYTORCH_SHAPE_OPS),
            use_fast_partitioner=False,
            use_python_runtime=False,
        )
        stage_partitions = _count_tensorrt_partitions(compiled)
        if stage_partitions < 1:
            raise RuntimeError(
                f"Swin stage {index} produced no sufficiently large TensorRT partition"
            )
        stages[index] = _CompiledStageProxy(
            compiled,
            len(tensor_inputs),
            example_inputs=tensor_inputs,
            has_downsample=stage.downsample is not None,
            partition_count=stage_partitions,
        )
        partition_count += stage_partitions
        del exported, fixed_call, stage
        gc.collect()
        torch.cuda.empty_cache()
        print(
            f"[swin-l-debug] compiled Swin stage {index}: "
            f"TensorRT partitions={stage_partitions}",
            flush=True,
        )
    return partition_count, rewrite_totals


def _make_artifact_temporary(output: Path) -> Path:
    output.parent.mkdir(parents=True, exist_ok=True)
    with NamedTemporaryFile(
        mode="wb",
        prefix=f".{output.name}.",
        suffix=".tmp",
        dir=output.parent,
        delete=False,
    ) as stream:
        return Path(stream.name)


def _save_stage_bundle(
    stages: torch.nn.ModuleList,
    artifact_path: Path,
    *,
    torch_tensorrt: Any,
) -> list[dict[str, Any]]:
    """Serialize compiled stages independently and package them as one artifact."""

    stage_metadata: list[dict[str, Any]] = []
    with TemporaryDirectory(
        prefix=f".{artifact_path.name}.stages.",
        dir=artifact_path.parent,
    ) as temporary_directory:
        temporary_path = Path(temporary_directory)
        for index, stage in enumerate(stages):
            if not isinstance(stage, _CompiledStageProxy):
                raise TypeError(f"Swin stage {index} is not a compiled stage proxy")
            stage_name = f"stage_{index}.ts"
            stage_path = temporary_path / stage_name
            # Torch-TensorRT 2.8's ExportedProgram serializer cannot encode a
            # multi-output execute_engine node when one output is consumed
            # directly instead of through operator.getitem. Its TorchScript
            # save path traces the already-compiled graph and preserves the
            # embedded engines without going through that exporter.
            serialization_inputs = stage.serialization_inputs()
            try:
                torch_tensorrt.save(
                    stage.compiled,
                    str(stage_path),
                    output_format="torchscript",
                    arg_inputs=serialization_inputs,
                )
            finally:
                del serialization_inputs
                gc.collect()
                if torch.cuda.is_available():
                    torch.cuda.empty_cache()
            if not stage_path.is_file() or stage_path.stat().st_size <= 0:
                raise RuntimeError(f"serialized Swin stage {index} is empty")
            stage_metadata.append(
                {
                    "index": index,
                    "file": stage_name,
                    "sha256": sha256_file(stage_path),
                    "serialization_format": "torchscript",
                    "tensor_input_count": stage.tensor_input_count,
                    "has_downsample": stage.downsample is not None,
                    "partition_count": stage.partition_count,
                }
            )

        with ZipFile(
            artifact_path,
            mode="w",
            compression=ZIP_STORED,
            allowZip64=True,
        ) as archive:
            for stage in stage_metadata:
                archive.write(temporary_path / stage["file"], arcname=stage["file"])
    return stage_metadata


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not torch.cuda.is_available():
        raise RuntimeError("TensorRT artifact build requires the target CUDA device")
    try:
        import tensorrt as trt
        import torch_tensorrt
    except ImportError as error:
        raise RuntimeError(
            "artifact build requires JetPack-compatible TensorRT and Torch-TensorRT"
        ) from error

    output = args.output.expanduser().resolve()
    manifest_path = (
        args.manifest_output.expanduser().resolve()
        if args.manifest_output is not None
        else output.with_suffix(output.suffix + ".json")
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    lock_path = output.with_suffix(output.suffix + ".lock")
    artifact_temporary: Path | None = None
    with lock_path.open("w", encoding="utf-8") as lock_stream:
        fcntl.flock(lock_stream, fcntl.LOCK_EX)

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
        expected_output_shape = (
            int(model.config.num_labels),
            DEFAULT_EVALUATION_SIZE[0],
            DEFAULT_EVALUATION_SIZE[1],
        )
        stages = _swin_stages(model)
        calls, reference = _capture_stage_calls(stages, wrapper, sample)
        if tuple(reference.shape) != expected_output_shape:
            raise RuntimeError(
                f"unexpected semantic output shape: {tuple(reference.shape)}"
            )
        del reference
        torch.cuda.empty_cache()

        partition_count, rewrites = _compile_swin_stages(
            stages,
            calls,
            torch_tensorrt=torch_tensorrt,
            optimization_level=args.optimization_level,
            workspace_size=args.workspace_mib * 1024 * 1024,
            min_block_size=args.min_block_size,
        )
        del calls
        with torch.inference_mode():
            hybrid_output = wrapper(sample)
            torch.cuda.synchronize(sample.device)
        if tuple(hybrid_output.shape) != expected_output_shape:
            raise RuntimeError("hybrid graph output shape is invalid")
        if hybrid_output.dtype != torch.float16:
            raise RuntimeError("hybrid graph output must remain FP16")
        if not bool(torch.isfinite(hybrid_output).all().item()):
            raise RuntimeError("hybrid graph validation produced NaN or Inf")
        del hybrid_output

        artifact_temporary = _make_artifact_temporary(output)
        try:
            stage_metadata = _save_stage_bundle(
                stages,
                artifact_temporary,
                torch_tensorrt=torch_tensorrt,
            )

            manifest = {
                "schema_version": ENGINE_MANIFEST_SCHEMA_VERSION,
                "artifact_format": HYBRID_ARTIFACT_FORMAT,
                "profile": profile.name,
                "model": {
                    "id": profile.model_id,
                    "revision": profile.model_revision,
                    "safetensors_sha256": checkpoint_manifest[
                        "safetensors_sha256"
                    ],
                    "id2label": {
                        str(key): value for key, value in model.config.id2label.items()
                    },
                },
                "checkpoint": {
                    "path": str(checkpoint),
                    "manifest_file": "checkpoint-manifest.json",
                    "safetensors_file": checkpoint_manifest["safetensors_file"],
                    "safetensors_sha256": checkpoint_manifest[
                        "safetensors_sha256"
                    ],
                },
                "input": {
                    "name": "pixel_values",
                    "shape": list(sample.shape),
                    "dtype": "float16",
                },
                "output": {
                    "name": "semantic_scores",
                    "shape": list(expected_output_shape),
                    "dtype": "float16",
                },
                "runtime": {
                    "tensorrt_version": trt.__version__,
                    "torch_version": torch.__version__,
                    "torch_tensorrt_version": getattr(
                        torch_tensorrt, "__version__", None
                    ),
                    "cuda_version": torch.version.cuda,
                    "compute_capability": list(torch.cuda.get_device_capability()),
                },
                "build": {
                    "optimization_level": args.optimization_level,
                    "workspace_mib": args.workspace_mib,
                    "min_block_size": args.min_block_size,
                },
                "partitioning": {
                    "pytorch_patch_embedding": True,
                    "pytorch_shape_ops": [str(op) for op in PYTORCH_SHAPE_OPS],
                    "tensorrt_swin_stage_count": len(stages),
                    "tensorrt_partition_count": partition_count,
                    "stages": stage_metadata,
                    "pytorch_mask2former_decoders": True,
                    "compatibility_rewrites": rewrites,
                },
                # Retain this key for existing monitoring/metadata consumers.
                "engine_sha256": sha256_file(artifact_temporary),
            }
            _publish_saved_artifact(
                artifact_temporary,
                manifest,
                output,
                manifest_path,
            )
            artifact_temporary = None
        finally:
            if artifact_temporary is not None:
                artifact_temporary.unlink(missing_ok=True)

    print(
        f"SWIN_L_TENSORRT_READY artifact={output} manifest={manifest_path} "
        f"partitions={partition_count}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
