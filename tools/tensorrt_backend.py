"""Trusted fixed-shape Torch-TensorRT hybrid runtime for Swin-L scores."""

from __future__ import annotations

import gc
import hashlib
import json
import shutil
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any
from zipfile import BadZipFile, ZipFile

import torch
from swin_l_tensorrt_model import SwinLSemanticScores, restore_swin_stage_outputs

ENGINE_MANIFEST_SCHEMA_VERSION = 4
HYBRID_ARTIFACT_FORMAT = "torch_tensorrt_stage_bundle"


def normalize_cuda_device(device: torch.device) -> torch.device:
    """Resolve an index-less CUDA device to the active concrete device."""

    if device.type == "cuda" and device.index is None:
        return torch.device("cuda", torch.cuda.current_device())
    return device


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_engine_manifest(path: Path) -> dict[str, Any]:
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"could not read TensorRT manifest: {path}") from error
    if not isinstance(manifest, dict):
        raise TypeError("TensorRT manifest must be a JSON object")
    return manifest


def validate_engine_manifest(
    manifest: dict[str, Any],
    *,
    profile: str,
    model_revision: str,
    input_shape: tuple[int, ...],
    output_shape: tuple[int, ...],
) -> None:
    if manifest.get("schema_version") != ENGINE_MANIFEST_SCHEMA_VERSION:
        raise RuntimeError("unsupported TensorRT manifest schema")
    if manifest.get("artifact_format") != HYBRID_ARTIFACT_FORMAT:
        raise RuntimeError("TensorRT artifact is not the required hybrid program")
    if manifest.get("profile") != profile:
        raise RuntimeError("TensorRT profile does not match the selected profile")
    model = manifest.get("model")
    if not isinstance(model, dict) or model.get("revision") != model_revision:
        raise RuntimeError("TensorRT model revision does not match the pinned revision")
    id2label = model.get("id2label")
    if not isinstance(id2label, dict) or len(id2label) != output_shape[0]:
        raise RuntimeError("TensorRT label metadata does not match the output classes")
    checkpoint = manifest.get("checkpoint")
    if not isinstance(checkpoint, dict):
        raise TypeError("TensorRT checkpoint metadata is missing")
    for key in ("path", "manifest_file", "safetensors_file"):
        if not isinstance(checkpoint.get(key), str) or not checkpoint[key]:
            raise RuntimeError(f"TensorRT checkpoint {key} is invalid")
    checkpoint_digest = checkpoint.get("safetensors_sha256")
    if not _is_sha256(checkpoint_digest):
        raise RuntimeError("TensorRT checkpoint SHA-256 is invalid")
    for key, expected_shape in (("input", input_shape), ("output", output_shape)):
        binding = manifest.get(key)
        if not isinstance(binding, dict):
            raise TypeError(f"TensorRT manifest is missing {key} metadata")
        if not isinstance(binding.get("name"), str) or not binding["name"]:
            raise RuntimeError(f"TensorRT {key} name is invalid")
        if tuple(binding.get("shape", ())) != expected_shape:
            raise RuntimeError(f"TensorRT {key} shape does not match runtime contract")
        if binding.get("dtype") != "float16":
            raise RuntimeError(f"TensorRT {key} must use float16")
    digest = manifest.get("engine_sha256")
    if not _is_sha256(digest):
        raise RuntimeError("TensorRT artifact SHA-256 is missing or invalid")
    partitioning = manifest.get("partitioning")
    if not isinstance(partitioning, dict):
        raise TypeError("TensorRT hybrid partition metadata is missing")
    if partitioning.get("pytorch_patch_embedding") is not True:
        raise RuntimeError("Swin patch embedding must remain in PyTorch")
    if partitioning.get("pytorch_mask2former_decoders") is not True:
        raise RuntimeError("Mask2Former decoders must remain in PyTorch")
    partition_count = partitioning.get("tensorrt_partition_count")
    if type(partition_count) is not int or partition_count < 1:
        raise RuntimeError("TensorRT hybrid artifact has no TensorRT partitions")
    stages = partitioning.get("stages")
    stage_count = partitioning.get("tensorrt_swin_stage_count")
    if type(stage_count) is not int or stage_count < 1:
        raise RuntimeError("TensorRT Swin stage count is invalid")
    if not isinstance(stages, list) or len(stages) != stage_count:
        raise RuntimeError("TensorRT Swin stage metadata count is invalid")
    stage_files: set[str] = set()
    recorded_partitions = 0
    for expected_index, stage in enumerate(stages):
        if not isinstance(stage, dict) or stage.get("index") != expected_index:
            raise RuntimeError("TensorRT Swin stage index is invalid")
        stage_file = stage.get("file")
        if (
            not isinstance(stage_file, str)
            or not stage_file
            or Path(stage_file).name != stage_file
            or stage_file in stage_files
        ):
            raise RuntimeError("TensorRT Swin stage file is invalid")
        stage_files.add(stage_file)
        if stage.get("serialization_format") != "torchscript":
            raise RuntimeError("TensorRT Swin stage serialization format is invalid")
        if not _is_sha256(stage.get("sha256")):
            raise RuntimeError("TensorRT Swin stage SHA-256 is invalid")
        tensor_input_count = stage.get("tensor_input_count")
        if type(tensor_input_count) is not int or tensor_input_count < 1:
            raise RuntimeError("TensorRT Swin stage input count is invalid")
        if type(stage.get("has_downsample")) is not bool:
            raise RuntimeError("TensorRT Swin stage downsample metadata is invalid")
        stage_partitions = stage.get("partition_count")
        if type(stage_partitions) is not int or stage_partitions < 1:
            raise RuntimeError("TensorRT Swin stage partition count is invalid")
        recorded_partitions += stage_partitions
    if recorded_partitions != partition_count:
        raise RuntimeError("TensorRT partition metadata totals do not match")


def _is_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value.lower())
    )


def _single_tensor_output(output: Any) -> torch.Tensor:
    if isinstance(output, torch.Tensor):
        return output
    if isinstance(output, (tuple, list)) and len(output) == 1:
        value = output[0]
        if isinstance(value, torch.Tensor):
            return value
    raise RuntimeError("hybrid TensorRT program did not return one tensor")


class _LoadedStageProxy(torch.nn.Module):
    """Restore the original Transformers stage signature around a TRT module."""

    def __init__(
        self,
        compiled: torch.nn.Module,
        tensor_input_count: int,
        *,
        has_downsample: bool,
    ) -> None:
        super().__init__()
        self.compiled = compiled
        self.tensor_input_count = tensor_input_count
        self.downsample = True if has_downsample else None

    def forward(self, *args: Any, **kwargs: Any) -> Any:
        from torch.utils._pytree import tree_flatten

        flat, _ = tree_flatten((args, kwargs))
        tensor_inputs = tuple(value for value in flat if isinstance(value, torch.Tensor))
        if len(tensor_inputs) != self.tensor_input_count:
            raise ValueError("Swin stage call no longer matches its TensorRT profile")
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


def _validate_checkpoint_files(
    checkpoint: Path,
    metadata: dict[str, Any],
    *,
    model_revision: str,
) -> None:
    manifest_path = checkpoint / metadata["manifest_file"]
    try:
        checkpoint_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(
            f"could not read prepared checkpoint manifest: {manifest_path}"
        ) from error
    if not isinstance(checkpoint_manifest, dict):
        raise TypeError("prepared checkpoint manifest must be a JSON object")
    if checkpoint_manifest.get("source_revision") != model_revision:
        raise RuntimeError("prepared checkpoint revision does not match the artifact")
    if checkpoint_manifest.get("safetensors_file") != metadata["safetensors_file"]:
        raise RuntimeError("prepared checkpoint filename does not match the artifact")
    expected_digest = metadata["safetensors_sha256"]
    if checkpoint_manifest.get("safetensors_sha256") != expected_digest:
        raise RuntimeError("prepared checkpoint manifest SHA-256 does not match")
    safetensors_path = checkpoint / metadata["safetensors_file"]
    if not safetensors_path.is_file():
        raise RuntimeError("prepared checkpoint safetensors file is missing")
    if sha256_file(safetensors_path) != expected_digest:
        raise RuntimeError("prepared checkpoint safetensors SHA-256 does not match")


def _extract_stage_bundle(
    artifact_path: Path,
    destination: Path,
    stage_metadata: list[dict[str, Any]],
) -> list[Path]:
    expected_names = {stage["file"] for stage in stage_metadata}
    try:
        with ZipFile(artifact_path, mode="r") as archive:
            actual_names = {
                entry.filename for entry in archive.infolist() if not entry.is_dir()
            }
            if actual_names != expected_names:
                raise RuntimeError(
                    "TensorRT stage bundle contents do not match its manifest"
                )
            extracted: list[Path] = []
            for stage in stage_metadata:
                stage_name = stage["file"]
                stage_path = destination / stage_name
                with archive.open(stage_name, mode="r") as source, stage_path.open(
                    "wb"
                ) as target:
                    shutil.copyfileobj(source, target, length=1024 * 1024)
                if sha256_file(stage_path) != stage["sha256"]:
                    raise RuntimeError(
                        f"TensorRT serialized stage {stage['index']} SHA-256 mismatch"
                    )
                extracted.append(stage_path)
            return extracted
    except BadZipFile as error:
        raise RuntimeError("TensorRT stage bundle is not a valid ZIP artifact") from error


class TensorRTSemanticBackend:
    """Execute TensorRT Swin stages inside the CUDA PyTorch Mask2Former model."""

    def __init__(
        self,
        engine_path: Path,
        manifest_path: Path,
        *,
        profile: str,
        model_revision: str,
        input_shape: tuple[int, ...],
        output_shape: tuple[int, ...],
        device: torch.device,
    ) -> None:
        if device.type != "cuda" or not torch.cuda.is_available():
            raise RuntimeError("TensorRT backend requires an available CUDA device")
        device = normalize_cuda_device(device)
        if not engine_path.is_file():
            raise RuntimeError(f"TensorRT artifact does not exist: {engine_path}")
        manifest = load_engine_manifest(manifest_path)
        validate_engine_manifest(
            manifest,
            profile=profile,
            model_revision=model_revision,
            input_shape=input_shape,
            output_shape=output_shape,
        )
        if sha256_file(engine_path) != manifest["engine_sha256"]:
            raise RuntimeError("TensorRT artifact SHA-256 does not match its manifest")

        try:
            import tensorrt as trt
            import torch_tensorrt
        except ImportError as error:
            raise RuntimeError(
                "hybrid backend requires JetPack-compatible TensorRT and Torch-TensorRT"
            ) from error

        runtime_metadata = manifest.get("runtime", {})
        expected_trt = runtime_metadata.get("tensorrt_version")
        if expected_trt and expected_trt != trt.__version__:
            raise RuntimeError(
                f"TensorRT version mismatch: artifact={expected_trt} "
                f"runtime={trt.__version__}"
            )
        expected_torch_trt = runtime_metadata.get("torch_tensorrt_version")
        actual_torch_trt = getattr(torch_tensorrt, "__version__", None)
        if expected_torch_trt and expected_torch_trt != actual_torch_trt:
            raise RuntimeError(
                "Torch-TensorRT version mismatch: "
                f"artifact={expected_torch_trt} runtime={actual_torch_trt}"
            )
        expected_cuda = runtime_metadata.get("cuda_version")
        if expected_cuda and expected_cuda != torch.version.cuda:
            raise RuntimeError(
                f"CUDA version mismatch: artifact={expected_cuda} "
                f"runtime={torch.version.cuda}"
            )
        expected_capability = runtime_metadata.get("compute_capability")
        actual_capability = list(torch.cuda.get_device_capability(device))
        if expected_capability and expected_capability != actual_capability:
            raise RuntimeError(
                "TensorRT artifact compute capability does not match this CUDA device"
            )

        checkpoint_metadata = manifest["checkpoint"]
        checkpoint = Path(checkpoint_metadata["path"])
        _validate_checkpoint_files(
            checkpoint,
            checkpoint_metadata,
            model_revision=model_revision,
        )
        try:
            from transformers import Mask2FormerForUniversalSegmentation
        except ImportError as error:
            raise RuntimeError("hybrid backend requires Transformers") from error

        artifact_directory = TemporaryDirectory(prefix="swin-l-trt-stages-")
        try:
            stage_metadata = manifest["partitioning"]["stages"]
            stage_paths = _extract_stage_bundle(
                engine_path,
                Path(artifact_directory.name),
                stage_metadata,
            )
            # Keep the full model on CPU until its original Swin stages have
            # been replaced. This avoids holding both copies of the backbone
            # in Jetson GPU memory during startup.
            model = Mask2FormerForUniversalSegmentation.from_pretrained(
                checkpoint,
                use_safetensors=True,
                local_files_only=True,
                dtype=torch.float16,
            ).eval()
            stages = _swin_stages(model)
            if len(stages) != len(stage_metadata):
                raise RuntimeError(
                    "prepared checkpoint Swin stage count does not match the artifact"
                )
            for stage, stage_path in zip(stage_metadata, stage_paths):
                try:
                    loaded = torch_tensorrt.load(str(stage_path))
                except Exception as error:
                    raise RuntimeError(
                        f"TensorRT Swin stage {stage['index']} load failed"
                    ) from error
                if isinstance(loaded, torch.export.ExportedProgram):
                    loaded = loaded.module()
                if not isinstance(loaded, torch.nn.Module):
                    raise TypeError(
                        f"TensorRT Swin stage {stage['index']} is not a module"
                    )
                stages[stage["index"]] = _LoadedStageProxy(
                    loaded,
                    stage["tensor_input_count"],
                    has_downsample=stage["has_downsample"],
                )
                gc.collect()

            self.device = device
            self.manifest = manifest
            self.engine_path = engine_path
            self._trt = trt
            self._torch_tensorrt = torch_tensorrt
            self._module = SwinLSemanticScores(
                model,
                evaluation_height=output_shape[1],
                evaluation_width=output_shape[2],
            ).eval().to(device)
            self.input_shape = input_shape
            self.output_shape = output_shape
            with torch.inference_mode():
                warmup_input = torch.zeros(
                    input_shape,
                    dtype=torch.float16,
                    device=device,
                )
                warmup_output = self.semantic_scores(warmup_input)
                torch.cuda.synchronize(device)
                if not bool(torch.isfinite(warmup_output).all().item()):
                    raise RuntimeError("hybrid TensorRT warm-up produced NaN or Inf")
        except Exception:
            artifact_directory.cleanup()
            raise
        self._artifact_directory = artifact_directory

    def semantic_scores(self, pixel_values: torch.Tensor) -> torch.Tensor:
        if pixel_values.device != self.device:
            raise ValueError("TensorRT input is on the wrong device")
        if pixel_values.dtype != torch.float16:
            raise ValueError("TensorRT input must be float16")
        if tuple(pixel_values.shape) != self.input_shape:
            raise ValueError("TensorRT input shape does not match the artifact")
        if not pixel_values.is_contiguous():
            pixel_values = pixel_values.contiguous()
        output = _single_tensor_output(self._module(pixel_values))
        if output.device != self.device:
            raise RuntimeError("hybrid TensorRT output is on the wrong device")
        if output.dtype != torch.float16:
            raise RuntimeError("hybrid TensorRT output is not float16")
        if tuple(output.shape) != self.output_shape:
            raise RuntimeError("hybrid TensorRT output shape is invalid")
        return output

    def metadata(self) -> dict[str, Any]:
        return {
            "backend": "torch-tensorrt-hybrid",
            "engine": str(self.engine_path),
            "engine_sha256": self.manifest["engine_sha256"],
            "tensorrt_version": self._trt.__version__,
            "torch_tensorrt_version": getattr(
                self._torch_tensorrt, "__version__", None
            ),
            "partitioning": self.manifest["partitioning"],
        }
