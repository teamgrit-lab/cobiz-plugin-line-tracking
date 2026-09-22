"""Trusted fixed-shape Torch-TensorRT hybrid runtime for Swin-L scores."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import torch

ENGINE_MANIFEST_SCHEMA_VERSION = 2
HYBRID_ARTIFACT_FORMAT = "torch_exported_program_hybrid"


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
    if not isinstance(digest, str) or len(digest) != 64:
        raise RuntimeError("TensorRT artifact SHA-256 is missing or invalid")
    partitioning = manifest.get("partitioning")
    if not isinstance(partitioning, dict):
        raise RuntimeError("TensorRT hybrid partition metadata is missing")
    if partitioning.get("pytorch_patch_embedding") is not True:
        raise RuntimeError("Swin patch embedding must remain in PyTorch")
    if partitioning.get("pytorch_mask2former_decoders") is not True:
        raise RuntimeError("Mask2Former decoders must remain in PyTorch")
    if int(partitioning.get("tensorrt_partition_count", 0)) < 1:
        raise RuntimeError("TensorRT hybrid artifact has no TensorRT partitions")


def _single_tensor_output(output: Any) -> torch.Tensor:
    if isinstance(output, torch.Tensor):
        return output
    if isinstance(output, (tuple, list)) and len(output) == 1:
        value = output[0]
        if isinstance(value, torch.Tensor):
            return value
    raise RuntimeError("hybrid TensorRT program did not return one tensor")


class TensorRTSemanticBackend:
    """Execute a serialized PyTorch-CUDA/TensorRT hybrid ExportedProgram."""

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

        try:
            loaded = torch_tensorrt.load(str(engine_path))
        except Exception as error:
            raise RuntimeError("hybrid TensorRT artifact load failed") from error
        if isinstance(loaded, torch.export.ExportedProgram):
            loaded = loaded.module()
        if not isinstance(loaded, torch.nn.Module):
            raise TypeError("hybrid TensorRT artifact did not contain a module")

        self.device = device
        self.manifest = manifest
        self.engine_path = engine_path
        self._trt = trt
        self._torch_tensorrt = torch_tensorrt
        self._module = loaded.to(device)
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
