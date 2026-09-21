"""Trusted, fixed-shape TensorRT runtime for Swin-L semantic scores."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import torch

ENGINE_MANIFEST_SCHEMA_VERSION = 1


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
        raise RuntimeError("TensorRT engine SHA-256 is missing or invalid")


class TensorRTSemanticBackend:
    """Execute one static TensorRT engine using PyTorch-owned CUDA buffers."""

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
        if not engine_path.is_file():
            raise RuntimeError(f"TensorRT engine does not exist: {engine_path}")
        manifest = load_engine_manifest(manifest_path)
        validate_engine_manifest(
            manifest,
            profile=profile,
            model_revision=model_revision,
            input_shape=input_shape,
            output_shape=output_shape,
        )
        if sha256_file(engine_path) != manifest["engine_sha256"]:
            raise RuntimeError("TensorRT engine SHA-256 does not match its manifest")

        try:
            import tensorrt as trt
        except ImportError as error:
            raise RuntimeError(
                "TensorRT backend requires JetPack-compatible tensorrt bindings"
            ) from error

        runtime_metadata = manifest.get("runtime", {})
        expected_trt = runtime_metadata.get("tensorrt_version")
        if expected_trt and expected_trt != trt.__version__:
            raise RuntimeError(
                f"TensorRT version mismatch: engine={expected_trt} runtime={trt.__version__}"
            )
        expected_cuda = runtime_metadata.get("cuda_version")
        if expected_cuda and expected_cuda != torch.version.cuda:
            raise RuntimeError(
                f"CUDA version mismatch: engine={expected_cuda} runtime={torch.version.cuda}"
            )
        expected_capability = runtime_metadata.get("compute_capability")
        actual_capability = list(torch.cuda.get_device_capability(device))
        if expected_capability and expected_capability != actual_capability:
            raise RuntimeError(
                "TensorRT engine compute capability does not match this CUDA device"
            )

        self.device = device
        self.manifest = manifest
        self.engine_path = engine_path
        self._trt = trt
        self._logger = trt.Logger(trt.Logger.WARNING)
        self._runtime = trt.Runtime(self._logger)
        self._engine = self._runtime.deserialize_cuda_engine(engine_path.read_bytes())
        if self._engine is None:
            raise RuntimeError("TensorRT engine deserialization failed")
        self._context = self._engine.create_execution_context()
        if self._context is None:
            raise RuntimeError("TensorRT execution-context creation failed")

        self.input_name = manifest["input"]["name"]
        self.output_name = manifest["output"]["name"]
        self.input_shape = input_shape
        self.output_shape = output_shape
        self._validate_engine_bindings()
        self._output = torch.empty(
            output_shape,
            dtype=torch.float16,
            device=device,
        )
        with torch.inference_mode():
            warmup_input = torch.zeros(
                input_shape,
                dtype=torch.float16,
                device=device,
            )
            warmup_output = self.semantic_scores(warmup_input)
            torch.cuda.synchronize(device)
            if not bool(torch.isfinite(warmup_output).all().item()):
                raise RuntimeError("TensorRT warm-up produced NaN or Inf")

    def _validate_engine_bindings(self) -> None:
        trt = self._trt
        names = {
            self._engine.get_tensor_name(index)
            for index in range(self._engine.num_io_tensors)
        }
        if {self.input_name, self.output_name} != names:
            raise RuntimeError("TensorRT engine bindings do not match its manifest")
        if self._engine.get_tensor_mode(self.input_name) != trt.TensorIOMode.INPUT:
            raise RuntimeError("TensorRT input binding has the wrong mode")
        if self._engine.get_tensor_mode(self.output_name) != trt.TensorIOMode.OUTPUT:
            raise RuntimeError("TensorRT output binding has the wrong mode")
        if tuple(self._engine.get_tensor_shape(self.input_name)) != self.input_shape:
            raise RuntimeError(
                "TensorRT engine input shape is not static or mismatched"
            )
        if tuple(self._engine.get_tensor_shape(self.output_name)) != self.output_shape:
            raise RuntimeError(
                "TensorRT engine output shape is not static or mismatched"
            )
        if self._engine.get_tensor_dtype(self.input_name) != trt.float16:
            raise RuntimeError("TensorRT engine input dtype is not float16")
        if self._engine.get_tensor_dtype(self.output_name) != trt.float16:
            raise RuntimeError("TensorRT engine output dtype is not float16")

    def semantic_scores(self, pixel_values: torch.Tensor) -> torch.Tensor:
        if pixel_values.device != self.device:
            raise ValueError("TensorRT input is on the wrong device")
        if pixel_values.dtype != torch.float16:
            raise ValueError("TensorRT input must be float16")
        if tuple(pixel_values.shape) != self.input_shape:
            raise ValueError("TensorRT input shape does not match the engine")
        if not pixel_values.is_contiguous():
            pixel_values = pixel_values.contiguous()
        stream = torch.cuda.current_stream(self.device)
        if not self._context.set_tensor_address(
            self.input_name, pixel_values.data_ptr()
        ):
            raise RuntimeError("TensorRT rejected the input buffer address")
        if not self._context.set_tensor_address(
            self.output_name, self._output.data_ptr()
        ):
            raise RuntimeError("TensorRT rejected the output buffer address")
        if not self._context.execute_async_v3(stream.cuda_stream):
            raise RuntimeError("TensorRT inference enqueue failed")
        return self._output

    def metadata(self) -> dict[str, Any]:
        return {
            "backend": "tensorrt",
            "engine": str(self.engine_path),
            "engine_sha256": self.manifest["engine_sha256"],
            "tensorrt_version": self._trt.__version__,
        }
