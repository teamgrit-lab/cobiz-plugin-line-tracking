"""Jetson FCN camera-owned inference. No ROS image or motion publisher."""
import argparse
import hashlib
import os
import tempfile
from pathlib import Path

import numpy as np

from fcn360_road import DEFAULT_INPUT_WIDTH, classify_panorama, model_shapes


class TrtFcn:
    """One fixed-size FP32 TensorRT 10 context shared by the four sequential views."""
    def __init__(self, model_path, sha, cache, input_width=DEFAULT_INPUT_WIDTH):
        import tensorrt as trt
        import torch

        raw = model_path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != sha:
            raise ValueError(f"FCN ONNX SHA-256 mismatch: {model_path}")
        self.torch = torch
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        self.input_width = input_width
        input_shape, output_shape = model_shapes(input_width)
        expected = {"input_0": input_shape, "output_0": output_shape}
        gpu = torch.cuda.get_device_properties(0)
        identity = f"native-fcn-v2-fp32-no-tf32-w{input_width}-{sha}-{trt.__version__}-{gpu.name}-{gpu.major}.{gpu.minor}-{torch.version.cuda}"
        cache.mkdir(parents=True, exist_ok=True)
        path = cache / (hashlib.sha256(identity.encode()).hexdigest() + ".plan")
        self.engine = self.runtime.deserialize_cuda_engine(path.read_bytes()) if path.is_file() else None
        plan = None
        if self.engine is None:
            print("Building FCN TensorRT engine for this Jetson", flush=True)
            builder = trt.Builder(self.logger)
            network = builder.create_network(0)
            parser = trt.OnnxParser(network, self.logger)
            if not parser.parse(raw):
                raise ValueError([str(parser.get_error(i)) for i in range(parser.num_errors)])
            config = builder.create_builder_config()
            config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 256 << 20)
            config.clear_flag(trt.BuilderFlag.TF32)
            plan = builder.build_serialized_network(network, config)
            if plan is None:
                raise RuntimeError("Could not build FCN TensorRT engine")
            self.engine = self.runtime.deserialize_cuda_engine(plan)
            if self.engine is None:
                raise RuntimeError("Could not deserialize new FCN engine")
        if self.engine.num_io_tensors != 2 or any(
            tuple(self.engine.get_tensor_shape(name)) != shape or self.engine.get_tensor_dtype(name) != trt.float32
            for name, shape in expected.items()
        ):
            raise ValueError("FCN engine does not match the camera/model contract")
        if plan is not None:
            with tempfile.NamedTemporaryFile(dir=cache, delete=False) as fp:
                fp.write(bytes(plan))
            Path(fp.name).replace(path)
        self.context = self.engine.create_execution_context()
        self.input = torch.empty(expected["input_0"], device="cuda", dtype=torch.float32)
        self.output = torch.empty(expected["output_0"], device="cuda", dtype=torch.float32)
        for name, tensor in (("input_0", self.input), ("output_0", self.output)):
            if not self.context.set_tensor_address(name, tensor.data_ptr()):
                raise RuntimeError(f"Cannot bind FCN tensor {name}")

    def __call__(self, image):
        if image.shape != tuple(self.input.shape) or image.dtype != np.float32:
            raise ValueError("FCN input does not match the configured engine")
        self.input.copy_(self.torch.from_numpy(image))
        if not self.context.execute_async_v3(self.torch.cuda.current_stream().cuda_stream):
            raise RuntimeError("TensorRT FCN inference failed")
        return self.output.cpu().numpy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video91")
    parser.add_argument("--model", type=Path, default=Path("/fcn-model/fcn_resnet18.onnx"))
    parser.add_argument("--cache", type=Path, default=Path("/fcn-engine-cache"))
    parser.add_argument('--port', type=int, default=8094)
    parser.add_argument("--input-width", type=int, choices=(512, 1024),
                        default=int(os.environ.get("LINE_TRACKING_FCN_INPUT_WIDTH", DEFAULT_INPUT_WIDTH)))
    args = parser.parse_args()
    sha = os.environ["LINE_TRACKING_FCN_SHA256"]
    infer = TrtFcn(args.model, sha, args.cache, args.input_width)
    print(f"FCN {args.input_width}x{args.input_width//2} · native {args.input_width//32}x{args.input_width//64} grid · {sha}", flush=True)
    from types import SimpleNamespace
    from direct_camera_service import DirectCameraService, UsbSource, serve
    # Camera-owned inference has no image publishers or ROS initialization.
    model = SimpleNamespace(reset=lambda: None, details=dict(
        backend=f'fcn{args.input_width}-360', weights_sha256=sha,
        input_width=args.input_width, class_grid_shape=[4, args.input_width//64, args.input_width//32]))
    def predict(panorama):
        masks, _ = classify_panorama(panorama, infer, args.input_width)
        return dict(classes=masks.tolist())
    serve(DirectCameraService(model, UsbSource(args.device), predict), args.port)


if __name__ == "__main__":
    main()
