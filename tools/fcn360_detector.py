"""Jetson FCN-ResNet18 perception publisher. This process has no motion publisher."""
import argparse
import hashlib
import os
import tempfile
from pathlib import Path

import cv2
import numpy as np

from fcn360_road import (BLIND, DEFAULT_INPUT_WIDTH, EGO, FRAME_ID, ROAD,
                         classify_panorama, mask_topic, model_shapes, observed_classes)


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


class Camera:
    """Pull the latest timestamped MJPEG frame; inference never queues old captures."""
    def __init__(self, device):
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
        self.gst = Gst
        Gst.init(None)
        self.pipeline = Gst.parse_launch(
            "v4l2src name=camera ! image/jpeg,width=2880,height=1440 ! "
            "appsink name=latest max-buffers=1 drop=true sync=false")
        self.pipeline.get_by_name("camera").set_property("device", device)
        self.sink = self.pipeline.get_by_name("latest")
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.close()
            raise RuntimeError(f"Cannot open 2880x1440 MJPEG capture on {device}")

    def read(self, clock):
        gst = self.gst
        sample = self.sink.emit("try-pull-sample", 200 * gst.MSECOND)
        if sample is None:
            error = self.pipeline.get_bus().pop_filtered(gst.MessageType.ERROR | gst.MessageType.EOS)
            if error is not None:
                raise RuntimeError(error.parse_error() if error.type == gst.MessageType.ERROR else "camera EOS")
            return None
        buffer = sample.get_buffer()
        pts = sample.get_segment().to_running_time(gst.Format.TIME, buffer.pts)
        if pts == gst.CLOCK_TIME_NONE:
            return None
        running = self.pipeline.get_clock().get_time() - self.pipeline.get_base_time()
        age = running - pts
        if not 0 <= age <= gst.SECOND:
            return None
        from rclpy.duration import Duration
        stamp = (clock.now() - Duration(nanoseconds=int(age))).to_msg()
        encoded = np.frombuffer(buffer.extract_dup(0, buffer.get_size()), np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("Cannot decode FCN camera JPEG")
        return image, stamp

    def close(self):
        self.pipeline.set_state(self.gst.State.NULL)


def overlay(views, masks):
    panels = []
    for name, view, grid, semantic in zip(("FRONT 90", "LEFT 180", "REAR 270", "RIGHT 0"), views,
                                         observed_classes(masks), masks):
        panel = cv2.resize(view, (512, 256), interpolation=cv2.INTER_AREA)
        labels = cv2.resize(grid, (512, 256), interpolation=cv2.INTER_NEAREST)
        panel[(labels == BLIND) | (labels == 0)] //= 2
        road = labels == ROAD
        panel[road] = .55 * panel[road] + np.array([15, 215, 95]) * .45
        ego = cv2.resize(semantic, (512, 256), interpolation=cv2.INTER_NEAREST) == EGO
        panel[ego] = .5 * panel[ego] + np.array([200, 70, 230]) * .5
        cv2.putText(panel, name, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 2)
        panels.append(panel)
    return np.vstack((np.hstack(panels[:2]), np.hstack(panels[2:])))


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video91")
    parser.add_argument("--model", type=Path, default=Path("/fcn-model/fcn_resnet18.onnx"))
    parser.add_argument("--cache", type=Path, default=Path("/fcn-engine-cache"))
    parser.add_argument("--input-width", type=int, choices=(512, 1024),
                        default=int(os.environ.get("LINE_TRACKING_FCN_INPUT_WIDTH", DEFAULT_INPUT_WIDTH)))
    args = parser.parse_args()
    sha = os.environ["LINE_TRACKING_FCN_SHA256"]
    infer = TrtFcn(args.model, sha, args.cache, args.input_width)
    print(f"FCN {args.input_width}x{args.input_width//2} · native {args.input_width//32}x{args.input_width//64} grid · {sha}", flush=True)
    rclpy.init()
    node = rclpy.create_node("fcn360_road_detector")
    masks_pub = node.create_publisher(Image, mask_topic(sha), qos_profile_sensor_data)
    preview_pub = node.create_publisher(CompressedImage, "/line_tracking/fcn360/overlay/compressed", qos_profile_sensor_data)
    camera = None
    def tick():
        nonlocal camera
        if not (masks_pub.get_subscription_count() or preview_pub.get_subscription_count()):
            if camera is not None:
                camera.close()
                camera = None
            return
        if camera is None:
            camera = Camera(args.device)
        capture = camera.read(node.get_clock())
        if capture is None:
            return
        image, stamp = capture
        masks, views = classify_panorama(image, infer, args.input_width)
        msg = Image()
        msg.header.frame_id, msg.header.stamp = FRAME_ID, stamp
        rows, columns = masks.shape[1:]
        msg.height, msg.width, msg.step, msg.encoding = 4 * rows, columns, columns, "mono8"
        msg.data = masks.tobytes()
        masks_pub.publish(msg)
        if preview_pub.get_subscription_count():
            ok, encoded = cv2.imencode(".jpg", overlay(views, masks))
            if ok:
                preview = CompressedImage()
                preview.header, preview.format, preview.data = msg.header, "jpeg", encoded.tobytes()
                preview_pub.publish(preview)

    node.create_timer(.25, tick)
    try:
        rclpy.spin(node)
    finally:
        if camera is not None:
            camera.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
