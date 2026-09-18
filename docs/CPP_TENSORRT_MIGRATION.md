# C++/TensorRT migration plan

The production migration must preserve the current fail-closed task and drive
contract.  The Python implementation remains the behavioral oracle until the
C++ node passes recorded-MCAP equivalence tests.

## Performance contract

Measure these values on the target Jetson Orin NX with the same power mode,
clocks, camera topic, LiDAR topic, and thermal state:

- sensor timestamp to published path/Joy latency: p50, p95, p99;
- model, model postprocess, path update, and complete pipeline latency;
- received, processed, and overwritten camera frames;
- GPU, CPU, EMC, RAM, temperature, and throttling from `tegrastats`;
- Road/Sidewalk mask IoU, lateral path error, and safety/drive decision parity.

Use batch 1 and the fixed `1x3x224x384` input shape. Throughput batching is not
appropriate for the live control path because it increases latency.

## Target package layout

```text
line_tracking_cpp/
  CMakeLists.txt
  package.xml
  include/line_tracking/
    tensorrt_segmenter.hpp
    local_path.hpp
    lidar_safety.hpp
    task_state.hpp
  src/
    line_tracking_component.cpp
    tensorrt_segmenter.cpp
    local_path.cpp
    lidar_safety.cpp
    task_state.cpp
  cuda/
    preprocess.cu
    semantic_postprocess.cu
```

Build it as an `rclcpp_components` component. The image and LiDAR callbacks
must only validate timestamps and replace a depth-one latest-message slot.
Inference and path extraction run outside the ROS callback thread. The 10 Hz
safety/control timer must never wait for model inference or overlay rendering.

## GPU data path

```text
sensor_msgs/Image
  -> newest-frame slot
  -> preallocated pinned input buffer
  -> asynchronous CUDA resize, BGR/RGB conversion, normalization
  -> TensorRT FP16 engine
  -> CUDA semantic-score EMA, hysteresis, argmax, class mapping
  -> one uint8 Road/Sidewalk/background mask copied to CPU
  -> cached OpenCV BEV remap and centerline fit
  -> nav_msgs/Path and sensor_msgs/Joy
```

Do not copy the full semantic score tensor to the CPU. At 65 classes and
`360x640` FP32, that transfer is about 57 MiB per inference. Temporal score
state stays in device memory. Only the final label mask and small diagnostics
cross to the CPU.

Use two preallocated input/output buffer sets and a non-default CUDA stream.
Synchronize with CUDA events at the point where the CPU mask is consumed;
avoid a process-wide `cudaDeviceSynchronize()` in the live loop.

## Migration phases

1. Export a fixed-shape ONNX model from the pinned Hugging Face revision.
2. Compare ONNX output with PyTorch FP32 on representative MCAP frames.
3. Build and benchmark a TensorRT FP16 engine on the target JetPack image.
4. Implement CUDA preprocessing and static semantic postprocessing.
5. Port cached BEV/path and LiDAR safety code to a dependency-free C++ library.
6. Add the ROS 2 component without a Joy publisher and run it in shadow mode.
7. Compare Python and C++ output on day/night/shadow/turn/obstacle clips.
8. Enable task lifecycle reporting, then gated Joy publication.
9. Evaluate INT8 only after FP16 parity is accepted.

Build TensorRT engines on the deployed JetPack/TensorRT version. Do not commit
an engine built for a different JetPack, CUDA, TensorRT, or Orin SKU.

## Acceptance gates

- No regression in timestamp, publisher-ownership, calibration, LiDAR-frame,
  stale-input, task timeout, SIGTERM, or zero-command behavior.
- Road/Sidewalk quality and path lateral error stay within agreed tolerances on
  the frozen validation MCAP set.
- Safety and drive decision reasons match the Python oracle for every replayed
  control tick.
- p95 inference age remains below the current 0.50 s watchdog and p95 LiDAR age
  remains below 0.35 s with margin.
- No unbounded queue, per-frame allocation growth, thermal throttling, or CUDA
  synchronization on the ROS control timer.

## Later model optimization

After the TensorRT C++ path is stable, distill the Swin-L teacher into a
three-class Road/Sidewalk/background student. Compare a small real-time
segmentation model at `224x384` against the same mask, path, and safety gates.
This is likely to produce a larger gain than further optimizing the Swin-L
Python wrapper.
