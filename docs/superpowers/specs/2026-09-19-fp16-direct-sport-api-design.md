# FP16 Direct Sport API Design

## Goal

Run the selected Swin-L line-tracking profile in FP16 and replace the
`/a2_control` `sensor_msgs/Joy` command path with direct Unitree Sport API
requests on `/api/sport/request`.

## Precision contract

- Keep `swin-l-aspect-224x384` as an explicit FP32 rollback profile.
- Add `swin-l-aspect-224x384-fp16` with the identical checkpoint, input size,
  evaluation size, class aggregation, temporal alpha, and hysteresis margin.
- Make the new FP16 profile the default and the only profile accepted by
  `task-drive` preflight.
- Cast floating model inputs and parameters to FP16 on CUDA/MPS only. Preserve
  integer inputs and the existing CPU FP32 score smoothing/post-processing.

## Robot command contract

`task-drive` publishes `unitree_api/msg/Request` directly to
`/api/sport/request`. Each move uses `header.identity.api_id = 1008` and compact
JSON in `parameter`:

```json
{"x":0.1,"y":0.0,"z":-0.1}
```

The current field-calibrated signs are preserved:

- `x = DriveDecision.vx`
- `y = -DriveDecision.vy`
- `z = -DriveDecision.yaw_rate`

Zero commands use the same Move API with all values set to `0.0`. Existing
startup, sensor-error, task-abort, SIGTERM, and shutdown zero-command behavior
remains unchanged. The publisher uses reliable QoS depth 10 and exists only
while a task owns control.

This is the user-selected direct-control option. It bypasses the
`teamgrit-navigation` emergency-stop forwarding node. The task listener does
not reject or abort when another `/api/sport/request` publisher exists, so
concurrent publishers may issue conflicting commands to the Unitree interface.

## ROS message packaging

Vendor the reference `unitree_api` interface package and BSD-3-Clause license
under `third_party/unitree_msgs`. Build it in `/unitree_ws` in the Jetson image,
source that overlay in the entrypoint, and fail the image build if the Python
`Request` type cannot be imported.

## FP32/FP16 validation

Use `/home/gritandgrind/Downloads/20260911_025838_teamgrit_rosbag_0.mcap` as the
fixed 720p camera/LiDAR corpus. Run FP32 and FP16 in separate processes against
identical offsets, frame counts, and timestamps. Persist masks, raw and smoothed
paths, dtype metadata, parameter bytes, and CUDA allocated/reserved peaks.

Compare:

- selected-label agreement and Road/Sidewalk IoU;
- path-validity agreement, lateral error, and confidence change;
- floating parameter bytes and incremental CUDA activation peak.

Initial acceptance gates are mask agreement at least 0.99, mean per-class IoU
at least 0.99, path-validity agreement 1.0, path lateral p95 at most 0.10 m,
floating parameter memory ratio at most 0.52, and no NaN, CUDA error, or
inference failure. The A6000 result establishes relative behavior; Jetson
absolute unified-memory use still requires `tegrastats` validation.

## Rollback

The FP32 profile remains selectable by name. Reverting the command transport is
independent from selecting FP32, so precision and robot I/O can be diagnosed
separately.
