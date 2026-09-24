# cobiz-plugin-line-tracking

`actual-activate` is a Cobiz `LINE_TRACKING` task listener for the Unitree A2.
For an accepted task it follows the selected Swin-L surface-center path with
the pinned FP16 TensorRT profile `swin-l-aspect-224x384-fp16`, and it uses AprilTag
detections to stop and complete that task. With no accepted task, it publishes
no Sport Move request. The default path class is sidewalk
(`SWIN_L_PATH_MASK_CLASS=2`); a task can request road (`1`) or road/sidewalk
combined (`0`) with
`payload.selected_mask`.

## Runtime contract

`teamgrit-slam` may publish `apriltag_msgs/msg/AprilTagDetectionArray` on
`/detections` when AprilTag task completion is available. The task can start
and track when that topic has zero publishers. In that mode it cannot complete
from an AprilTag and instead ends through its duration or another lifecycle
event. Empty `detections` arrays still expose detector liveness in metrics.

- A valid Cobiz payload begins with a two-second zero-command startup hold.
- Motion requires fresh camera/inference inputs and an available local path,
  or a saved path-derived yaw when the path-stop bypass is enabled.
- The first AprilTag candidate immediately sends a hard zero-command stop.
- The task completes only after three frames for the same tag ID arrive across
  a full one-second confirmation window. Completion sends the hard stop before
  `TASK_COMPLETED` is published.
- A one- or two-hit false positive remains stopped, then can resume when the
  configured drive checks permit motion.
- If `/detections` stops during confirmation, an unconfirmed candidate is
  released as a false positive after the full confirmation window.
- This service supplies no obstacle avoidance. Use independent, appropriate
  protection and a physical emergency stop for real-world operation.

The direct Sport interface is `/api/sport/request` (`unitree_api/msg/Request`,
Move API ID `1008`). It bypasses navigation-level command arbitration and does
not reject or abort a task when other publishers exist. Concurrent publishers
can therefore issue conflicting commands, and the downstream Unitree interface
determines which command takes effect. Move serialization preserves the existing
axis mapping: `x=vx`, `y=-vy`, and `z=-yaw_rate`. Forward speed defaults to
`0.50 m/s`, can be adjusted through
`LINE_TRACKING_MAX_FORWARD_MPS`, and is rejected above the hard `1.00 m/s`
limit.

## Topics and settings

| Direction | Default | Type | Purpose |
|---|---|---|---|
| input | `/a2/front_camera/image_raw` | `sensor_msgs/Image` | A2 front camera |
| input | `/detections` | `apriltag_msgs/msg/AprilTagDetectionArray` | optional tag detection and liveness metrics |
| input | `/task_event` | `std_msgs/String` | Cobiz task lifecycle event |
| output | `/api/sport/request` | `unitree_api/msg/Request` | accepted-task Move requests |
| output | `/line_tracking/swin_l/local_path` | `nav_msgs/Path` | selected surface-center path in `base_link` |
| output | `/line_tracking/swin_l/metrics` | `std_msgs/String` | path, task, and AprilTag state JSON |
| output | `/task_state` | `std_msgs/String` | `TASK_STARTED`, `TASK_COMPLETED`, or abort/reject state |

The deployment defaults are literal and may be overridden in `.env`:

```dotenv
SWIN_L_APRILTAG_DETECTIONS_TOPIC=/detections
SWIN_L_APRILTAG_MAX_AGE_SEC=1.0
SWIN_L_APRILTAG_CONFIRM_WINDOW_SEC=1.0
SWIN_L_APRILTAG_CONFIRM_MIN_HITS=3
LINE_TRACKING_MAX_FORWARD_MPS=0.50
```

A single `.env` switch bypasses all three path-based stop checks:

```dotenv
LINE_TRACKING_BYPASS_PATH_STOPS=false
```

Set it to `true` to bypass `path_unavailable`, `path_low_confidence` (including
NaN/Inf confidence), and `path_lateral_target_large`. With usable coordinates,
tracking still uses the computed yaw, capped at 0.18 rad/s. If the path is
missing or has no usable coordinates, the controller holds the last valid
yaw from the current task and continues at the configured forward speed.
Until a valid target has been obtained in that task, it still stops with
`path_unavailable`; it does not invent an initial heading.

The held command has no independent timeout. It ends when a valid path returns,
a camera/inference check fails, or another stop/lifecycle condition applies.
Camera/inference faults clear the saved yaw, as does ending or starting a task.
The four camera/inference stops (`camera_stale`, `inference_stale`,
`camera_timestamp_invalid`, and `camera_conversion_error`) always remain active.
AprilTag policy, startup hold, explicit cancellation, task duration, faults,
shutdown, and speed limits also remain active.

The master switch overrides the two individual path-quality switches below;
it does not override the AprilTag switch. Its default is `false`. Rebuild the
image with this code and recreate the service after changing the setting.
Metrics expose `path_stop_bypass` and `path_yaw_held`; held-yaw motion reports
`tracking_path_hold` and can continue even while `path_tracked` is false and
the published Path is empty. The task treats held-yaw motion as permitted
motion, so it does not abort merely because the path has been missing for two
seconds. This setting does not correct the steering sign conversion.

Three automatic stop checks can also be configured independently in `.env`:

```dotenv
LINE_TRACKING_STOP_ON_LOW_CONFIDENCE=true
LINE_TRACKING_STOP_ON_LATERAL_TARGET=true
LINE_TRACKING_STOP_ON_APRILTAG=true
```

Set a flag to `false` to disable that check, then recreate the task service with
an image containing this code. Defaults preserve the existing behavior.

| Setting | Behavior when `false` |
|---|---|
| `LINE_TRACKING_STOP_ON_LOW_CONFIDENCE` | A finite low confidence value does not stop tracking. Unrestricted path mode already bypasses this threshold. |
| `LINE_TRACKING_STOP_ON_LATERAL_TARGET` | A lateral target beyond 0.75 m can be tracked; yaw remains capped at 0.18 rad/s. |
| `LINE_TRACKING_STOP_ON_APRILTAG` | Tags neither stop nor complete a task, including a tag visible at startup. Detection-stream liveness is still reported. |

Camera loss/invalid timestamps, the 5-second camera and inference age limits,
startup hold, task cancellation/end, and fault/shutdown stops remain active.
Without the master bypass, a missing or nonnumeric path and non-finite
confidence also stop motion. Speed limits and Unitree hardware protections are
not changed. There is no environment switch to disable camera-disconnection stopping. The
`stop_checks` object in metrics reports the effective checks, and
`apriltag.stop_enabled` reports whether tag stopping is enabled. Changing a stop
flag does not change the selected road/sidewalk class or create a missing path.

`path_lateral_target_large` means that a path exists, but its lateral coordinate
at the 4 m lookahead exceeds 0.75 m in absolute value. This is a stop decision,
not an inference failure. A split or off-center segmentation region, sparse
path support, or inaccurate camera-to-ground geometry can produce that target.
With `LINE_TRACKING_STOP_ON_LATERAL_TARGET=false`, the controller instead uses
`yaw = clip(atan2(y_at_4m, 4), -0.18, 0.18)` rad/s and the configured forward
speed, provided the remaining checks pass. This does not repair the estimated
path or guarantee that a sharp bend can be followed.

`debugging-swin-l` is an explicit debug profile and never publishes Sport
requests. It can be used to inspect the camera path and metrics before a live
task run.

## Start the task listener

`cobiz-core` must register `LINE_TRACKING` in `actions.custom`, and its task
lifecycle bridge must publish `/task_event`. On the Jetson, prepare the DDS
directory and configure the camera and path geometry. Start `teamgrit-slam` if
AprilTag-based completion is required.

```bash
cp .env.example .env
mkdir -p .cache/huggingface models/swin-l-checkpoint models
docker compose --profile engine build
docker compose run --rm prepare-swin-l-checkpoint
docker compose run --rm build-swin-l-engine
docker compose config --quiet
docker compose up -d actual-activate
docker compose logs -f actual-activate

ros2 topic echo /detections
ros2 topic echo /line_tracking/swin_l/metrics
ros2 topic echo /task_state
```

Use a finite task payload such as:

```json
{"duration_sec": 30, "selected_mask": 2}
```

`selected_mask` accepts `0` (road or sidewalk), `1` (road only), or `2`
(sidewalk only). With `0`, both surface labels from the same inference form
one combined region for path generation and tracking. Either surface alone
can produce a path, and adjacent road/sidewalk regions can form one wider
region. Semantic background pixels (label `0`) are still excluded. This mode
does not prefer sidewalk over road or run the model a second time.

To use this mode when a task omits `selected_mask`, set
`SWIN_L_PATH_MASK_CLASS=0` in `.env` and recreate the service with an updated
image. An explicit task value overrides the environment default. Metrics
report `path_mask_class: 0` and `path_surface: "ROAD_OR_SIDEWALK"`. Existing
stop checks still apply; no road/sidewalk region means no path.

The default duration is 500 seconds. Requests above 1000 seconds are capped at
1000 seconds. The listener
reports task state on `/task_state`; core owns any corresponding HTTP report.
It hard-stops on server cancellation, `SIGTERM`, publish errors, stale required
camera/inference inputs, an unavailable path, or tag confirmation.
Detection-stream loss is reported in metrics but does not abort or block the task.
No process can publish a final command after power loss or `SIGKILL`.

## Camera and path calibration

`.env.example` is not a calibrated deployment file. Before operation, validate
the camera-to-`base_link` geometry and Swin-L path against the installed A2.

Road and sidewalk share the same ROI. Its default bottom width is 84% of the
image, top width is 24%, and height is 55% (top at image y=45%):

```dotenv
SWIN_L_ROI_POLYGON=0.08,1.00,0.92,1.00,0.62,0.45,0.38,0.45
```

An existing deployment `.env` overrides this default. Update its value and
recreate the service to apply it. The ROI also defines the camera-to-ground
homography: moving its top changes which pixels map to the configured 8 m far
distance. Check known ground points after changing it; this setting does not
measure the distance from the camera.

1. In the debug profile, adjust `SWIN_L_ROI_POLYGON` while inspecting the
   `nav_msgs/Path` local path in RViz; use the offline overlay workflow for a
   rendered camera view.
2. Measure known ground points to tune `SWIN_L_NEAR_DISTANCE_M`,
   `SWIN_L_FAR_DISTANCE_M`, and `SWIN_L_GROUND_HALF_WIDTH_M`.
3. Select `SWIN_L_PATH_MASK_CLASS=1` for road or `2` for sidewalk.
4. Verify the camera timestamp, inference freshness, coordinate axes, speed limits,
   and behavior with any concurrently active Sport publishers before a live task.

For a 1280x720 Jetson camera, retain the 640x360 evaluation size and set only
the actual image topic as needed:

```dotenv
SWIN_L_IMAGE_TOPIC=/a2/front_camera/image_raw
SWIN_L_EVALUATION_WIDTH=640
SWIN_L_EVALUATION_HEIGHT=360
```

## Path generation and motion gates

The road (`1`), sidewalk (`2`), or combined (`0`) region is projected into a 280-by-160
ground grid spanning the configured 3-8 m forward range and +/-3.5 m sideways.
A 5-by-5 morphological closing fills small mask gaps. Each forward-distance
row selects a contiguous region at least 0.12 m wide, favoring width and
continuity with the preceding row. Its center contributes to a quadratic fit,
which is sampled into 20 path points and clipped to +/-3.5 m laterally.

With the deployed default `SWIN_L_UNRESTRICTED_PATH_MODE=true`, at least two
usable rows are sufficient. No path is produced when fewer than two rows have
a qualifying region, including when the selected class is absent from the
ROI. An invalid new estimate clears the previous path. Valid-ratio and drive
confidence thresholds, temporal smoothing, and the smoother's hold expiry are
bypassed in this mode. Sparse observations can therefore be extrapolated over
the full forward range; neither fitting nor gap filling establishes obstacle
clearance. With unrestricted mode disabled, the default valid-row requirement
is 35% (56 of 160 rows), and the smoother can retain a previous valid path for
up to 0.90 seconds.

A visible path does not imply permission to move. The default drive gates are:

| Condition | Result |
|---|---|
| No accepted task | No Move publisher after control release |
| First 2 seconds of a task | Zero velocity |
| Camera or inference source age greater than 5 seconds, missing, or invalid | Zero velocity |
| Missing path or no usable numeric x/y points | With the master bypass, hold this task's last valid yaw; otherwise zero velocity (`path_unavailable`). No saved yaw always means stop. |
| Absolute lateral target at x=4 m greater than 0.75 m | Zero velocity when the master bypass is off and `LINE_TRACKING_STOP_ON_LATERAL_TARGET=true` |
| Non-finite confidence | Zero velocity unless the master bypass is on |
| Confidence below 0.49 when unrestricted mode is disabled | Zero velocity when the master bypass is off and `LINE_TRACKING_STOP_ON_LOW_CONFIDENCE=true` |
| AprilTag candidate | When `LINE_TRACKING_STOP_ON_APRILTAG=true`, hard stop during confirmation; confirmed tag completes the task |

The controller has no separate path-age cutoff and does not require the path
to span x=4 m or arrive in increasing x order. It discards non-finite points,
sorts by forward distance, and uses the first finite point at each duplicate
distance. A single finite point is sufficient. If x=4 m is outside the available
range, the nearest endpoint's lateral coordinate supplies the target. A path
with no usable numeric x/y points remains unavailable; the master bypass may
reuse a saved yaw but does not publish an invented path. The heading calculation
still uses the 4 m lookahead, and the 0.75 m lateral target limit applies when
its stop check is enabled and the master bypass is off.

Path age remains a diagnostic measured from inference-result availability.
Camera and inference freshness also account for the original sensor timestamp;
repeating a path at 10 Hz does not reset these timestamps. Tracking sends the
configured forward speed (default 0.50 m/s), zero lateral velocity, and a heading-based yaw
rate capped at +/-0.18 rad/s. A task aborts if tracking is unavailable at the
end of startup hold, or if an unsafe tracking condition persists for 2 seconds
after tracking has begun. Server cancellation, task termination, inference or
publish faults, and handled shutdown also stop motion. AprilTag confirmation
has its own stop-and-confirm lifecycle; detector-stream loss alone does not
block tracking.

## FP16 TensorRT Swin-L and Jetson image

The live task profile is fixed to `swin-l-aspect-224x384-fp16`:

- model: `facebook/mask2former-swin-large-mapillary-vistas-semantic`
- revision: `4772b6bf101d91f2534c106dc524d906aeb3c68a`
- model input: 224x384; score map: 640x360; FP16 on CUDA/MPS
- CPU falls back to FP32 for compatibility

Build the CUDA base image once on the Jetson host with jetson-containers, then
use it as the Compose build base:

```bash
cd ~/tools/jetson-containers
PYTORCH_VERSION=2.8 CUDA_VERSION=12.6 \
jetson-containers build \
  --base=cobiz:jetson \
  --name=cobiz:jetson-swin-l \
  --skip-packages=ffmpeg,opencv,ros \
  pytorch:2.8

docker image inspect cobiz:jetson-swin-l-l4t-r36.5.0
cd /path/to/cobiz-plugin-line-tracking
mkdir -p .cache/huggingface models/swin-l-checkpoint models
docker compose --profile engine build
docker compose run --rm prepare-swin-l-checkpoint
docker compose run --rm build-swin-l-engine
docker compose --profile debug up -d --build debugging-swin-l
docker compose logs -f debugging-swin-l
```

The image build installs the vendored `unitree_api` and `apriltag_msgs`
interfaces. Checkpoint preparation explicitly loads the pinned Hub
`model.safetensors`, records any checkpoint-initialized values, and saves one
complete local safetensors file. The engine build then compiles a static
`1x3x224x384` FP16 input into a `65x360x640` semantic-score output and writes a
SHA-256-protected manifest next to the plan. TensorRT plans must be generated on
the target Jetson class and rebuilt after a TensorRT/CUDA/GPU change.

The listener validates the plan checksum, model revision, binding shapes,
TensorRT version, and GPU compute capability before accepting inference. It
never falls back automatically in `task-drive` mode. Set
`SWIN_L_BACKEND=pytorch` explicitly to use the retained FP16 PyTorch rollback;
`SWIN_L_ALLOW_BACKEND_FALLBACK=true` is allowed only for debug/offline use.

## Jetson inference stability

Live inference always obeys `SWIN_L_INFERENCE_HZ`, including when
`SWIN_L_UNRESTRICTED_PATH_MODE=true`. The latest-frame queue holds one frame;
it replaces pending frames while waiting for the next inference start. A late
inference does not trigger a burst of catch-up jobs. Rate limiting reduces
average load, but cannot guarantee latency or cap instantaneous GPU power.

The hybrid backend and the complete segmentation/temporal pipeline run in
`torch.inference_mode()` in the calling worker thread. This prevents the
PyTorch decoder's autograd graph from being retained across frames by the
temporal score average. `model.eval()` alone does not disable autograd.
See the [PyTorch autograd documentation](https://docs.pytorch.org/docs/stable/notes/autograd).

Compose defaults the OpenMP, MKL, and OpenBLAS thread pools to two threads.
Engine auto-build is disabled by default (`SWIN_L_TRT_AUTO_BUILD=false`);
prepare artifacts with the engine services during maintenance, with live
inference stopped. Existing `.env` values still override these defaults.
Run only one live Swin-L model at a time on the Jetson.

For an **inference-only** 1 Hz acceptance run, use debug mode with
`SWIN_L_INFERENCE_HZ=1.25` to leave scheduling margin. Warm up first, then
measure for at least five minutes under the normal camera and service load:

- `/line_tracking/swin_l/metrics`: `performance.completion_fps >= 1.0`,
  `completion_gap_max_ms <= 1000`, and processing p95/p99 latency. These are
  rolling-window metrics; record the entire run to detect intermittent stalls.
- `tegrastats`: total RAM, swap activity, GPU load, temperature, and `VDD_IN`.
  `cuda_memory` in the metrics covers only the PyTorch allocator, not all
  TensorRT allocations or host memory. Memory should plateau after warm-up.
- Kernel OOM logs, container restart count, and `oc*_event_cnt`: no increases
  during the run. A five-minute pass is a smoke test, not a long-term guarantee.

The task-driving controller reuses the available path between inference results
without a separate 0.45-second path timeout. A 1-1.5 Hz update rate alone no
longer inserts zero commands between valid results. The live inference target
remains 4 Hz. Missing paths and the 5-second camera/inference freshness checks
still stop motion; other task and drive gates also remain active. In restricted
path mode, the smoother's separate 0.90-second hold expiry can still make the
path unavailable.

If an over-current warning appears, inspect the board's actual power modes
with `nvpmodel -q` and `/etc/nvpmodel.conf`, then validate a supported power
budget with the carrier board and supply. Do not disable hardware throttling
or assume that a lower inference frequency alone prevents current spikes.
See [NVIDIA's power and throttling documentation](https://docs.nvidia.com/jetson/archives/r36.5/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html).

## Offline camera overlay

Convert an MCAP camera topic to MP4 without ROS 2:

```bash
uv run --with mcap --with mcap-ros2-support \
  python tools/rosbag_mcap_to_mp4.py \
  --input /path/to/input.mcap \
  --topic /a2/front_camera/res_360p/image_raw \
  --output rosbag-results/camera.mp4
```

Run the Swin-L overlays locally, or use the `test-swin-l` Compose profile on a
Jetson:

```bash
uv run tools/swin_l_rosbag_overlay.py sidewalk --input /path/to/input.mcap --open
uv run tools/swin_l_rosbag_overlay.py local-path --input /path/to/input.mcap --open

docker compose run --rm --no-deps test-swin-l local-path \
  --input /bags/input.mcap --device cuda
```

These overlays validate segmentation and path geometry; they do not validate a
live robot operation.

## Verification

```bash
python -m pytest -q test/
docker compose config --quiet
```
