# cobiz-plugin-line-tracking

`actual-activate` is a Cobiz `LINE_TRACKING` task listener for the Unitree A2.
For an accepted task it follows the selected Swin-L surface-center path with
the pinned FP16 TensorRT profile `swin-l-aspect-224x384-fp16`, and it uses AprilTag
detections to stop and complete that task. With no accepted task, it publishes
no Sport Move request. The default path class is sidewalk
(`SWIN_L_PATH_MASK_CLASS=2`); a task can request road (`1`) with
`payload.selected_mask`.

## Runtime contract

`teamgrit-slam` may publish `apriltag_msgs/msg/AprilTagDetectionArray` on
`/detections` when AprilTag task completion is available. The task can start
and track when that topic has zero publishers. In that mode it cannot complete
from an AprilTag and instead ends through its duration or another lifecycle
event. Empty `detections` arrays still expose detector liveness in metrics.

- A valid Cobiz payload begins with a two-second zero-command startup hold.
- Motion requires fresh camera, inference, and local-path data.
- The first AprilTag candidate immediately sends a hard zero-command stop.
- The task completes only after three frames for the same tag ID arrive across
  a full one-second confirmation window. Completion sends the hard stop before
  `TASK_COMPLETED` is published.
- A one- or two-hit false positive remains stopped, then can resume only after
  camera and path health recover.
- If `/detections` stops during confirmation, an unconfirmed candidate is
  released as a false positive after the full confirmation window.
- This service supplies no obstacle avoidance. Use independent, appropriate
  protection and a physical emergency stop for real-world operation.

The direct Sport interface is `/api/sport/request` (`unitree_api/msg/Request`,
Move API ID `1008`). It bypasses navigation-level command arbitration and does
not reject or abort a task when other publishers exist. Concurrent publishers
can therefore issue conflicting commands, and the downstream Unitree interface
determines which command takes effect. Move serialization preserves the existing
calibrated axes: `x=vx`, `y=-vy`, and `z=-yaw_rate`. Forward speed defaults to
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

The default duration is 500 seconds. Requests above 1000 seconds are capped at
1000 seconds. The listener
reports task state on `/task_state`; core owns any corresponding HTTP report.
It hard-stops on server cancellation, `SIGTERM`, publish errors, stale required
camera/path inputs, or tag confirmation. Detection-stream loss is reported in
metrics but does not abort or block the task.
No process can publish a final command after power loss or `SIGKILL`.

## Camera and path calibration

`.env.example` is not a calibrated deployment file. Before operation, validate
the camera-to-`base_link` geometry and Swin-L path against the installed A2.

1. In the debug profile, adjust `SWIN_L_ROI_POLYGON` while inspecting the
   `nav_msgs/Path` local path in RViz; use the offline overlay workflow for a
   rendered camera view.
2. Measure known ground points to tune `SWIN_L_NEAR_DISTANCE_M`,
   `SWIN_L_FAR_DISTANCE_M`, and `SWIN_L_GROUND_HALF_WIDTH_M`.
3. Select `SWIN_L_PATH_MASK_CLASS=1` for road or `2` for sidewalk.
4. Verify the camera timestamp, path freshness, coordinate axes, speed limits,
   and behavior with any concurrently active Sport publishers before a live task.

For a 1280x720 Jetson camera, retain the 640x360 evaluation size and set only
the actual image topic as needed:

```dotenv
SWIN_L_IMAGE_TOPIC=/a2/front_camera/image_raw
SWIN_L_EVALUATION_WIDTH=640
SWIN_L_EVALUATION_HEIGHT=360
```

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

The task-driving path watchdog still expires at **0.45 seconds**. A 1 Hz
inference rate therefore causes intermittent stops even if inference is
stable. The live default remains 4 Hz; continuous driving requires measured
path-update gaps below 0.45 seconds. Do not extend this watchdog just to hide
slow inference without reviewing speed, stopping distance, and perception age.

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
