# Jetson inference stability investigation

## Observed on dangjin-a2

Read-only inspection on 2026-09-23 (Asia/Seoul), before deploying this patch:

- NVIDIA Jetson Orin NX 16 GB; Jetson Linux R36.5; `nvpmodel` mode `MAXN`.
- Installed modes: ID 0 MAXN, ID 1 10W, ID 2 15W, ID 3 25W.
- Live service: `cobiz-plugin-line-tracking-actual-activate`, source commit
  `6f950ef`, hybrid Torch-TensorRT backend, fixed FP16 224x384 profile.
- Live settings: inference target 4 Hz, unrestricted path mode enabled,
  engine auto-build enabled, camera `/a2/front_camera/res_720p/image_raw`.
- Kernel logs repeatedly killed Python processes with about 12.3 GiB of
  anonymous RSS. One event at 13:26:21 killed PID 69786, previously observed
  running `swin_l_local_path_debug.py task-drive`, with `anon-rss:12909108kB`.
  Its container restarted at 13:26:26. This directly associates at least one
  global OOM with the live inference process.
- Initial 1-minute load average was 54.17 on eight CPU cores. A short
  `tegrastats` sample during startup showed approximately 50–54 C and 11–12 W;
  these are startup observations, not steady-state inference measurements.
- `oc1_event_cnt`, `oc2_event_cnt`, and `oc3_event_cnt` were all zero at
  inspection. The user's reported “overdrive” warning was not reproduced or
  identified as an over-current event in this session.

## Defects reproduced and changes

The hybrid runtime kept PyTorch patch embedding and Mask2Former decoders.
Only startup warm-up was inside `torch.inference_mode()`. Runtime forward
passes retained gradients, and the temporal score average linked successive
frames' graphs through `_previous_scores`. `eval()` does not disable this.

The patch decorates both `TensorRTSemanticBackend.semantic_scores()` and
`BestSoFarSegmenter.segment()` with `torch.inference_mode()`, and detaches the
retained score history. The outer guard also covers temporal smoothing and
executes inside the inference worker thread.

Unrestricted path mode set the live inference period to zero. The patch
always derives the live period from `SWIN_L_INFERENCE_HZ`, while preserving
the existing single-frame replacement queue and start-to-start scheduler.
It adds processing p95/p99, maximum completion gap, and configured rate to
the existing metrics. Offline MCAP frame-selection behavior is unchanged.

Compose now provides two-thread OpenMP/MKL/OpenBLAS defaults and disables
automatic engine building in the live task service by default. An existing
`.env` override must be updated separately. This patch does not change the
model, input resolution, robot commands, or freshness watchdog thresholds.

## Local validation

The local test runtime used Python 3.12, PyTorch 2.13.0, and Transformers
5.16.1. The inspected Jetson image uses PyTorch 2.8.0; CUDA/TensorRT runtime
and throughput still require an on-device acceptance run.

- Focused runtime/deployment tests: **61 passed**.
- Complete checked-in test directory: **230 passed, 8 failed**.
- Unmodified HEAD exported to a temporary directory: **222 passed, the same
  8 tests failed**. No new failures in the complete suite.
- New graph-retention and unrestricted-mode scheduling tests against
  unmodified runtime: three failures reproduced the missing guards and rate
  bypass. The restricted-mode scheduling case passed on both versions.
- Ruff on changed Python files, `git diff --check`, and
  `docker compose --env-file .env.example config --quiet`: passed.

The pre-existing failures are one AprilTag false-positive recovery test,
three AprilTag old-window-boundary cases, three drive-input gating cases,
and the local-path MCAP frame-policy test. They concern existing freshness,
confidence, and unrestricted-mode expectations and remain unresolved.
Run `pytest test/` explicitly: root-wide discovery also includes untracked
rosbag experiment scripts with duplicate module names.

## Acceptance still required

No operational container was modified or restarted during diagnosis.
No stable on-device 1 Hz result is claimed by the local tests.

With the operational model stopped, run the modified inspection-only ROS
mode at 1.25 Hz for at least five minutes after warm-up under normal camera
and companion-service load. Record completion count/rate and maximum gaps,
processing p95/p99, total RAM and swap, power/temperature, restart count, OOM
events, and changes in OC counters. Require at least 1 Hz completed inference
and no gaps above one second throughout the recorded steady-state interval,
with stable memory and no OOM/restarts/OC increments. Follow a successful
smoke test with a longer endurance run before claiming production stability.

Continuous driving is a separate requirement: the current path watchdog is
0.45 seconds. At 1 Hz it will issue intermittent stops. The task service's
default remains 4 Hz; do not loosen the watchdog to conceal insufficient
throughput. If the corrected Swin-L pipeline cannot sustain the required
path-update rate, evaluate a smaller model/resolution with segmentation and
path-quality validation before changing the driving contract.

## References

- [PyTorch 2.8 inference mode](https://docs.pytorch.org/docs/2.8/generated/torch.autograd.grad_mode.inference_mode.html):
  inference mode disables autograd tracking and is thread-local.
- [NVIDIA Jetson R36.5 power management](https://docs.nvidia.com/jetson/archives/r36.5/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html):
  supported power budgets, hardware throttling, and OC event counters.
  Frequency limiting alone is not an instantaneous power cap.
