# Runtime cleanup validation — 2026-11-05

Baseline: `master` at `33dd30d`. The local deployment selects
`r50-finetuned-fp16-640x360`, PyTorch/CUDA, and mask mode 0. Configuration migration
preserves that selection, model path, revision, manifest pin and controller values.
Private values are not included in this report.

## Behavior comparison

The original and updated R50 postprocessing were run in separate interpreters
with the same deterministic 360×640 score sequences. Both the public and local
R50 profiles were exercised, with path modes 0/1/2. All 64 captured arrays match
exactly: selected masks, temporal history, smoothed path points and velocity/yaw
commands. This verifies the retained computations; it does not measure model
quality on new real footage. Deployment weights are not present on this Mac.

The camera JSON was copied byte-for-byte from the original simulator location to
`config/camera360-profile.json`. Its canonical profile SHA-256 remains
`77c4d25f4e4da0555369bd6ed607011a22b969c8a67974cb2ae32ff278318cdf`.
Real sensor datasets and model package schemas/checksum fields are unchanged.

## Regression and packaging checks

- R50/configuration/routing subset: 362 passed, 16 skipped.
- Road camera/native-resolution/recovery/isolation subset: 82 passed.
- Final full regression, excluding the known baseline cases below: 1071 passed,
  25 skipped, 2 deselected in a clean staged-source checkout.
- That checkout contains no local simulator, private `.env`, or model caches.
- Base Compose and each Connectome, front, 360, FCN and navigation override resolve
  successfully using `.env.example`. A placeholder bundle hash was used solely
  for Compose interpolation; no model inference was claimed from that value.
- Maintained Python imports have no dependency on retired or local-only modules.
- Both Dockerfiles copy only maintained paths. Shell syntax and local simulator
  `--help` checks pass. The simulator remains on disk and is ignored by Git/Docker.
- Runtime diagrams were regenerated from their Mermaid sources.
- The pre-existing untracked `tools/validate_connectome360_videos.py` is unchanged
  and is not included in these commits.

## Existing baseline failures

An unfiltered `pytest -q test` already fails collection before these changes:
`test_road_corner.py` imports the removed prototype type `RoadPathResult`, and
`test_road_corner_ros.py` imports that test module. Those unrelated prototype
tests were left unchanged apart from the common module-name migration.

With those two modules excluded, the original and updated code both fail these
AprilTag cases in the local OpenCV environment:

- `test_range_recovers_distance_through_the_wide_angle_distortion[pose2-3.0]`
- `test_range_recovers_distance_through_the_wide_angle_distortion[pose2-6.0]`

The clean-checkout regression command was:

```bash
python -m pytest -q test \
  --ignore=test/test_road_corner.py --ignore=test/test_road_corner_ros.py \
  --deselect='test/test_apriltag_stop.py::test_range_recovers_distance_through_the_wide_angle_distortion[pose2-3.0]' \
  --deselect='test/test_apriltag_stop.py::test_range_recovers_distance_through_the_wide_angle_distortion[pose2-6.0]'
```

GPU-dependent tests skip without the corresponding device. This Mac's Docker
daemon has no Jetson base image or NVIDIA runtime. Jetson image building, CUDA
model loading and physical camera/robot verification therefore remain deployment
checks; no live robot services were started.

## Follow-up cleanup — 2026-11-09

Removed the unused `update_path_smoothers`, `_now_stamp`, and
`SegmentationModel._as_numpy` helpers, plus the unused NumPy import in
`direct_camera_service.py`. The path-mode isolation test now exercises the
retained `extract_path_estimates` and smoother APIs directly.

Removed the disconnected `road_corner.py` prototype, its obsolete
`validate_surface_navigation.py` replay tool, and both corner test modules.
The corner collection failures and `--ignore` options above describe the
November 5 baseline; those test files are no longer present.

An AST comparison against the pre-cleanup commit confirms that the retained
runtime code is identical after excluding only the removed helpers/import.
No tracked Python source in `tools/` or `test/` still references the removed
helpers or corner modules. Camera transport, supported model configurations,
and the active Sidewalk/Road control paths are unchanged.

The two AprilTag distance failures above were reproduced before cleanup.
After cleanup, the full maintained suite completed with **1098 passed,
25 skipped, 2 deselected** using:

```bash
python -m pytest -q test \
  --deselect='test/test_apriltag_stop.py::test_range_recovers_distance_through_the_wide_angle_distortion[pose2-3.0]' \
  --deselect='test/test_apriltag_stop.py::test_range_recovers_distance_through_the_wide_angle_distortion[pose2-6.0]'
```

Validation ran on the Mac; no Jetson services or physical robot commands were
started.

## Optional stop removal — 2026-11-09

Removed the five optional timestamp, immediate missing-path, confidence,
untracked-duration and lateral-target stop conditions, their CLI/environment
settings, dedicated thresholds, telemetry flags and obsolete enabled-guard tests.
Normal tracking retains the former disabled behavior: a usable sharp/low-confidence
path is followed with the existing speed/yaw caps; a missing path holds a valid
previous command or waits for the first one. Waiting alone does not abort a task.

Direct Road capture timestamps are still validated. An invalid or expired capture
is discarded without refreshing the last result or aborting the task, and the
worker requests another frame. Model identity, response format, service-error,
camera freshness, AprilTag and explicit cancellation checks remain independent.
The separately configured path-loss limit and recovery behavior remain supported.

Existing deployment files can retain retired keys without enabling the removed
checks. The configuration migration tool removes those keys and dedicated limits
while preserving remaining controls and model selection.

Regression: **1040 passed, 25 skipped, 2 deselected**, using the same two pre-existing
AprilTag exclusions documented above. Coverage includes prolonged waiting without
task abort, tracking sharp/low-confidence paths, missing-path command holding,
bad capture time followed by a current result, and retired-setting migration.
Base Compose and all six maintained overrides resolve with `.env.example`; the
flow-planner check uses a placeholder bundle hash only for interpolation.
No Jetson services or physical robot commands were started.

## Navigation integration removal — 2026-11-09

Removed the Navigation Compose override, worker mode, segment/lease and shared
ownership code, map/pose-based waypoint guidance, dedicated documentation and
tests. Runtime/container entrypoints retain `ros2` inspection and `task-drive`;
the offline runtime also retains `mcap`. A regression test verifies that the
retired worker mode cannot be selected.

Standalone LINE_TRACKING still accepts CoBiz tasks directly, keeps models loaded,
and handles Road/Sidewalk camera and model routing, path tracking, AprilTag and
task cancellation as before. The task-drive SLAM STOP behavior and its tests are
retained. No weights, camera profiles, speed limits or deployment model selections
were changed. The optional flow planner retains its local road/sidewalk geometry
and building-cost behavior; only external waypoint/pose inputs were removed.

Regression: **982 passed, 19 skipped, 2 deselected**, using the two pre-existing
AprilTag exclusions above. The base Compose file and five supported Road overrides
resolve with `.env.example`; shell syntax and tracked Python syntax checks pass.
The flow-planner configuration check uses a placeholder bundle hash only for
interpolation. No Jetson services or physical robot commands were started.
