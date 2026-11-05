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
