# FP16 Direct Sport API Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Swin-L task driving use FP16 and publish Unitree Move requests directly to `/api/sport/request`, with repeatable FP32/FP16 validation on the supplied MCAP.

**Architecture:** Preserve the FP32 profile as rollback, add a precision-only FP16 profile, and keep post-processing in FP32. Isolate A2 command serialization in a ROS-free module, then let the live node wrap the serialized move in `unitree_api/msg/Request`. Build the vendored ROS interface into the Jetson image.

**Tech Stack:** Python 3.12, PyTorch, Transformers Mask2Former, NumPy/OpenCV, ROS 2 Humble, `unitree_api`, Docker Compose, pytest, MCAP.

**Spec:** `docs/superpowers/specs/2026-09-19-fp16-direct-sport-api-design.md`

## Global Constraints

- `swin-l-aspect-224x384` remains the FP32 rollback profile.
- FP16 and FP32 use checkpoint revision `4772b6bf101d91f2534c106dc524d906aeb3c68a` and score map `360x640`.
- Direct commands use `/api/sport/request`, `unitree_api/msg/Request`, and Move API ID `1008`.
- Preserve A2 signs as `x=vx`, `y=-vy`, and `z=-yaw_rate`.
- No command publisher exists before an accepted task or after its zero-command release window.
- Direct control bypasses the navigation emergency-stop node and requires exclusive final-topic ownership.

---

### Task 1: Add and pin the FP16 Swin-L profile

**Files:**
- Modify: `tools/best_so_far_runtime.py`
- Modify: `tools/swin_l_local_path_debug.py`
- Modify: `test/test_best_so_far_profiles.py`
- Modify: `test/test_swin_l_drive_control.py`

**Interfaces:**
- Produces: `SWIN_L_ASPECT_FP16_PROFILE = "swin-l-aspect-224x384-fp16"`.
- Produces: `task-drive` preflight accepting only that profile.

- [ ] Add tests proving the FP16 profile differs from the FP32 rollback only by name and precision, becomes the default, and is pinned for task driving.
- [ ] Run those tests and verify they fail because the profile does not exist.
- [ ] Add the profile and update the preflight/default selection.
- [ ] Run the focused tests and confirm they pass.

### Task 2: Add Unitree Move serialization

**Files:**
- Create: `tools/unitree_sport_api.py`
- Create: `test/test_unitree_sport_api.py`

**Interfaces:**
- Produces: `ROBOT_SPORT_API_ID_MOVE: int = 1008`.
- Produces: `drive_to_sport_move(vx: float, vy: float, yaw_rate: float) -> SportMove`.
- Produces: `SportMove.parameter_json() -> str`.
- Produces: `populate_move_request(request: Any, move: SportMove) -> Any`.

- [ ] Add tests with literal expected values for sign conversion, compact JSON, exact zero normalization, and Request population.
- [ ] Run the test and verify import failure for the missing module.
- [ ] Implement the immutable move value and Request population functions.
- [ ] Run the focused test and confirm it passes.

### Task 3: Replace Joy publishing with direct Sport requests

**Files:**
- Modify: `tools/swin_l_local_path_debug.py`
- Modify: `test/test_cobiz_task_selected_mask_ros.py`
- Modify: `test/test_swin_l_debug_topics.py`

**Interfaces:**
- Consumes: `populate_move_request()` and `ROBOT_SPORT_API_ID_MOVE` from Task 2.
- Produces: task argument `sport_request_topic`, default `/api/sport/request`.

- [ ] Update the fake-ROS integration test to provide a complete `unitree_api.msg.Request` and assert API ID, literal JSON payload, reliable QoS, zero release, and no `/a2_control` output.
- [ ] Run the integration test and verify it fails against Joy publishing.
- [ ] Replace Joy imports, arguments, ownership counting, publisher construction, and message creation with the direct Request path.
- [ ] Run the ROS-focused tests and confirm they pass.

### Task 4: Vendor and build the Unitree interface package

**Files:**
- Create: `third_party/unitree_msgs/LICENSE`
- Create: `third_party/unitree_msgs/unitree_api/CMakeLists.txt`
- Create: `third_party/unitree_msgs/unitree_api/package.xml`
- Create: `third_party/unitree_msgs/unitree_api/msg/*.msg`
- Modify: `Dockerfile.swin-l-debug`
- Modify: `docker/swin_l_debug_entrypoint.sh`
- Modify: `test/test_deployment_contract.py`

**Interfaces:**
- Produces: `/unitree_ws/install/setup.bash` and importable `unitree_api.msg.Request`.

- [ ] Add deployment-contract tests for the vendored package, colcon build, overlay source, and build-time Python import.
- [ ] Run the deployment tests and verify they fail on the missing package/build.
- [ ] Add the reference IDL and license, install ROS build dependencies, build `/unitree_ws`, source it, and add the import smoke test.
- [ ] Run deployment tests and Docker Compose config validation.

### Task 5: Update deployment configuration and operator documentation

**Files:**
- Modify: `docker-compose.yml`
- Modify: `.env.example`
- Modify: `README.md`
- Modify: `test/test_deployment_contract.py`

**Interfaces:**
- Produces: `LINE_TRACKING_SPORT_REQUEST_TOPIC=/api/sport/request`.

- [ ] Add tests requiring FP16 live profiles, the final Sport request topic, and removal of the Joy contract.
- [ ] Run tests and verify they fail against the previous Compose configuration.
- [ ] Update Compose, environment examples, comments, and operating warnings for direct exclusive control.
- [ ] Run deployment tests and `docker compose config`.

### Task 6: Add precision validation artifacts and run the supplied MCAP

**Files:**
- Modify: `tools/validate_swin_l_self_quality.py`
- Create: `tools/compare_swin_l_precision.py`
- Create: `test/test_compare_swin_l_precision.py`
- Generate outside Git: `rosbag-results/fp16-validation/*`

**Interfaces:**
- Produces: profile-specific NPZ/JSON captures and a comparison JSON containing mask, path, parameter-memory, and CUDA-peak metrics.

- [ ] Add pure metric tests using hand-derived masks and paths.
- [ ] Run them and verify failure for the missing comparison module.
- [ ] Allow the capture tool to select profile/device and persist local-path samples plus memory metadata; implement the offline comparator.
- [ ] Run unit tests, then capture identical FP32 and FP16 frames from `/home/gritandgrind/Downloads/20260911_025838_teamgrit_rosbag_0.mcap`.
- [ ] Generate the comparison report and inspect the worst-difference frames.

### Task 7: Final verification, commits, and push

**Files:**
- Verify all modified files and generated reports; do not commit model caches or validation outputs.

**Interfaces:**
- Produces: pushed branch `codex/fp16-direct-sport-api`.

- [ ] Run the complete pytest suite in an explicit dependency environment.
- [ ] Run syntax compilation, Compose config validation, MCAP comparison, and Git diff inspection.
- [ ] Commit coherent changes with descriptive messages.
- [ ] Push `codex/fp16-direct-sport-api` to `origin` and report the commit IDs and validation limitations.
