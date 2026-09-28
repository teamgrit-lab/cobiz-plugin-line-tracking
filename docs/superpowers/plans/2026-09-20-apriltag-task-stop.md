# AprilTag-Gated Line-Tracking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a valid COBIZ `LINE_TRACKING` task drive the A2 without manual arm flags, remove LiDAR STOP completely, and use teamgrit-slam `/detections` to stop immediately and complete the task after a one-second same-ID confirmation.

**Architecture:** Vendor only teamgrit-slam's `apriltag_msgs` interface and subscribe to its existing `/detections` publisher. Keep tag liveness and confirmation in a ROS-independent monitor, keep motion geometry in the existing drive-control module, and integrate both through the task-driving ROS node. `StopMove(1003)` followed by zero `Move(1008)` is the hard-stop primitive.

**Tech Stack:** Python 3.10+, ROS 2 Humble `rclpy`, `apriltag_msgs`, `unitree_api`, NumPy, OpenCV, Docker Compose, pytest, ruff.

**Spec:** `docs/superpowers/specs/2026-09-20-apriltag-task-stop-design.md`

## Global Constraints

- Do not add an AprilTag detector, AprilTag C library, or second AprilTag camera pipeline to this repository.
- `/detections` has type `apriltag_msgs/msg/AprilTagDetectionArray`, reliable/volatile QoS, depth 1.
- An empty detection array is a valid heartbeat. A discovered publisher without a received message is not ready.
- Detection freshness is 1.0 second, measured with the receiving process's monotonic clock.
- Confirmation observes the full 1.0 second window and requires one ID in at least 3 distinct messages.
- The first tag candidate must immediately publish `StopMove(1003, {})` followed by zero `Move(1008)`.
- Confirmed tags produce `TASK_COMPLETED` and remain latched; disappearing tags never restart the completed task.
- Detection loss after tracking started and a new competing Sport publisher are immediate hard-stop plus `TASK_ABORTED` faults.
- Remove `SWIN_L_DRIVE_ENABLED`, `SWIN_L_CALIBRATION_CONFIRMED`, all `SWIN_L_LIDAR_*`, `PointCloud2`, safety-stop, and clearance contracts.
- Retain the current FP16 profile, camera/inference freshness limits (0.50 s), path freshness limit (0.45 s), confidence floor (0.70), 4.0 m lookahead, 0.75 m lateral bound, and speed clamps.
- A task may start in zero-speed hold before dynamic inputs are ready. It aborts at the end of the 2.0 second startup hold if motion readiness is still false.
- Preserve exclusive ownership of `/api/sport/request` and never publish a non-zero command without an active task.
- Use TDD for every behavioral change and preserve unrelated user changes.

---

## File Structure

### New files

- `third_party/apriltag_msgs/apriltag_msgs/CMakeLists.txt` — ROS interface build.
- `third_party/apriltag_msgs/apriltag_msgs/package.xml` — interface dependencies and MIT metadata.
- `third_party/apriltag_msgs/apriltag_msgs/LICENSE` — MIT license carried with the vendored package.
- `third_party/apriltag_msgs/README.md` — upstream repository, pinned commit, and copied-file provenance.
- `third_party/apriltag_msgs/apriltag_msgs/msg/Point.msg` — tag pixel point.
- `third_party/apriltag_msgs/apriltag_msgs/msg/AprilTagDetection.msg` — one detection.
- `third_party/apriltag_msgs/apriltag_msgs/msg/AprilTagDetectionArray.msg` — stamped detection array.
- `tools/apriltag_stop.py` — ROS-independent liveness and confirmation state machine.
- `test/test_apriltag_stop.py` — deterministic state-machine tests.
- `test/test_apriltag_task_stop_ros.py` — fake-ROS end-to-end task stop tests.

### Modified files

- `Dockerfile.swin-l-debug` — build/import both ROS interface packages.
- `tools/unitree_sport_api.py` and `test/test_unitree_sport_api.py` — encode `StopMove`.
- `tools/cobiz_line_tracking_task.py` and `test/test_cobiz_line_tracking_task.py` — accept before dynamic readiness and enforce startup deadline.
- `tools/swin_l_drive_control.py` and `test/test_swin_l_drive_control.py` — path/camera/detection-only motion gates.
- `tools/local_path.py` and `test/test_local_path.py` — delete LiDAR classes and PointCloud2 decoder.
- `tools/swin_l_local_path_debug.py` — remove LiDAR runtime and integrate tag subscription, hard stop, telemetry, and task completion.
- `tools/swin_l_rosbag_overlay.py` and `test/test_swin_l_rosbag_overlay.py` — camera/path-only replay.
- `test/test_swin_l_debug_topics.py` — assert the reduced debug surface.
- `test/test_cobiz_task_selected_mask_ros.py` — adapt fake ROS to the new readiness signature and typed detection dependency.
- `test/test_deployment_contract.py` — interface build and no-LiDAR/no-arm contract.
- `docker-compose.yml`, `.env.example`, and `README.md` — deployment and operator contract.

---

### Task 1: Vendor and Build `apriltag_msgs`

**Files:**
- Create: `third_party/apriltag_msgs/apriltag_msgs/CMakeLists.txt`
- Create: `third_party/apriltag_msgs/apriltag_msgs/package.xml`
- Create: `third_party/apriltag_msgs/apriltag_msgs/LICENSE`
- Create: `third_party/apriltag_msgs/README.md`
- Create: `third_party/apriltag_msgs/apriltag_msgs/msg/Point.msg`
- Create: `third_party/apriltag_msgs/apriltag_msgs/msg/AprilTagDetection.msg`
- Create: `third_party/apriltag_msgs/apriltag_msgs/msg/AprilTagDetectionArray.msg`
- Modify: `Dockerfile.swin-l-debug`
- Modify: `test/test_deployment_contract.py`

**Interfaces:**
- Consumes: teamgrit-slam commit `d03bbf21724f35b4304c688793b05c28e98802a0`, package version `2.0.2`.
- Produces: importable `apriltag_msgs.msg.AprilTagDetectionArray` in `/unitree_ws/install`.

- [ ] **Step 1: Add a failing deployment-contract test**

Extend `test_jetson_image_builds_and_sources_unitree_request_interface` into a two-interface test with these exact checks:

```python
apriltag_root = ROOT / "third_party" / "apriltag_msgs" / "apriltag_msgs"
assert (apriltag_root / "LICENSE").is_file()
assert "d03bbf21724f35b4304c688793b05c28e98802a0" in (
    ROOT / "third_party" / "apriltag_msgs" / "README.md"
).read_text()
assert (apriltag_root / "msg" / "Point.msg").read_text().splitlines() == [
    "float64 x",
    "float64 y",
]
assert (
    apriltag_root / "msg" / "AprilTagDetectionArray.msg"
).read_text().splitlines() == [
    "std_msgs/Header header",
    "AprilTagDetection[] detections",
]
assert (
    "COPY third_party/apriltag_msgs/apriltag_msgs "
    "/unitree_ws/src/apriltag_msgs"
) in dockerfile
assert (
    "colcon build --merge-install --packages-select unitree_api apriltag_msgs"
) in dockerfile
assert "from apriltag_msgs.msg import AprilTagDetectionArray" in dockerfile
```

- [ ] **Step 2: Run the focused test and verify RED**

Run: `python -m pytest test/test_deployment_contract.py::test_jetson_image_builds_and_sources_unitree_request_interface -q`

Expected: FAIL because the vendored package does not exist.

- [ ] **Step 3: Add the exact message definitions**

Copy these source files byte-for-byte from
`../teamgrit-slam/slam/src/apriltag_msgs`: `CMakeLists.txt`, `package.xml`, and
the three `msg/*.msg` files. Add `LICENSE` using the MIT text and Christian
Rauch copyright from `../teamgrit-slam/slam/src/apriltag_ros/LICENSE` because
the message package declares MIT but does not carry a separate license file in
the reference tree. Add `third_party/apriltag_msgs/README.md` recording
`https://github.com/teamgrit-lab/teamgrit-slam.git`, commit
`d03bbf21724f35b4304c688793b05c28e98802a0`, source directory
`slam/src/apriltag_msgs`, and the three hashes below.

Verify the three message SHA-256 values:

```text
AprilTagDetectionArray.msg 04265d730aef8b08214eeebf8f820b63954a655b53b464fd1d1a59b017d67ced
AprilTagDetection.msg      ad5e493f9d18bfc221b6ed1790b95ea4a5960bfea639f2ae7d4eb16b38846b2d
Point.msg                  952153897409af9b2dc9129e8c520766d0e47be8ca8d26979a5d613ddd2f496c
```

- [ ] **Step 4: Build both packages in the image contract**

Change the Docker workspace section to:

```dockerfile
WORKDIR /unitree_ws
COPY third_party/unitree_msgs/unitree_api /unitree_ws/src/unitree_api
COPY third_party/apriltag_msgs/apriltag_msgs /unitree_ws/src/apriltag_msgs
RUN source /opt/ros/humble/setup.bash \
    && colcon build --merge-install --packages-select unitree_api apriltag_msgs \
    && source /unitree_ws/install/setup.bash \
    && /opt/venv/bin/python -c \
       'from unitree_api.msg import Request; from apriltag_msgs.msg import AprilTagDetectionArray; request = Request(); request.header.identity.api_id = 1008; detections = AprilTagDetectionArray(); print("[swin-l-debug] ROS interfaces ready")'
```

The entrypoint already sources `/unitree_ws/install/setup.bash`; retain that single source operation.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run: `python -m pytest test/test_deployment_contract.py -q`

Expected: PASS.

- [ ] **Step 6: Commit the interface package**

```bash
git add third_party/apriltag_msgs Dockerfile.swin-l-debug test/test_deployment_contract.py
git commit -m "build: vendor apriltag message interface"
```

---

### Task 2: Implement ROS-Independent AprilTag Confirmation

**Files:**
- Create: `tools/apriltag_stop.py`
- Create: `test/test_apriltag_stop.py`

**Interfaces:**
- Produces: `AprilTagPolicy`, `AprilTagDecision`, and `AprilTagStopMonitor`.
- `AprilTagStopMonitor.observe(ids, frame_key, now, task_active)` records heartbeat and one frame.
- `AprilTagStopMonitor.tick(now, task_active)` advances time without inventing a message.
- `AprilTagStopMonitor.stream_ready(now)` reports callback freshness.
- `AprilTagStopMonitor.reset_task()` clears confirmation state but preserves stream heartbeat.

- [ ] **Step 1: Write policy and empty-heartbeat tests**

Create `test/test_apriltag_stop.py` with:

```python
from apriltag_stop import AprilTagPolicy, AprilTagStopMonitor


def monitor():
    return AprilTagStopMonitor(
        AprilTagPolicy(max_age_sec=1.0, confirm_window_sec=1.0, min_hits=3)
    )


def test_empty_detection_is_a_heartbeat_not_a_candidate():
    tags = monitor()
    result = tags.observe(ids=[], frame_key=(1, 0), now=10.0, task_active=True)
    assert result.state == "no_tag"
    assert not result.stop_now
    assert tags.message_age_sec(10.25) == 0.25
    assert tags.stream_ready(10.99)
    assert not tags.stream_ready(11.01)
```

- [ ] **Step 2: Write first-hit, deduplication, and per-ID tests**

Add:

```python
def test_first_candidate_stops_and_duplicate_frame_does_not_count_twice():
    tags = monitor()
    first = tags.observe(ids=[7, 7], frame_key=(2, 0), now=0.0, task_active=True)
    duplicate = tags.observe(ids=[7], frame_key=(2, 0), now=0.1, task_active=True)
    assert first.state == "verifying"
    assert first.stop_now
    assert dict(first.hit_counts) == {7: 1}
    assert not duplicate.stop_now
    assert dict(duplicate.hit_counts) == {7: 1}


def test_different_ids_do_not_combine_hits():
    tags = monitor()
    tags.observe(ids=[1], frame_key=1, now=0.0, task_active=True)
    tags.observe(ids=[2], frame_key=2, now=0.2, task_active=True)
    tags.observe(ids=[3], frame_key=3, now=0.4, task_active=True)
    result = tags.observe(ids=[], frame_key=4, now=1.0, task_active=True)
    assert result.state == "no_tag"
    assert result.false_positive
    assert result.confirmed_id is None


def test_inactive_task_records_heartbeat_without_starting_confirmation():
    tags = monitor()
    result = tags.observe(ids=[7], frame_key=1, now=0.0, task_active=False)
    assert result.state == "no_tag"
    assert not result.stop_now
    assert tags.stream_ready(0.0)


def test_nonempty_message_after_failed_window_starts_next_window_without_gap():
    tags = monitor()
    tags.observe(ids=[7], frame_key=1, now=0.0, task_active=True)
    result = tags.observe(ids=[7], frame_key=2, now=1.0, task_active=True)
    assert result.state == "verifying"
    assert result.false_positive
    assert result.stop_now
    assert dict(result.hit_counts) == {7: 1}
```

- [ ] **Step 3: Write full-window confirmation and latch tests**

Add:

```python
def test_same_id_three_frames_confirms_only_after_full_window():
    tags = monitor()
    tags.observe(ids=[7], frame_key=1, now=0.0, task_active=True)
    tags.observe(ids=[7], frame_key=2, now=0.1, task_active=True)
    early = tags.observe(ids=[7], frame_key=3, now=0.2, task_active=True)
    assert early.state == "verifying"
    assert tags.tick(now=0.99, task_active=True).state == "verifying"
    confirmed = tags.tick(now=1.0, task_active=True)
    assert confirmed.state == "confirmed"
    assert confirmed.just_confirmed
    assert confirmed.confirmed_id == 7
    assert tags.observe(ids=[], frame_key=4, now=1.1, task_active=True).state == "confirmed"


def test_reset_task_clears_latch_but_preserves_fresh_heartbeat():
    tags = monitor()
    for frame, now in ((1, 0.0), (2, 0.1), (3, 0.2)):
        tags.observe(ids=[7], frame_key=frame, now=now, task_active=True)
    tags.tick(now=1.0, task_active=True)
    tags.reset_task()
    assert tags.snapshot(now=1.0).state == "no_tag"
    assert tags.stream_ready(1.0)
```

- [ ] **Step 4: Run the new test file and verify RED**

Run: `python -m pytest test/test_apriltag_stop.py -q`

Expected: FAIL with `ModuleNotFoundError: apriltag_stop`.

- [ ] **Step 5: Implement the minimal state machine**

Create these public types in `tools/apriltag_stop.py`:

```python
from __future__ import annotations

from collections.abc import Hashable, Iterable
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AprilTagPolicy:
    max_age_sec: float = 1.0
    confirm_window_sec: float = 1.0
    min_hits: int = 3

    def validate(self) -> None:
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (self.max_age_sec, self.confirm_window_sec)
        ):
            raise ValueError("AprilTag timing limits must be positive and finite")
        if type(self.min_hits) is not int or self.min_hits <= 0:
            raise ValueError("AprilTag min_hits must be a positive integer")


@dataclass(frozen=True)
class AprilTagDecision:
    state: str
    stop_now: bool
    just_confirmed: bool
    false_positive: bool
    confirmed_id: int | None
    hit_counts: tuple[tuple[int, int], ...]
    window_elapsed_sec: float | None


class AprilTagStopMonitor:
    def __init__(self, policy: AprilTagPolicy | None = None) -> None:
        self.policy = policy or AprilTagPolicy()
        self.policy.validate()
        self._last_message_at: float | None = None
        self._window_started_at: float | None = None
        self._hits: dict[int, set[Hashable]] = {}
        self._confirmed_id: int | None = None

    def _decision(
        self,
        now: float,
        *,
        stop_now: bool = False,
        just_confirmed: bool = False,
        false_positive: bool = False,
    ) -> AprilTagDecision:
        state = (
            "confirmed"
            if self._confirmed_id is not None
            else "verifying"
            if self._window_started_at is not None
            else "no_tag"
        )
        elapsed = (
            None
            if self._window_started_at is None
            else max(now - self._window_started_at, 0.0)
        )
        return AprilTagDecision(
            state=state,
            stop_now=stop_now,
            just_confirmed=just_confirmed,
            false_positive=false_positive,
            confirmed_id=self._confirmed_id,
            hit_counts=tuple(
                (tag_id, len(frames))
                for tag_id, frames in sorted(self._hits.items())
            ),
            window_elapsed_sec=elapsed,
        )

    def _winner(self) -> int | None:
        winners = sorted(
            tag_id
            for tag_id, frames in self._hits.items()
            if len(frames) >= self.policy.min_hits
        )
        return winners[0] if winners else None

    def _start_window(
        self, ids: tuple[int, ...], frame_key: Hashable, now: float
    ) -> None:
        self._window_started_at = now
        self._hits = {tag_id: {frame_key} for tag_id in ids}

    def _clear_window(self) -> None:
        self._window_started_at = None
        self._hits = {}

    def observe(
        self,
        *,
        ids: Iterable[int],
        frame_key: Hashable,
        now: float,
        task_active: bool,
    ) -> AprilTagDecision:
        normalized = tuple(sorted({int(tag_id) for tag_id in ids}))
        self._last_message_at = now
        if not task_active:
            self.reset_task()
            return self.snapshot(now=now)
        if self._confirmed_id is not None:
            return self.snapshot(now=now)
        false_positive = False
        if (
            self._window_started_at is not None
            and now - self._window_started_at >= self.policy.confirm_window_sec
        ):
            winner = self._winner()
            if winner is not None:
                self._confirmed_id = winner
                self._clear_window()
                return self._decision(now, just_confirmed=True)
            self._clear_window()
            false_positive = True
        if self._window_started_at is None and normalized:
            self._start_window(normalized, frame_key, now)
            return self._decision(
                now,
                stop_now=True,
                false_positive=false_positive,
            )
        if self._window_started_at is not None:
            for tag_id in normalized:
                self._hits.setdefault(tag_id, set()).add(frame_key)
        return self._decision(now, false_positive=false_positive)

    def tick(self, *, now: float, task_active: bool) -> AprilTagDecision:
        if not task_active:
            self.reset_task()
            return self.snapshot(now=now)
        if (
            self._confirmed_id is None
            and self._window_started_at is not None
            and now - self._window_started_at >= self.policy.confirm_window_sec
        ):
            winner = self._winner()
            if winner is not None:
                self._confirmed_id = winner
                self._clear_window()
                return self._decision(now, just_confirmed=True)
        return self.snapshot(now=now)

    def snapshot(self, *, now: float) -> AprilTagDecision:
        return self._decision(now)

    def message_age_sec(self, now: float) -> float | None:
        if self._last_message_at is None:
            return None
        return max(now - self._last_message_at, 0.0)

    def stream_ready(self, now: float) -> bool:
        age = self.message_age_sec(now)
        return age is not None and age <= self.policy.max_age_sec

    def reset_task(self) -> None:
        self._window_started_at = None
        self._hits = {}
        self._confirmed_id = None
```

Use `dict[int, set[Hashable]]` for per-ID distinct-frame accounting. `observe`
always updates `_last_message_at`, including for empty arrays and inactive
tasks. `tick` may confirm a qualifying window at its deadline, but an
insufficient window does not resume until a post-deadline detection message
arrives; this prevents a non-zero command gap between adjacent non-empty
windows. When that post-deadline message is non-empty, close the false window
and start the next window in the same call.

Validate all timing values as positive finite numbers and `min_hits` as a
positive non-boolean integer.

- [ ] **Step 6: Run confirmation tests and verify GREEN**

Run: `python -m pytest test/test_apriltag_stop.py -q`

Expected: PASS.

- [ ] **Step 7: Commit the confirmation unit**

```bash
git add tools/apriltag_stop.py test/test_apriltag_stop.py
git commit -m "feat: add apriltag confirmation state machine"
```

---

### Task 3: Add Unitree `StopMove` Encoding

**Files:**
- Modify: `tools/unitree_sport_api.py`
- Modify: `test/test_unitree_sport_api.py`

**Interfaces:**
- Produces: `ROBOT_SPORT_API_ID_STOP_MOVE = 1003`.
- Produces: `populate_stop_move_request(request: Any) -> Any`.

- [ ] **Step 1: Write the failing request-encoding test**

Import the new names and add:

```python
def test_populate_stop_move_request_sets_unitree_contract():
    request = SimpleNamespace(
        header=SimpleNamespace(identity=SimpleNamespace(api_id=0)),
        parameter="",
        binary=[],
    )

    returned = populate_stop_move_request(request)

    assert returned is request
    assert ROBOT_SPORT_API_ID_STOP_MOVE == 1003
    assert request.header.identity.api_id == 1003
    assert request.parameter == "{}"
```

- [ ] **Step 2: Run the focused test and verify RED**

Run: `python -m pytest test/test_unitree_sport_api.py::test_populate_stop_move_request_sets_unitree_contract -q`

Expected: FAIL on the missing import.

- [ ] **Step 3: Implement `StopMove`**

Add to `tools/unitree_sport_api.py`:

```python
ROBOT_SPORT_API_ID_STOP_MOVE = 1003


def populate_stop_move_request(request: Any) -> Any:
    """Fill a generated request with one Unitree Sport StopMove command."""
    request.header.identity.api_id = ROBOT_SPORT_API_ID_STOP_MOVE
    request.parameter = "{}"
    return request
```

- [ ] **Step 4: Run the Sport API tests and verify GREEN**

Run: `python -m pytest test/test_unitree_sport_api.py -q`

Expected: PASS.

- [ ] **Step 5: Commit the encoder**

```bash
git add tools/unitree_sport_api.py test/test_unitree_sport_api.py
git commit -m "feat: encode unitree stop move requests"
```

---

### Task 4: Separate Task Acceptance from Dynamic Readiness

**Files:**
- Modify: `tools/cobiz_line_tracking_task.py`
- Modify: `test/test_cobiz_line_tracking_task.py`

**Interfaces:**
- Replaces: `LineTrackingTasks.handle_event(event, *, now, ready_reason)`.
- Produces: `LineTrackingTasks.handle_event(event, *, now, rejection_reason=None)`.
- Retains: `tick(*, now: float, drive_reason: str)` and
  `finish(state_type: str, reason: str | None = None)`.

- [ ] **Step 1: Update tests to express the new acceptance contract**

Change normal calls to pass `rejection_reason=None`, then replace the old
calibration-rejection test with:

```python
def test_dynamic_inputs_do_not_reject_a_valid_task():
    tasks = LineTrackingTasks()
    state = tasks.handle_event(event(), now=0, rejection_reason=None)
    assert state["type"] == "TASK_STARTED"
    assert tasks.active is not None


def test_static_control_conflict_rejects_without_activation():
    tasks = LineTrackingTasks()
    state = tasks.handle_event(
        event(), now=0, rejection_reason="multiple_control_publishers"
    )
    assert state["type"] == "TASK_REJECTED"
    assert state["reason"] == "multiple_control_publishers"
    assert tasks.active is None
```

- [ ] **Step 2: Add the startup deadline test**

```python
def test_startup_hold_aborts_at_deadline_when_never_ready():
    tasks = LineTrackingTasks(TaskPolicy(default_duration_sec=10, max_duration_sec=10))
    tasks.handle_event(event(), now=0, rejection_reason=None)
    assert tasks.tick(now=1.99, drive_reason="path_unavailable") is None
    stopped = tasks.tick(now=2.0, drive_reason="path_unavailable")
    assert stopped["type"] == "TASK_ABORTED"
    assert stopped["reason"] == "startup:path_unavailable"
```

Retain the transient post-start failure test but replace LiDAR reasons with
`path_unavailable` and `camera_stale`.

- [ ] **Step 3: Run lifecycle tests and verify RED**

Run: `python -m pytest test/test_cobiz_line_tracking_task.py -q`

Expected: FAIL because the method still requires `ready_reason` and rejects
dynamic unready states.

- [ ] **Step 4: Implement the new lifecycle behavior**

Change the signature to:

```python
def handle_event(
    self,
    event: Any,
    *,
    now: float,
    rejection_reason: str | None = None,
) -> dict[str, Any] | None:
```

Validate identity and payload before applying `rejection_reason`. If it is not
`None`, return `TASK_REJECTED`; otherwise activate the task regardless of
camera/path/detection state.

Change the start of `tick` to:

```python
elapsed = now - active.started_at
if elapsed < self.policy.startup_hold_sec:
    return None
if not self.tracking_seen and drive_reason != "tracking":
    return self.finish("TASK_ABORTED", f"startup:{drive_reason}")
```

Then retain the post-start unsafe timer and finite duration semantics.

- [ ] **Step 5: Run lifecycle tests and verify GREEN**

Run: `python -m pytest test/test_cobiz_line_tracking_task.py -q`

Expected: PASS.

- [ ] **Step 6: Commit the lifecycle change**

```bash
git add tools/cobiz_line_tracking_task.py test/test_cobiz_line_tracking_task.py
git commit -m "feat: start line tracking tasks in safe hold"
```

---

### Task 5: Remove LiDAR from Path, Drive, and Offline Replay

**Files:**
- Modify: `tools/local_path.py`
- Modify: `tools/swin_l_drive_control.py`
- Modify: `tools/swin_l_local_path_debug.py`
- Modify: `tools/swin_l_rosbag_overlay.py`
- Modify: `test/test_local_path.py`
- Modify: `test/test_swin_l_drive_control.py`
- Modify: `test/test_swin_l_rosbag_overlay.py`

**Interfaces:**
- Deletes: `LidarSafetyConfig`, `LidarSafetyResult`, `LidarSafetyMonitor`, `pointcloud2_xyz`, and `lidar_frame_matches_base`.
- Produces: `decide_drive(path, *, camera_age_sec, inference_age_sec, detections_ready, other_control_publishers, config)`.
- Produces: path overlay rendering with no safety object.

- [ ] **Step 1: Rewrite drive tests around the reduced signature**

Remove `_safety` and all LiDAR cases. Define:

```python
def _decide(path=None, **overrides):
    arguments = dict(
        camera_age_sec=0.1,
        inference_age_sec=0.1,
        detections_ready=True,
        other_control_publishers=False,
        config=DriveConfig(),
    )
    arguments.update(overrides)
    return decide_drive(path or _path(), **arguments)
```

The stop parameterization must include:

```python
[
    ({"detections_ready": False}, "apriltag_detections_stale"),
    ({"other_control_publishers": True}, "multiple_control_publishers"),
    ({"camera_age_sec": None}, "camera_stale"),
    ({"camera_age_sec": 0.6}, "camera_stale"),
    ({"inference_age_sec": 0.6}, "inference_stale"),
    ({"path": _path(age=0.6)}, "path_stale"),
    ({"path": _path(confidence=0.5)}, "path_low_confidence"),
    ({"path": _path(lateral=1.0)}, "path_lateral_target_large"),
]
```

- [ ] **Step 2: Delete LiDAR tests from local-path and replay tests**

Delete the PointCloud2 decoding and safety-monitor tests in
`test/test_local_path.py`. In `test/test_swin_l_rosbag_overlay.py`, assert
that local-path mode requests only the image topic and does not forward a
`--lidar-topic` argument.

Add a parser contract test:

```python
def test_active_parsers_expose_no_lidar_arguments():
    for argv in (["ros2"], ["task-drive"]):
        args = debug.parse_args(argv)
        assert not any("lidar" in name.lower() for name in vars(args))
        assert not hasattr(args, "safety_stop_topic")
        assert not hasattr(args, "clearance_topic")
```

- [ ] **Step 3: Run focused tests and verify RED**

Run: `python -m pytest test/test_local_path.py test/test_swin_l_drive_control.py test/test_swin_l_rosbag_overlay.py -q`

Expected: FAIL while LiDAR APIs and parser fields still exist.

- [ ] **Step 4: Delete LiDAR core types and reduce drive control**

Remove the LiDAR dataclasses, point-cloud decoder, and monitor from
`tools/local_path.py`. Remove the import and these `DriveConfig` fields from
`tools/swin_l_drive_control.py`:

```python
max_lidar_age_sec
min_clearance_m
```

Use this drive signature:

```python
def decide_drive(
    path: SmoothedPath | None,
    *,
    camera_age_sec: float | None,
    inference_age_sec: float | None,
    detections_ready: bool,
    other_control_publishers: bool,
    config: DriveConfig,
) -> DriveDecision:
```

Check `other_control_publishers`, `detections_ready`, camera age, inference
age, and the existing path conditions in that order. Delete `enabled` and
`calibrated`; task ownership is now the only motion-enablement boundary.

- [ ] **Step 5: Remove LiDAR from replay and overlay internals**

In `tools/swin_l_local_path_debug.py`:

- remove `_lidar_config_from_args`;
- remove LiDAR imports and arguments;
- make `run_mcap` read only camera messages;
- remove `safety` from `render_local_path_overlay` and replace the LiDAR line
  with an optional `status_text: str | None = None`; and
- remove `lidar_topic` and `lidar_safety` from reports.

In `tools/swin_l_rosbag_overlay.py`, remove `DEFAULT_LIDAR_TOPIC`,
`--lidar-topic`, and command forwarding for that argument. Keep both
segmentation and local-path overlay modes camera-only.

- [ ] **Step 6: Run focused tests and verify GREEN**

Run: `python -m pytest test/test_local_path.py test/test_swin_l_drive_control.py test/test_swin_l_rosbag_overlay.py -q`

Expected: PASS.

- [ ] **Step 7: Commit the LiDAR core removal**

```bash
git add tools/local_path.py tools/swin_l_drive_control.py tools/swin_l_local_path_debug.py tools/swin_l_rosbag_overlay.py test/test_local_path.py test/test_swin_l_drive_control.py test/test_swin_l_rosbag_overlay.py
git commit -m "refactor: remove lidar stop pipeline"
```

---

### Task 6: Integrate `/detections`, Hard Stop, and Task Completion

**Files:**
- Modify: `tools/swin_l_local_path_debug.py`
- Create: `test/test_apriltag_task_stop_ros.py`
- Modify: `test/test_cobiz_task_selected_mask_ros.py`
- Modify: `test/test_swin_l_debug_topics.py`

**Interfaces:**
- Consumes: `AprilTagStopMonitor`, `populate_stop_move_request`, and the reduced `decide_drive`.
- Produces: typed `/detections` subscription and ordered hard-stop publication.
- Produces CLI/env fields: `apriltag_detections_topic`, `apriltag_max_age_sec`, `apriltag_confirm_window_sec`, and `apriltag_confirm_min_hits`.

- [ ] **Step 1: Add parser and debug-surface tests**

Assert these task-drive defaults:

```python
args = debug.parse_args(["task-drive"])
assert args.apriltag_detections_topic == "/detections"
assert args.apriltag_max_age_sec == 1.0
assert args.apriltag_confirm_window_sec == 1.0
assert args.apriltag_confirm_min_hits == 3
assert not hasattr(args, "drive_enabled")
assert not hasattr(args, "calibration_confirmed")
```

Update `test_swin_l_debug_topics.py` so inspection mode publishes only:

```python
[args.local_path_topic, args.metrics_topic]
```

and task-drive mode has no safety/clearance topic.

- [ ] **Step 2: Adapt the selected-mask fake ROS test**

Provide `apriltag_msgs.msg.AprilTagDetectionArray` in the fake modules and
remove `PointCloud2`, `Bool`, `Float32`, and `LidarSafetyResult`. Change its
stub to return `(None, DriveDecision(0.1, 0.0, 0.0, "tracking"))` from `drive_readiness`, and change
task acceptance to the new `rejection_reason` contract. Keep assertions that
the selected mask reaches both preflight and control. The fake
`rclpy.qos` module must expose
`DurabilityPolicy.VOLATILE` in addition to the existing history and
reliability constants.

- [ ] **Step 3: Write a fake-ROS hard-stop integration test**

Create `test/test_apriltag_task_stop_ros.py` using the existing `FakeNode`,
`FakeQoS`, `Request`, and `Message` pattern. Capture subscriptions by topic and
published messages by topic. During fake `spin(node)`:

```python
clock = {"monotonic": 0.0}
monkeypatch.setattr(debug.time, "monotonic", lambda: clock["monotonic"])
node.on_task_event(Message(json.dumps({
    "type": "TASK_REGISTERED",
    "action_name": "LINE_TRACKING",
    "task_id": "tag-stop-1",
    "payload": {"duration_sec": 30},
})))
assert json.loads(published["/task_state"][-1].data)["type"] == "TASK_STARTED"

for frame, stamp in enumerate((0.0, 0.1, 0.2), start=1):
    clock["monotonic"] = stamp
    subscriptions["/detections"](detection_message(tag_id=7, frame=frame))

sport = published["/api/sport/request"]
assert [message.header.identity.api_id for message in sport[-2:]] == [1003, 1008]
assert json.loads(sport[-1].parameter) == {"x": 0.0, "y": 0.0, "z": 0.0}

clock["monotonic"] = 1.0
node.publish_state()
assert json.loads(published["/task_state"][-1].data)["type"] == "TASK_COMPLETED"
assert "apriltag_confirmed:7" in json.loads(published["/task_state"][-1].data)["reason"]
```

Also assert that no `Move` with a non-zero value is published while the tag
monitor state is `verifying`.

- [ ] **Step 4: Add false-positive and stale-stream integration tests**

Add one scenario with one candidate followed by empty arrays after the 1.0 s
window and assert that `tracking` commands may resume only after readiness is
healthy. Add another scenario that first establishes tracking, advances the
monotonic clock beyond 1.0 s with no detection callback, invokes
`publish_state`, and asserts ordered API IDs `[1003, 1008]` followed by
`TASK_ABORTED` reason `apriltag_detections_stale`.

- [ ] **Step 5: Run ROS-facing tests and verify RED**

Run: `python -m pytest test/test_apriltag_task_stop_ros.py test/test_cobiz_task_selected_mask_ros.py test/test_swin_l_debug_topics.py -q`

Expected: FAIL because `/detections` and hard-stop integration do not exist.

- [ ] **Step 6: Add task-drive arguments and typed imports**

In task-drive mode add:

```python
live.add_argument(
    "--apriltag-detections-topic",
    default=_env("SWIN_L_APRILTAG_DETECTIONS_TOPIC", "/detections"),
)
live.add_argument(
    "--apriltag-max-age-sec",
    type=float,
    default=_env_float("SWIN_L_APRILTAG_MAX_AGE_SEC", 1.0),
)
live.add_argument(
    "--apriltag-confirm-window-sec",
    type=float,
    default=_env_float("SWIN_L_APRILTAG_CONFIRM_WINDOW_SEC", 1.0),
)
live.add_argument(
    "--apriltag-confirm-min-hits",
    type=int,
    default=_env_int("SWIN_L_APRILTAG_CONFIRM_MIN_HITS", 3),
)
```

Import `AprilTagDetectionArray` only inside `run_ros2` with the other generated
ROS types so offline tooling remains importable without ROS installed.

- [ ] **Step 7: Build the monitor, subscription, and frame key**

Construct `AprilTagPolicy` from the four arguments and create a reliable,
volatile, depth-1 subscription in task-drive mode. The callback must derive:

```python
apriltag_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
self.apriltag_subscription = self.create_subscription(
    AprilTagDetectionArray,
    args.apriltag_detections_topic,
    self.on_apriltag_detections,
    apriltag_qos,
)

stamp_ns = _stamp_ns(message.header)
self.apriltag_callback_sequence += 1
frame_key = stamp_ns if stamp_ns > 0 else self.apriltag_callback_sequence
ids = [int(detection.id) for detection in message.detections]
decision = self.apriltags.observe(
    ids=ids,
    frame_key=frame_key,
    now=time.monotonic(),
    task_active=self.tasks.active is not None,
)
```

Call hard stop immediately when `decision.stop_now` is true.

- [ ] **Step 8: Implement ordered hard stop and terminal handling**

Add methods with these responsibilities:

```python
def publish_zero_move(self, reason: str) -> None:
    self.publish_drive(DriveDecision.stop(reason))

def publish_hard_stop(self, reason: str) -> None:
    if self.command_publisher is None:
        return
    assert Request is not None
    self.command_publisher.publish(populate_stop_move_request(Request()))
    self.publish_zero_move(reason)

def complete_apriltag_task(self, tag_id: int) -> None:
    self.publish_hard_stop(f"apriltag_confirmed:{tag_id}")
    body = self.tasks.finish(
        "TASK_COMPLETED", f"apriltag_confirmed:{tag_id}"
    )
    if body is not None:
        self.publish_task_state(body)
    self.stop_until = time.monotonic() + 1.0
```

`complete_apriltag_task` must call `publish_hard_stop`, then
`tasks.finish("TASK_COMPLETED", f"apriltag_confirmed:{tag_id}")`, publish the
task state, and enter the existing one-second zero-command release hold.

Refactor `publish_drive` to remain Move-only. Do not represent `StopMove` as a
`DriveDecision`.

- [ ] **Step 9: Integrate readiness and urgent faults**

Make `drive_readiness` return `(path, decision)` and pass
`self.apriltags.stream_ready(now)` to `decide_drive`.

At task registration, compute only static rejection reasons:

- `control_release_pending` when the previous publisher is still held; or
- `multiple_control_publishers` when a competing Sport publisher exists.

Do not reject for detection, camera, inference, or path readiness.

In `publish_state`, apply this order:

1. advance tag confirmation and complete if newly confirmed;
2. abort immediately on a new competing publisher;
3. if tracking was previously established and detections are stale, hard stop
   and abort immediately;
4. force zero `apriltag_verifying` while verifying and do not call
   `tasks.tick` until that bounded verification window resolves;
5. otherwise apply startup hold and normal path decision;
6. pass the resulting reason to `tasks.tick`.

Reset only confirmation state when a task starts or releases; preserve the
detection heartbeat across tasks. Use hard stop for server abort, SIGTERM,
detection loss, competing publisher, tag candidate, and confirmed tag.

- [ ] **Step 10: Add tag metrics and overlay text**

Replace LiDAR metrics with:

```python
"apriltag": {
    "topic": args.apriltag_detections_topic,
    "stream_ready": self.apriltags.stream_ready(now),
    "message_age_sec": self.apriltags.message_age_sec(now),
    "state": tag_status.state,
    "hit_counts": dict(tag_status.hit_counts),
    "confirmed_id": tag_status.confirmed_id,
    "window_elapsed_sec": tag_status.window_elapsed_sec,
}
```

Use the `message_age_sec(now) -> float | None` method and its heartbeat test
from Task 2. Pass text such as
`APRILTAG VERIFY 2/3` or `APRILTAG CONFIRMED ID 7` to the overlay renderer.

- [ ] **Step 11: Run ROS-facing and core tests and verify GREEN**

Run:

```bash
python -m pytest \
  test/test_apriltag_stop.py \
  test/test_apriltag_task_stop_ros.py \
  test/test_cobiz_task_selected_mask_ros.py \
  test/test_swin_l_debug_topics.py \
  test/test_swin_l_drive_control.py \
  test/test_cobiz_line_tracking_task.py -q
```

Expected: PASS.

- [ ] **Step 12: Commit runtime integration**

```bash
git add tools/apriltag_stop.py tools/swin_l_local_path_debug.py test/test_apriltag_stop.py test/test_apriltag_task_stop_ros.py test/test_cobiz_task_selected_mask_ros.py test/test_swin_l_debug_topics.py
git commit -m "feat: stop line tracking on confirmed apriltags"
```

---

### Task 7: Update Deployment Configuration and Operator Documentation

**Files:**
- Modify: `docker-compose.yml`
- Modify: `.env.example`
- Modify: `README.md`
- Modify: `test/test_deployment_contract.py`

**Interfaces:**
- Produces the four `SWIN_L_APRILTAG_*` settings from the design.
- Deletes all manual-arm and LiDAR settings/topics from the active deployment contract.

- [ ] **Step 1: Write failing Compose and environment contract tests**

Update `test_default_compose_is_cobiz_task_listener`:

```python
assert listener["environment"]["SWIN_L_APRILTAG_DETECTIONS_TOPIC"].endswith(
    "/detections}"
)
assert listener["environment"]["SWIN_L_APRILTAG_MAX_AGE_SEC"].endswith(":-1.0}")
assert listener["environment"]["SWIN_L_APRILTAG_CONFIRM_WINDOW_SEC"].endswith(
    ":-1.0}"
)
assert listener["environment"]["SWIN_L_APRILTAG_CONFIRM_MIN_HITS"].endswith(
    ":-3}"
)
```

Add a repository contract:

```python
def test_active_deployment_has_no_manual_arm_or_lidar_contract():
    active = "\n".join(
        (ROOT / path).read_text()
        for path in ("docker-compose.yml", ".env.example")
    )
    for forbidden in (
        "SWIN_L_DRIVE_ENABLED",
        "SWIN_L_CALIBRATION_CONFIRMED",
        "SWIN_L_LIDAR_",
        "SWIN_L_SAFETY_STOP_TOPIC",
        "SWIN_L_CLEARANCE_TOPIC",
    ):
        assert forbidden not in active
```

- [ ] **Step 2: Run deployment tests and verify RED**

Run: `python -m pytest test/test_deployment_contract.py -q`

Expected: FAIL on old arm/LiDAR settings and absent AprilTag settings.

- [ ] **Step 3: Change Compose and `.env.example`**

Remove all LiDAR, safety-stop, clearance, drive-enabled, and
calibration-confirmed environment entries. Add to `actual-activate`:

```yaml
SWIN_L_APRILTAG_DETECTIONS_TOPIC: ${SWIN_L_APRILTAG_DETECTIONS_TOPIC:-/detections}
SWIN_L_APRILTAG_MAX_AGE_SEC: ${SWIN_L_APRILTAG_MAX_AGE_SEC:-1.0}
SWIN_L_APRILTAG_CONFIRM_WINDOW_SEC: ${SWIN_L_APRILTAG_CONFIRM_WINDOW_SEC:-1.0}
SWIN_L_APRILTAG_CONFIRM_MIN_HITS: ${SWIN_L_APRILTAG_CONFIRM_MIN_HITS:-3}
```

Document the same literal defaults in `.env.example`.

- [ ] **Step 4: Rewrite the README around the new runtime contract**

Keep the FP16/Jetson build and camera/path calibration guidance, but replace
all LiDAR STOP and manual-arm passages with:

- teamgrit-slam must run with tag detection enabled;
- `/detections` empty arrays are the detector heartbeat;
- a valid COBIZ payload starts a two-second zero hold;
- motion needs fresh camera, inference, path, and detection heartbeat;
- first tag candidate hard-stops immediately;
- one second plus three same-ID frames completes the task;
- one/two-hit false positives may resume when path health returns;
- stale `/detections` after tracking hard-stops and aborts; and
- this service no longer supplies obstacle avoidance.

Update topic tables and commands to show `/detections` and remove
`/unitree/slam_lidar/*`, `safety_stop`, and `clearance_m`.

- [ ] **Step 5: Run deployment tests and static contract scans**

Run:

```bash
python -m pytest test/test_deployment_contract.py -q
rg -n -i 'SWIN_L_DRIVE_ENABLED|SWIN_L_CALIBRATION_CONFIRMED|SWIN_L_LIDAR_|PointCloud2|safety_stop|clearance_m|LIDAR STOP' \
  tools test docker-compose.yml .env.example README.md Dockerfile.swin-l-debug
```

Expected: pytest PASS; `rg` returns no matches.

- [ ] **Step 6: Commit deployment and docs**

```bash
git add docker-compose.yml .env.example README.md test/test_deployment_contract.py
git commit -m "docs: deploy line tracking with apriltag heartbeat"
```

---

### Task 8: Full Verification, On-Device Checklist, and Push

**Files:**
- Modify only files required by failures found in this task.
- Verify: all changed files and branch history.

**Interfaces:**
- Consumes all prior task deliverables.
- Produces a clean, pushed `codex/fp16-direct-sport-api` branch.

- [ ] **Step 1: Run formatting and static checks**

Run:

```bash
python -m ruff check tools test
python -m compileall -q tools test
git diff --check
```

Expected: all commands exit 0.

- [ ] **Step 2: Run the full test suite**

Run: `python -m pytest -q`

Expected: all tests pass with no skips newly introduced for this feature.

- [ ] **Step 3: Validate Compose expansion**

Run: `docker compose config --quiet`

Expected: exit 0. Do not build or run the Jetson image on this non-Jetson PC.

- [ ] **Step 4: Prove active LiDAR and manual-arm contracts are gone**

Run:

```bash
rg -n -i 'SWIN_L_DRIVE_ENABLED|SWIN_L_CALIBRATION_CONFIRMED|SWIN_L_LIDAR_|PointCloud2|safety_stop|clearance_m|LIDAR STOP' \
  tools test docker-compose.yml .env.example README.md Dockerfile.swin-l-debug
```

Expected: no matches. Historical design/spec files and generated validation
artifacts are intentionally outside this scan.

- [ ] **Step 5: Review the final diff and commit verification fixes**

Run:

```bash
git status --short
git diff --stat origin/codex/fp16-direct-sport-api...HEAD
git diff origin/codex/fp16-direct-sport-api...HEAD -- tools test Dockerfile.swin-l-debug docker-compose.yml .env.example README.md third_party/apriltag_msgs
```

Expected: the worktree is clean because each prior task committed its own
verified deliverable. If this review finds a failure, stop before pushing,
return to the owning task, add a failing regression test, fix it, rerun the
task checks, and commit that task's explicit file list.

- [ ] **Step 6: Push the branch**

Run: `git push origin codex/fp16-direct-sport-api`

Expected: the remote branch advances to the verified local HEAD.

- [ ] **Step 7: Record mandatory Jetson/A2 validation as pending**

The local PC cannot close these hardware checks. Report them explicitly:

```bash
ros2 topic type /detections
ros2 topic info -v /detections
ros2 topic echo /detections --once
ros2 topic info -v /api/sport/request
```

Expected on A2:

- `/detections` type is `apriltag_msgs/msg/AprilTagDetectionArray`;
- empty arrays arrive continuously when no tag is visible;
- publisher/subscriber QoS is compatible;
- no competing Sport publisher exists during the test;
- first tag candidate produces immediate `1003` then zero `1008`;
- three same-ID frames complete the task after the full one-second window;
- stopping teamgrit-slam aborts within the one-second freshness bound; and
- removing a confirmed tag never resumes the completed task.
