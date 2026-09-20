# AprilTag-Gated Line-Tracking Design

Date: 2026-09-20

## Purpose

Change the COBIZ `LINE_TRACKING` service so that a valid server task can drive
the A2 through Unitree Sport API requests without manual arm environment
variables, while an AprilTag reported by teamgrit-slam stops and completes the
task after false-positive confirmation.

The change also removes the LiDAR STOP subsystem in full. After this change,
LiDAR is not subscribed, evaluated, published in metrics, or required for
motion.

## System boundary

teamgrit-slam owns image-based AprilTag detection and publishes
`/detections`. Line-tracking owns:

- the COBIZ task lifecycle;
- detection-stream liveness monitoring;
- false-positive confirmation;
- Unitree `StopMove` and `Move` requests; and
- the decision to complete or abort the active task.

teamgrit-slam never publishes robot motion commands or task states.
Line-tracking does not run another AprilTag detector and does not subscribe to
a second camera stream for AprilTag processing.

```text
teamgrit-slam
  camera -> AprilTag detector -> /detections
                                   |
                                   | apriltag_msgs/AprilTagDetectionArray
                                   v
cobiz-plugin-line-tracking
  /task_event -> task lifecycle -> path control -> /api/sport/request
                       |
                       `-> tag confirmation -> STOPMOVE -> zero Move
                                             -> TASK_COMPLETED
```

This makes detection dependent on the teamgrit-slam process, but keeps the
stop policy and robot control independent from the SLAM/localization logic.

## ROS interface packaging

The Line-tracking repository will vendor a minimal, buildable copy of the
exact `apriltag_msgs` interface used by teamgrit-slam under:

```text
third_party/apriltag_msgs/apriltag_msgs/
  CMakeLists.txt
  package.xml
  LICENSE
  msg/Point.msg
  msg/AprilTagDetection.msg
  msg/AprilTagDetectionArray.msg
```

The message package name and all field definitions must remain byte-for-byte
compatible with the teamgrit-slam copy. Its source revision and license will
be preserved. The Docker build will build it with `colcon` alongside the
vendored `unitree_api` messages and source the resulting install space at
runtime.

Line-tracking will not install `apriltag_ros`, an AprilTag C library, or an
AprilTag detector executable.

The runtime subscription contract is:

- default topic: `/detections`;
- type: `apriltag_msgs/msg/AprilTagDetectionArray`;
- QoS: reliable, volatile, depth 1; and
- configurable topic name:
  `SWIN_L_APRILTAG_DETECTIONS_TOPIC=/detections`.

The two containers must use the same ROS domain and compatible DDS networking.
Humble/Jazzy interoperability is accepted only after the on-robot topic type
and QoS checks in the validation section pass.

## Readiness definitions

Topic discovery alone is never readiness. A publisher visible in the ROS
graph but producing no data is treated as unavailable.

### Detection readiness

The detection stream is ready when:

1. at least one typed `AprilTagDetectionArray` callback has completed since
   Line-tracking started; and
2. the last callback was received no more than 1.0 second ago.

An empty `detections` array means "detector alive, no tag" and satisfies
readiness. A non-empty array satisfies liveness and also starts or advances
tag confirmation.

Liveness uses the receiver's monotonic clock. It does not depend on
`header.stamp`, because the publisher and subscriber may use different ROS
clock configurations.

Default configuration:

```text
SWIN_L_APRILTAG_MAX_AGE_SEC=1.0
```

### Camera and inference readiness

The camera is ready only after a ROS image was successfully converted and the
source timestamp passed the existing live and strictly increasing checks. The
effective camera age must be at most 0.50 seconds.

Inference is independently ready only after a successful segmentation result
whose effective age is at most 0.50 seconds.

### Local-path readiness

A path is ready only when all of the following hold:

- a `SmoothedPath` exists;
- its age is at most 0.45 seconds;
- confidence is at least 0.70;
- there are at least two finite `(x, y)` points;
- forward `x` values are strictly increasing;
- the first point is ahead of the robot and no farther than the 4.0 m
  lookahead;
- the path reaches at least the 4.0 m lookahead; and
- interpolated lateral displacement at 4.0 m is at most 0.75 m in magnitude.

The smoother may retain a path longer for visualization, but a retained path
older than the drive threshold is not motion-ready.

### Control ownership

The current check for competing `/api/sport/request` publishers remains.
A task is rejected if another motion publisher already owns that topic. If a
new competing publisher appears during a task, Line-tracking performs a hard
stop and aborts the task.

## COBIZ task acceptance and motion start

The manual runtime gates are removed:

- `SWIN_L_DRIVE_ENABLED`; and
- `SWIN_L_CALIBRATION_CONFIRMED`.

A valid, non-conflicting COBIZ `TASK_REGISTERED` event for `LINE_TRACKING`
authorizes Line-tracking to acquire the Sport request publisher. Dynamic
sensor readiness is no longer an immediate task-rejection condition.

On acceptance Line-tracking:

1. publishes `TASK_STARTED` through the existing task-state path;
2. creates the direct `/api/sport/request` publisher;
3. publishes zero `Move`; and
4. enters a 2.0 second startup hold.

Non-zero motion starts only when detection, camera, inference, local-path, and
control-ownership readiness all pass. If they do not all pass within the
startup hold, Line-tracking performs a hard stop and publishes `TASK_ABORTED`
with the failing readiness reason.

After motion has started, transient camera, inference, or path failure holds
zero speed and uses the existing 2.0 second unsafe timeout. Detection-stream
staleness and competing control publishers are stronger faults: they cause an
immediate hard stop and task abort, without automatic recovery.

## AprilTag confirmation state machine

AprilTag confirmation is implemented as a ROS-independent pure-Python unit so
its timing and deduplication can be tested without a robot.

The state machine is scoped to the active `LINE_TRACKING` task and has three
states:

```text
NO_TAG -- first non-empty message --> VERIFYING
VERIFYING -- no ID reaches threshold --> NO_TAG
VERIFYING -- one ID reaches threshold --> CONFIRMED (latched)
```

Defaults:

```text
SWIN_L_APRILTAG_CONFIRM_WINDOW_SEC=1.0
SWIN_L_APRILTAG_CONFIRM_MIN_HITS=3
```

Any detection item emitted by teamgrit-slam is a candidate. Upstream
teamgrit-slam configuration remains the source of truth for families, IDs,
and hamming limits; Line-tracking does not duplicate its allowlist.

### First candidate

On the first candidate in an active task, Line-tracking immediately:

1. publishes Unitree `StopMove` API ID `1003` with parameter `{}`;
2. publishes Unitree `Move` API ID `1008` with zero velocity;
3. suppresses all non-zero commands; and
4. opens a 1.0 second verification window.

Zero `Move` continues at the normal control cadence throughout verification.

If the task starts while the latest fresh detection array is already
non-empty, it enters verification before any non-zero command is allowed.

### Hit counting

Hits are counted per tag ID. One ID appearing multiple times in one array is
one hit. Different IDs are never added together.

Distinct messages use `header.stamp` as the frame key. When the stamp is zero,
the subscriber callback sequence is used as the fallback key. Replayed or
duplicate callbacks with an already-counted key do not add hits.

The full 1.0 second window is observed even if three hits arrive early. This
preserves the requested deliberate false-positive check rather than
completing after only a few fast frames.

### Confirmation

At the end of the window, confirmation succeeds when any one ID appeared in
at least three distinct messages. Line-tracking then:

1. repeats `StopMove` followed by zero `Move`;
2. publishes `TASK_COMPLETED` for the active `LINE_TRACKING` task with the
   confirmed tag ID in the reason/metrics;
3. keeps zero speed during the existing control-release hold; and
4. destroys the Sport publisher.

The confirmed state is latched until the task is released. A tag disappearing
cannot restart the completed task.

### False positive

If no ID reaches three hits by the end of the window, the candidate is cleared.
The same task may resume only after all normal motion-readiness checks pass.

If another non-empty array arrives as the old window expires, a new window
starts without allowing a non-zero command between windows.

If task duration expires during verification, lifecycle completion waits for
the bounded verification window. A confirmed tag completes with the tag
reason. An unconfirmed window falls back to the normal duration-end result.

## Robot stop operations

`tools/unitree_sport_api.py` will expose separate encoders for:

- `StopMove`: API ID `1003`, JSON parameter `{}`; and
- `Move`: API ID `1008`, JSON velocity payload.

A "hard stop" means ordered publication of `StopMove` and then zero `Move`.
It is used for:

- the first AprilTag candidate;
- confirmed AprilTag completion;
- detection-stream loss during an active task;
- competing control publisher detection; and
- unrecoverable task abort/release paths.

Routine zero holds may publish zero `Move` without repeating `StopMove` on
every timer tick.

## Removal of LiDAR STOP

The implementation removes, rather than disables, the LiDAR safety subsystem:

- no `PointCloud2` import or subscription;
- no `LidarSafetyConfig`, `LidarSafetyMonitor`, or `LidarSafetyResult`;
- no point-cloud decoder used by Line-tracking;
- no LiDAR age, frame, clearance, corridor, or obstacle checks;
- no LiDAR CLI arguments or `SWIN_L_LIDAR_*` environment variables;
- no `safety_stop` or `clearance_m` debug publishers;
- no LiDAR section in runtime metrics;
- no LiDAR input or `LIDAR STOP` state in rosbag overlay tooling; and
- no LiDAR deployment instructions or LiDAR-specific tests.

Generated historical validation artifacts are not rewritten, but active code,
configuration, tests, and current documentation must contain no LiDAR STOP
contract.

This explicitly removes obstacle-stop protection from this service. AprilTag
completion is not an obstacle-avoidance replacement.

## Component changes

### `third_party/apriltag_msgs`

Provide the exact generated ROS interface source and licensing needed for a
typed Python subscription.

### `tools/apriltag_stop.py`

Own the confirmation policy, per-ID hit accounting, frame deduplication,
window expiration, and latched result. It has no ROS imports.

### `tools/unitree_sport_api.py`

Add the `StopMove` request constant and encoder while retaining the existing
clamped `Move` encoder.

### `tools/swin_l_drive_control.py`

Remove LiDAR input from `decide_drive`. Keep camera, inference, path geometry,
speed clamping, and exclusive-publisher checks.

### `tools/swin_l_local_path_debug.py`

Remove all LiDAR state and ROS surfaces. Add the typed `/detections`
subscription, detection heartbeat state, tag confirmation integration, hard
stop sequencing, and task-lifecycle transitions.

### `tools/cobiz_line_tracking_task.py`

Separate static task acceptance from dynamic motion readiness so a valid
payload can start in a zero-speed hold. Preserve one-task-at-a-time handling,
duration validation, unsafe timeout, and server cancellation behavior.

### Deployment and documentation

Build and source `apriltag_msgs`, expose only the four AprilTag settings in
this design, delete LiDAR settings, delete manual arm flags, and document the
teamgrit-slam runtime prerequisite.

## Observability

Runtime metrics and the debug overlay will expose:

- detection stream received/not received and receive age;
- tag state: `no_tag`, `verifying`, or `confirmed`;
- candidate IDs and per-ID hit counts;
- verification window elapsed/remaining time;
- confirmed ID;
- motion-readiness reason; and
- task-state reason.

The overlay may show text such as `APRILTAG VERIFY 2/3` or
`APRILTAG CONFIRMED ID 7`. It will not draw a tag box unless image and
detection frame timestamps are explicitly matched; textual state is the
required behavior.

## Test strategy

Implementation follows test-driven development.

### Unit tests

- `StopMove` uses API ID `1003` and parameter `{}`.
- A first candidate requests an immediate hard stop.
- Duplicate IDs in one array count once.
- A duplicate frame key does not count twice.
- Different IDs do not combine toward the threshold.
- Three distinct frames for one ID within the window confirm only after the
  full 1.0 second window.
- One or two hits expire as a false positive.
- Confirmation remains latched after empty arrays.
- Empty arrays update liveness but do not start verification.
- Detection liveness is false before the first callback and after 1.0 second.
- Drive decisions no longer accept or inspect LiDAR data.
- Existing camera, inference, path, geometry, speed, and publisher gates
  remain enforced.
- A valid COBIZ task starts in zero hold without manual arm flags.
- A false positive permits resume only with healthy inputs.
- Confirmation produces `TASK_COMPLETED` and cannot resume.
- Detection staleness during a task produces hard stop plus `TASK_ABORTED`.

### Deployment-contract tests

- Docker builds both `unitree_api` and `apriltag_msgs`.
- The entrypoint sources the interface workspace.
- Compose contains the detection topic and confirmation settings.
- Compose and `.env.example` contain no manual arm or LiDAR variables.
- Runtime source contains no `PointCloud2` subscription or LiDAR debug topic.

### Regression tests

- Run the complete Python test suite.
- Run lint/format checks used by the repository.
- Run the existing FP16 segmentation and local-path tests to ensure the model
  and path calculation are unchanged.
- Use camera-only rosbag overlay generation to verify that removal of LiDAR
  does not change segmentation or path rendering.

## A2 integration validation

On the robot, before a moving test:

1. verify `/detections` has type
   `apriltag_msgs/msg/AprilTagDetectionArray`;
2. verify publisher and subscriber QoS with `ros2 topic info -v`;
3. verify empty arrays arrive continuously with no visible tag;
4. verify both containers use the intended `ROS_DOMAIN_ID` and DDS network;
5. verify no competing `/api/sport/request` publisher exists; and
6. perform all initial command checks with the robot lifted or otherwise made
   physically safe.

Behavioral acceptance cases:

1. A valid COBIZ payload starts the task and begins low-speed tracking after
   all readiness gates pass.
2. One or two tag frames cause immediate stop, then resume after the complete
   confirmation window if inputs remain healthy.
3. Three frames of the same ID cause immediate stop followed by
   `TASK_COMPLETED` after the full window.
4. Removing the confirmed tag never restarts the completed task.
5. Stopping teamgrit-slam or `/detections` causes a hard stop and
   `TASK_ABORTED` within the 1.0 second freshness bound.
6. A competing Sport publisher prevents or aborts motion.
7. No LiDAR topic is required for startup or motion.

## Success criteria

The change is complete when all automated tests pass, the container exposes
the vendored `apriltag_msgs` type, a COBIZ task can drive without manual arm
flags, LiDAR STOP has been fully removed, and the A2 integration cases above
have been demonstrated or explicitly left as on-device validation steps when
robot hardware is unavailable.
