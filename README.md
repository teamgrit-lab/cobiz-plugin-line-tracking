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

A single `.env` switch bypasses path-quality stops and briefly tolerates path loss:

```dotenv
LINE_TRACKING_BYPASS_PATH_STOPS=false
```

Set it to `true` to temporarily bypass `path_unavailable` and bypass
`path_low_confidence` (including NaN/Inf confidence) and
`path_lateral_target_large`. With usable coordinates,
tracking still uses the computed yaw, capped at 0.18 rad/s. If the path is
missing or has no usable coordinates, the controller holds the last valid
yaw from the current task and continues at the configured forward speed.
Until a valid target has been obtained in that task, it still stops with
`path_unavailable`; it does not invent an initial heading.

Only the first four consecutive completed inferences with an unavailable path
can use the saved yaw. On the **fifth consecutive unavailable result**, the next
control tick sends zero velocity with `path_unavailable`, even with the bypass
enabled. If that unsafe state then lasts `LINE_TRACKING_UNSAFE_TIMEOUT_SEC`
(default 2 seconds), the task aborts with `unsafe:path_unavailable`. A usable
path before the abort resets the counter to zero and resumes tracking.

The counter follows the task's selected mask (`0`, `1`, or `2`). It increments
once per completed inference whose resulting path has no usable target, never
per camera callback or 10 Hz output tick. Intervening valid results reset it
even when no control tick occurs between results. A new task starts at zero.
In restricted mode, a still-usable path retained by the smoother is not a
missing-path result; after it expires, unavailable inference results count.
Five failures is an inference count, not a five-second timeout. Independently,
a camera/inference check failure or another stop/lifecycle condition ends motion.
Camera/inference faults clear the saved yaw, as does ending or starting a task.
The four camera/inference stops (`camera_stale`, `inference_stale`,
`camera_timestamp_invalid`, and `camera_conversion_error`) always remain active.
AprilTag policy, startup hold, explicit cancellation, task duration, faults,
shutdown, and speed limits also remain active.

The master switch overrides the two individual path-quality switches below;
it does not override the AprilTag switch. Its default is `false`. Rebuild the
image with this code and recreate the service after changing the setting.
Metrics expose `path_stop_bypass`, `path_yaw_held`,
`path_unavailable_inferences` (the consecutive count), and
`path_unavailable_limit` (`5`). Held-yaw motion reports
`tracking_path_hold` and can continue even while `path_tracked` is false and
the published Path is empty. The task treats held-yaw motion as permitted
motion during failures 1–4, so those held-yaw intervals do not start the unsafe
timeout. `stop_checks.path_available` becomes true at the fifth failure.
This setting does not correct the steering sign conversion.

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

The default duration is 500 seconds. Requests above 10000 seconds are capped at
10000 seconds. Existing deployments with `LINE_TRACKING_MAX_DURATION_SEC=1000`
in `.env` must change it to `10000` and recreate the service to apply the new limit.
The listener
reports task state on `/task_state`; core owns any corresponding HTTP report.
It sends stop commands on server cancellation, `SIGTERM`, publish errors, stale
required camera/inference inputs, an unavailable path that cannot use the
limited yaw hold, or tag confirmation.
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
| Missing path or no usable numeric x/y points | With the master bypass and a saved yaw, hold through failures 1–4; the fifth consecutive unavailable inference stops with `path_unavailable`. Without bypass or a saved yaw, stop immediately. |
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

## 정지·작업 중단·시작 거절 사유

아래는 `actual-activate`의 Python `task-drive` 코드 기준이다. 주행 판단의
`drive_reason`과 `ready_reason`은 `/line_tracking/swin_l/metrics`에서,
작업 결과의 `type`과 `reason`은 `/task_state`에서 확인한다.
`drive_reason`은 시작 대기·AprilTag 확인 등까지 반영한 출력 판단이고,
`ready_reason`은 카메라·추론·Path 검사 결과다. 정지 명령 전송과 작업 중단은
별개이며, 아래의 "즉시 정지"는 다음 제어 주기(기본 10 Hz)에 0 속도를 보내는
동작이다. 카메라 콜백의 오류와 AprilTag 검출은 콜백에서 바로 정지를 요청한다.

### 주행 중 정지 조건

| 사유 | 발생 조건 | 정지 우회 설정의 영향 |
|---|---|---|
| `camera_stale` | 카메라 수신 또는 원본 센서 시각 기준 나이가 5초 초과. 수신 이력이 없거나 계산한 나이가 음수·NaN·Inf여도 정지 | 항상 검사. 저장한 회전 명령도 삭제 |
| `inference_stale` | 마지막 추론 완료 시각 또는 추론에 사용한 원본 이미지 시각 기준 나이가 5초 초과. 이력이 없거나 나이가 잘못된 경우도 포함 | 항상 검사. 저장한 회전 명령도 삭제 |
| `camera_timestamp_invalid` | 이미지 시각이 0 이하, 현재보다 50 ms 초과 미래, 5초 초과 과거, 또는 직전 수락한 이미지보다 같거나 이전 시각 | 콜백에서 0 속도 전송, 카메라 유효 상태와 저장 회전 명령 삭제. 이후 제어 주기에는 보통 `camera_stale`로 표시 |
| `camera_conversion_error` | 카메라 메타데이터 검사 또는 선택 프레임 변환 중 예외 | 콜백 또는 워커에서 0 속도 전송, 카메라 유효 상태와 저장 회전 명령 삭제. 이후 보통 `camera_stale`로 표시 |
| `path_unavailable` | 선택한 클래스의 Path가 없거나 유한한 x/y 목표점을 만들 수 없음 | 기본은 즉시 정지. `LINE_TRACKING_BYPASS_PATH_STOPS=true`이고 현재 작업의 저장 회전 명령이 있으면 연속 실패 1–4회만 유지. **5회째부터 정지**. 저장 명령이 없으면 첫 실패부터 정지 |
| `path_low_confidence` | 신뢰도가 NaN/Inf이거나 활성 임계값 0.49 미만 | 전체 Path 우회가 켜지면 검사 생략. 개별 `LINE_TRACKING_STOP_ON_LOW_CONFIDENCE=false` 또는 unrestricted 모드는 유한한 저신뢰도만 허용하며 NaN/Inf는 계속 정지 |
| `path_lateral_target_large` | 전방 4 m 기준 목표점의 좌우 좌표 절댓값이 0.75 m 초과 | 전체 Path 우회 또는 `LINE_TRACKING_STOP_ON_LATERAL_TARGET=false`로 생략. 회전 속도 제한 ±0.18 rad/s는 유지 |
| `apriltag_verifying` | AprilTag 후보 검출 후 확인 중 | `LINE_TRACKING_STOP_ON_APRILTAG=true`일 때 `StopMove`와 0 속도 전송. Path 우회와 무관. 기본 1초 확인 창에서 같은 ID가 서로 다른 프레임 3개에 검출되면 정상 완료 |

### 정지가 작업 중단으로 이어지는 조건

| `/task_state.reason` | 조건과 결과 |
|---|---|
| `startup:<사유>` | 시작 후 2초 대기가 끝났는데 추종 또는 허용된 회전 유지가 한 번도 시작되지 못한 경우 `TASK_ABORTED` |
| `unsafe:<사유>` | 추종을 시작한 뒤 허용되지 않는 상태가 연속 `LINE_TRACKING_UNSAFE_TIMEOUT_SEC`(기본 2초) 지속되면 `TASK_ABORTED`. 이유가 바뀌어도 그 사이 정상 추종/허용된 회전 유지가 없으면 타이머는 계속 진행 |
| `tracking_unavailable:<사유>` | 작업 제한 시간에 도달했지만 정상 완료 조건(이전에 추종했고 현재도 추종/허용된 회전 유지 중)을 만족하지 못하면 `TASK_ABORTED`. 앞의 startup/unsafe 조건이 먼저 충족되면 그 사유가 우선 |
| `task_aborted_by_server` | 현재 task ID에 대한 서버의 `TASK_ABORTED` 이벤트 수신. 즉시 정지 요청 후 작업 중단 |
| `inference_error` | 모델 추론 또는 Path 처리 워커에서 예외. 정지 요청, 작업 중단 후 ROS 종료. 예외로 전달된 CUDA OOM도 여기에 포함 |
| `publish_error` | 제어 타이머에서 명령·Path·metrics 발행 등을 처리하다 예외. 정지 재시도 후 작업 중단 |
| `stop_publish_error` | AprilTag 확인/완료 중 `StopMove` 또는 0 속도 발행 실패. 정지 재시도 후 작업 중단 |
| `sigterm` | SIGTERM 수신 시 활성 작업을 중단하고 정지 요청 후 종료 |
| `shutdown` | Ctrl+C 또는 ROS 실행 종료의 정리 단계에서 아직 활성인 작업을 중단하고 정지 요청 |

따라서 Path 실패가 5회가 되는 순간에는 먼저 정지하고, 그 후에도 복구되지 않은
상태가 기본 2초 유지될 때 `unsafe:path_unavailable`로 중단된다. 중단 전에
유효 Path가 돌아오면 연속 실패 횟수와 unsafe 타이머가 초기화된다. 중단이 이미
완료된 작업은 Path가 돌아와도 자동 재시작하지 않는다. AprilTag 확인 중에는
정지를 유지하면서 별도 확인 창을 처리하므로 이 일반 타이밍과 다를 수 있다.

### 오류가 아닌 정지·정상 완료

| 상태/사유 | 동작 |
|---|---|
| `startup_hold` | 작업 시작 후 2초 동안 0 속도 유지 |
| `task_idle` | 활성 작업 없음. 종료 직후 약 1초 동안 0 속도를 반복한 뒤 Sport publisher 해제 |
| `task_complete` / `TASK_COMPLETED` | 정상 추종/허용된 회전 유지 상태로 작업 시간 종료. 기본 500초, 요청 최대 10000초. 정지 후 완료 보고하며 `/task_state`에 reason은 생략될 수 있음 |
| `apriltag_confirmed:<ID>` / `TASK_COMPLETED` | AprilTag 확인 성공. 정지 후 완료 보고 |
| `inputs_not_ready` | 최초 판단 전 `ready_reason`의 초기값. 실제 입력 검사는 위 카메라·추론·Path 사유로 구체화 |

### 작업 시작 거절과 실행 전 실패

다음은 `TASK_REJECTED` 사유이며, 다른 작업의 시작 거절이 이미 실행 중인 작업을
중단시키지는 않는다.

| 사유 | 조건 |
|---|---|
| `another_line_tracking_task_active` | 다른 LINE_TRACKING 작업 실행 중 |
| `control_release_pending` | 이전 작업의 정지 명령 전송·publisher 해제가 아직 끝나지 않음 |
| `control_publisher_error` | 새 작업의 Sport publisher 생성 또는 최초 정지 명령 처리 실패 |
| `invalid_payload_json` | 문자열 payload의 JSON 문법 오류 |
| `invalid_payload` | payload가 JSON 객체가 아님 (`null`/누락은 기본값 사용) |
| `invalid_selected_mask` | 정수 `0`, `1`, `2` 이외 값. 문자열이나 bool도 거절 |
| `invalid_duration_sec` | duration이 숫자가 아님. bool도 거절 |
| `duration_sec_out_of_range` | duration이 NaN/Inf이거나 3초 미만. 최대값 초과는 거절하지 않고 설정 최대값으로 제한 |

잘못된 최상위 이벤트 JSON, 잘못된 task ID/다른 action, 중복 등록 이벤트 등은
대체로 무시하며 별도 주행 정지 사유가 아니다. 프로세스 실행 전에는 ROS 메시지
패키지·CUDA·모델/엔진 로딩 실패, TensorRT manifest/체크섬/버전 불일치, 잘못된
환경변수·ROI·속도/시간 설정, 고정 profile/360×640 출력/`base_link` 조건 위반,
출력 주기 10 Hz 미만, `use_sim_time=true`, 자동 backend fallback 허용 등이
시작 자체를 막을 수 있다. 이 경우 아직 작업을 수락하지 않았으므로
`/task_state` 대신 컨테이너 시작 로그를 확인한다.

1 Hz 미만 추론, Path 나이 0.45초 초과, Path가 전방 4 m에 못 미치는 것,
AprilTag 검출 스트림 단절만으로는 별도 정지하지 않는다. 단, 추론/카메라 5초
검사는 계속 적용하고 짧은 Path는 가장 가까운 끝점으로 목표를 계산한다.
전원 상실·SIGKILL·운영체제 OOM kill은 Python 예외 처리 없이 프로세스를 종료할
수 있어 마지막 정지 명령이나 `TASK_ABORTED` 발행을 보장하지 못한다. OC3 같은
보드 보호 동작은 이 애플리케이션의 reason 코드와 별개다.

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

The camera callback validates timestamps, encoding, dimensions, row stride and
buffer length, then queues the original ROS image message. Only the selected
message is decoded by the worker. Native `rgb8` images reach the model as RGB
without a BGR round trip; packed rows share the retained message buffer, while
padded rows are made contiguous after selection. Other encodings are converted
directly to RGB by `cv_bridge`. Camera conversion failures still stop motion.
Offline BGR image/overlay callers keep the existing default input convention.
The worker's processing latency now also includes selected-message decoding;
previously callback decoding was outside that measurement. TensorRT transfers
only `pixel_values`, leaving the unused processor `pixel_mask` on the CPU.

BEV projection maps, metric axes and the closing kernel are cached by image
dimensions and the frozen local-path configuration (up to eight entries).
Road, sidewalk and union paths share this immutable geometry, but each current
mask is remapped and each path/smoother is updated independently. ROI,
calibration, image size, BEV size or kernel changes select a new cache entry.

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
remains 4 Hz. Missing paths beyond the configured bypass behavior and the
5-second camera/inference freshness checks still stop motion; other task and
drive gates also remain active. In restricted
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
