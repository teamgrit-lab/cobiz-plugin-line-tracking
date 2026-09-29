# cobiz-plugin-line-tracking

`actual-activate` is a Cobiz `LINE_TRACKING` task listener for the Unitree A2.
For an accepted task it follows the selected surface-center path, defaulting to
the pinned Swin-L FP16 TensorRT profile `swin-l-aspect-224x384-fp16`. The existing
MaskFormer R50 profile `r50-fp16-640x360` is also supported with the PyTorch
backend. AprilTag-based stopping and completion can be enabled explicitly. With no
accepted task, it publishes
no Sport Move request. The default path class is sidewalk
(`SWIN_L_PATH_MASK_CLASS=2`); a task can request road (`1`) or road/sidewalk
combined (`0`) with
`payload.selected_mask`.

## Runtime contract

All configurable automatic stops default to `false`; each `LINE_TRACKING_STOP_ON_*` setting
uses `true` to enable its condition and `false` to disable it. Explicit server
cancellation, image/model/publish errors, and handled shutdown still stop motion.
No accepted task means no motion command, and speed limits remain enforced.
When left/right fork selection is enabled, its confirmation, ambiguity and selected-
branch-loss guards also stop motion independently of these switches.

- There is no startup hold or automatic task timeout by default.
- `actual-activate` loads the model once and keeps it in memory. With no
  accepted task, its inference worker sleeps on a condition variable and the
  image callback skips validation, decoding, and queueing. ROS task reception
  and status publication remain active.
- After acceptance and successful initial zero-command publication, inference
  starts on the next received camera frame. `TASK_STARTED` can precede motion;
  the robot waits at zero until this task has a usable path. Initial camera
  arrival, inference, and path calculation add to the time before movement.
- Completion, cancellation, and error/shutdown cleanup suspend inference and
  clear pending frames, paths, and saved commands. An already-running inference
  may finish, but its result is discarded. Model temporal history is reset by
  the worker before the next task's first frame; model weights are not reloaded.
- `debugging-swin-l` (`ros2` mode) still infers continuously without tasks.
  `metrics.inference_enabled` reports whether the worker accepts frames;
  `inference_count` counts published results over the process lifetime. Timing
  samples are cleared between tasks so idle time does not distort throughput.
- With path stops disabled, loss of a path holds the current task's last valid
  forward speed and yaw without a failure-count limit. Camera/inference stalls
  alone do not stop motion when their checks are disabled.
  A lost or ambiguous committed fork is an exception: it publishes zero motion
  and clears the saved command until the selected branch can be matched again.
- `LINE_TRACKING_STOP_ON_APRILTAG=true` enables the existing stop-and-confirm
  behavior: a candidate sends a hard stop, and the same ID in three distinct
  frames across a one-second window completes the task. An unconfirmed candidate
  can resume after that window, subject to enabled drive checks.
- `teamgrit-slam` may supply tags on `/detections`. Missing or empty detection
  streams only affect liveness metrics. With AprilTag stopping disabled, tags
  never stop or complete the task.
- This service supplies no obstacle avoidance. Hardware protections and external
  controllers are outside these application settings.

The direct Sport interface is `/api/sport/request` (`unitree_api/msg/Request`,
Move API ID `1008`). It bypasses navigation-level command arbitration and does
not reject or abort a task when other publishers exist. Concurrent publishers
can therefore issue conflicting commands, and the downstream Unitree interface
determines which command takes effect. Move serialization preserves the existing
axis mapping: `x=vx`, `y=-vy`, and `z=-yaw_rate`. The forward speed ceiling defaults to
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
LINE_TRACKING_MAX_TARGET_HEADING_DEG=60.0
```

## 갈림길에서 좌우 경로 우선

ROS 실행과 MCAP local-path 모드는 기본적으로 `SWIN_L_BRANCH_PREFERENCE=right`를
사용한다. 일반 단일 통로는 기존 중앙선을 유지한다. 공통 진입로에서 갈라진
후보가 확인되면, `right`는 유효한 후보 중 상대적으로 오른쪽인 경로의 중앙선을,
`left`는 왼쪽인 경로의 중앙선을 선택한다. 통로의 경계에 붙이는 기능은 아니다.
`none`은 기존 추출 방식으로 복귀한다.

```dotenv
SWIN_L_BRANCH_PREFERENCE=right
SWIN_L_BRANCH_MIN_WIDTH_M=0.60
SWIN_L_BRANCH_MARGIN_M=0.10
SWIN_L_BRANCH_CONFIRM_FRAMES=2
SWIN_L_BRANCH_HOLD_SEC=1.50
```

왼쪽 분기를 우선하려면 `.env`에 `SWIN_L_BRANCH_PREFERENCE=left`를 지정한다.
좌우 모두 동일한 분기 확인·선택 유지·경로 소실 정지 조건을 적용한다.

직접 실행에서는 `--branch-preference left|right|none`, `--branch-min-width-m`,
`--branch-margin-m`, `--branch-confirm-frames`, `--branch-hold-sec`로 덮어쓸 수 있다.
환경 설정을 변경한 뒤 Compose 서비스를 재생성해야 적용된다.

- 마스크의 틈을 메우기 **전** BEV에서 행별 구간의 겹침을 추적한다. 가까운 공통
  진입로와 연결되지 않은 조각은 분기 후보로 쓰지 않으며, 끊긴 행을
  보간해서 연결하지 않는다. 갈라졌다 다시 합쳐지는 구간은 합류점에서 통합한다.
- 두 후보가 분기점 이후 같은 전방 거리에서 최소 0.75m 진행하고, 중심 간격
  0.60m 이상·방향 차이 15° 이상일 때 분기로 판단한다. 후보가 과도하게 늘어나는
  마스크는 `branch_graph_ambiguous`로 정지한다.
- **서로 다른 새 추론 2회**에서 같은 후보를 확인한 뒤 선택한다. 확인 중에는
  `branch_confirming`으로 0 명령을 보낸다. 10Hz 제어 타이머는 확인 횟수를 늘리지
  않는다. 설정한 폭과 여유를 확보하지 못하거나 연결 경로를 만들 수 없는 후보는
  제외한다. 선호 방향 후보만 부적합하면 유효한 다른 후보를 선택할 수 있다.
- 선택 후에는 매번 화면의 선호 방향을 다시 고르지 않고 이전 경로의 먼 구간과 대응시킨다.
  기본 1.5초가 지나고 새 추론 3회에서 단일 통로가 확인되면 선택 상태를 해제한다.
  작업 시작·종료 시 초기화하며, 도로·보도·통합 마스크의 상태는 각각 독립적이다.
- 선택된 가지가 사라지거나 대응이 모호하면 `branch_path_lost` 또는
  `branch_match_ambiguous`로 정지한다. `LINE_TRACKING_STOP_ON_PATH_UNAVAILABLE=false`
  등 일반 정지 설정으로 이 분기 보호를 우회하지 않는다. 선택 경로가 다시 확인되면
  재개하며, 복구되지 않으면 작업 취소/재시작으로 선택 상태를 초기화할 수 있다.
- 분기 중앙선은 선택된 구간의 여유 범위 안에서 공간적으로 다듬고, 점 사이의 선분도
  마스크 내부인지 확인한다. 두 가지를 평균 내는 시간 평활화는 적용하지 않는다.
  분기 경로에는 BEV 행별 점을 유지하므로 기존 `SWIN_L_PATH_POINTS=20`보다 점이
  많을 수 있다. `unrestricted_path_mode`에서도 분기 판정과 선택 유지는 동작한다.

`metrics.branch_selection`에는 분기 검출 여부, 후보 방향, 유효 후보 수, 확인 횟수,
선택 상태와 이유, 선호 방향 적용 여부를 기록한다. 정지 중에는 `path_tracked=false`와
빈 Path를 발행한다. MCAP 화면의 흰 선은 선택된 경로이고 주황 선은 기존 원시 추출이다.

폭·여유·거리 임계값은 **설정된 BEV 좌표 기준 초기값**이다. ROI와 카메라 자세의
보정 오차를 포함하므로 차체 폭, 회전 시 차체가 차지하는 공간, 실제 경계 여유를
보장하지 않는다. 이 구현은 x 방향으로 진행하는 Y자 분기가 대상이며 90° 코너를
해결하지 않는다. IMU/odom 보정도 추가하지 않는다. 프레임 간 경로 이동이 커서
대응할 수 없으면 추측해서 다른 가지로 바꾸지 않고 정지한다. 실제 로봇 적용 전에는
기록 영상과 저속 주행으로 분기 판정·회전 방향·여유 폭을 확인해야 한다.

## 목표 방향에 따른 전진 감속

급하게 꺾어야 할수록 전진 속도를 줄인다. 목표 방향은 기존 방식대로
`heading = atan2(y_at_lookahead, lookahead)`로 계산하며 기본 lookahead는 4m다.
허용 목표각은 좌우 각각 **60° 이하**로, 이전 횡방향 ±0.75m 제한을 대체한다.
각도 제한이 켜져 있을 때 60°를 초과하면 `path_lateral_target_large`로 정지한다.
60°는 Path 목표각이며 회전 속도는 기존 **±0.18rad/s(약 ±10.31°/초)**다.

```text
requested_yaw = heading_gain × heading_radians
yaw = clip(requested_yaw, -max_yaw, +max_yaw)
forward_speed = max_forward × max_yaw / max(max_yaw, abs(requested_yaw))
```

회전 요구가 상한을 넘으면 전진 속도도 같은 비율로 줄여 명령의 회전/전진 비율을
유지한다. 기본 gain=1, 전진 상한=0.50m/s, yaw 상한=0.18rad/s일 때:

| 목표각 절댓값 | 전진 속도 | 회전 속도 절댓값 |
|---|---|---|
| 0° | 0.500m/s | 0°/초 |
| 10° | 0.500m/s | 10°/초 |
| 15° | 0.344m/s | 10.31°/초 |
| 30° | 0.172m/s | 10.31°/초 |
| 45° | 0.115m/s | 10.31°/초 |
| 60° | 0.086m/s | 10.31°/초 |

이전 곡률·영상 나이 기반 감속, 1.2초 `perception_delay_stop`, 정지 후 방향 정렬,
두 프레임 확인, 회전 펄스, 가속 제한은 제거했다. 추론 결과를 기다리는 추가 단계 없이
기본 10Hz 제어 주기에서 적용하며 추론 루프는 그대로다. 기존 5초 카메라/추론
검사, 시작 2초 대기와 AprilTag 정지는 아래 설정으로 켤 수 있다. 명시적 취소와
오류·프로세스 종료 시 정지는 항상 유지한다.

감속 중 `drive_reason`은 `tracking_slow_turn`이며 정상 추종으로 처리한다.
metrics의 `turn_speed_control`에서 목표각, 허용각, 전진·회전 명령을 확인할 수 있다.
60°에서 전방 4m 기준 횡방향 한계는 약 6.93m다. 현재 BEV 반폭 4m와 Path 추출
범위는 그대로이므로 허용각을 높여도 카메라가 60° 또는 90° 코너의 Path를 추가로
검출하게 되지는 않는다. 감속은 코너 통과나 도로 경계 안의 주행을 보장하지 않는다.

Jetson 적용 시 사용자가 `.env`의 `LINE_TRACKING_MAX_TARGET_HEADING_DEG=60.0`을
확인하고 `docker compose up -d --build actual-activate`로 빌드·재생성한다.
제거된 `LINE_TRACKING_ADAPTIVE_CONTROL`, `LINE_TRACKING_TURN_*` 등 예전 설정은
사용하지 않는다. 모델/백엔드 설정은 유지한다.

## 자동 정지 설정

모든 자동 정지 조건은 **`true`=사용, `false`=해제**이며 기본값은 전부 `false`다.
기존 조건과 임계값은 유지하고, 각 조건의 적용 여부만 독립적으로 선택한다.

```dotenv
LINE_TRACKING_STOP_ON_CAMERA_STALE=false
LINE_TRACKING_STOP_ON_INFERENCE_STALE=false
LINE_TRACKING_STOP_ON_CAMERA_TIMESTAMP_INVALID=false
LINE_TRACKING_STOP_ON_PATH_UNAVAILABLE=false
LINE_TRACKING_STOP_ON_PATH_LOSS_LIMIT=false
LINE_TRACKING_STOP_ON_LOW_CONFIDENCE=false
LINE_TRACKING_STOP_ON_LATERAL_TARGET=false
LINE_TRACKING_STOP_ON_APRILTAG=false
LINE_TRACKING_STOP_ON_STARTUP_HOLD=false
LINE_TRACKING_STOP_ON_STARTUP_UNREADY=false
LINE_TRACKING_STOP_ON_UNSAFE_TIMEOUT=false
LINE_TRACKING_STOP_ON_TASK_TIMEOUT=false
```

아래 이름에는 공통 접두사 `LINE_TRACKING_STOP_ON_`을 붙인다.

| 설정 | `true`일 때 적용되는 조건 |
|---|---|
| `CAMERA_STALE` | 카메라 수신/원본 시각 기준 5초 초과, 이력 없음 또는 잘못된 나이이면 정지 |
| `INFERENCE_STALE` | 추론 완료/추론 원본 이미지 시각 기준 5초 초과, 이력 없음 또는 잘못된 나이이면 정지 |
| `CAMERA_TIMESTAMP_INVALID` | 0 이하, 50ms 초과 미래, 5초 초과 과거, 중복·역순 이미지 시각을 거절하고 정지 |
| `PATH_UNAVAILABLE` | 유한한 x/y 목표점을 만들 수 없으면 즉시 정지 |
| `PATH_LOSS_LIMIT` | 선택한 Path가 연속 5회 추론에서 없으면 정지. 즉시 정지를 꺼도 독립 적용 |
| `LOW_CONFIDENCE` | 신뢰도 0.49 미만 또는 NaN/Inf이면 정지. unrestricted 모드에서도 적용 |
| `LATERAL_TARGET` | 목표각 절댓값이 설정 허용각(기본 60°)을 초과하면 정지 |
| `APRILTAG` | 후보 검출 즉시 정지하고, 1초 확인 창에서 같은 ID의 새 프레임 3회 확인 시 작업 완료 |
| `STARTUP_HOLD` | 작업 시작 후 2초간 0 속도로 대기 |
| `STARTUP_UNREADY` | 시작 2초 이후에도 추종을 한 번도 시작하지 못했다면 작업 중단 |
| `UNSAFE_TIMEOUT` | 추종할 수 없는 상태가 연속 2초 지속되면 작업 중단 |
| `TASK_TIMEOUT` | 요청한 작업 시간 도달 시 정지·종료. 기본 500초, 최대 10000초 |

`PATH_UNAVAILABLE=false`, `PATH_LOSS_LIMIT=true`이면 이전처럼 실패 1–4회는
마지막 전진·회전 명령을 유지하고 5회째 정지한다. 둘 다 `false`이면 실패 횟수에
상관없이 그 명령을 유지한다. 급회전 때문에 줄어든 전진 속도도 함께 유지한다.
현재 작업에서 아직 유효한 명령을 얻지 못했다면 `waiting_for_path`로 0 속도 대기한다.
작업 시작·종료 시 저장 명령은 초기화하며, 다른 작업의 명령을 재사용하지 않는다.
실패 횟수는 카메라 수신/10Hz 발행 횟수가 아니라 선택한 클래스의 추론 완료 횟수다.
유효 Path가 돌아오면 0으로 초기화한다. 제한 모드에서는 smoother가 보존하는 유효
Path가 만료된 뒤부터 실패로 센다.

`STARTUP_HOLD=false`이면 별도의 2초 시작 대기는 없지만, 작업 수락 후 새 영상의
첫 추론과 경로 계산이 끝나기 전까지는 0 속도로 대기한다. 모델은 메모리에 유지하며
작업 전·종료 후에는 영상 추론을 수행하지 않는다. `STARTUP_UNREADY=true`이면
이 첫 추론 대기 시간도 기존의 시작 제한 시간에 포함된다.
`STARTUP_UNREADY`와 `UNSAFE_TIMEOUT`은 서로 독립적인 중단 조건이다.
`TASK_TIMEOUT=false`이면 payload의 `duration_sec`에 도달해도 작업은 계속된다.
명시적 취소, 처리 오류·프로세스 종료, 또는 별도로 켠 자동 종료 조건이 작업을 끝낸다.

명시적 서버 취소, 이미지 변환 오류, 추론·발행 오류, SIGTERM/정상 종료 시 정지는
해제하지 않는다. 이미지 변환 오류는 오류 이후 수락한 새 프레임의 추론이 성공할
때까지 정지를 유지한다. 전진 상한 1.0m/s, 회전 상한 ±0.18rad/s, 급회전 시 전진 감속,
설정·명령 유효성 검사도 유지한다. Path 생성과 속도 제한을 끄는 설정은 아니다.

기존 역방향 설정 `LINE_TRACKING_BYPASS_PATH_STOPS`와 `--bypass-path-stops`는
제거했다. 기존 `.env`에서 해당 키를 삭제하고 위 설정으로 교체한다. 직접 실행 환경에
옛 키가 남아 있으면 설정 오류로 안내하며, Compose는 그 키를 전달하지 않는다.
기존 `.env`에 명시한 `STOP_ON_*=true`는 계속 적용되므로 전부 해제하려면 기존 값도
`false`로 변경한다. 코드를 반영한 이미지를 빌드하고 서비스를 재생성해야 적용된다.
CLI는 `--stop-on-camera-stale`처럼 켜고 `--no-stop-on-camera-stale`처럼 끈다.

metrics의 `stop_checks`에 12개 설정을 표시한다. `path_yaw_held`,
`path_unavailable_inferences`, `path_unavailable_limit`(`5`)로 명령 유지와 실패 횟수를
확인한다. `tracking_path_hold`이면 Path 메시지가 비어 있어도 저장 명령으로 움직일
수 있다. 호환 진단 키 `stop_checks.path_available`은 즉시 정지가 켜졌거나,
연속 실패 제한이 켜지고 5회에 도달했을 때만 `true`다.
`apriltag.stop_enabled`는 AprilTag 정지 설정을 나타낸다.

`debugging-swin-l`은 Sport 요청을 발행하지 않는 디버그 프로필이다.

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

Example task payload (`duration_sec` ends the task only when `TASK_TIMEOUT=true`):

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
It always sends stop commands on server cancellation, handled shutdown, and
processing/publish errors. Automatic stops apply only when their flags are enabled.
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
ROI. An invalid new estimate clears the previous path. Valid-ratio filtering,
temporal smoothing, and the smoother's hold expiry are bypassed in this mode.
The independent `STOP_ON_LOW_CONFIDENCE=true` drive check still applies at 0.49.
Sparse observations can therefore be extrapolated over
the full forward range; neither fitting nor gap filling establishes obstacle
clearance. With unrestricted mode disabled, the default valid-row requirement
is 35% (56 of 160 rows), and the smoother can retain a previous valid path for
up to 0.90 seconds.

A visible path does not imply an active task. Motion requires an accepted task,
a usable target or a command retained from that task, no active processing fault,
and satisfaction of any enabled automatic checks listed above. The controller
has no separate path-age cutoff and does not require a path to span x=4 m or
arrive in increasing x order. It discards non-finite points, sorts by forward
distance, and uses the first finite point at each duplicate distance. A single
finite point is sufficient. Outside the path range, the nearest endpoint supplies
the lateral target. Holding a saved command never creates an invented Path message.

Path age is measured from inference-result availability. Camera and inference
freshness metrics also account for the original sensor timestamp; repeating a
path at 10 Hz never resets them. Their stop flags determine whether stale values
block motion. The heading calculation uses the 4 m lookahead; when enabled,
the default 60° target-angle limit corresponds to about ±6.93 m laterally.
Tracking sends heading-regulated forward speed (default ceiling 0.50 m/s),
zero lateral velocity, and yaw capped at ±0.18 rad/s. Disabling angle stopping
does not repair an inaccurate path or guarantee a sharp bend can be followed.

## 정지·작업 중단·시작 거절 사유

아래는 `actual-activate`의 Python `task-drive` 코드 기준이다. 주행 판단의
`drive_reason`과 `ready_reason`은 `/line_tracking/swin_l/metrics`에서,
작업 결과의 `type`과 `reason`은 `/task_state`에서 확인한다.
`drive_reason`은 시작 대기·AprilTag 확인 등까지 반영한 출력 판단이고,
`ready_reason`은 카메라·추론·Path 검사 결과다. 정지 명령 전송과 작업 중단은
별개이며, 아래의 "즉시 정지"는 다음 제어 주기(기본 10 Hz)에 0 속도를 보내는
동작이다. 카메라 콜백의 오류와 AprilTag 검출은 콜백에서 바로 정지를 요청한다.

### 주행 중 정지 조건

각 자동 조건은 위 표의 해당 `STOP_ON_*` 값이 `true`일 때만 적용한다.

| 사유 | 발생 조건/동작 |
|---|---|
| `camera_stale` | 카메라 나이 검사 실패. 저장한 전진·회전 명령 삭제 |
| `inference_stale` | 추론 나이 검사 실패. 저장한 전진·회전 명령 삭제 |
| `camera_timestamp_invalid` | 이미지 시각 검사 실패. 콜백에서 0 속도 전송 및 저장 명령 삭제. 이후 새 유효 프레임의 추론 성공까지 정지하며, 카메라 검사도 켜져 있으면 보통 `camera_stale`로 표시 |
| `camera_conversion_error` | 메타데이터 검사 또는 선택 프레임 변환 예외. 모든 자동 조건이 꺼져도 정지 및 저장 명령 삭제. 새 유효 프레임의 추론 성공까지 유지하며 `camera_fault_reason`으로 확인 |
| `path_unavailable` | 즉시 Path 정지 또는 5회 실패 제한이 활성화되어 정지 |
| `waiting_for_path` | Path 정지는 꺼져 있지만 현재 작업에 저장된 유효 명령도 없어 0 속도 대기 |
| `path_low_confidence` | 신뢰도 0.49 미만 또는 NaN/Inf |
| `path_lateral_target_large` | 목표각 절댓값이 허용각(기본 60°) 초과 |
| `apriltag_verifying` | 후보 확인 중. `StopMove`와 0 속도로 정지 |

### 정지가 작업 중단으로 이어지는 조건

| `/task_state.reason` | 조건과 결과 |
|---|---|
| `startup:<사유>` | `STOP_ON_STARTUP_UNREADY=true`이고 시작 후 2초가 지났는데 추종 또는 허용된 명령 유지가 한 번도 시작되지 못한 경우 `TASK_ABORTED`. 시작 대기 설정과 독립적으로 적용 |
| `unsafe:<사유>` | `STOP_ON_UNSAFE_TIMEOUT=true`이고 추종할 수 없는 상태가 연속 `LINE_TRACKING_UNSAFE_TIMEOUT_SEC`(기본 2초) 지속되면 `TASK_ABORTED`. 이유가 바뀌어도 그 사이 정상 추종/허용된 회전 유지가 없으면 타이머는 계속 진행 |
| `tracking_unavailable:<사유>` | `STOP_ON_TASK_TIMEOUT=true`이고 작업 제한 시간에 도달했지만 정상 완료 조건(이전에 추종했고 현재도 추종/허용된 회전 유지 중)을 만족하지 못하면 `TASK_ABORTED`. 앞의 startup/unsafe 조건이 먼저 충족되면 그 사유가 우선 |
| `task_aborted_by_server` | 현재 task ID에 대한 서버의 `TASK_ABORTED` 이벤트 수신. 즉시 정지 요청 후 작업 중단 |
| `inference_error` | 모델 추론 또는 Path 처리 워커에서 예외. 정지 요청, 작업 중단 후 ROS 종료. 예외로 전달된 CUDA OOM도 여기에 포함 |
| `publish_error` | 제어 타이머에서 명령·Path·metrics 발행 등을 처리하다 예외. 정지 재시도 후 작업 중단 |
| `stop_publish_error` | AprilTag 확인/완료 중 `StopMove` 또는 0 속도 발행 실패. 정지 재시도 후 작업 중단 |
| `sigterm` | SIGTERM 수신 시 활성 작업을 중단하고 정지 요청 후 종료 |
| `shutdown` | Ctrl+C 또는 ROS 실행 종료의 정리 단계에서 아직 활성인 작업을 중단하고 정지 요청 |

예를 들어 즉시 Path 정지는 끄고 `PATH_LOSS_LIMIT`과 `UNSAFE_TIMEOUT`을 켜면,
5회째 Path 실패에서 정지하고 그 상태가 2초 지속될 때 `unsafe:path_unavailable`로
중단된다. 중단 전에 유효 Path가 돌아오면 실패 횟수와 unsafe 타이머가 초기화된다.
중단된 작업은 Path가 돌아와도 자동 재시작하지 않는다. 두 설정 모두 `false`이면
이 실패 횟수나 타이머로 정지·중단하지 않는다. AprilTag 정지를 켠 경우에는
확인 창을 먼저 처리하므로 일반 작업 타이머의 판정 시점이 늦춰질 수 있다.

### 오류가 아닌 정지·정상 완료

| 상태/사유 | 동작 |
|---|---|
| `startup_hold` | `STOP_ON_STARTUP_HOLD=true`일 때 작업 시작 후 2초 동안 0 속도 유지 |
| `task_idle` | 활성 작업 없음. 종료 직후 약 1초 동안 0 속도를 반복한 뒤 Sport publisher 해제 |
| `task_complete` / `TASK_COMPLETED` | `STOP_ON_TASK_TIMEOUT=true`일 때 정상 추종/허용된 회전 유지 상태로 작업 시간 종료. 기본 500초, 요청 최대 10000초. 정지 후 완료 보고하며 `/task_state`에 reason은 생략될 수 있음 |
| `apriltag_confirmed:<ID>` / `TASK_COMPLETED` | `STOP_ON_APRILTAG=true`일 때 AprilTag 확인 성공. 정지 후 완료 보고 |
| `inputs_not_ready` | 최초 판단 전 `ready_reason`의 초기값. 실제 입력 검사는 위 카메라·추론·Path 사유로 구체화 |

### 작업 시작 거절과 실행 전 실패

다음은 `TASK_REJECTED` 사유이며, 다른 작업의 시작 거절이 이미 실행 중인 작업을
중단시키지는 않는다.

| 사유 | 조건 |
|---|---|
| `another_line_tracking_task_active` | 다른 LINE_TRACKING 작업 실행 중 |
| `control_release_pending` | 이전 작업의 정지 명령 전송·publisher 해제가 아직 끝나지 않음 |
| `control_publisher_error` | 새 작업의 Sport publisher 생성 또는 최초 제어 명령 처리 실패. publisher가 있으면 정지 명령 재시도 후 해제 |
| `invalid_payload_json` | 문자열 payload의 JSON 문법 오류 |
| `invalid_payload` | payload가 JSON 객체가 아님 (`null`/누락은 기본값 사용) |
| `invalid_selected_mask` | 정수 `0`, `1`, `2` 이외 값. 문자열이나 bool도 거절 |
| `invalid_duration_sec` | duration이 숫자가 아님. bool도 거절 |
| `duration_sec_out_of_range` | duration이 NaN/Inf이거나 3초 미만. 최대값 초과는 거절하지 않고 설정 최대값으로 제한 |

잘못된 최상위 이벤트 JSON, 잘못된 task ID/다른 action, 중복 등록 이벤트 등은
대체로 무시하며 별도 주행 정지 사유가 아니다. 프로세스 실행 전에는 ROS 메시지
패키지·CUDA·모델/엔진 로딩 실패, TensorRT manifest/체크섬/버전 불일치, 잘못된
환경변수·ROI·속도/시간 설정, 허용 profile·고정 checkpoint/360×640 출력/`base_link` 조건 위반,
출력 주기 10 Hz 미만, `use_sim_time=true`, 자동 backend fallback 허용 등이
시작 자체를 막을 수 있다. 이 경우 아직 작업을 수락하지 않았으므로
`/task_state` 대신 컨테이너 시작 로그를 확인한다.

1 Hz 미만 추론, Path 나이 0.45초 초과, Path가 전방 4 m에 못 미치는 것,
AprilTag 검출 스트림 단절만으로는 별도 정지하지 않는다. 추론/카메라 5초 검사는
해당 설정이 `true`일 때 적용하고 짧은 Path는 가장 가까운 끝점으로 목표를 계산한다.
전원 상실·SIGKILL·운영체제 OOM kill은 Python 예외 처리 없이 프로세스를 종료할
수 있어 마지막 정지 명령이나 `TASK_ABORTED` 발행을 보장하지 못한다. OC3 같은
보드 보호 동작은 이 애플리케이션의 reason 코드와 별개다.

## FP16 TensorRT Swin-L and Jetson image

The default live task profile is `swin-l-aspect-224x384-fp16`:

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

## MaskFormer R50 on Jetson

Both `actual-activate` and `debugging-swin-l` read `SWIN_L_PROFILE` from `.env`.
To select the existing R50 runtime, set both the profile and backend:

```dotenv
SWIN_L_PROFILE=r50-fp16-640x360
SWIN_L_BACKEND=pytorch
SWIN_L_DEVICE=cuda
SWIN_L_TRT_AUTO_BUILD=false
SWIN_L_ALLOW_BACKEND_FALLBACK=false
```

R50 uses `facebook/maskformer-resnet50-vistas` at revision
`ae4b8c2590c0a090fc32d5c217d78738a2dd4b19`. Its PyTorch weights are loaded through
the existing Hugging Face cache, then run in FP16 on CUDA. The image already
includes its dependencies; no ONNX conversion or TensorRT engine build is needed.
An R50/TensorRT combination fails before any startup engine build.

The 720p camera topic can stay unchanged: the processor resizes each selected
frame to 640x360 before model-specific padding, and the output score map remains
640x360 for BEV and Path calculation. Latest-frame selection, RGB handling,
BEV caching, task-selected masks 0/1/2, and Sport control use the shared pipeline.
CUDA, checkpoint pinning, camera freshness, task lifecycle, and control checks
still apply. The live inference target remains `SWIN_L_INFERENCE_HZ` (default 4).

R50 retains its existing label aggregation and cleanup: Bike Lane and Manhole
join the sidewalk group, while Parking and Service Lane are excluded from the
road group. Its temporal hysteresis margin defaults to zero. This can change
the resulting Path, including the union selected by mask 0; changing models
does not establish equivalent segmentation quality or a Jetson speedup.

After updating `.env`, rebuild/recreate the selected service on the Jetson:

```bash
docker compose up -d --build actual-activate
docker compose logs -f actual-activate
```

For inspection without Sport commands, use `debugging-swin-l` instead of
`actual-activate`. Avoid running both while comparing inference speed because
they share the GPU. Check the metrics `profile`, `inference_count`,
and `performance` together with the generated Path. The Swin-L-specific
`swin_l_rosbag_overlay.py` wrapper remains pinned to Swin-L; use
`swin_l_local_path_debug.py mcap --profile r50-fp16-640x360 --backend pytorch`
with the input/output arguments for offline R50 Path inspection.

To restore the default runtime, set `SWIN_L_PROFILE=swin-l-aspect-224x384-fp16`
and `SWIN_L_BACKEND=tensorrt`, then recreate the service with its matching
Swin-L engine and manifest.

## Jetson inference stability

While enabled, live inference obeys `SWIN_L_INFERENCE_HZ`, including when
`SWIN_L_UNRESTRICTED_PATH_MODE=true`. The latest-frame queue holds one frame;
it replaces pending frames while waiting for the next inference start. A late
inference does not trigger a burst of catch-up jobs. Rate limiting reduces
average load, but cannot guarantee latency or cap instantaneous GPU power.
The first frame of a newly accepted task does not wait for the previous task's
rate-limit deadline. Task-driven mode sleeps between tasks; debug mode does not.
TensorRT's one-time startup validation with a synthetic input is retained;
this is separate from camera inference and can briefly use the GPU at startup.

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
remains 4 Hz. Missing paths and stale camera/inference inputs stop motion only
when the corresponding automatic flags are enabled. In restricted
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
