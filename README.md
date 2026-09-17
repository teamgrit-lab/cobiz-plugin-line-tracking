# cobiz-plugin-line-tracking

기본 Docker Compose 서비스 `actual-activate`는 Cobiz 서버의 커스텀 액션
`LINE_TRACKING`을 기다립니다. 작업이 도착하고 현장 보정·카메라·LiDAR 안전
검사를 통과한 경우에만 Swin-L `swin-l-aspect-224x384`의 선택된 영역 중심 경로로
Unitree A2용 `sensor_msgs/Joy` 명령을 생성합니다. 작업이 없으면 Joy publisher도
없습니다. 기본은 인도(`SWIN_L_PATH_MASK_CLASS=2`)이며, `.env`에서 차도(`1`)로
선택할 수 있습니다. Cobiz 작업의 `payload.selected_mask`가 있으면 그 작업에만
적용합니다. 기존 YOLOP 노란 선 추종은 별도 `legacy` 프로필로 보관합니다.

아래 YOLOP 흐름은 **레거시** 서비스 설명입니다.

Google Docs의 권장 흐름을 코드로 옮겼습니다.

```text
Image -> YOLOP(road + lane-line, one ONNX forward pass) -> road-gated line mask
      -> trapezoid ROI -> bird's-eye transform -> quadratic centerline
      -> lateral/heading/curvature error -> vx/vy/yaw_rate
      -> confidence gate -> low-pass filter -> rate limit -> A2 Joy
```

레거시 YOLOP를 선택할 때는 `models/README.md`의 ONNX 파일을 `models/`에 넣고
`.env`의 `CAMERA_PROFILE`과 `SEGMENTATION_MODEL_PATH`를 맞춥니다. 모델 경로가
비어 있으면 HSV/LAB 검출기로 fallback하므로 운영 주행에 사용하면 안 됩니다.

## 안전 계약

기본 작업 수신형 Swin-L 서비스는 아래의 공통 `/a2_control` 계약에 더해
작업 ID·보정 플래그·센서 freshness·LiDAR 프레임·제어 publisher 단독 소유를
확인합니다. 아래의 선 신뢰도 0.4와 카메라 watchdog 설명은 레거시 YOLOP
서비스에 해당합니다.

- 출력 기본 토픽은 `/a2_control`이며 `cobiz-plugin-a2`의 `a2_control_node`가
  Unitree Sport API 명령으로 변환합니다.
- 선 신뢰도가 `0.4` 미만이거나 선을 잃으면 즉시 0 속도를 발행합니다.
- 카메라 프레임이 기본 `0.5 s` 동안 오지 않아도 0 속도를 발행합니다.
- `cobiz-plugin-a2`는 Joy를 `Move(vx=-axes[1], vy=-axes[0], yaw=-axes[2])`로
  해석합니다. 보고된 기체 동작에서 좌우 이동과 좌우 회전이 경로 좌표계와 반대였으므로
  이 플러그인은 `[vy, -vx, yaw_rate]`를 발행해 두 방향을 보정합니다. 버튼
  10개는 항상 0으로 유지해 자세·보행 버튼 동작을 방지합니다. 기본 Swin-L
  주행은 `vy=0`이므로 실제로 사용하는 좌우 보정은 회전축입니다.
- 실제 주행 전 `perspective_source`, 지면 폭/거리와 Joy 축 부호를 현장 카메라
  장착 상태에 맞춰 보정해야 합니다.
- `/a2_control`은 로봇으로 직접 이어지는 수동 제어 경로이므로 다른 gamepad
  publisher 또는 Navigation 제어와 동시에 사용하지 않아야 합니다.

현재 제공된 원근점은 기능 확인용 초기값입니다. 측량하지 않은 기본값으로 무인
주행을 시작하면 안 됩니다.

## 토픽

| 방향 | 토픽 기본값 | 형식 | 설명 |
|---|---|---|---|
| 입력 | `/a2/front_camera/image_raw` | `sensor_msgs/Image` | A2 전방 영상 |
| 출력 | `/a2_control` | `sensor_msgs/Joy` | A2 속도 제어 명령 |
| 출력 | `/line_tracking/confidence` | `std_msgs/Float32` | 0~1 검출 신뢰도 |
| 출력 | `/line_tracking/debug_image` | `sensor_msgs/Image` | ROI와 마스크 overlay |
| 출력 | `/line_tracking/mask` | `sensor_msgs/Image` | 도로로 gating한 선 이진 마스크 |

테스트 모드(`TEST_MODE=true`)에서는 다음 토픽도 발행합니다.

| 방향 | 기본 토픽 | 형식 | 설명 |
|---|---|---|---|
| 출력 | `/line_tracking/test/debug_image` | `sensor_msgs/Image` | road=초록, line=노랑, 중심선=흰색 overlay |
| 출력 | `/line_tracking/test/road_mask` | `sensor_msgs/Image` | YOLOP 도로 영역 mask |
| 출력 | `/line_tracking/test/raw_line_mask` | `sensor_msgs/Image` | road gating 전 YOLOP 선 mask |
| 출력 | `/line_tracking/test/line_mask` | `sensor_msgs/Image` | road gating 후 선 mask |
| 출력 | `/line_tracking/test/birdseye_mask` | `sensor_msgs/Image` | 경로 fitting에 사용한 bird's-eye mask |
| 출력 | `/line_tracking/test/centerline` | `nav_msgs/Path` | `base_link` 기준 추정 중심 경로 |
| 출력 | `/line_tracking/test/metrics` | `std_msgs/String` | confidence/error/추종 여부 JSON |

테스트 모드에서는 안전을 위해 `/a2_control`에 0 속도 Joy를 발행합니다.
실제 제어까지 함께 시험하려면 `PUBLISH_CONTROL_IN_TEST_MODE=true`를 명시해야
합니다.

로봇 좌표는 `x=전방`, `y=왼쪽`, `yaw=반시계 방향 양수`를 사용합니다.

## Cobiz 작업 수신형 실행 (기본)

`cobiz-core`의 `cobiz_bridge/config/config.yaml`에서 `actions.custom` 항목에
`LINE_TRACKING`을 등록했습니다. Core를 재빌드·재시작해 서버에 갱신된 액션을
등록해야 합니다. Core의 `health_check_node`가 서버의 `TASK_REGISTERED`와
`TASK_ABORTED`를 `/task_event`로 전달하고, 이 플러그인은 `/task_state`에
`TASK_STARTED/start`, `TASK_COMPLETED/complete`, `TASK_REJECTED/reject`,
`TASK_ABORTED/abort`를 발행합니다. HTTP 보고는 Core의 `request_manager`가 맡습니다.

Jetson에서 먼저 Swin-L용 CUDA 베이스 이미지, DDS 설정 파일, 카메라·LiDAR
토픽을 준비합니다. `cp .env.example .env` 후 실제 토픽과 카메라-to-`base_link`
homography, LiDAR-to-`base_link` 변환 및 장애물 범위를 **실측**해야 합니다.
`.env.example`은 보정값이 아닙니다. 물리 비상정지와 `/a2_control` 단독 소유를
검증한 뒤에만 `.env`의 `SWIN_L_CALIBRATION_CONFIRMED=true`와
`SWIN_L_DRIVE_ENABLED=true`를 설정합니다. 두 값이 `false`면 컨테이너는
대기하지만 작업을 `drive_not_armed`로 거절합니다.

```bash
cp .env.example .env                  # 최초 1회, Jetson 값으로 보정
docker compose up -d --build           # 최초 빌드: actual-activate 하나만 시작
# 이후에는 docker compose up -d 또는 docker compose up -d actual-activate
docker compose logs -f actual-activate
ros2 topic echo /line_tracking/swin_l/metrics
ros2 topic echo /task_state
```

기본 작업 시간은 60초, 최대 300초입니다. 서버 작업 `payload`에
`{"duration_sec": 30, "selected_mask": 1}`을 넣으면 30초 동안 차도(`1`)를
추종합니다. 인도는 `2`이며, `selected_mask`를 생략하면 `.env`의
`SWIN_L_PATH_MASK_CLASS`를 사용합니다. `0`·그 밖의 값은 `invalid_selected_mask`로
거절합니다. 작업 종료 후에는 `.env` 기본값으로 돌아갑니다. 센서·모델 경로가 준비되지
않았거나 다른 `/a2_control` publisher가 있으면 작업을 거절합니다. 수락 시
2초간 0 명령을 보낸 뒤 추종하고, 서버 취소·시간 만료·안전 조건 위반 2초 지속 시
0 명령 후 종료 상태를 보고합니다. 무한 주행 작업은 지원하지 않습니다.
정상 종료와 `SIGTERM`에도 0 명령을 시도하지만 전원 차단·`SIGKILL` 시에는
발행할 수 없습니다. 현재 A2 제어 노드의 Joy 미수신 watchdog은 경고 로그만
남기므로, 무인 실주행 전 독립적인 하위 제어 정지 장치/물리 비상정지를 확인해야
합니다.

기존 YOLOP 테스트가 필요하면 기본 Swin-L 서비스를 먼저 중지하고
`docker compose --profile legacy up -d --build line-tracking`을 명시합니다.
두 서비스를 동시에 실행하지 마세요. 종료는 `docker compose down`입니다.

## 동영상 segmentation overlay 생성

ROS2 카메라 토픽이 없어도 동영상 파일을 YOLOP에 넣어 도로와 선 mask를
overlay한 새 동영상을 만들 수 있습니다. 입력 동영상이 1280x720이면
`--profile 720p`, 640x360이면 `--profile 360p`를 사용합니다.

```bash
PYTHONPATH=ros_ws/src/line_tracking python3 tools/segment_video.py \
  --input /path/to/input.mp4 \
  --output /path/to/output_yolop_overlay.mp4 \
  --model models/yolop-720-1280.onnx \
  --profile 720p
```

출력 overlay 색상은 초록=도로 영역, 빨강=raw 선 segmentation,
노랑=도로 mask로 gating된 최종 선 segmentation입니다. 기본 출력에는
입력 영상의 오디오가 포함되지 않으며, OpenCV codec 문제로 출력이 열리지
않으면 `--codec avc1` 또는 `--codec mp4v`를 시도합니다.

YOLOP의 도로·선 mask를 모두 사용한 뒤, 도로로 gating된 모델 선 안에서
OpenCV 노란색·선 형태 검출을 한 번 더 적용하려면 `mix` backend를 사용합니다.

```bash
uv run python tools/segment_video.py \
  --backend mix \
  --input /path/to/input.mp4 \
  --output /path/to/output_mix_overlay.mp4 \
  --model models/yolop-720-1280.onnx \
  --profile 720p
```

Mix overlay는 초록=YOLOP 도로, 빨강=YOLOP raw 선, 청록=도로로 제한된
YOLOP 선, 노랑=OpenCV 색상·형태 조건까지 통과한 최종 선, 흰색=ROI입니다.
최종 노란 mask는 항상 도로로 제한된 YOLOP 선 mask의 부분집합입니다.

노란색과 흰색 차선만 색상으로 먼저 제한한 뒤 Canny와 HoughLinesP로 선분을
검출하려면 `lane-only` backend를 사용합니다. 이 backend는 경로 fitting이나
중앙선 선택을 하지 않고, 색상으로 지지되는 차선 선분만 출력합니다. 초록색·도로
전체 mask는 사용하지 않으며, 노란색 선은 노랑, 흰색 선은 흰색으로 표시합니다.

```bash
uv run python tools/segment_video.py \
  --backend lane-only \
  --input /Users/kangminwoo/Downloads/roadline_test.mp4 \
  --output /Users/kangminwoo/Downloads/lane_only_validation.mp4 \
  --lane-min-length-px 60 \
  --lane-draw-width-px 5
```

Colab의 Advanced-Lane-Lines 흐름처럼 자동차 도로의 좌·우 차선을 함께 검출하고
그 사이 주행 영역을 표시하려면 `advanced-lane` 테스트 backend를 사용합니다.
기존 `lane-only`의 노란색/흰색 후보를 재사용한 뒤 `bird's-eye 변환 → sliding
window → 좌우 2차 곡선 fitting → 역원근 overlay`를 수행합니다. 초록색은 주행
영역, 노란색/흰색은 좌·우 차선, 청록색은 차선 중심입니다. 곡률과 차량의 차선
중심 이탈량도 영상 왼쪽 위에 표시됩니다.

```bash
uv run python tools/segment_video.py \
  --backend advanced-lane \
  --input /path/to/driving_video.mp4 \
  --output /path/to/advanced_lane_overlay.mp4
```

실차 영상에서는 카메라 장착 상태에 맞게 `VisionConfig.perspective_source`를 먼저
보정해야 합니다. 차선 폭·가시 거리는 각각 `--advanced-lane-width-m`,
`--advanced-lane-visible-distance-m`으로 맞출 수 있으며, 검출이 끊기면
`--advanced-lane-margin-px`와 `--advanced-lane-min-points`를 조정합니다. 이
backend는 테스트 영상용이며 ROS의 기존 단일 중앙선 제어 출력은 변경하지 않습니다.

한 색상만 확인하려면 `--lane-color yellow` 또는 `--lane-color white`를
추가합니다. 두 색상을 동시에 표시할 때 겹치는 Hough 선분은 빨간색
(`OVERLAP`)으로 표시됩니다. `lane-only`는 원본 입력 영상에서 실행해야 하며, 이미 overlay가
입혀진 결과 영상을 다시 입력으로 사용하면 overlay 자체가 검출 후보가 됩니다.

처리 순서는 `노란색/흰색 HSV 후보 → 그레이스케일 Gaussian blur → Canny →
대칭 하단 ROI → 색상 지지 HoughLinesP`입니다. 최소 선 길이는
`--lane-min-length-px`, 선분 연결 간격은 `--lane-max-gap-px`, 선분 방향 조건은
`--lane-min-vertical-ratio`, 색상 지지율은 `--lane-min-color-support-ratio`로
조정할 수 있습니다. 현재 카메라의 색상 편향 때문에 노란색에는 별도의 hue wrap과
BGR 채널 차이 조건도 적용합니다.
흰색은 현재 카메라의 보라색 아스팔트가
흰색 후보로 번지는 것을 줄이기 위해 기본적으로 `S<=40`, `V>=200`으로
제한하며, 다른 카메라에서는 `--lane-white-saturation-max`와
`--lane-white-value-min`으로 조정할 수 있습니다.

선 구조를 먼저 찾고 그 선 후보 안에서 노란색을 segmentation하려면
`line-first` backend를 사용합니다. 청록색은 Canny/Hough 선 후보 주변,
노란색은 색상·형태 조건까지 통과한 최종 중앙선입니다.

YOLOP 도로 영역 안에서 OpenCV 선 후보만 확인하려면 `road-lines` backend를
사용합니다. 이 단계에서는 노란색 판정이나 중앙선 선택을 수행하지 않고,
초록색으로 YOLOP 도로 전체 mask, 청록색으로 도로 mask 내부의 Canny/Hough
선 후보만 표시합니다.

```bash
uv run python tools/segment_video.py \
  --backend road-lines \
  --input /Users/kangminwoo/Downloads/roadline_test.mp4 \
  --output /Users/kangminwoo/Downloads/road_lines_only_validation.mp4 \
  --model models/yolop-720-1280.onnx \
  --profile 720p
```

YOLOP 없이 대칭 ROI 안의 회색 도로를 OpenCV로 먼저 mask하고, 그 mask 안에서
선 후보만 찾으려면 `gray-road-lines` backend를 사용합니다. 카메라의 색상 편향을
고려해 ROI 하단 중앙의 LAB 색도를 기준으로 회색 도로를 적응적으로 분리합니다.
이 backend도 노란색 판정과 중앙선 선택은 수행하지 않습니다. 초록색은 OpenCV
회색 도로 mask, 청록색은 그 mask 내부의 Canny/Hough 선 후보, 흰색은 ROI입니다.

```bash
uv run python tools/segment_video.py \
  --backend gray-road-lines \
  --input /path/to/input.mp4 \
  --output /path/to/output_gray_road_lines.mp4
```

현재 검증 영상에서 최소 선 길이 60px, 표시 폭 7px로 실행하려면:

```bash
uv run python tools/segment_video.py \
  --backend gray-road-lines \
  --input /Users/kangminwoo/Downloads/roadline_test.mp4 \
  --output /Users/kangminwoo/Downloads/gray_road_lines_validation.mp4 \
  --road-line-min-length-px 60 \
  --road-line-corridor-width-px 7
```

회색 도로 분리는 `--gray-road-lab-tolerance`, `--gray-road-min-luminance`,
`--gray-road-max-luminance`, `--gray-road-top-y`로 조정하고, 선 검출은 `--road-line-min-length-px`,
`--road-line-max-gap-px`, `--road-line-hough-threshold`로 조정합니다.

OpenCV 도로 mask 안에서 `lane-only`처럼 노란색/흰색을 각각 색상 필터링한 뒤
Canny와 HoughLinesP로 검출하려면 `gray-road-lane-only`를 사용합니다. 이 backend는
YOLOP를 사용하지 않으며, 초록색은 OpenCV 도로 mask, 노란색/흰색은 해당 색상의
선분, 빨간색은 두 선분이 겹친 부분입니다. `--lane-color yellow` 또는
`--lane-color white`로 한 색상만 표시할 수 있습니다.

```bash
uv run python tools/segment_video.py \
  --backend gray-road-lane-only \
  --lane-color both \
  --input /Users/kangminwoo/Downloads/roadline_test.mp4 \
  --output /Users/kangminwoo/Downloads/gray_road_lane_validation.mp4 \
  --lane-min-length-px 60 \
  --lane-draw-width-px 5
```

```bash
uv run python tools/segment_video.py \
  --backend line-first \
  --input /path/to/input.mp4 \
  --output /path/to/output_line_first.mp4
```

검출 민감도는 `--line-first-canny-low`, `--line-first-canny-high`,
`--line-first-hough-threshold`, `--line-first-min-length-px`,
`--line-first-max-gap-px`, `--line-first-corridor-width-px`로 조정합니다.
굵은 도색은 Hough로 찾은 양쪽 경계를 씨앗으로 사용해 전체 색상 영역을
복원합니다. 복원 폭은 `--line-first-recovery-width-px`, 양쪽 경계를 하나의
마스크로 합치는 폭은 `--line-first-band-close-kernel-px`로 조정합니다.

## MCAP 카메라 토픽을 MP4로 변환

ROS 2 설치 없이 MCAP rosbag의 `sensor_msgs/msg/Image` 또는
`sensor_msgs/msg/CompressedImage` 카메라 토픽을 MP4로 변환할 수 있습니다.
입력 bag은 read-only로 열며, FPS를 생략하면 메시지 timestamp의 중앙값 간격으로
자동 계산합니다. FFmpeg와 Python 패키지 `mcap`, `mcap-ros2-support`가 필요합니다.

```bash
uv run --with mcap --with mcap-ros2-support \
  python tools/rosbag_mcap_to_mp4.py \
    --input "$HOME/Downloads/20260827_063215_teamgrit_rosbag" \
    --topic /a2/front_camera/res_360p/image_raw \
    --output rosbag-results/20260827_063215_camera.mp4
```

`--input`에는 rosbag 디렉터리 또는 단일 `.mcap` 파일을 줄 수 있습니다. 긴 bag의
일부만 확인하려면 `--max-frames 100`, 용량과 처리 시간을 줄이려면
`--frame-step 2`를 사용합니다. 기존 출력 파일을 교체하려면 `--overwrite`를
명시해야 합니다.

AI 모델 없이 원본 카메라 영상의 노란 중앙선을 확인하려면 OpenCV backend를
사용합니다. 이 카메라는 보라색 색감이 강해 BGR의 R-B/R-G 채널 차이와
LAB 조건, 중앙 도로 ROI, PCA 기반 선 형태(주축/부축 비율), 가장 큰 연속
선 후보를 함께 사용합니다. 일반 HSV 검출은 warm-camera 후보가 없는
프레임에서만 fallback으로 사용합니다.

```bash
uv run python tools/segment_video.py \
  --backend opencv \
  --input /path/to/input.mp4 \
  --output /path/to/output_opencv_yellow.mp4
```

OpenCV 결과의 노란색은 실제 중앙선 색상 검출 mask이고, 흰색 선은 좌우 대칭
ROI 경계입니다. 필요하면 `--hsv-lower H S V`, `--hsv-upper H S V`,
`--lab-b-min N`, `--red-blue-min N`, `--red-green-min N`으로 조정할 수
있습니다. 선 형태 필터 기준은 `--line-min-elongation N`으로 조정하고,
비교 테스트가 필요하면 `--no-line-feature`로 끌 수 있습니다. 다른 카메라는
`line_roi_polygon`을 카메라에 맞게 보정해야 합니다.

다른 카메라/제어 토픽을 쓰는 경우 `.env`의 `IMAGE_TOPIC`과 `JOY_TOPIC`을
명시적으로 바꿉니다. `CONTROL_TOPIC`은 `JOY_TOPIC`의 별칭입니다. 같은 설정은
`ros_ws/src/line_tracking/config/line_tracking.yaml`의 `image_topic`과
`joy_topic`을 직접 편집해도 됩니다. `.env` 값이 있으면 launch 시 config 값을
override합니다.

카메라 해상도에 맞추려면 `.env`에서 `CAMERA_PROFILE=360p` 또는 `720p`를
선택합니다. profile은 각각 `yolop-360-640.onnx`/`yolop-720-1280.onnx`와
입력 크기(각각 640x384/1280x736)를 함께 선택합니다. 모델 export와 파일 배치는
`models/README.md`를 참고합니다.

## 보정과 튜닝

운영 설정은
`ros_ws/src/line_tracking/config/line_tracking.yaml`이며 Compose가 read-only로
mount하므로 값 변경 후 `docker compose restart`로 반영할 수 있습니다.

1. 정지 상태에서 debug image를 보며 `roi_polygon`을 실제 도로 영역에 맞춥니다.
2. 지면의 알려진 네 점을 이용해 `perspective_source`와
   `near_distance_m`, `far_distance_m`, `half_width_m`을 보정합니다.
3. YOLOP의 `road_threshold`, `line_threshold`, `road_gate_kernel`을 조정합니다.
4. 맑음, 흐림, 그늘, 젖은 노면 영상을 현장 데이터로 fine-tuning합니다.
5. 낮은 속도에서 `lateral_kp`, `lateral_kd`, `yaw_kp` 순으로 조정합니다.
6. 마지막에 속도 제한과 confidence threshold를 올립니다.

YOLOP 모델 경로가 비어 있을 때만 HSV/LAB fallback의 밝기 보정과 색상 임계값이
사용됩니다. 운영 모드에서는 모델의 도로 마스크로 line mask를 먼저 gating하므로
풀숲이나 인도의 노란색 물체가 경로 추종에 직접 들어가지 않습니다.

## 로컬 테스트

OpenCV, NumPy, pytest가 설치된 환경에서:

```bash
python3 -m pytest \
  ros_ws/src/line_tracking/test \
  test
docker compose config --quiet
```

## Mapillary segmentation profile 전환과 Swin-L 복구

선택된 Mapillary 기본 profile은 **`swin-l-aspect-224x384`**입니다. 같은
checkpoint의 기존 정사각형 `swin-l-best-so-far`와 R50은 비교·복구용으로
남겨 두었습니다. 이 기본값은 benchmark/전체 영상/Swin-L 디버그·MCAP 도구에
적용되며, 기존 `line-tracking` 서비스의 YOLOP 주행 코드를 몰래 바꾸지는
않습니다. Swin-L 실제 주행은 아래의 별도 `drive-swin-l` 서비스로 연결합니다.

```bash
# 과거 384x384 결과 재현
uv run tools/benchmark_best_so_far.py mcap \
  --profile swin-l-best-so-far \
  --input /path/to/input.mcap \
  --output-report rosbag-results/benchmarks/swin-l-restored.json
```

선택된 `swin-l-aspect-224x384`의 고정 계약은 다음과 같습니다.

- model: `facebook/mask2former-swin-large-mapillary-vistas-semantic`
- revision: `4772b6bf101d91f2534c106dc524d906aeb3c68a`
- model input: `224x384`, score map: `640x360`, precision: FP32
- temporal alpha `0.62`, hysteresis margin `0.07`
- Road/Bike Lane/Crosswalk/Parking/Service Lane/Lane Marking을 Road로 통합
- Sidewalk/Pedestrian Area/Curb Cut을 Sidewalk로 통합

2026-09-16 전방 카메라 `test-one` 검증에서 16:9에 가까운 입력 크기를 쓰는
`swin-l-aspect-224x384`는 16:9 카메라에 가까운 모델 입력을 사용하며,
위 결과를 재현하기 위해 기본값으로 고정했습니다. 구형 384×384 프로필은
명시적으로 선택할 때만 사용합니다. 두 참조 overlay는
ADE20K B5 모델 출력이므로 정확한 수동 라벨이 아니며, 실제로 27초와 57~58초
그늘진 타일을 Road로 잘못 칠하는 구간이 있습니다. 표본 비교와 재현 명령은
`rosbag-results/test-one-swin-validation-20260916/INITIAL_REPORT.md`에 있습니다.

화질 우선 실험 프로필 `swin-l-aspect-448x768`도 선택할 수 있습니다. 두 영상의
표본 비교에서 224×384보다 참조 일치도가 높았고, 두 번째 영상의 69~70초
보도→도로 오분류를 줄였습니다. 대신 이 장비의 연속 80프레임 실험에서
처리 시간이 프레임당 약 0.18초에서 0.51초로 늘었습니다. 현장 정확도를
보증하는 기본값은 아니므로 주행 프로필로 선택하지 않았습니다.

## MCAP overlay 테스트를 명령 한 줄로 실행

Mac/일반 PC에서는 저장소 루트에서 `uv`로 실행한다. Jetson은 아래의
[MCAP 테스트 컨테이너](#jetson에서-mcap-overlay-테스트)를 사용한다.
두 명령 모두 위의 `swin-l-aspect-224x384`
모델·revision·FP32·224×384 입력·temporal 설정을 고정한다. `.env`의
다른 모델 선택은 적용하지 않으며, Local Path 기하 설정은 기존 `.env`를 사용한다.

```bash
# 1. 인도 검출 overlay: 원본 카메라의 모든 프레임을 추론
uv run tools/swin_l_rosbag_overlay.py sidewalk --input /path/to/input.mcap --open

# 2. Local Planning overlay: 인도 중심 경로·평활·LiDAR 상태
uv run tools/swin_l_rosbag_overlay.py local-path --input /path/to/input.mcap --open
```

기본은 처음 200프레임(현재 20Hz bag에서 약 10초)이다. 전체를 처리하려면
`--max-frames 0`, 문제 구간부터 보려면 `--start-offset 90`을 추가한다.
입력은 단일 `.mcap`이며 기본 토픽은 카메라
`/a2/front_camera/res_360p/image_raw`, LiDAR `/unitree/slam_lidar/points2`다.
다른 bag은 `--image-topic`, `--lidar-topic`, `--output-fps`로 맞춘다.

완료하면 영상이 자동으로 열리고, 터미널에 MP4와 JSON의 절대경로가 나온다.
매 실행마다 `rosbag-results/swin-l-tests/` 아래 새 폴더를 만든다. 영상만
생성하려면 `--open`을 생략한다. 첫 실행에는 모델과 의존성을 내려받는다.

- `sidewalk`: 초록=도로, 마젠타=인도. 경로와 LiDAR를 계산하지 않는다.
- `local-path`: 초록=차도, 마젠타=인도, 주황=추정 경로, 흰색=평활 경로. 경로 대상은
  `SWIN_L_PATH_MASK_CLASS`로 선택한다. 기존 4Hz 목표 추론과 hold를 재사용하므로
  `TRACKED`라도 이전 경로일 수 있다. 장애물 우회 planner는 아니다.

## Swin-L 선택 영역 중심 local path 디버그

`tools/swin_l_local_path_debug.py`는 Swin-L profile에서 선택한 인도 또는 차도
mask를 카메라 전방 3~8m의 metric bird's-eye grid로 옮긴 뒤, 각 거리에서 영역의 중심을
추출해 `base_link` 기준 `nav_msgs/Path`로 만든다. 매 프레임마다 경로를
갈아끼우지 않고 최신 카메라 프레임만 유지하는 depth-1 큐, Swin-L 기본 추론
4Hz, 0.8초 EMA, 0.9초 경로 hold를 사용한다. 따라서 출력 타이머는 기본 10Hz여도
실제 Swin-L 추론이 4Hz보다 느리면 유효한 경로 갱신은 더 느려질 수 있다.

유효한 결과는 `local_path.poses`가 2개 이상이고 metrics에서
`path_tracked=true`, `path_confidence>0`인 상태다. `ros2 topic hz`가 약 10Hz라는
것만으로 경로가 갱신되는 것은 아니다. `poses=[]`, `path_tracked=false`,
`reason=path_unavailable`이면 영상은 들어오지만 선택된 영역의 중심선을 추출하지
못한 상태다. LiDAR가 연결되어 있어도 mask/ROI/
homography가 맞지 않으면 이 상태가 된다.

LiDAR가 오래되었거나 path corridor 안에 3m 이내의 점이 3개 이상 있으면
`safety_stop` 디버그 토픽이 `true`가 된다. 이 프로세스는 `/a2_control`을 발행하지
않으므로 기존 제어 노드와 분리된 검사 전용이다.

### 토픽과 주요 설정

영상과 LiDAR 토픽은 `.env`에서 정의한다. 첨부 rosbag의 640x360 스트림은
`res_360p` 토픽을 사용하지만, 현재 Jetson의 1280x720 카메라 스트림은 보통
`/a2/front_camera/image_raw`를 사용한다. LiDAR도 장치에 따라 `points1` 또는
`points2`가 될 수 있으므로 실제 토픽 목록과 컨테이너 로그를 확인한다.

| 방향 | `.env` 변수 | 예시 |
|---|---|---|
| 입력 영상 | `SWIN_L_IMAGE_TOPIC` | `/a2/front_camera/image_raw` |
| 입력 LiDAR | `SWIN_L_LIDAR_TOPIC` | `/unitree/slam_lidar/points1` |
| 경로 대상 클래스 | `SWIN_L_PATH_MASK_CLASS` | `2`=인도(기본), `1`=차도 |
| 출력 overlay (주행 모드만) | `SWIN_L_OVERLAY_TOPIC` | `/line_tracking/swin_l/overlay` |
| 출력 경로 | `SWIN_L_LOCAL_PATH_TOPIC` | `/line_tracking/swin_l/local_path` |
| 안전 상태 | `SWIN_L_SAFETY_STOP_TOPIC` | `/line_tracking/swin_l/safety_stop` |
| 여유 거리 (주행 모드만) | `SWIN_L_CLEARANCE_TOPIC` | `/line_tracking/swin_l/clearance_m` |
| 진단 metrics | `SWIN_L_METRICS_TOPIC` | `/line_tracking/swin_l/metrics` |

`debugging-swin-l`에서는 경로·안전 상태·metrics만 발행한다. LiDAR 여유 거리의
상세값은 별도 토픽 대신 `metrics.lidar.clearance_m`에서 확인할 수 있다.
`metrics.path_mask_class`와 `metrics.path_surface`에서 현재 선택을 확인할 수 있다.
`0`(배경)과 그 밖의 `.env` 값은 시작 시 거부한다. `.env`를 바꾼 뒤에는 해당 컨테이너를
재생성해야 적용된다. Cobiz `LINE_TRACKING` 작업의 `payload.selected_mask`로는
`1` 또는 `2`를 지정할 수 있으며, 수락된 작업에만 적용된다. 모델이 분할한 두
영역의 경로를 각각 유지해 요청한 영역의 경로·LiDAR 판정으로 작업을 수락한다.
인도→차도 자동 대체는 하지 않는다.
차도 추종을 실제 주행에 적용하기 전에는 카메라 원근 보정, 경로 폭·중심선과
LiDAR 안전 구간을 차도 장면에서 별도로 검증해야 한다.

카메라 입력 해상도와 모델 평가 해상도는 별개다. 예를 들어 카메라가 1280x720이어도
`SWIN_L_EVALUATION_WIDTH=640`, `SWIN_L_EVALUATION_HEIGHT=360`으로 두면 모델은
640x360으로 평가한다. 이는 Swin-L 계산량을 줄이기 위한 설정이며 원본 토픽의
해상도를 변경하지 않는다.

`SWIN_L_ROI_POLYGON`과 `SWIN_L_GROUND_HALF_WIDTH_M`은 카메라 pitch와 장착 위치에
따라 반드시 현장에서 보정해야 한다. Rosbag에는 `CameraInfo`는 있지만
camera-to-base extrinsic/TF가 없으므로 기본 homography는 초기 디버그값이다.
LiDAR는 x=전방, y=왼쪽으로 `base_link`에 정렬되어 있다고 가정한다. 실차에서는
extrinsic을 확인한 뒤 `SWIN_L_LIDAR_Z_*`, corridor 폭과 stop 거리를 조정한다.

### Rosbag을 동영상으로 먼저 확인

ROS 2 없이 같은 rosbag을 동영상 overlay로 확인할 수 있다. camera 20Hz 출력
프레임은 유지하면서 Swin-L update만 기본 4Hz로 실행하고 LiDAR 상태와 raw/
평활 경로를 overlay한다.

```bash
uv run tools/swin_l_local_path_debug.py mcap \
  --input /Users/kangminwoo/Downloads/20260827_062352_teamgrit_rosbag_0.mcap \
  --output rosbag-results/swin-l-local-path.mp4 \
  --report rosbag-results/swin-l-local-path.json \
  --max-frames 400
```

Overlay에서 마젠타는 Swin-L 인도 mask, 주황색은 최신 raw 중심선, 흰색은 평활된
local path다. 기본 `ros2` 모드는 디버깅 전용이며, 아래 별도 `drive` 모드에
연결하기 전 homography, LiDAR frame 정렬, 장애물 z 범위를 검증해야 한다.

## Swin-L 실제 주행 모드 (보정 확인 전에는 비활성)

기본 `actual-activate`는 Cobiz 작업 수신형입니다. 수동 `drive-swin-l`은
별도 `drive` 프로필로 남겨 두었으며, 둘을 동시에 실행하면 안 됩니다.
`drive-swin-l`은 위 224×384 checkpoint를 강제하고, Swin-L 보도 중심 경로와
LiDAR gate를 이용해 `/a2_control` Joy를 **최대 0.10m/s, 0.18rad/s**로 발행한다.
카메라/추론/경로가 오래되거나 경로 신뢰도가 낮거나 LiDAR가 없거나 장애물이
가깝거나 다른 control publisher가 보이면 10Hz로 영속적인 0 명령을 보낸다.
시작 후 2초 동안도 0 명령만 보내고, LiDAR PointCloud2의 `frame_id`가
`base_link`가 아니면 변환 없이 사용하는 대신 즉시 정지한다.
카메라와 LiDAR의 ROS timestamp가 현재 시스템 시각과 맞지 않거나 `/clock`
시뮬레이션 시간이 활성화되어 있어도 주행하지 않는다.
`debugging-swin-l`에는 Joy publisher가 없다.

현재 `.env.example`의 카메라 homography 및 LiDAR `base_link` 정렬은 현장
실측값이 아니므로 **이 저장소에서는 주행 모드를 켜지 않는다.** Jetson에서
좌표계·거리·토픽·CUDA 처리 지연을 검증하고 물리 비상정지 수단을 준비한
후에만 `.env`의 `SWIN_L_CALIBRATION_CONFIRMED=true`와
`SWIN_L_DRIVE_ENABLED=true`를 직접 설정한다. 코드/컨테이너의 기본값은
둘 다 `false`다. 기존 `line-tracking`의 `/a2_control` 발행을 중지하고,
동시에 두 주행 서비스를 실행하지 않는다.

```bash
# 보정과 비상정지 검증 후 Jetson에서만 실행. 이 명령은 실제 움직임을 유발한다.
docker compose stop actual-activate line-tracking debugging-swin-l
docker compose --profile drive up -d --build drive-swin-l
docker compose logs -f drive-swin-l
# 중지
docker compose stop drive-swin-l
```

ROS metrics의 `drive_reason`이 `tracking`일 때만 비영(非零) Joy가 발행된다.
앱의 실주행 검증을 실행했다는 뜻은 아니며, 현장 보정·비상정지·속도 검증이
끝나기 전에는 두 플래그를 활성화하지 말아야 한다.

## Docker debug 컨테이너

아래 구성은 Jetson에서 `debugging-swin-l`만 실행해 local path와 metrics를 확인하고
로봇을 주행시키지 않는 절차다. `debugging-swin-l`은 `/a2_control`을 발행하지
않으므로 주행용 `line-tracking` 서비스와 분리해서 사용할 수 있다.

`jetson-containers`는 이 저장소 안에 있을 필요가 없다. Jetson 호스트의 별도
디렉터리(예: `~/tools/jetson-containers`)에 clone해도 되며, 이미지 build가 끝난
뒤 계속 실행 중일 필요도 없다.

```bash
cd ~/dev/dangjin-a2/cobiz-plugin-line-tracking
test -f .env || cp .env.example .env
```

현재 Jetson 카메라가 1280x720이라면 `.env`에서 입력 토픽만 실제 장치에 맞추고,
모델 평가 해상도는 640x360으로 유지한다.

```dotenv
SWIN_L_IMAGE_TOPIC=/a2/front_camera/image_raw
SWIN_L_LIDAR_TOPIC=/unitree/slam_lidar/points1
SWIN_L_EVALUATION_WIDTH=640
SWIN_L_EVALUATION_HEIGHT=360
SWIN_L_BASE_IMAGE=cobiz:jetson-swin-l-l4t-r36.5.0
```

첨부 rosbag처럼 640x360 stream을 재생할 때는
`/a2/front_camera/res_360p/image_raw`를 사용한다. LiDAR도 장치에 따라 `points1`
또는 `points2`가 될 수 있으므로 `ros2 topic list`와 컨테이너 로그의
`image=... lidar=...`를 기준으로 `.env`를 맞춘다.

```bash
# Jetson 호스트에서 한 번 수행한다. clone 위치는 프로젝트 폴더와 달라도 된다.
cd ~/tools/jetson-containers
jetson-containers --help
docker image inspect cobiz:jetson >/dev/null

# cobiz:jetson에는 ROS Humble이 이미 있으므로 ROS/OpenCV/FFmpeg stage는
# 다시 빌드하지 않고 Jetson CUDA 12.6용 PyTorch만 추가한다.
PYTORCH_VERSION=2.8 CUDA_VERSION=12.6 \
jetson-containers build \
  --base=cobiz:jetson \
  --name=cobiz:jetson-swin-l \
  --skip-packages=ffmpeg,opencv,ros \
  pytorch:2.8
```

이 명령은 dependency stage를 여러 개 만들 수 있어 시간이 오래 걸린다.
`--simulate`는 dependency 해석만 보여주고 Docker image를 만들지 않는다. 실제
image가 필요한 경우에는 위의 `build` 명령을 실행해야 한다. FFmpeg/OpenCV/ROS
stage를 포함한 기존 시도에서 각각 `dav1d`, OpenCV package, rosdep default
source 중복 문제가 발생했기 때문에 `cobiz:jetson`에서는 해당 stage를 건너뛴다.

```bash
# Jetson-containers가 L4T 태그를 자동으로 붙이는지 확인한다.
docker images 'cobiz:jetson-swin-l*'

# 최종 alias가 없고 intermediate tag만 있는 경우에는 재빌드하지 않고
# alias만 추가한다.
if ! docker image inspect cobiz:jetson-swin-l-l4t-r36.5.0 >/dev/null 2>&1; then
  docker image inspect cobiz:jetson-swin-l-l4t-r36.5.0-pytorch_2.8 >/dev/null
  docker tag \
    cobiz:jetson-swin-l-l4t-r36.5.0-pytorch_2.8 \
    cobiz:jetson-swin-l-l4t-r36.5.0
fi

# Compose build 전 CUDA PyTorch와 ROS를 빠르게 검증한다.
docker run --rm --runtime=nvidia \
  --entrypoint python3 \
  cobiz:jetson-swin-l-l4t-r36.5.0 \
  -c 'import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())'

docker run --rm --runtime=nvidia \
  --entrypoint bash \
  cobiz:jetson-swin-l-l4t-r36.5.0 \
  -lc 'source /opt/ros/humble/setup.bash; python3 -c "import rclpy; print(rclpy.__file__)"'

# 위 base image가 제공하는 PyTorch를 사용하고, Compose 빌드에서
# Transformers, Jetson CUDA 12.6용 torchvision, cv_bridge와 Cyclone DDS를
# 보강한다. 정상적인 핵심 출력은 `2.8.0 12.6 True`다.
cd ~/dev/dangjin-a2/cobiz-plugin-line-tracking
docker compose stop line-tracking
docker compose up -d --build debugging-swin-l
docker compose logs -f debugging-swin-l
```

Compose profile을 명시하고 싶다면 같은 작업을 다음처럼 실행할 수 있다.

```bash
docker compose --profile debug up -d --build debugging-swin-l
```

컨테이너는 다음 토픽만 디버깅용으로 발행한다.

```text
/line_tracking/swin_l/local_path
/line_tracking/swin_l/safety_stop
/line_tracking/swin_l/metrics
```

오버레이 이미지 복사·렌더링과 별도 `clearance_m` 토픽 발행은 디버그 모드에서
실행하지 않는다. LiDAR 여유 거리와 원인 코드는 `metrics`의 `lidar` 항목에 남는다.

호스트에서 결과를 확인한다.

```bash
ros2 topic echo /line_tracking/swin_l/local_path
ros2 topic echo /line_tracking/swin_l/safety_stop
ros2 topic echo /line_tracking/swin_l/metrics
rviz2  # Fixed Frame=base_link, Path topic=/line_tracking/swin_l/local_path
```

출력 타이머 기본값은 10Hz이고 Swin-L 목표 추론률은 4Hz다. 따라서 10Hz가
측정되어도 이전 경로를 재발행하는 중일 수 있다. 다음을 함께 판단한다.

```text
정상: poses >= 2, path_tracked=true, path_confidence > 0
비정상: poses == [], path_tracked=false, reason=path_unavailable
```

2026-09-03의 이전 구성에서 기록한 MCAP에서는 다섯 output topic이 약 10Hz였지만
20.5초 동안
`local_path.poses`가 계속 비어 있고 `path_unavailable`이었다. overlay 자체는
갱신됐으므로 이 경우는 publisher 고장이 아니라 인도 mask/ROI/homography가
유효한 중심선을 만들지 못한 상황이다. 실내 영상, 인도가 보이지 않는 장면,
카메라 pitch가 기본값과 다른 경우를 먼저 확인하고 ROI와 homography를 보정한다.
LiDAR가 `lidar_available=true`인데도 Path가 비어 있다면 LiDAR보다 영상 경로
추출을 먼저 점검한다.

### MCAP으로 재현 결과 저장

결과만 짧게 기록하면 카메라 원본을 다시 저장하는 것보다 디스크를 크게 아낄 수
있다. Jetson host에서 실행하고 Ctrl-C로 중지한다.

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/rosbags/swin_l
ros2 bag record -s mcap \
  -o ~/rosbags/swin_l/debug_result_$(date +%Y%m%d_%H%M%S) \
  /line_tracking/swin_l/local_path \
  /line_tracking/swin_l/safety_stop \
  /line_tracking/swin_l/metrics
```

원인 분석을 위해 입력까지 기록할 때만 카메라와 LiDAR 토픽을 추가한다.
1280x720 RGB 영상은 약 20Hz에서 분당 수 GB가 될 수 있고 LiDAR도 수십 MB/s가
될 수 있으므로 짧게 기록한다.

```bash
ros2 bag info ~/rosbags/swin_l/debug_result_YYYYMMDD_HHMMSS
mcap info ~/rosbags/swin_l/debug_result_YYYYMMDD_HHMMSS/*.mcap
```

### Jetson 부하와 통신 확인

`network_mode: host`이므로 `docker stats`의 Net I/O가 0으로 보여도 실제 ROS
트래픽이 없다는 뜻은 아니다. 다음 명령으로 host 인터페이스와 컨테이너 부하를
확인한다.

```bash
docker stats cobiz-plugin-line-tracking-debugging-swin-l
tegrastats
ip -s link show enP8p1s0
ros2 topic bw /a2/front_camera/image_raw
ros2 topic bw /unitree/slam_lidar/points1
```

`ros2 topic bw`는 측정 subscriber가 수신한 payload 기준이므로 실제 wire
utilization과 완전히 같지 않으며, 측정을 위해 임시 subscriber 트래픽도 만든다.
Jetson의 `enP8p1s0`↔A2 `eth0` 링크는 1Gbps full-duplex이고, 직접 TCP 측정은
약 905~913Mbps였다. debug 컨테이너가 실행 중일 때 관측한 payload는 카메라
40~47MB/s, LiDAR 약 30MB/s 수준까지 나와 합계 약 560~616Mbps가 될 수 있다.
`/livox/lidar`를 A2로 전달하는 경우에는 약 0.52MB@10Hz, 즉 42Mbps 정도를
추가로 예상한다. Swin-L 추론이 Jetson CPU/GPU를 크게 사용할 수 있으므로 녹화
시간과 model input 해상도를 제한한다.

### 종료와 주행 안전

일반 `docker compose up -d`는 `actual-activate` 작업 수신기를 시작하지만
보정 플래그가 꺼져 있거나 수락된 작업이 없으면 Joy publisher는 없다. 현장
보정 완료 후 서버의 `LINE_TRACKING` 작업을 받으면 실제 움직임을 유발할 수
있다. local path만 확인할 때는 debug 서비스를 명시한다.

```bash
docker compose stop line-tracking
docker compose stop actual-activate
docker compose stop debugging-swin-l
docker compose rm -f debugging-swin-l
```

모델은 `${SWIN_L_MODEL_CACHE_DIR:-./.cache/huggingface}`에 캐시되어 다음
컨테이너 재생성 때 재사용된다. 처음 실행 시 Swin-L checkpoint 다운로드와
Docker image build 때문에 시간이 오래 걸릴 수 있으며, Jetson의 여유 디스크도
미리 확인한다.

### Jetson에서 MCAP overlay 테스트

`test-swin-l`은 위 실시간 `debugging-swin-l`과 같은 Dockerfile·CUDA 이미지를
사용한다. 녹화된 MCAP을 직접 읽으므로 `ros2 bag play`, ROS/DDS source와
실시간 토픽 연결은 필요 없다. 컨테이너의 `/opt/venv/bin/python`으로 실행해
Jetson용 PyTorch 2.8/CUDA 12.6을 유지한다. 여기서는 `uv run`을 사용하지 않는다.

**최초 준비 — Jetson 호스트의 저장소 루트에서:** 기존 실시간 디버깅 이미지가
있어도 새 테스트 스크립트를 포함하도록 Compose 이미지를 다시 빌드한다.
`SWIN_L_BASE_IMAGE`가 아직 없다면 위 Docker debug 절차의 base 이미지 빌드를
먼저 마친다. 아래 rosbag 디렉터리는 실제 `.mcap` 파일이 있는 폴더로 바꾼다.

```bash
cd ~/dev/dangjin-a2/cobiz-plugin-line-tracking
test -f .env || cp .env.example .env
export SWIN_L_ROSBAG_DIR="$HOME/rosbags"
export SWIN_L_TEST_UID="$(id -u)" SWIN_L_TEST_GID="$(id -g)"
export SWIN_L_TEST_RESULTS_DIR="$PWD/rosbag-results/swin-l-tests"
export SWIN_L_TEST_CACHE_DIR="$PWD/.cache/huggingface-tests"
test -f "$SWIN_L_ROSBAG_DIR/20260827_062352_teamgrit_rosbag_0.mcap"
mkdir -p "$SWIN_L_TEST_RESULTS_DIR" "$SWIN_L_TEST_CACHE_DIR"
docker compose build test-swin-l
docker compose run --rm --no-deps --entrypoint /opt/venv/bin/python test-swin-l -c 'import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available()); assert torch.cuda.is_available(), "Jetson CUDA unavailable"'
```

CUDA 확인 출력은 `2.8.0 12.6 True` 계열이어야 한다. 각 `export`는 새 터미널에서
다시 실행하거나 `.env`에 실제 절대경로·UID·GID 값으로 저장한다. 테스트 캐시는
호스트 사용자 권한으로 쓰기 위해 실시간 디버깅 캐시와 별도로 둔다. 최초 실행은
Swin-L 모델 다운로드가 필요하고 다음 실행부터 같은 캐시를 재사용한다.

**테스트 — 같은 터미널에서 각각 한 줄:**

```bash
# 1. 인도 검출 overlay
docker compose run --rm --no-deps test-swin-l sidewalk --input /bags/20260827_062352_teamgrit_rosbag_0.mcap --device cuda

# 2. Local Planning overlay
docker compose run --rm --no-deps test-swin-l local-path --input /bags/20260827_062352_teamgrit_rosbag_0.mcap --device cuda
```

`/bags`는 `SWIN_L_ROSBAG_DIR`의 읽기 전용 mount다. 두 명령은 동일한 최고 성능
선택된 Swin-L 224×384 모델·revision·FP32 설정을 고정하고 기본 200프레임을 처리한다.
전체는 `--max-frames 0`, 90초 이후는 `--start-offset 90`을 추가한다. 카메라·LiDAR
기본 토픽은 앞의 MCAP 테스트와 같고, `.env`의 실시간 토픽보다 CLI 기본값이
우선한다. 다른 토픽은 `--image-topic`, `--lidar-topic`으로 명시한다.

**결과 확인:** 터미널의 `VIDEO=`·`REPORT=`에서 `/workspace/`를 호스트의 저장소
경로로 바꾸면 된다(위 기본 결과 경로 기준). 실행마다 새 폴더에
`sidewalk-overlay.mp4`·`sidewalk-report.json` 또는
`local-path-overlay.mp4`·`local-path-report.json`이 생성된다.
컨테이너가 종료되어도 호스트의 `rosbag-results/swin-l-tests/`에 남는다.
Jetson 데스크톱에서 MP4를 열거나 SSH 작업 시 PC로 복사해 재생한다.
`--open`은 컨테이너 안에서 사용하지 않는다.

```bash
# Jetson 데스크톱: VIDEO=에서 확인한 호스트 경로를 넣는다.
xdg-open /absolute/path/to/sidewalk-overlay.mp4
```

이 테스트의 Local Path 4Hz 설정은 bag 시간에 따른 추론 간격이다. 처리 속도가
실시간 4Hz라는 뜻은 아니며, 실제 Jetson 속도는 별도 벤치마크로 확인한다.

비교용 `r50-fp16-640x360`은 별도로 선택할 수 있으며
다음 실행 계약을 사용합니다.

- model: `facebook/maskformer-resnet50-vistas`
- revision: `ae4b8c2590c0a090fc32d5c217d78738a2dd4b19`
- native `640x360` input, FP16 on MPS/CUDA, `640x360` score map
- CPU에서는 호환성을 위해 FP32로 자동 fallback

두 profile의 같은 프레임 결과와 속도를 직접 비교하려면:

```bash
uv run tools/compare_segmentation_profiles.py \
  --input /path/to/camera.mp4 \
  --start-frame 0 \
  --max-frames 200 \
  --output-dir rosbag-results/profile-comparisons
```

## best-so-far 실시간 Hz 벤치마크

첨부 rosbag에서 확인한 카메라 계약은
`/a2/front_camera/res_360p/image_raw`, `sensor_msgs/msg/Image`, RGB8,
640x360, 약 20 Hz입니다. ROS 2 없이 MCAP에서 바로 실시간 조건을 모사하려면:

```bash
uv run tools/benchmark_best_so_far.py mcap \
  --profile r50-fp16-640x360 \
  --input /Users/kangminwoo/Downloads/20260827_062352_teamgrit_rosbag_0-001.mcap \
  --topic /a2/front_camera/res_360p/image_raw \
  --playback-mode realtime \
  --max-frames 200 \
  --snapshot-dir rosbag-results/benchmarks/snapshots \
  --output-report rosbag-results/benchmarks/best-so-far-realtime.json
```

`realtime`은 bag timestamp에 맞춰 입력을 재생하고 depth-1 최신 프레임 큐를
사용합니다. 따라서 모델이 20 Hz보다 느리면 오래된 프레임을 쌓지 않고 교체하며,
report의 `overwritten_frames`, `drop_ratio`, `effective_output`이 라이브 동작에
가까운 값을 보여줍니다. 순수 최대 처리 성능은 `--playback-mode throughput`으로
측정합니다. 모델 다운로드/로딩 시간은 `model_load_seconds`로 따로 기록되고 Hz
계산에서는 제외됩니다.

여러 MCAP을 한 번에 각각 측정할 수도 있습니다. `--max-frames`는 파일마다
적용되고 temporal history는 파일 경계에서 초기화됩니다.

```bash
uv run tools/benchmark_best_so_far.py mcap \
  --profile r50-fp16-640x360 \
  --input \
    /path/to/first.mcap \
    /path/to/second.mcap \
  --playback-mode throughput \
  --max-frames 50 \
  --output-report rosbag-results/benchmarks/best-so-far-throughput.json
```

라이브 ROS 2 토픽은 ROS 환경의 `rclpy`와 `cv_bridge`를 사용해야 하므로 ROS를
source한 Python 환경에서 실행합니다. 그 환경에는 위 스크립트 상단에 명시된
PyTorch/Transformers 의존성도 설치되어 있어야 합니다.

```bash
source /opt/ros/humble/setup.bash
python3 tools/benchmark_best_so_far.py ros2 \
  --topic /a2/front_camera/res_360p/image_raw \
  --duration 30 \
  --expected-input-hz 20 \
  --output-report rosbag-results/benchmarks/best-so-far-live.json
```

라이브 overlay는 `/best_so_far/benchmark/overlay`, 진행 metrics JSON은
`/best_so_far/benchmark/metrics`에 발행됩니다. overlay 발행 비용까지 피한 순수
추론 측정은 `--overlay-topic ''`을 사용합니다. `rates_hz.segmentation_compute`는
segmentation 자체의 지속 가능 Hz, `rates_hz.effective_output`은 큐 대기와 실제
출력 간격을 반영한 Hz, `verdict.can_keep_up`은 입력 약 20 Hz를 따라갈 수 있는지
나타냅니다.
