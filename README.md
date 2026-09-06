# cobiz-plugin-line-tracking

전방 카메라에서 도로 영역과 도로 위 선을 **하나의 YOLOP 모델**로
segmentation하고, 그 선의 중심 경로를 추종해 Unitree A2 같은 4족보행 로봇용
`vx`, `vy`, `yaw_rate`와 A2 제어용 `sensor_msgs/Joy`를 생성하는 ROS 2 Humble
플러그인입니다.

Google Docs의 권장 흐름을 코드로 옮겼습니다.

```text
Image -> YOLOP(road + lane-line, one ONNX forward pass) -> road-gated line mask
      -> trapezoid ROI -> bird's-eye transform -> quadratic centerline
      -> lateral/heading/curvature error -> vx/vy/yaw_rate
      -> confidence gate -> low-pass filter -> rate limit -> A2 Joy
```

`models/README.md`의 360p 또는 720p YOLOP ONNX 파일을 `models/`에 넣고
`.env`의 `CAMERA_PROFILE`과 `SEGMENTATION_MODEL_PATH`를 맞춰 실행합니다. 모델 경로가
비어 있으면 가중치 없이 bring-up할 수 있도록 기존 HSV/LAB 검출기로
fallback하지만, 풀숲/인도 환경의 운영 주행에는 YOLOP를 장착하고 현장 데이터로
fine-tuning해야 합니다.

## 안전 계약

- 출력 기본 토픽은 `/a2_control`이며 `cobiz-plugin-a2`의 `a2_control_node`가
  Unitree Sport API 명령으로 변환합니다.
- 선 신뢰도가 `0.4` 미만이거나 선을 잃으면 즉시 0 속도를 발행합니다.
- 카메라 프레임이 기본 `0.5 s` 동안 오지 않아도 0 속도를 발행합니다.
- Joy 축은 A2 계약에 맞춰 `[-vy, -vx, -yaw_rate]`로 발행하고 버튼 10개는
  항상 0으로 유지해 자세·보행 버튼 동작을 방지합니다.
- 실제 주행 전 `perspective_source`, 지면 폭/거리와 Joy 축 부호를 현장 카메라
  장착 상태에 맞춰 보정해야 합니다.
- `/a2_control`은 로봇으로 직접 이어지는 수동 제어 경로이므로 다른 gamepad
  publisher 또는 Navigation 제어와 동시에 사용하지 않아야 합니다.

현재 기본 원근점은 기능 확인용 초기값입니다. 측량하지 않은 기본값으로 무인
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

## 실행

카메라 bridge가 먼저 영상을 발행하고 있어야 합니다.

```bash
cd cobiz-plugin-line-tracking
cp .env.example .env
# models/README.md의 profile에 맞는 ONNX 파일을 먼저 models/에 배치
docker compose up -d --build
```

확인:

```bash
docker compose logs -f line-tracking
ros2 topic hz /a2/front_camera/image_raw
ros2 topic echo /line_tracking/confidence
ros2 topic echo /a2_control

# 테스트 모드 결과 확인
TEST_MODE=true docker compose up -d --build
rqt_image_view /line_tracking/test/debug_image
ros2 topic echo /line_tracking/test/metrics
# RViz2에서 nav_msgs/Path 토픽 /line_tracking/test/centerline 추가
```

종료:

```bash
docker compose down
```

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

실시간 기본 profile은 `r50-fp16-640x360`입니다. 기존에 가장 안정적으로 보였던
Swin-L 결과는 `swin-l-best-so-far`로 고정해 두었으므로 모델 실험 후에도 아래
옵션 하나로 즉시 복구할 수 있습니다.

```bash
# 기존 Swin-L best-so-far 설정으로 복구
uv run tools/benchmark_best_so_far.py mcap \
  --profile swin-l-best-so-far \
  --input /path/to/input.mcap \
  --output-report rosbag-results/benchmarks/swin-l-restored.json
```

`swin-l-best-so-far`의 고정 계약은 다음과 같습니다.

- model: `facebook/mask2former-swin-large-mapillary-vistas-semantic`
- revision: `4772b6bf101d91f2534c106dc524d906aeb3c68a`
- model input: `384x384`, score map: `640x360`, precision: FP32
- temporal alpha `0.62`, hysteresis margin `0.07`
- Road/Bike Lane/Crosswalk/Parking/Service Lane/Lane Marking을 Road로 통합
- Sidewalk/Pedestrian Area/Curb Cut을 Sidewalk로 통합

## MCAP overlay 테스트를 명령 한 줄로 실행

Mac/일반 PC에서는 저장소 루트에서 `uv`로 실행한다. Jetson은 아래의
[MCAP 테스트 컨테이너](#jetson에서-mcap-overlay-테스트)를 사용한다.
두 명령 모두 위의 `swin-l-best-so-far`
모델·revision·FP32·384×384 입력·temporal 설정을 고정한다. `.env`의
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
- `local-path`: 마젠타=인도, 주황=추정 경로, 흰색=평활 경로. 기존 4Hz 목표
  추론과 hold를 재사용하므로 `TRACKED`라도 이전 경로일 수 있다. 현재 경로는
  인도 중심 후보이며 장애물 우회 planner는 아니다.

## Swin-L 인도 중심 local path 디버그

`tools/swin_l_local_path_debug.py`는 Swin-L profile의 `Sidewalk` mask를 카메라
전방 3~8m의 metric bird's-eye grid로 옮긴 뒤, 각 거리에서 인도 영역의 중심을
추출해 `base_link` 기준 `nav_msgs/Path`로 만든다. 매 프레임마다 경로를
갈아끼우지 않고 최신 카메라 프레임만 유지하는 depth-1 큐, Swin-L 기본 추론
4Hz, 0.8초 EMA, 0.9초 경로 hold를 사용한다. 따라서 출력 타이머는 기본 10Hz여도
실제 Swin-L 추론이 4Hz보다 느리면 유효한 경로 갱신은 더 느려질 수 있다.

유효한 결과는 `local_path.poses`가 2개 이상이고 metrics에서
`path_tracked=true`, `path_confidence>0`인 상태다. `ros2 topic hz`가 약 10Hz라는
것만으로 경로가 갱신되는 것은 아니다. `poses=[]`, `path_tracked=false`,
`reason=path_unavailable`이면 영상은 들어오고 overlay timer도 돌지만 인도
중심선을 추출하지 못한 상태다. LiDAR가 연결되어 있어도 인도 mask/ROI/
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
| 출력 overlay | `SWIN_L_OVERLAY_TOPIC` | `/line_tracking/swin_l/overlay` |
| 출력 경로 | `SWIN_L_LOCAL_PATH_TOPIC` | `/line_tracking/swin_l/local_path` |
| 안전 상태 | `SWIN_L_SAFETY_STOP_TOPIC` | `/line_tracking/swin_l/safety_stop` |
| 여유 거리 | `SWIN_L_CLEARANCE_TOPIC` | `/line_tracking/swin_l/clearance_m` |
| 진단 metrics | `SWIN_L_METRICS_TOPIC` | `/line_tracking/swin_l/metrics` |

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
local path다. 이 코드는 디버깅용이므로 실제 보행 제어기에 연결하기 전 homography,
LiDAR frame 정렬, 장애물 z 범위를 검증해야 한다.

## Docker debug 컨테이너

아래 구성은 Jetson에서 `debugging-swin-l`만 실행해 local path와 overlay를 확인하고
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
/line_tracking/swin_l/overlay
/line_tracking/swin_l/local_path
/line_tracking/swin_l/safety_stop
/line_tracking/swin_l/clearance_m
/line_tracking/swin_l/metrics
```

호스트에서 결과를 확인한다.

```bash
ros2 topic echo /line_tracking/swin_l/local_path
ros2 topic echo /line_tracking/swin_l/safety_stop
rqt_image_view /line_tracking/swin_l/overlay
rviz2  # Fixed Frame=base_link, Path topic=/line_tracking/swin_l/local_path
```

출력 타이머 기본값은 10Hz이고 Swin-L 목표 추론률은 4Hz다. 따라서 10Hz가
측정되어도 이전 경로를 재발행하는 중일 수 있다. 다음을 함께 판단한다.

```text
정상: poses >= 2, path_tracked=true, path_confidence > 0
비정상: poses == [], path_tracked=false, reason=path_unavailable
```

2026-09-03 결과 MCAP에서는 다섯 output topic이 약 10Hz였지만 20.5초 동안
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
  /line_tracking/swin_l/overlay \
  /line_tracking/swin_l/local_path \
  /line_tracking/swin_l/safety_stop \
  /line_tracking/swin_l/clearance_m \
  /line_tracking/swin_l/metrics \
  /tf /tf_static
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

일반 `docker compose up -d`는 주행용 `line-tracking` 서비스를 시작할 수 있고
`/a2_control`을 publish할 수 있다. local path만 확인할 때는 반드시 debug
서비스를 명시하고 주행 서비스를 먼저 정지한다.

```bash
docker compose stop line-tracking
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
보관 Swin-L 모델·revision·FP32 설정을 고정하고 기본 200프레임을 처리한다.
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

새 기본 `r50-fp16-640x360`은 같은 label aggregation과 temporal 설정을 유지하면서
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
