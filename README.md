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
