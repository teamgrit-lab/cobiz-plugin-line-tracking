# cobiz-plugin-line-tracking

Unitree A2의 전방 카메라와 LiDAR를 이용해 선택된 주행면의 중심 경로를 만들고,
Cobiz `LINE_TRACKING` 작업이 승인된 동안에만 `/a2_control` Joy 명령을 보내는
C++20/ROS 2 플러그인입니다. 런타임과 도구, 테스트는 Python 없이 C++로
구성되어 있습니다.

기본 프로필은 `swin-l-aspect-224x384`입니다. 고정된 Mapillary Vistas
Mask2Former 체크포인트를 ONNX로 내보낸 모델을 OpenCV DNN으로 실행하며,
640×360 score map에 시간 평활과 hysteresis를 적용합니다.

## 안전 계약

`actual-activate`는 아래 조건을 모두 통과한 경우에만 Joy publisher를 만듭니다.

- Cobiz에서 유효한 `LINE_TRACKING` 작업을 수신함
- `SWIN_L_DRIVE_ENABLED=true`와 `SWIN_L_CALIBRATION_CONFIRMED=true`
- 카메라·추론·LiDAR·경로 timestamp가 허용 범위 안에 있음
- LiDAR frame과 경로 frame이 모두 `base_link`임
- `/a2_control`에 다른 publisher가 없음
- 선택 경로의 confidence, 기하와 clearance가 제한을 통과함

작업 시작 후 2초 동안 0 명령을 보내며, unsafe 상태가 기본 2초간 지속되면
작업을 중단합니다. 작업 완료·서버 취소·SIGTERM 때도 0 명령을 시도합니다.
전원 차단이나 `SIGKILL`에서는 발행할 수 없으므로 독립적인 하위 제어 watchdog과
물리 비상정지는 반드시 별도로 유지해야 합니다.

`cobiz-plugin-a2`는 Joy를
`Move(vx=-axes[1], vy=-axes[0], yaw=-axes[2])`로 해석합니다. 이 플러그인은
`[vy, -vx, yaw_rate]`를 발행하며 버튼 10개는 항상 0입니다.

## 구성

주요 소스는 다음과 같습니다.

| 경로 | 역할 |
|---|---|
| `include/line_tracking`, `src` | 경로, LiDAR, 제어, 작업, 분할 라이브러리 |
| `ros/line_tracking_node.cpp` | ROS 2 debug/task-drive 노드 |
| `apps/line_tracking_cli.cpp` | 영상 overlay, benchmark, 평가 도구 |
| `tests` | ROS 비의존 네이티브 안전·수치 계약 테스트 |
| `Dockerfile.swin-l-debug` | Jetson C++ 빌드·런타임 이미지 |

ROS 2가 없는 개발 환경에서는 core, inference, CLI와 테스트가 빌드됩니다.
ROS 2 Humble이 source된 환경에서는 `line_tracking_node`도 자동으로 빌드됩니다.

## 모델 계약

모델 weight는 Git에 포함하지 않습니다. 아래 파일을 준비합니다.

```text
models/mask2former-swin-l-mapillary-224x384.onnx
```

ONNX 입력은 `[1,3,224,384]` RGB float tensor이며 ImageNet mean/std 정규화를
노드가 수행합니다. 출력은 다음 중 하나여야 합니다.

1. semantic logits `[1,C,H,W]`
2. Mask2Former class logits `[1,Q,C+1]`와 mask logits `[1,Q,H,W]`

Mapillary class ID는 Road 계열 `7,8,10,13,14,23,24`, Sidewalk 계열
`9,11,15`를 사용합니다. 모델의 원본 체크포인트와 revision은
[models/README.md](models/README.md)에 고정되어 있습니다. ONNX graph는 배포 대상
OpenCV/TensorRT 조합에서 사전에 검증해야 합니다.

## 로컬 빌드와 테스트

OpenCV 4, CMake 3.22+, C++20 compiler, nlohmann-json이 필요합니다.

```bash
cmake -S . -B build-cpp \
  -DLINE_TRACKING_BUILD_ROS2=OFF \
  -DLINE_TRACKING_BUILD_TESTS=ON
cmake --build build-cpp -j
ctest --test-dir build-cpp --output-on-failure
```

지원 프로필 확인:

```bash
./build-cpp/line_tracking_cli profiles
```

영상 분할 overlay:

```bash
./build-cpp/line_tracking_cli segment-video \
  --input videos/input.mp4 \
  --output results/surfaces.mp4 \
  --model models/mask2former-swin-l-mapillary-224x384.onnx \
  --report results/surfaces.json
```

경로 overlay는 영상에 LiDAR가 없으므로 화면에 fail-closed LiDAR 상태를 표시합니다.

```bash
./build-cpp/line_tracking_cli local-path-video \
  --input videos/input.mp4 \
  --output results/local-path.mp4 \
  --model models/mask2former-swin-l-mapillary-224x384.onnx \
  --path-mask-class 2
```

benchmark와 mask IoU 평가:

```bash
./build-cpp/line_tracking_cli benchmark \
  --input videos/input.mp4 \
  --model models/mask2former-swin-l-mapillary-224x384.onnx \
  --max-frames 100

./build-cpp/line_tracking_cli evaluate \
  --candidate results/candidate-mask.png \
  --reference results/reference-mask.png
```

MCAP은 Python decoder를 내장하지 않습니다. ROS 2의 `ros2 bag play`로 camera와
LiDAR 토픽을 재생하고 `debugging-swin-l` 노드의 출력 토픽을 기록합니다.

## Jetson 실행

`.env.example`을 복사하고 실제 장비 값을 설정합니다.

```bash
cp .env.example .env
docker compose config --quiet
docker compose up -d --build
docker compose logs -f actual-activate
```

기본 서비스는 `actual-activate` 하나이며, `.env`의 두 arm 변수가 `false`이면
작업을 `drive_not_armed`로 거절합니다. inspection-only 노드는 명시적으로 실행합니다.

```bash
docker compose --profile debug up -d --build debugging-swin-l
```

네이티브 CLI를 같은 Jetson 이미지에서 실행하려면 다음처럼 command를 덮어씁니다.

```bash
docker compose --profile test run --rm test-swin-l \
  benchmark --input /videos/input.mp4 \
  --model /models/mask2former-swin-l-mapillary-224x384.onnx \
  --max-frames 100
```

## ROS 토픽

| 방향 | 기본 토픽 | 형식 | 설명 |
|---|---|---|---|
| 입력 | `/a2/front_camera/image_raw` | `sensor_msgs/Image` | 전방 영상 |
| 입력 | `/unitree/slam_lidar/points1` | `sensor_msgs/PointCloud2` | base_link 정렬 LiDAR |
| 입력 | `/task_event` | `std_msgs/String` | Cobiz 작업 JSON |
| 출력 | `/a2_control` | `sensor_msgs/Joy` | 승인된 작업의 저속 제어 |
| 출력 | `/line_tracking/swin_l/local_path` | `nav_msgs/Path` | 중심 경로 |
| 출력 | `/line_tracking/swin_l/safety_stop` | `std_msgs/Bool` | LiDAR 안전 정지 |
| 출력 | `/line_tracking/swin_l/metrics` | `std_msgs/String` | 상태 JSON |
| 출력 | `/task_state` | `std_msgs/String` | Cobiz 작업 상태 JSON |

`debugging-swin-l`은 `/a2_control`, overlay, clearance를 발행하지 않습니다.
`actual-activate`는 task가 활성화된 동안에만 Joy publisher를 소유합니다.

## Cobiz payload

기본 작업 시간은 60초, 최대 300초입니다.

```json
{"duration_sec": 30, "selected_mask": 2}
```

`selected_mask=1`은 Road, `2`는 Sidewalk입니다. 값이 없으면
`SWIN_L_PATH_MASK_CLASS`를 사용하며 다른 값은 `invalid_selected_mask`로 거절합니다.
두 surface의 경로 smoother는 task-drive에서 독립적으로 유지되며 자동 대체하지
않습니다.

## 현장 보정

`.env.example`의 ROI, 지면 폭·거리, LiDAR z/corridor/stop 값은 초기값일 뿐입니다.
실제 카메라 장착 자세와 LiDAR-to-`base_link` extrinsic으로 반드시 다시 측정해야
합니다. 확인 전에는 `SWIN_L_CALIBRATION_CONFIRMED`와
`SWIN_L_DRIVE_ENABLED`를 활성화하지 마십시오.
