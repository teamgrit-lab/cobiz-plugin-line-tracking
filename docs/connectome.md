# R50 표면 검출 + Flyvis 중앙 경로

`LINE_TRACKING_PATH_PLANNER=connectome`은 기존 R50 FP16 결과를 재사용해 별도
Flyvis 경로 서비스로 전달하고, 검증한 경로를 기존 line-tracking 제어기로 추종한다.
R50을 다시 실행하지 않으며 A2 이동 API 발행자는 기존 line-tracking 하나뿐이다.

```text
동일 카메라 → R50 → 인도/차도 라벨 + 현재 grass/curb 제외 마스크
    └→ 원본 이미지 → Flyvis 광류 ──┤
                                   ↓
              표면별 여유 영역 → 중앙 후보 생성·광류 보조 평가
                                   ↓ HTTP 좌표 응답
line-tracking 현재 마스크 재검사 → Path → 기존 추종·LiDAR·작업 중지 → Sport API
```

## 경로와 제어

- 참고 채팅 `정리 초파리 커넥톰 순찰 설계`의 제안대로 원본 영상을 실제 pretrained
  Flyvis 1.2.0 회로·flow decoder에 입력한다. 마스크는 후단 기하 경로 생성기에 준다.
  **학습된 neural waypoint head가 직접 경로를 출력하는 구현은 아니다.**
- 회로 state를 프레임 사이에 유지하고 최대 0.02초 간격으로 내부 적분한다.
  camera/task stream 변경 시 초기화한다. 중복·역행 시각과 긴 공백은 거부한다.
- 라벨은 0=배경, 1=차도, 2=인도다. 선택 모드 1·2는 해당 표면만 사용한다.
  통합 모드 0은 시작할 때 가까운 관측 통로를 선택하고 거리가 같으면 인도를 우선한다.
  선택 표면이 사라져도 다른 표면으로 자동 전환하지 않는다. 새 작업은 선택을 초기화한다.
  이 정책은 LiDAR 회피와 최종 명령 검사에도 적용한다.
- 기존 `LIDAR_OBSTACLE_ROBOT_HALF_WIDTH_M`(0.30m), `LIDAR_OBSTACLE_MARGIN_M`(0.15m)를
  공유해 허용 영역을 줄인다. 구멍을 채우거나 떨어진 포장 영역을 연결하지 않는다.
  현재 logits의 Vegetation/Curb는 표면 EMA보다 우선한다.
- 중앙 ±0.20m 안의 후보를 생성한다. 전역 이동을 제거한 상대 광류는 보조 비용이다.
  광류를 거리·장애물 확률·실측 odom으로 해석하지 않는다. 먼 곳에서 경로가 불명확하면
  확인된 가까운 구간까지만 계획하고 미관측 영역으로 연장하지 않는다.
- 응답 request/task/source stamp가 일치해야 한다. 수신 측도 현재 라벨·위험 영역·
  로봇 여유·중앙 범위에서 **모든 선분**을 다시 검사한다. 검증된 좌표를 이전 EMA로
  옮기거나 경로 소실 시 유지하지 않는다. `confidence`는 관측된 전방 행의 비율이며
  보정된 R50 분류 확률이 아니다.
- 응답 실패·경로 없음·마스크 위반·지연은 정지한다. `LINE_TRACKING_STOP_ON_*`가
  꺼져 있어도 이 검사를 유지하며 직진 복구·회전 탐색으로 전환하지 않는다.
  기존 작업 취소·AprilTag·LiDAR 정지는 유지한다.

`metrics.path_planner`에는 준비 상태·거부 이유·왕복 지연·선택 표면, 실제 bundle
SHA-256·회로 크기·추론 프레임 수와 `trained_waypoint_head=false`가 표시된다.

## Odom/IMU 명령 세분화

`LINE_TRACKING_MOTION_ENABLED=true`로 촬영 시각부터 현재까지의 이동·회전을 보정하고
20Hz로 명령을 갱신할 수 있다. 입력은 `/dog_odom`, `/dog_imu_raw`이며, frame 변환이
없거나 센서가 만료되면 정지한다. 설정·검증은 [motion-tracking.md](motion-tracking.md)를 참고한다.

## 모델과 실행

기존 `fly_connectom`의 검증된 `flyvis_encoder.pt`를 사용할 수 있다. 모델 바이너리는
Git에 포함하지 않는다. 공식 archive로부터 준비할 때는 다음 변환기가 archive
체크섬을 확인하고 `flynav.encoder.v1` 형식으로 내보낸다.

```bash
python tools/prepare_connectome_bundle.py /path/to/official-flow-archive.zip models/flyvis_encoder.pt
sha256sum models/flyvis_encoder.pt
```

`.env`에 **실제 파일의 해시**를 설정한다.

```dotenv
CONNECTOME_MODEL_DIR=./models
CONNECTOME_BUNDLE=/models/flyvis_encoder.pt
CONNECTOME_BUNDLE_SHA256=<확인한 64자리 해시>
CONNECTOME_DEVICE=cuda
LINE_TRACKING_CONNECTOME_TIMEOUT_SEC=0.8
LINE_TRACKING_CONNECTOME_MAX_AGE_SEC=1.0
```

Jetson CUDA/ROS 기본 이미지를 먼저 빌드한 뒤 별도 Flyvis 이미지를 사용한다.
Flyvis 설치는 기본 torch·torchvision·NumPy의 정확한 버전을 고정하고 기존 추론
환경을 교체하지 않는다.

```bash
docker compose build actual-activate
docker compose -f docker-compose.yml -f docker-compose.connectome.yml config --quiet
docker compose -f docker-compose.yml -f docker-compose.connectome.yml build connectome-planner
docker compose -f docker-compose.yml -f docker-compose.connectome.yml up -d connectome-planner actual-activate
```

override가 `actual-activate`와 `debugging-swin-l`을 connectome 모드로 선택한다.
기본 Compose는 `local`을 유지한다. 서비스는 127.0.0.1:8091에만 바인드하며 모델이
준비된 뒤 `/health`를 통과한다. 카메라마다 서비스 하나를 사용한다.

`POST /v1/plan`의 schema는 `line_tracking.connectome.v1`이다. 요청은 stream/request ID,
source stamp, 원본 이미지 PNG, 라벨 PNG, 제외 마스크 PNG, ROI/그리드·여유 설정이다.
응답 `paths`의 키는 `0/1/2`이며 값은 좌표·표면·관측 비율 또는 `null`이다.
요청/응답 크기와 PNG 크기를 제한한다. 서비스가 끊기면 정지하며 같은 stream의
새 정상 응답 뒤에만 추종한다.

호스트에서는 별도 환경에 `requirements-connectome.txt`를 설치한 뒤 CPU로 검사할 수 있다.

```bash
python tools/connectome_server.py --bundle /path/to/flyvis_encoder.pt --sha256 <해시> --device cpu
python tools/swin_l_local_path_debug.py mcap --path-planner connectome \
  --profile r50-fp16-640x360 --input /path/to/input.mcap --output /tmp/connectome.mp4
```

## 검증과 한계

```bash
python -m pytest test/test_connectome.py test/test_connectome_ros.py -q
CONNECTOME_TEST_BUNDLE=/path/to/flyvis_encoder.pt CONNECTOME_TEST_SHA256=<해시> \
  MPLBACKEND=Agg python -m pytest test/test_connectome_real.py -q
```

검사는 중앙 경로, 좁은 통로·구멍·위험 영역, 표면 유지, 실제 HTTP, 잘못된 응답,
task 제어·지연·장애 정지를 포함한다. 선택 실제 모델 검사는 45,669개 뉴런의
pretrained 회로 → HTTP → R50 마스크 → 경로 재검사 → Sport 명령 직렬화를 확인하며
실제 명령을 발행하지 않는다. `CONNECTOME_TEST_CACHE`에는 기존 `frames/frame-00001.png`,
`masks/mask-00000.npz` 형식의 추론 캐시를 지정할 수 있다. 이 검사는 연결 검사이며
실측 ego-motion 재생이나 주행 성공률 검증은 아니다.

2026-10-19 로컬 검사: 새 단위/HTTP/ROS 테스트 22개와 실제 pretrained 모델·R50
캐시 연결 검사 1개가 통과했다. 전체 테스트는 843개 통과·17개 제외·2개 실패였다.
실패는 기존 `test_range_recovers_distance_through_the_wide_angle_distortion`의
`lateral_m=0.6`, 거리 3m·6m 항목이며 변경 전 HEAD에서도 동일하게 재현했다.
기본 Compose와 connectome override 설정 검사는 통과했다. Jetson CUDA 이미지 빌드와
현장 주행은 이 로컬 검사에 포함하지 않았다.

거리·여유는 기존 ROI 지면 변환 기준이다. 실제 카메라 보정, 근거리 사각의 차체
안전성, 현장 코너 성공은 이 기능만으로 보장하지 않는다. LiDAR 검사를 유지하며
전역 순찰 경로·지도 분기·90도 회전 제어를 새로 구현하지 않는다. 코드 변경·푸시는
Jetson 배포나 실제 출발을 수행하지 않는다.

참고: [Flyvis 공식 구현](https://github.com/TuragaLab/flyvis),
[Network API](https://turagalab.github.io/flyvis/reference/network/).
