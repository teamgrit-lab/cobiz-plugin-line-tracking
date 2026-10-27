# 인도 로컬 경로 + 차도 Flyvis 중앙 경로

`LINE_TRACKING_PATH_PLANNER=connectome`은 차도에만 별도 Flyvis 경로 서비스를 사용한다.
인도는 기존 R50 결과의 로컬 중심선 추출·다항식 피팅·`LocalPathSmoother`를 사용한다.
인도 작업은 HTTP 요청을 보내지 않으며 차도 서버의 오류·타임아웃으로 차단되지 않는다.
R50을 다시 실행하지 않으며 A2 이동 API 발행자는 기존 line-tracking 하나뿐이다.

| 선택 모드 | 경로 생성 | ROI |
| --- | --- | --- |
| Mode 1 차도 | 커넥톰/Flyvis + 현재 차도 마스크 재검사 | 확장 사다리꼴 |
| Mode 2 인도 | 기존 로컬 추출·피팅·평활화 | 기존 사다리꼴 |
| Mode 0 자동 | 처음 선택한 노면이 차도면 커넥톰, 인도면 로컬 | 기존 사다리꼴 |

Mode 0은 한 작업 안에서 노면·백엔드를 자동으로 바꾸지 않는다. 작업 취소·AprilTag·
LiDAR 정지 및 활성화된 odom/IMU 보정·20Hz 명령 제한은 두 방식에 공통으로 적용된다.
`LINE_TRACKING_PATH_PLANNER=local`은 모든 모드에서 종전 로컬 경로를 사용한다.
아래 Flyvis 마스크 검사·실패 정지 정책은 커넥톰을 사용하는 차도 경로에 해당한다.

```text
동일 카메라 → R50 → 인도 라벨 → 기존 중심선 피팅·평활화 ──┐
                └→ 차도 라벨 + 위험 영역 → Flyvis HTTP   │
                     → 현재 마스크·여유 영역 재검사 ─────┤
                                                       ↓
                          Path → 추종·공통 정지 검사 → Sport API
```

## 경로와 제어

- Mode 1에만 `Building`의 관측된 지면 접점에 대한 근접 비용을 추가한다.
  R50 현재 프레임의 원래 분류에서 별도 건물 마스크를 꺼내며 추가 신경망 추론은 없다.
  건물 하단에 차도·인도·curb·vegetation 증거가 있는 접점만 기존 ROI 좌표로 투영한다.
  벽·지붕 전체나 건물 사각형을 바닥 장애물로 투영하지 않는다. 작은 성분, 영상 아래쪽에서
  잘린 건물, 장식물 제외 영역은 접점 증거로 사용하지 않는다.
  `LINE_TRACKING_ROAD_BUILDING_COST_WEIGHT=2.0`,
  `LINE_TRACKING_ROAD_BUILDING_DISTANCE_M=2.0`이 기본값이다. weight 0으로 해제한다.
  거리는 기존 가정에 따른 좌표이며 실측 거리가 아니다. 비용은 거리와 함께 감소하며
  distance의 두 배 밖에서는 0이다. 기존 중심 범위 ±0.20m와 모든 선분의 현재 차도·위험
  영역·로봇 폭 검사를 유지한다. 구간별 허용 폭에 맞춘 추가 후보도 검사해 좁은 구간에서
  통로를 잃지 않고 건물 반대편 후보를 평가한다. 최장 유효 구간을 먼저 선택한다.
  이는 부드러운 경로 선호도이며 열린 입구 바닥 전체를 진입 금지로 인식하는 기능은 아니다.
  Mode 0·2 경로와 AprilTag, 속도 상한, 감속·정지 검사는 유지한다.
- Mode 1은 `SWIN_L_ROAD_ROI_POLYGON`의 측면을 확장한 사다리꼴 ROI를 사용한다.
  기본값은 `0.07,1.00,0.93,1.00,0.68,0.45,0.32,0.45`이다.
  위쪽 폭 36%, 아래쪽 폭 86%, 위쪽 높이 45%로, 원래 사다리꼴보다 좌우를
  넓히면서 A2 손잡이 제외 영역과 간격을 둔다.
  Mode 0·2는 기존 `SWIN_L_ROI_POLYGON`의 사다리꼴을 유지한다. Mode 0이
  차도를 선택해도 ROI는 바뀌지 않는다. 서버 경로 생성과 수신 측 마스크 재검사,
  오버레이 투영 모두 요청 모드의 ROI를 사용한다. 거리 매핑은 기존 전방 3~8m와
  횡방향 ±4m 설정을 유지하며 실제 카메라 장착 자세에 따른 실측 보정은 별도다.
  `SWIN_L_ROAD_ROI_EXCLUSION_POLYGON`으로 오른쪽 위 카메라 장식물/손잡이 영역을
  추가로 제외한다. 이 제외 마스크는 Mode 1의 서버 경로 생성·수신 경로 검사·
  오버레이에 적용하며 네 꼭짓점의 거리 변환과 Mode 0·2 ROI는 바꾸지 않는다.
  원본 카메라 영상과 AprilTag 입력은 유지한다. 빈 문자열로 설정하면 제외를 해제한다.

- 참고 채팅 `정리 초파리 커넥톰 순찰 설계`의 제안대로 원본 영상을 실제 pretrained
  Flyvis 1.2.0 회로·flow decoder에 입력한다. 전송은 회로의 첫 연산과 동일한
  회색조를 무손실 PNG로 보내며 R50의 컬러 입력은 유지한다.
  마스크는 후단 기하 경로 생성기에 준다.
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
- 차도에서는 현재 프레임의 단면들을 공간적으로 연결해 주통로 기준선을 만든다.
  넓은 입구·앞마당의 중앙값은 낮게 가중하고 좁고 관측된 통로는 높게 가중한다.
  가까운 단면을 고정하는 비용과 곡률 변화 비용을 사용하고, 각 점은 관측된 여유 영역
  안으로 제한한다. `LINE_TRACKING_ROAD_CORRIDOR_SMOOTHING_M=0.30`이 기본이며
  0이면 종전 행별 중앙값으로 되돌린다. 이 설정의 거리는 ROI 거리 매핑 기준이다.
  Mode 2의 기존 로컬 경로에는 적용하지 않는다.
- 주통로 기준선 ±0.20m 안의 경로 중심 후보를 생성한다. 옆 입구의 포장면도
  차도로 관측될 수 있지만 이 범위 밖의 경로 후보는 서버와 수신 측 모두 거부한다.
  실제 코너에서는 관측된 곡선 통로를 따라 기준선이 휜다. 이 기능은 현재 영상의
  기하적인 주통로 선택이며, 우사 내부를 명시적으로 식별하는 영구 금지구역 지도는
  별도의 의미 검출·지도 정보가 필요하다. LiDAR의 의미 마스크·정지 정책은 유지한다.
  전역 이동을 제거한 상대 광류는 보조 비용이다.
  광류를 거리·장애물 확률·실측 odom으로 해석하지 않는다. 먼 곳에서 경로가 불명확하면
  확인된 가까운 구간까지만 계획하고 미관측 영역으로 연장하지 않는다.
- 응답 request/task/source stamp가 일치해야 한다. 수신 측도 현재 라벨·위험 영역·
  로봇 여유·중앙 범위에서 **모든 선분**을 다시 검사한다. 검증된 좌표를 이전 EMA로
  옮기거나 경로 소실 시 유지하지 않는다. `confidence`는 관측된 전방 행의 비율이며
  보정된 R50 분류 확률이 아니다.
- 응답 실패·경로 없음·마스크 위반·지연은 정지한다. `LINE_TRACKING_STOP_ON_*`가
  꺼져 있어도 이 검사를 유지하며 직진 복구·회전 탐색으로 전환하지 않는다.
  기존 작업 취소·AprilTag·LiDAR 정지는 유지한다.

`metrics.path_planner.backend`는 현재 작업의 `local` 또는 `connectome`을 표시한다.
준비 상태·거부 이유·선택 표면을 공통으로 표시하고, 차도에는 왕복 지연·실제 bundle
SHA-256·회로 크기·추론 프레임 수와 `trained_waypoint_head=false`도 표시한다.
HTTP 요청은 사용 중인 차도 모드만 지정하며 다른 모드 응답으로 해당 경로를 거부하지 않는다.

## Odom/IMU 명령 세분화

`LINE_TRACKING_MOTION_ENABLED=true`로 촬영 시각부터 현재까지의 이동·회전을 보정하고
20Hz로 명령을 갱신할 수 있다. 입력은 `/dog_odom`, `/dog_imu_raw`이며, frame 변환이
없거나 센서가 만료되면 정지한다. 설정·검증은 [motion-tracking.md](motion-tracking.md)를 참고한다.

## Jetson 지연 최적화

CUDA/PyTorch R50에서는 `SWIN_L_CUDA_GRAPH=true`, CUDA Flyvis에서는
`CONNECTOME_CUDA_GRAPH=true`로 같은 모델의 반복 실행을 CUDA graph로 재생한다.
모델·FP16 설정·ROI·프레임 주기·검사 기준은 바꾸지 않는다. R50 전처리와 표면
후처리는 그대로이며, 재사용되는 GPU 출력과 시간 EMA의 저장 공간은 분리한다.
입력 shape/dtype/device가 바뀌거나 capture가 실패하면 오류로 종료하며 다른
모델이나 CPU로 대체하지 않는다. 첫 프레임의 warmup/capture 동안은 신선한 경로가
없으므로 이동할 수 없다.

Flyvis는 native `_next_state`의 모든 적분 단계를 재생한다. 같은 입력이 유지되는
단계들은 1/4/16단계 graph로 묶어서 제출하며 적분 횟수는 줄이지 않는다. 내부 dt는 기존처럼
최대 0.02초이며 입력 시각에 따라 단계 수가 달라진다. 1초 stream gap 제한도 유지한다.
학습된 파라미터는 고정하고 DecoderGAVP는 native eager CUDA로 실행한다. 장시간 공백은
회로를 초기화하지만 Mode 0의 선택 표면은 새 작업까지 유지한다.

표면 projection/footprint 격자만 유한 캐시에 저장한다. 행별 경계와 선분 검사점을
배열 단위로 계산하고 immutable 광류 receptor의 공간 검색도 재사용한다. 매 프레임의 허용 영역과
hazard는 새로 계산하며 Mode 0과 같은 표면의 후보 평가를 공유한다. 먼 구간이
막힐 때의 최장 유효 prefix는 같은 선분 검사로 이진 탐색한다. 수신 측의 현재
마스크 재검증은 유지한다. timer와 추론 worker 사이의 경로/시각도 한 snapshot으로
교체해 경로 소실 순간의 경쟁을 막는다.

`metrics.pipeline_timing`은 segmentation/planner 시간과 완료 시 촬영 나이를,
`path_planner.timing`은 Flyvis/표면 계산 시간을 제공한다. 이 수치와 source 나이로
판정하며 단순 명령 timer 주파수로 인식 성능을 판단하지 않는다.

실행 예시(장비에서 확인한 frame 변환은 별도 설정):

```dotenv
SWIN_L_CUDA_GRAPH=true
CONNECTOME_CUDA_GRAPH=true
LINE_TRACKING_MOTION_ENABLED=true
SWIN_L_INFERENCE_HZ=2.5
SWIN_L_TELEMETRY_HZ=5.0
SWIN_L_INFERENCE_PROCESS=true
```

이 장비의 목표는 매초 최소 1회 새 추론을 완료하는 것이다. 2.5Hz는 마감 여유를
두기 위한 요청 주기이며 실제 달성률은 완료 간격으로 확인한다. 단순히 요청
주기를 1Hz로 두면 처리 지연이 더해져 기존 1초 촬영 나이 제한 사이에 정지
구간이 생길 수 있다. 영상/센서 만료 한계는 늘리지 않는다.

`SWIN_L_TELEMETRY_HZ`는 Path·metrics에만 적용한다(0이면 기존 매 timer 발행).
상태 직렬화·Pose 생성·GPU 메모리 조회는 5Hz로 줄이고, 이동 명령과 모든 정지
검사는 20Hz로 계속 실행한다. `control_loop.observed_hz`와 `gap_max_sec`로 실제
제어 간격을 확인한다. OpenCV 형태학 처리의 CPU pool도 2개 스레드로 제한한다.

`SWIN_L_INFERENCE_PROCESS=true`는 R50·HTTP·현재 마스크 재검증을 spawn 자식
프로세스에서 실행한다. 부모는 최신 프레임 하나만 보내고 응답의 generation/sequence를
검사한 뒤 기존 task/epoch/source-age 검사를 적용한다. 자식은 ROS/이동 발행자를
만들지 않는다. CUDA graph warmup은 ready 이전에 수행하고 모델의 시간 이력은
초기화한다. 자식 오류·종료·응답 마감 실패는 추론 오류이며 fallback하지 않는다.
부모의 센서 수신/20Hz 정지 검사는 계산 응답을 기다리지 않는다.

당진 A2에서는 8개 코어의 load average가 35 이상이고, 평균 처리보다 간헐적인
CPU 대기 지연이 컸다. `LINE_TRACKING_CPU_SHARES=2048`,
`CONNECTOME_CPU_SHARES=2048`로 두 컨테이너의 CPU 경쟁 가중치를 기본 1024의
두 배로 설정할 수 있다. [Docker의 CPU shares](https://docs.docker.com/engine/containers/resource_constraints/#cpu)
는 경쟁 시 상대 가중치이며 코어 예약이나 실시간 마감 보장은 아니다. 다른
서비스를 종료하거나 CPU 개수를 제한하지 않는다. 기본 Compose 값은 1024다.

`camera_input`은 기존 카메라 구독에서 수신 개수·최대 수신 공백·1초 초과 공백을
기록한다. 성능 수집기가 원격 raw 영상에 별도 구독자를 추가하면 DDS 송신량을
늘릴 수 있으므로, 실제 주행 부하 측정은 이 상태 정보를 사용한다. AprilTag 검출기는
계속 활성화한다. 아래 설정은 relay 하나가 원격 원본을 받아 주행 노드에 그대로
공유한다. 원본 픽셀·encoding·촬영 stamp·frame과 CameraInfo는 유지하며, 원본 공유는
AprilTag 6Hz 제한이나 CameraInfo 준비 여부에 묶이지 않는다. AprilTag에는 기존의
동기화된 영상/보정 정보를 전달한다. 검출·확인·인계와 정지 조건은 변경하지 않는다.

```dotenv
SWIN_L_APRILTAG_DETECTOR_ENABLED=true
SWIN_L_APRILTAG_IMAGE_TOPIC=/a2/front_camera/res_360p/image_raw
SWIN_L_APRILTAG_CAMERA_INFO_TOPIC=/a2/front_camera/res_360p/camera_info
SWIN_L_SHARED_CAMERA_IMAGE_TOPIC=/line_tracking/camera/image_raw
SWIN_L_SHARED_CAMERA_INFO_TOPIC=/line_tracking/camera/camera_info
SWIN_L_IMAGE_TOPIC=/line_tracking/camera/image_raw
SWIN_L_CAMERA_INFO_TOPIC=/line_tracking/camera/camera_info
```

기본 Compose의 `actual-activate`는 task-drive다. 검사 용도로 만든 로컬 override의
`SWIN_L_MODE=ros2` 또는 disabled Sport 토픽이 남아 있으면 주행하지 않는다.
주행 배포는 task-drive와 `/api/sport/request`를 사용하며, 기존 작업 수락 후에만
명령을 발행한다. `ros2` 성능 검사나 가상 ROS domain 재생은 현장 주행 성공을
대신하지 않는다.

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

## 당진 우사 브라우저 순환 시험

[별도 로컬 시뮬레이터](../environments/dangjin/README.md)는 360° 합성 카메라의
서로 90° 간격인 네 RGB 투영을 실제 Flyvis 시각 회로에 넣고, 34개 출력 세포형의
신경 활동에서 새로 학습한 readout으로 차도를 검출한다. 원래 optical-flow 사전학습
회로를 고정했으며, 생물학적 회로가 원래 차도 분류기로 학습되었다고 가정하지 않는다.
readout에 RGB 우회 입력이나 지도·pose를 주지 않는다. 현재 기본 `--control neural`은
손상된 네 방향 검출 마스크만 학습된 행동 CNN에 넣어 속도·회전·정지를
직접 출력한다. 지도 목표점·pose는 행동 입력에서 제외하며, 지도는 완주 평가에만 사용한다.
Pure Pursuit와 비상 정지 검사는 실행하지 않으며, footprint·금지 구역·도로
이탈은 평가 기록에만 남긴다. Flyvis 회로와 차도 출력층은 고정하고 행동 출력층을 새로
학습한다. `--control baseline`은 기존 광류·표면 경로 검사와 Pure Pursuit를 재현한다.
정답 마스크는 오프라인 인식기 학습에만 쓰며 주행 API는 RGB만 받는다.
시뮬레이션 위치와 운동학 모델을 사용하는 별도 실행 경로이며, 실제 로봇의 Sport 제어
소유권·ROS 실행 설정에는 영향을 주지 않는다.
