# 360° FCN으로 Road 작업 실행

기본 검출기는 당진에서 파인튜닝한 **FCN-ResNet18 1024×512**다. CoBiz `LINE_TRACKING`의 `selected_mask=1`은 Insta 카메라와 FCN을, `selected_mask=2`는 기존 전면 카메라·인도 모델·제어기를 사용한다. 아래 명령은 Jetson의 `/home/teamgrit/dev/dangjin-a2/cobiz-plugin-line-tracking/`에서 Compose를 실행하는 기준이다.

현재 Compose는 `fcn360-road`가 USB 카메라를 직접 읽고 TensorRT로 검출한 작은
분류 격자만 로컬 HTTP `127.0.0.1:8094/v1/direct-road`로 반환한다.
`actual-activate`가 기존 FCN 경로·명령을 계산한다. 영상과 마스크의 ROS 중계는
사용하지 않는다. [공통 직접 입력 안내](direct-camera.md)를 참고한다.

Compose의 직접 입력은 Jetson 하드웨어 MJPEG 디코딩과 VIC BGRx 변환 후,
시스템 메모리 staging을 통해 CUDA로 올린다. 네 방향 투영·RGB 정규화·TensorRT
추론은 GPU에서 수행하며 원래 분류 격자만 CPU에 반환한다. zero-copy 구현은 아니며
지원 플러그인/CUDA가 없으면 오류로 알린다. 전면 카메라·Sidewalk 경로는 유지한다.

## 로컬 검증

브라우저 시험은 [로컬 시뮬레이션 안내](local-simulation.md)에 따라 별도 실행한다.
실기 검출·제어 코드는 시뮬레이션 파일 없이 사용할 수 있다.

## Jetson에 코드와 모델 복사

새 코드는 1024×512 추론, 원래 16×32 격자의 ROS 전송·경로 계산을 지원한다. 모델 파일만 바꾸면 이전 512 전용 코드는 새 규격을 처리하지 못하므로 코드도 반영해야 한다.

커밋된 변경을 Jetson에 pull해서 반영한 경우 아래 코드 복사를 생략한다. 현재 Mac 작업 파일을 직접 적용하려면 **Mac 터미널에서** 다음 네 파일을 복사한다.

```bash
cd /Users/kangminwoo/Documents/GitHub/cobiz-plugin-line-tracking
rsync -avR \
  tools/fcn360_road.py \
  tools/fcn360_detector.py \
  tools/line_tracking_runtime.py \
  docker-compose.fcn360.yml \
  dangjin-a2:/home/teamgrit/dev/dangjin-a2/cobiz-plugin-line-tracking/
```

학습 패키지 전체를 복사한다. 512 패키지와 파일 이름이 같으므로 새 디렉터리에 둔다.

```bash
ssh dangjin-a2 'mkdir -p /home/teamgrit/dev/dangjin-a2/cobiz-plugin-line-tracking/models/fcn1024-road-20261103'
rsync -av --exclude='._*' \
  '/Volumes/MY_HDD/02_학습모델_데이터셋/03_FCN_ResNet18/모델/fcn1024-road-runs/20261103-081851-c90tq27l/' \
  dangjin-a2:/home/teamgrit/dev/dangjin-a2/cobiz-plugin-line-tracking/models/fcn1024-road-20261103/
```

추론에 사용하는 파일은 `fcn_resnet18.onnx`다. `best.pt`, `config.json`, `complete.json`, `classes.txt`는 같은 패키지의 학습·설정·해시 근거로 보관한다. ONNX SHA-256은 다음과 같다.

```text
77197ab82305f61eb7e83d4bff7f2dfad0122a618ea9332b4c80236862733e7f
```

## Jetson .env 설정

기존 `.env`에서 다음 항목을 맞춘다. `.env.example` 전체를 덮어쓰지 말고 기존 인도 모델·카메라·DDS 설정은 현재 값으로 둔다. FCN override는 front-connectome override와 함께 선택하지 않는다.

```dotenv
COMPOSE_FILE=docker-compose.yml:docker-compose.fcn360.yml
FCN360_MODEL_DIR=./models/fcn1024-road-20261103
FCN360_MODEL_SHA256=77197ab82305f61eb7e83d4bff7f2dfad0122a618ea9332b4c80236862733e7f
FCN360_INPUT_WIDTH=1024
FCN360_VIDEO_DEVICE=/dev/video91
FCN360_HEIGHT_M=1.6
FCN360_MAX_AGE_SEC=1.0
```

`FCN360_INPUT_WIDTH`는 검출기와 제어기에 함께 전달된다. `FCN360_HEIGHT_M`에는 렌즈 중심의 실측 높이를 넣는다. 현재 1.6 m·수평 장착은 추정값이다. 실기의 전진 상한은 기존 `LINE_TRACKING_MAX_FORWARD_MPS` 설정을 사용하며 현재 코드 상한은 1.0 m/s다. 브라우저의 10.0 m/s는 별도 시뮬레이션 배율이다.

## 빌드와 반영

진행 중인 Road 작업을 종료한 뒤 **Jetson 터미널에서** 실행한다.

```bash
cd /home/teamgrit/dev/dangjin-a2/cobiz-plugin-line-tracking
sha256sum ./models/fcn1024-road-20261103/fcn_resnet18.onnx
docker compose config --quiet
docker compose build actual-activate
docker compose up -d --no-deps --force-recreate actual-activate fcn360-road
docker compose logs --tail=100 fcn360-road
```

두 서비스가 같은 이미지를 사용하므로 `actual-activate` 이미지를 먼저 빌드하고 두 컨테이너를 재생성한다. 첫 실행은 모델 해시·입력 크기·GPU·TensorRT·CUDA를 포함한 새 캐시 키로 FP32 엔진을 생성한다. 카메라는 2880×1440 MJPEG를 제공해야 한다.

검출기 로그에 `FCN 1024x512 · native 32x16 grid`와 새 ONNX 해시가 표시되는지 확인한다. `fcn360-road`에는 주행 명령 퍼블리셔가 없으며, CoBiz Road 작업이 활성화됐을 때 `actual-activate`가 명령을 계산한다. 검출 서비스만 먼저 바꾸려면 위 `up` 명령에서 `actual-activate`를 제외한다.

## 검출과 작업 전환 확인

- 같은 ROS 도메인·DDS 설정의 ROS 2 뷰어에서 `/line_tracking/fcn360/overlay/compressed`를 `CompressedImage`, best-effort QoS로 구독한다. 앞·좌·후·우 순서의 2×2 미리보기이며 초록은 차도, 분홍은 고정 로봇 영역이다. 표시만 512×256 패널로 축소하고 추론·경로 계산은 원래 해상도를 유지한다.
- Road 작업에서 `/line_tracking/metrics`의 `inference_backend`는 `fcn1024-360`이다. `fcn360.front_path_valid`, `maneuver`, `observed_horizon_m`, `speed_fraction`을 확인한다. Sidewalk 작업의 backend는 `r50`이며 기존 전면 카메라와 인도 제어기를 사용한다.
- 직접 입력 HTTP 응답의 `classes`에는 네 방향의 원래 `4×16×32` 클래스 격자와 원본 촬영 시각이 포함된다. 격자를 축소하지 않는다. 0은 미관측, 1은 고정 로봇, 3은 차도다. ROS 마스크·미리보기 토픽은 기존 검사 CLI를 `--direct-camera` 없이 실행할 때만 제공한다.
- 전방 고정 팔 영역은 0 기준 14~17열·10~15행, 1 기준 15~18열·11~16행이다. 512 기준 2열×3행과 같은 화면 비율의 4열×6행이며 경로 계산에서는 미관측으로 제외한다. 그 밖의 로봇 예측도 미관측으로 처리한다.
- 전방과 연결된 노면과 좌우 차도를 기준으로 경로를 선택한다. 작은 중앙 보정은 확인 대기 없이 적용한다. 새 큰 회전은 방향 기울기 0.40(약 22°)에서 확인을 시작하고 0.25(약 14°)까지 유지하는 히스테리시스로 직진·회전 왕복을 줄인다. 새 회전은 마스크 세 장에서 연속 확인한다.
- 회전 확인 중에는 이전 진행 방향을 현재 마스크의 연결·폭 조건으로 재검증하고, 통과하는 구간만 현재 카메라 좌표에서 다시 생성해 감속 주행한다. 이전 좌표·검출 마스크를 그대로 재사용하지 않는다. 유지할 통로가 없으면 아래의 측면 회전 재탐색을 시도한다. 전체 검출 누락에서는 정지하고 방향 확인을 초기화한다. 0.65 m 여유 폭과 최소 3.2 m 관측 길이는 유지한다.
- `path_status`와 `candidate_path_valid`로 경로 미검출과 회전 확인 대기를 구분한다. `path_source`는 현재 후보·현재 영상으로 재검증한 이전 방향·현재의 작은 보정 통로를 구분한다. `front_path_valid`는 실제 주행 경로 반환 여부다. 큰 회전 후보가 있어도 유지할 통로가 없으면 후보만 유효하고 주행 경로는 없을 수 있다.
- 후방은 진단에 사용한다. 유효 경로와 측면 회전 근거가 모두 없거나 촬영·추론 시각이 기본 1초를 넘으면 정지한다. LiDAR·odom·지도·목표점은 이 Road 경로 생성의 입력이 아니다.

### 경로 소실 시 차도가 많은 쪽으로 제자리 회전

`native-fcn-grid-v6-side-recovery`는 전방 경로를 반환할 수 없을 때 좌우 카메라의 같은 관측 영역을 비교한다. 영역은 하단 3/8·가로 중앙 3/4이며 512에서는 3×12칸, 1024에서는 6×24칸이다. 각 영역에서 차도 클래스의 가장 큰 4방향 연결 성분 비율을 구해 고립된 몇 칸을 제외한다.

- 더 큰 쪽의 연결 차도가 25% 이상이고 좌우 차이가 10%p 이상일 때만 방향 후보로 사용한다. 차이가 작거나 둘 다 근거가 부족하면 정지한다.
- 같은 쪽이 서로 다른 촬영 시각의 두 프레임에서 확인되면 `vx=0`, `vy=0`, `yaw_rate=±0.18 rad/s`로 제자리 회전한다. 왼쪽은 양수, 오른쪽은 음수다. Jetson에서는 설정된 회전 상한으로 더 제한한다. 최대 전진 10 m/s와 회전속도는 별도다.
- 매 새 영상에서 방향을 다시 확인한다. 우세 방향이 바뀌거나 검출이 사라지면 즉시 회전을 멈추고 재확인한다. 전체 검출 누락·오래된 영상·작업 취소에서는 회전하지 않는다.
- 한 경로 소실 구간의 재탐색 시간은 촬영 시각 기준 최대 10초다. 잠깐의 검출 소실이나 방향 변경으로 이 시간을 연장하지 않는다. 유효 경로가 다시 생기거나 새 작업·시험을 시작하면 초기화한다.
- 경로가 다시 생기면 즉시 일반 추종으로 복귀한다. 회전 중 `front_path_valid=false`와 빈 Path는 유지하며 존재하지 않는 전진 경로를 만들지 않는다.

진단 필드는 `recovery_left_road_fraction`, `recovery_right_road_fraction`, `recovery_direction`, `recovery_status`, `recovery_confirmation_frames`, `recovery_elapsed_sec`, `recovery_remaining_sec`, `recovery_yaw_rate`다. 브라우저 명령 사유는 `road_side_recovery`, ROS 제어기는 `fcn_side_recovery`를 표시한다. 이 동작은 360° Road 제어기에 적용하며 전면 한 장을 사용하는 Flyvis 행동 모델에는 추가되지 않는다.

기존 512 모델도 해당 디렉터리·해시와 `FCN360_INPUT_WIDTH=512`를 함께 지정하면 원래 `4×8×16` 격자로 사용할 수 있다. 두 모델 모두 같은 전방 시야·정규화와 화면 비율의 팔 마스크를 사용한다.

과거 512/1024 비교 영상은 1024 결과를 8×16으로 축소한 호환 재생 및 팔 고정 영역 변경 전 기록이다. 이번 네이티브 16×32 제어의 주행 결과로 해석하지 않는다. Jetson의 실제 TensorRT 생성·메모리·FPS 및 실기 완주는 현장에서 확인해야 한다.

2026-11-03 적용 확인에서는 실제 ONNX 네 방향 추론, 기본 실행기의 모델 선택, 브라우저의 2048×1024 입력 전달 및 두 프레임 응답을 확인했다. 이 짧은 브라우저 시험은 누락 꺼짐·보행 흔들림 켜짐이며, 시작 위치에서 두 번 모두 `path_unavailable`로 정지했다. 완주 검증이 아니다.

### 연속 추종 개선의 짧은 브라우저 확인

이후 `native-fcn-grid-v5-continuous`로 위 회전 확인·경로 유지·히스테리시스·상태 표시를 수정했다. 같은 FCN1024 가중치, 최대 2.0 m/s, 보행 흔들림 켜짐, 시드 9027로 누락 꺼짐·보통 누락 켜짐을 각 150프레임 시험했다.

| 검출 누락 | 같은 입력에서 이전 제어기의 정지 판단 | 수정 제어기의 정지 프레임 | 수정 제어기의 실제 가상 이동 |
| --- | ---: | ---: | ---: |
| 꺼짐 | 46 / 150 | 0 / 150 | 23.39 m |
| 켜짐 | 69 / 150 | 48 / 150 | 16.29 m |

수정 제어기로 실제 브라우저 폐루프를 진행하고, 그때 추론된 동일 격자를 이전 제어기에도 별도로 넣어 정지 판단을 비교했다. 이전 제어기로 별도의 폐루프를 실행한 결과는 아니다. 누락 켜짐의 정지 48프레임은 전체 누락 16프레임과 미관측 재확인 32프레임이다. 두 짧은 시험의 이탈·접촉 기록은 0회이며 우사 완주나 실기 성능을 검증한 결과가 아니다.

기록: `/Volumes/MY_HDD/01_동영상/02_360도_Insta360/분석결과/fcn1024-road-continuity-20261103/`. Python 관련 검사 119개와 JavaScript 검사 25개가 통과했다.
