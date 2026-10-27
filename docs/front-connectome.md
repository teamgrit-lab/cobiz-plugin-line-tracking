# 전면 카메라 Connectome Road 모델

Mac에서 학습한 전면 카메라용 실험 모델과 `cobiz-plugin-line-tracking` 연결 방법입니다.
Jetson에 설치하거나 실제 로봇을 움직인 결과는 포함하지 않습니다.

## 모델 구성

```text
전면 BGR 영상 1장 (480 × 270)
 → 사전 학습 Flyvis 1.2.0 시각 회로 (가중치 고정)
 → 학습한 차도 분할 출력층
 → 학습한 속도·회전·정지 출력층
 → vx [m/s], yaw_rate [rad/s]
```

추론에 360도 영상, LiDAR, 지도, 목표점, 로봇 위치를 넣지 않습니다. 현재 행동 출력층은 한 프레임의 차도 마스크를 사용하며 영상 이력을 학습한 모델은 아닙니다. 시각 회로는 매 영상마다 회색 기준 상태에서 0.02초 간격으로 10회 적분합니다. Flyvis 전체를 처음부터 학습한 것이 아니라 기존 차도 출력층을 파인튜닝하고 전면 행동 출력층을 새로 학습했습니다.

차도 마스크와 실제 영상의 전진·회전·정지 목표는 자동 추정 라벨입니다. 기록된 운전자 명령이나 전문가 정답으로 간주하면 안 됩니다. 시뮬레이션의 지도 기반 교사는 학습 데이터 생성에만 사용하며 추론 입력에는 포함하지 않습니다. 학습·검증은 촬영 세션으로 분리했습니다. 자동 라벨 교사의 기존 학습 데이터 중복 여부는 감사되지 않았습니다.

## 인도 데이터 제외 후 재학습

v2는 인도 주행, 보행 공간이 섞인 구간, 자동 라벨의 보행 영역 후보와 앞뒤 3초를 제외했습니다. 9월 촬영의 시작 흙바닥 및 마지막 혼합 구간도 보수적으로 제외했습니다. 실제 학습 표본은 2,138 → 1,755개, 검증은 849 → 772개, 테스트는 171 → 86개입니다. 별도의 시뮬레이션 표본 896개는 유지했습니다. 제외한 라벨 프레임은 총 545개이며 문맥 프레임까지 포함하면 2,541개입니다.

정확한 ID, 이유, 기준, 대표 이미지는 `data-selection/` 및 `/Volumes/MY_HDD/connectum_data_selections/front-road-only-v2/`에 있습니다. 원본 데이터는 삭제하지 않았습니다. 기존 v1의 학습된 출력층을 재사용하지 않고 동일한 시뮬레이션 초기 차도 출력층에서 다시 학습했습니다. 고정 시각 회로의 영상 특징 캐시만 재사용합니다. 자동 후보 검출 및 대표 장면 검토에 기반한 보수적 선별이며 모든 픽셀을 사람이 검수한 데이터는 아닙니다. 평가 모집단도 달라졌으므로 v1과 v2 IoU를 그대로 비교하면 안 됩니다.

## 저장물

기본 패키지: `/Volumes/MY_HDD/connectum_models/front-road-v2-road-only-20261027/`

- `front_model.pt`: CPU `state_dict`로 저장한 차도 및 행동 가중치
- `flyvis_encoder.pt`: 사전 학습 시각 회로 가중치, 회로 설정 및 출처
- `model.json`: 입력 크기, 임계값, 속도 범위, 가중치 SHA-256
- `runtime/`: 실제 모델 구조와 서비스 실행에 필요한 Python 코드
- `infer.py`, `serve.py`: 이미지 추론 및 로컬 HTTP 서비스 진입점
- `source/`, `initial-road-readout.pt`, `training-samples.json`, `provenance.json`: 학습·통합 코드와 재현 정보
- `road-training.json`, `actor-training.json`, `validation/`: 평가 지표와 원본 주행 기록
- `SHA256SUMS.json`: 파일 무결성 목록

`.pt` 파일만 옮기지 말고 이 폴더 전체를 옮깁니다. Mac MPS 전용 바이너리나 TensorRT 엔진이 아니라 가중치와 Python 모델 구조이므로 호환 환경에서 CPU/CUDA로 로드할 수 있습니다. 실제 Jetson의 Flyvis·PyTorch·CUDA 호환성과 처리 속도는 별도 확인해야 합니다.

## Mac에서 재현

저장소 루트에서 기존 Flyvis 환경을 사용합니다.

```sh
MPLBACKEND=Agg .cache/connectome-venv/bin/python tools/train_front_connectome.py \
  --output /Volumes/MY_HDD/connectum_models/front-road-v2-road-only-20261027 \
  --cache .cache/front-road-v2 \
  --exclusions /Volumes/MY_HDD/connectum_data_selections/front-road-only-v2/exclusions.json
MPLBACKEND=Agg .cache/connectome-venv/bin/python tools/validate_front_connectome.py \
  --model /Volumes/MY_HDD/connectum_models/front-road-v2-road-only-20261027
MPLBACKEND=Agg .cache/connectome-venv/bin/python tools/dangjin_sim_server.py \
  --front-model /Volumes/MY_HDD/connectum_models/front-road-v2-road-only-20261027 \
  --device cpu --road-device mps --port 8773
```

학습에는 원본 데이터셋과 저장소의 `.cache/dangjin-connectome-training`, `.cache/dangjin-policy-training`, 초기 차도 가중치가 필요합니다. `training-samples.json`은 정확한 입력 목록입니다. `--stage actor`로 차도 학습 완료 후 행동 학습만 재개할 수 있습니다. 실행 환경 버전은 `provenance.json`에 기록합니다.

브라우저에서 `http://127.0.0.1:8773/` → **로봇 실험** → **추종 시작**을 선택합니다. 전면 모델이 연결되면 360도 미리보기는 숨겨지고 전면 영상만 서버로 전송됩니다. 전체 누락 시 행동 모델이 정지를 학습하며 다음 관측에서 재출발합니다. 완주 여부, 이탈 및 금지구역 접촉은 지도와 충돌 형상을 사용하는 별도 평가에서 기록합니다. 평가가 로봇 위치를 교정하거나 조향 명령을 만들지는 않습니다.

## Jetson으로 옮긴 뒤 연결

Jetson 저장소와 Compose 실행 위치는 `~/dev/danjin-a2/cobiz-plugin-line-tracking/`입니다.
기존 `models/` 안에 버전별 모델 패키지 폴더를 둡니다. Mac에서 복사하는 예이며,
이 문서는 명령을 자동 실행하지 않습니다.

```sh
ssh dangjin-a2 \
  'mkdir -p ~/dev/danjin-a2/cobiz-plugin-line-tracking/models/front-road-v2-road-only-20261027'
rsync -av /Volumes/MY_HDD/connectum_models/front-road-v2-road-only-20261027/ \
  dangjin-a2:~/dev/danjin-a2/cobiz-plugin-line-tracking/models/front-road-v2-road-only-20261027/
```

Jetson 저장소에도 이 변경의 `tools/`, `Dockerfile.connectome`,
`requirements-connectome.txt`, `docker-compose.front-connectome.yml`이 필요합니다.
Docker는 저장소의 `tools/`를 이미지에 복사하므로 모델 패키지의 `runtime/`만
교체해서는 플러그인 코드가 갱신되지 않습니다. NVIDIA용 기존 PyTorch를 일반
PyPI wheel로 덮어쓰지 마세요. `Dockerfile.connectome`은 기존 tracking 이미지의
PyTorch를 재사용하고 버전 제약을 검사합니다.

### Compose 설정과 시작

Jetson 저장소의 `.env`에서 다음 항목을 추가하거나 같은 이름의 값을 변경합니다.
기존 센서와 보정 설정은 유지합니다.

```dotenv
COMPOSE_FILE=docker-compose.yml:docker-compose.front-connectome.yml
CONNECTUM_FRONT_MODEL_DIR=./models/front-road-v2-road-only-20261027
CONNECTUM_FRONT_SHA256=43cddf61ba3781b7d241f274d9a6259b1409c0541d650dd3d31aa91b35cf6f88
SWIN_L_IMAGE_TOPIC=/a2/front_camera/res_360p/image_raw
SWIN_L_BACKEND=pytorch
SWIN_L_TRT_AUTO_BUILD=false
```

`COMPOSE_FILE`로 두 Compose 파일을 함께 읽고, 지정한 모델 폴더를 추론 컨테이너의
`/front-model`에 읽기 전용으로 연결합니다. 실제 구독 토픽은 `SWIN_L_IMAGE_TOPIC`이
결정합니다. `actual-activate`의 기본 토픽은 `/a2/front_camera/image_raw`이므로
360p 입력을 사용하려면 위 값을 명시합니다. 토픽의 `360p`는 해상도이며 360도
파노라마 입력을 의미하지 않습니다.

커넥톰 이미지는 기본 tracking 이미지를 기반으로 빌드합니다. 현재 Compose에는
이미지 빌드 의존성이 선언되지 않아 최초 설치나 기본 이미지 변경 시 기본 이미지를
먼저 빌드합니다. `depends_on`은 서비스 시작 순서를 지정합니다.

```sh
cd ~/dev/danjin-a2/cobiz-plugin-line-tracking
docker compose config --quiet
docker compose build actual-activate
docker compose up -d --build front-connectome
```

모델 초기화가 끝난 뒤 단독 추론 서비스의 상태를 확인합니다.

```sh
curl http://127.0.0.1:8092/health
```

`ready: true`, `camera_views: ["front"]`, 예상한 `weights_sha256`을 확인한 다음
전체 서비스를 시작합니다.

```sh
docker compose up -d --build
```

Road 모드 `1`, `--path-planner local`, 별도 inference process 및 motion 보정 비활성 상태에서 사용합니다. 로컬 경로 추정은 표시와 기존 장애물 검사에 쓰이며 속도·회전은 새 행동 모델에서 가져옵니다. 잘못된 모드, 응답 해시 불일치, 오래된 카메라 영상, 서비스 오류, 작업 취소는 주행 허용 조건을 만족하지 못합니다. 기존 작업 권한·AprilTag·LiDAR 정지 검사는 유지합니다.

전면 영상은 480×270으로 변환되어 `http://127.0.0.1:8092/v1/front-road`로 전달됩니다.
서비스 시작 후 Road 모드의 `LINE_TRACKING` 작업을 수락하면 추론과 추종을 수행합니다.
이 문서 작성 과정에서는 Jetson 서비스를 시작하거나 실기 주행하지 않았습니다.
기본 Compose와 기존 R50 모드는 바뀌지 않습니다.

### 다른 모델로 교체

`CONNECTUM_FRONT_SHA256`은 **`front_model.pt` 파일의 SHA-256**입니다. 패키지의
`jetson.env.example`, `model.json`의 `weights_sha256["front_model.pt"]`,
`SHA256SUMS.json`에서 확인할 수 있습니다. 복사한 파일에서 직접 계산하려면:

```sh
# Jetson 저장소 루트에서 실행; 새모델폴더를 실제 폴더 이름으로 변경합니다.
sha256sum ./models/새모델폴더/front_model.pt
# Mac에서 계산할 때:
shasum -a 256 /Volumes/MY_HDD/connectum_models/새모델폴더/front_model.pt
```

호환 모델 패키지를 새 버전 폴더에 복사하고 `.env`의 모델 경로와 SHA-256을 함께
변경합니다. 가중치와 일치하는 `model.json`도 필요합니다. 파일을 덮어써도 실행 중인
프로세스가 자동으로 모델을 다시 읽지는 않으므로 컨테이너를 다시 생성합니다.

```sh
docker compose up -d --build --force-recreate front-connectome actual-activate
```

### Docker 없이 패키지 추론

Flyvis 1.2.0 및 호환 CUDA PyTorch가 설치된 Python 환경에서는 독립 패키지로도
저장 영상과 서비스 로딩을 확인할 수 있습니다. Compose 서비스와 동시에 같은 포트를
사용하지 않습니다.

```sh
python /절대경로/front-road-v2-road-only-20261027/infer.py 영상.png --device cuda --mask 예측.png
python /절대경로/front-road-v2-road-only-20261027/serve.py --device cuda --port 8092
```

직접 실행하는 플러그인의 연결 설정은 `LINE_TRACKING_FRONT_CONNECTOME_URL`,
`LINE_TRACKING_FRONT_CONNECTOME_SHA256`입니다. Compose는 각각 서비스 URL과
`CONNECTUM_FRONT_SHA256` 값을 전달합니다. 추론 응답 제한 시간은
`LINE_TRACKING_FRONT_CONNECTOME_TIMEOUT_SEC`, 입력 최대 나이는
`LINE_TRACKING_CONNECTOME_MAX_AGE_SEC`이며 기본값은 각각 1초입니다.

## 평가를 읽는 방법

`road-training.json`의 실제 영상 IoU는 전문가 정답 정확도가 아니라 자동 라벨과의 일치도입니다. 검증 세션과 테스트 세션을 별도로 확인해야 합니다. `validation/portability.json`은 Mac CPU와 MPS 수치 비교이고 Jetson 성능 측정이 아닙니다. 브라우저 폐루프 시험은 같은 농장 형상의 합성 영상과 운동학 기반이며 실제 A2 보행 안정성, 카메라 흔들림, 지면 접촉, 제동 거리까지 검증하지 않습니다.


### v2에서 확인된 한계

차도 전용 보류 세트에서 자동 라벨과의 IoU는 검증 0.9678(772장), 테스트 0.9288(86장)입니다. 정지 표본은 매우 적습니다. 남은 검증 명령 662개 중 자동 추정 정지 표본 4개를 모두 놓쳤습니다. 빈 검출 마스크에서는 학습된 정지 출력이 나오지만, 완전히 검은 카메라 영상에서는 차도를 오검출하여 약 0.20 m/s를 출력했습니다. 따라서 이 모델은 실험용이며 실제 A2 주행에 바로 적용할 준비가 된 모델로 분류하지 않습니다. 전문가가 확인한 정지·장애물·카메라 이상 장면을 보강하고 Jetson 지연 및 실기 검증이 필요합니다.

실제 HTTP 서비스와 플러그인 어댑터의 입력 색상, 모델 해시, 타임스탬프 및 불허 입력 검사를 확인했습니다. ROS 동작은 테스트 하네스에서 검증했고 실제 ROS/Sport 연결은 하지 않았습니다. Mac CPU/MPS 비교 8장에서는 마스크가 일치했고, 모델 폴더만 `/tmp`에서 실행하는 독립 추론도 통과했습니다. 상세 수치는 `validation/`의 보고서와 로그를 확인하세요.


### 폐루프 브라우저 시험

동일 가중치로 전면 합성 RGB만 입력한 실제 추론을 실행했습니다. 시드 9001(보통)은 1,031.04m를 완주하며 전체 누락 81회 뒤 재출발했고, 시드 9002(강함, 초기 횡방향 오차 0.6m)는 1,030.59m를 완주하며 전체 누락 160회 뒤 재출발했습니다. 두 시험 모두 도로 이탈·금지구역·구조물 접촉 기록은 0회였습니다. 지도/목표점/위치는 행동 모델에 입력하지 않았고, 평가기는 명령 적분을 대조하여 위치 보정이 없었음을 확인했습니다. 이 결과는 두 시나리오의 성공이며 모든 누락 패턴에 대한 보장은 아닙니다.

`validation/summary.json`에서 전체 결과를 볼 수 있습니다. 보통 시험 뒤 출력 상한의 float32 반올림을 수정했고, 최종 런타임은 강한 누락 시험·수치 경계 단위 테스트·CPU/MPS 비교·HTTP 시험으로 확인했습니다. 보통 시험의 이전 런타임 소스도 보고서와 함께 보존했습니다. Python 관련 테스트 227개와 지도/움직이는 차량 형상/누락 UI 로직 테스트 14개가 통과했습니다. 차량 형상 단위 테스트가 실제 모델의 차량 회피 성능을 의미하지는 않습니다.
