# 360° FCN으로 Road 작업 실행

기본 검출기는 당진에서 파인튜닝한 **FCN-ResNet18 1024×512**다. CoBiz `LINE_TRACKING`의 `selected_mask=1`은 Insta 카메라와 FCN을, `selected_mask=2`는 기존 전면 카메라·인도 모델·제어기를 사용한다. 아래 명령은 Jetson의 `/home/teamgrit/dev/dangjin-a2/cobiz-plugin-line-tracking/`에서 Compose를 실행하는 기준이다.

## Mac에서 브라우저 실행

```bash
cd /Users/kangminwoo/Documents/GitHub/cobiz-plugin-line-tracking
./tools/run_dangjin_360.sh
```

`Ready` 후 <http://127.0.0.1:8775/>에서 로봇 실험을 연다. 기본 모델은 아래 2026-11-03 패키지이며 최대속도는 시뮬레이션 기준 2.0 m/s다. 이미 켜 둔 서버는 종료 후 다시 실행하고 브라우저를 새로고침한다. 다른 포트는 `--port 8776`으로 지정한다.

이전에 모델 경로를 지정했다면 다음처럼 새 모델을 명시할 수 있다.

```bash
DANGJIN_FCN_MODEL_DIR='/Volumes/MY_HDD/02_학습모델_데이터셋/03_FCN_ResNet18/모델/fcn1024-road-runs/20261103-081851-c90tq27l' \
  ./tools/run_dangjin_360.sh
```

## Jetson에 코드와 모델 복사

새 코드는 1024×512 추론, 원래 16×32 격자의 ROS 전송·경로 계산을 지원한다. 모델 파일만 바꾸면 이전 512 전용 코드는 새 규격을 처리하지 못하므로 코드도 반영해야 한다.

커밋된 변경을 Jetson에 pull해서 반영한 경우 아래 코드 복사를 생략한다. 현재 Mac 작업 파일을 직접 적용하려면 **Mac 터미널에서** 다음 네 파일을 복사한다.

```bash
cd /Users/kangminwoo/Documents/GitHub/cobiz-plugin-line-tracking
rsync -avR \
  tools/fcn360_road.py \
  tools/fcn360_detector.py \
  tools/swin_l_local_path_debug.py \
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

`FCN360_INPUT_WIDTH`는 검출기와 제어기에 함께 전달된다. `FCN360_HEIGHT_M`에는 렌즈 중심의 실측 높이를 넣는다. 현재 1.6 m·수평 장착은 추정값이다. 실기의 전진 상한은 기존 `LINE_TRACKING_MAX_FORWARD_MPS` 설정을 사용하며 현재 코드 상한은 1.0 m/s다. 브라우저의 2.0 m/s는 별도 시뮬레이션 배율이다.

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
- Road 작업에서 `/line_tracking/swin_l/metrics`의 `inference_backend`는 `fcn1024-360`이다. `fcn360.front_path_valid`, `maneuver`, `observed_horizon_m`, `speed_fraction`을 확인한다. Sidewalk 작업의 backend는 `legacy`이며 기존 전면 카메라와 인도 제어기를 사용한다.
- 모델 해시가 포함된 `/line_tracking/fcn360/masks/model_<SHA>`에는 네 방향의 원래 `4×16×32` 클래스 격자가 전송된다. ROS `Image`는 `mono8`, `height=64`, `width=32`, `step=32`, 데이터 2,048바이트다. 0은 미관측, 1은 고정 로봇, 3은 차도다.
- 전방 고정 팔 영역은 0 기준 14~17열·10~15행, 1 기준 15~18열·11~16행이다. 512 기준 2열×3행과 같은 화면 비율의 4열×6행이며 경로 계산에서는 미관측으로 제외한다. 그 밖의 로봇 예측도 미관측으로 처리한다.
- 전방과 연결된 노면과 좌우 차도를 기준으로 경로를 선택하고, 새 회전 방향은 새 마스크 세 장에서 연속 확인한다. 후방은 진단에 사용한다. 유효 경로가 없거나 촬영·추론 시각이 기본 1초를 넘으면 정지한다. LiDAR·odom·지도·목표점은 이 Road 경로 생성의 입력이 아니다.

기존 512 모델도 해당 디렉터리·해시와 `FCN360_INPUT_WIDTH=512`를 함께 지정하면 원래 `4×8×16` 격자로 사용할 수 있다. 두 모델 모두 같은 전방 시야·정규화와 화면 비율의 팔 마스크를 사용한다.

과거 512/1024 비교 영상은 1024 결과를 8×16으로 축소한 호환 재생 및 팔 고정 영역 변경 전 기록이다. 이번 네이티브 16×32 제어의 주행 결과로 해석하지 않는다. Jetson의 실제 TensorRT 생성·메모리·FPS 및 실기 완주는 현장에서 확인해야 한다.

2026-11-03 적용 확인에서는 실제 ONNX 네 방향 추론, 기본 실행기의 모델 선택, 브라우저의 2048×1024 입력 전달 및 두 프레임 응답을 확인했다. 이 짧은 브라우저 시험은 누락 꺼짐·보행 흔들림 켜짐이며, 시작 위치에서 두 번 모두 `path_unavailable`로 정지했다. 완주 검증이 아니다.
