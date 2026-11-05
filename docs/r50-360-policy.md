# 360° R50 v7 검출 + 기존 학습형 주행 모델

Road 검출기만 `r50-surface-v7/best`로 교체한다. 주행 모델은 그림자 증강 이전
`connectome360-both-20261103-135251-10mps`의 `policy/drive-readout.pt`를 그대로 쓴다.
주행 모델 재학습이나 Flyvis 가중치 수정은 하지 않는다. 이 구성의 차도 검출에는
Flyvis 인코더·차도 헤드를 실행하지 않는다.

## 입력과 명령

```text
/dev/video91 → MJPEG JPEG 디코딩 → 2880×1440 BGR 파노라마
 → 상대방위 전방 90°·왼쪽 180°·후방 270°·오른쪽 0°, 각 480×270
 → 동일 R50 v7 가중치로 네 방향 배치 추론 (CUDA FP16)
 → 65개 의미 클래스 중 Road 묶음=1, 그 외=0
 → 각 마스크 480×270 → 행 [135:257] → 64×32, 네 방향 순서 유지
 → 기존 DriveReadout: 전진 속도·회전 속도·정지 여부
 → 로컬 HTTP로 명령과 촬영 시각 반환 → actual-activate → Sport Move
```

R50은 640×360 입력으로 리사이즈한 후 프로세서가 패딩한다. 출력 의미 점수는
640×360으로 복원하고 클래스 argmax를 취한다. 기존 R50의 Road 묶음은 `Road`,
`Crosswalk - Plain`, `Lane Marking - Crosswalk`, `Lane Marking - General`,
`Manhole`이다. 풀·연석·인도·로봇과 그 밖의 클래스는 주행 모델 입력에서 0이다.
0은 주행 모델에 넣지 않는 영역이며, 가려진 부분을 관측한 비차도 정답으로
확정한다는 뜻은 아니다. 팔 사이 차도를 임의로 채우지 않는다.

네 방향을 하나의 시간 필터에 순서대로 넣지 않는다. 이 Road 검출은 프레임별
배치 추론이며 기존 Sidewalk의 단일 카메라 시간 필터나 후처리 상태를 공유하지 않는다.
JPEG 디코딩·투영은 현재 master와 같은 CPU 처리이고 R50·주행 모델은 GPU에서 실행한다.
별도의 하드웨어 디코딩 변경을 포함하지 않는다.

입력 차원은 기존 주행 모델과 동일한 `[4,32,64]`이고 지도·목표점·LiDAR·odom은
넣지 않는다. 명시적인 중앙 경로나 Pure Pursuit로 교체하지 않고 기존 CNN 주행
모델이 명령을 결정한다. 모델의 원래 10 m/s 출력 척도를 유지한 뒤 속도·회전을 같은
비율로 줄인다. 어댑터의 전진 상한은 0.80 m/s이고 최종 발행에는
`LINE_TRACKING_MAX_FORWARD_MPS`와 기존 회전 상한을 적용한다. 횡이동은 0이다.

## 필요한 모델과 설정

저장소 루트의 `models/`에 다음 파일들이 있어야 한다. R50 모델을 이미 설치했다면
그대로 사용한다. 두 원본 모델 폴더는 읽기 전용으로 마운트하며 수정하지 않는다.

```text
models/r50-surface-v7/best/
  config.json
  preprocessor_config.json
  model.safetensors
  checkpoint-manifest.json
models/connectome360-both-20261103-135251-10mps/
  model.json
  flyvis_encoder.pt
  road/road-readout.pt
  policy/drive-readout.pt
  policy/training.json
```

인코더와 이전 차도 가중치는 실행하지 않지만 원본 패키지 무결성 검사를 위해 필요하다.
R50 manifest와 원본 주행 패키지의 `model.json`을 각각 핀으로 검증한다. 원본 주행
체크포인트에 기록된 학습 당시 검출기 SHA와 실제 주행 가중치 SHA도 계속 확인한다.
검출기 교체는 `detector_override=true`로 구분하며 R50이 주행 학습에 사용됐다고
위장하지 않는다.

`.env`에 다음을 지정한다. 다른 Road override와 함께 선택하지 않는다.
Sidewalk용 `LINE_TRACKING_PROFILE`, `LINE_TRACKING_MODEL_ID`,
`LINE_TRACKING_CHECKPOINT_SHA256`, 전면 카메라 토픽 및 제어 설정은 유지한다.

```dotenv
COMPOSE_FILE=docker-compose.yml:docker-compose.r50-360-policy.yml
R50_360_MODEL_DIR=./models/r50-surface-v7/best
R50_360_MODEL_SHA256=1fab9a4d1e6ca4b9d1bd6efc9d6a6367daff42d0e40a6c3bcda2045a6b9a31a8
CONNECTUM_360_MODEL_DIR=./models/connectome360-both-20261103-135251-10mps
CONNECTUM_360_MODEL_SHA256=1dd52c0e04834aaaf21f863201b1208ecb3725eb7ea49b84d1b7ee27f5d2b4e0
CONNECTUM_360_VIDEO_DEVICE=/dev/video91
LINE_TRACKING_CONNECTOME360_TIMEOUT_SEC=2.0
LINE_TRACKING_CONNECTOME_MAX_AGE_SEC=1.5
LINE_TRACKING_MAX_FORWARD_MPS=0.80
```

기존 `.env`에 더 낮은 전진 상한이 있으면 그 값을 우선한다. 0.80 m/s를 쓰려면 위처럼
명시한다. 서비스와 실행기 양쪽의 R50 핀은 Compose가 같은 값으로 전달한다.

## 빌드와 실행

Jetson에서 위 `.env`를 설정하고 기존 `connectome360`·`fcn360`처럼 `/dev/video91`이나
8093 포트를 쓰는 이전 Road 서비스는 먼저 종료한다. 카메라 소유자는 하나만 실행한다.

```bash
docker compose config --quiet
docker compose build actual-activate r50-360-policy
docker compose up -d --force-recreate --remove-orphans actual-activate r50-360-policy
docker compose logs --tail=100 r50-360-policy actual-activate
curl --fail http://127.0.0.1:8093/health
```

두 서비스가 같은 기본 런타임 이미지를 사용한다. 별도 Flyvis 이미지 빌드는 필요 없다.
`r50-360-policy`는 직접 카메라 입력·검출·주행 모델 추론을 맡고 Sport 명령을 발행하지
않는다. `actual-activate`는 Cobiz 작업 수락, 응답 검증, 속도 상한, Sport 발행을 맡는다.

서비스 시작 때는 합성 영상으로 가중치만 준비한다. 실카메라는 첫 Road 요청에서 열고
추론 요청이 **5초** 동안 없으면 해제한다. 컨테이너는 유지하고 다음 요청에서 다시 연다.
이 설정은 입력 영상 최대 나이(1.5초), HTTP 응답 제한(2초)과 별개다.

Cobiz `LINE_TRACKING`의 `selected_mask=1`은 새 Road 구성,
`selected_mask=2`는 기존 Sidewalk 검출기·제어기를 사용한다. 혼합 모드 0은 거부한다.
`/line_tracking/metrics`에서 Road는 `inference_backend=r50-360-policy`,
`camera_transport=direct`, `camera_topic=null`이다. `connectome360` 항목에는 네 방향
차도 비율, 추론 시간, 주행 모델의 원시 출력·정지 확률·최종 명령을 담는다.
Sidewalk는 기존처럼 `inference_backend=r50`, `camera_transport=ros`다.

연결 실패·응답 지연·식별값 불일치는 해당 Road 작업을 중단한다. 오래된 명령은
재사용하지 않고 기존 정지·취소 처리를 유지한다. 기존 세 구성으로 돌아갈 때는 해당
Compose override 하나를 다시 선택한다. 각 override는 R50 검출기 교체 핀을 해제한다.

## 확인 범위

단위·HTTP·가상 ROS 시험으로 네 방향 입력, 풀·연석·로봇 제외, 기존 주행 가중치와
출력 일치, 두 모델 식별값 검사, 실제 속도 상한, Road/Sidewalk 전환과 취소를 확인한다.
2026-11-05 관련 회귀 검사 463개와 Compose 설정 검증이 통과했다. Volume의 실제 R50 v7과
원본 주행 가중치를 Mac MPS에 로딩해 2026-11-04 녹화 영상 한 프레임을 네 방향으로
추론하고 명령을 생성했다. 이는 Jetson 처리 속도 측정이나 실주행 시험은 아니다.
검출기 교체로 주행 모델이 학습 때와 다른 마스크 분포를 받는다. 이 연결만으로 실제
연석 회피나 주행 안정성을 검증한 것은 아니며 Jetson 실주행 검증은 별도로 필요하다.
