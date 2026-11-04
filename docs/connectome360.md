# 360도 커넥톰 — Jetson Road 연결

브라우저 없이 기존 360도 검출·주행 모델 쌍을 실행한다. 기본은 그림자 증강 이전
`connectome360-both-20261103-135251-10mps`이며 그림자 버전으로 자동 교체하지 않는다.
모델 원본과 `model.json`의 학습·검증 이력은 수정하지 않는다.

## 실행 경로

```text
/dev/video91 (2880×1440, stitched ERP MJPEG)
 → connectome360 안에서 직접 수집·상대방위 90°를 전방으로 투영
 → 전방·좌측·후방·우측 480×270 배열을 메모리에서 직접 추론
 → 고정 Flyvis → 차도 검출 헤드 → 주행·정지 헤드
 → 로컬 HTTP로 명령·촬영 시각만 actual-activate에 반환
 → 실제 속도·회전 상한 적용 → 기존 Unitree Sport Move 발행
```

Road(`selected_mask=1`)에서만 360도 영상과 학습한 명령을 사용한다. Sidewalk(`2`)는
기존 `SWIN_L_IMAGE_TOPIC`, `SWIN_L_PROFILE` 검출기·제어기를 사용한다. 생략한 작업은
Road가 기본이며 혼합 모드 `0`은 거부한다. 지도·목표점·LiDAR·odom·브라우저 서버는
360도 모델 추론에 넣지 않는다. 컨테이너는 `connectome360`, `actual-activate` 두 개다.
Road 요청 때 카메라를 열고, 요청이 끊긴 후 유휴 1.5초가 지나면 해제한다.
영상이 1.5초 없다고 강제 종료하는 설정이 아니다. 프로그램은 유지하며 다음 요청에서 다시 연다.
영상의 ROS·PNG·HTTP 재전송은 없다. [직접 입력 구조와 전환 방법](direct-camera.md)을 참고한다.

두 모델의 Road 입력·학습 방식은 그대로다. 정답이나 미리 정한 경로로 검출 결과를
보완하지 않고, 실기에 합성 그림자·보행 흔들림·랜덤 검출 누락을 주입하지 않는다.

## 모델 복사와 설정

Jetson의 저장소 루트에서 Compose를 실행한다. 모델 폴더는 `models/` 아래에 둔다.
최소 추론 파일은 다음과 같다. 전체 모델 폴더를 복사해도 된다.

```text
models/connectome360-both-20261103-135251-10mps/
  model.json
  flyvis_encoder.pt
  road/road-readout.pt
  road/simulation-training.json
  policy/drive-readout.pt
  policy/training.json
```

데이터셋·학습 캐시·시뮬레이션 파일은 실기 추론에 필요하지 않다. 실행 코드는
복사한 모델 폴더의 `runtime/` 대신 현재 저장소를 빌드한 이미지에서 사용한다.

`.env`에서 다음을 설정한다. 전면 커넥톰·FCN·기존 광류 커넥톰 Compose override를
함께 지정하지 않는다. Sidewalk의 기존 모델 관련 설정과 전면 카메라 토픽은 유지한다.
`/dev/video91`을 쓰는 기존 FCN 검출 서비스나 다른 카메라 프로그램도 중복 실행하지 않는다.

```dotenv
COMPOSE_FILE=docker-compose.yml:docker-compose.connectome360.yml
CONNECTUM_360_MODEL_DIR=./models/connectome360-both-20261103-135251-10mps
CONNECTUM_360_MODEL_SHA256=1dd52c0e04834aaaf21f863201b1208ecb3725eb7ea49b84d1b7ee27f5d2b4e0
CONNECTUM_360_VIDEO_DEVICE=/dev/video91
LINE_TRACKING_CONNECTOME360_TIMEOUT_SEC=2.0
LINE_TRACKING_CONNECTOME_MAX_AGE_SEC=1.5
LINE_TRACKING_MAX_FORWARD_MPS=0.50
```

`CONNECTUM_360_MODEL_SHA256`는 **model.json 자체**의 SHA-256이다. 시작할 때 이 파일의
해시와 안에 기록한 인코더·검출·주행 가중치의 실제 해시, 검출기와 주행 모델의 결합,
학습 당시 카메라 프로필을 확인한다. 다른 버전을 선택할 때는 경로와 해시를 함께 바꾼다.

```bash
sha256sum ./models/connectome360-both-20261103-135251-10mps/model.json
```

카메라는 이미 스티칭된 2880×1440 ERP 영상이어야 한다. 두 어안 원본을 그대로 넣거나
영상을 임의로 크롭·확대하지 않는다. 상대방위 90°가 전방이며 장착 높이·기울기는
기존 미확인 추정값을 유지한다. 이 추정값을 실제 교정 결과라고 해석하지 않는다.

## 빌드와 실행

베이스 이미지에 새 카메라 실행 코드가 포함돼야 하고 Flyvis 이미지는 그 이미지를
바탕으로 만들어지므로, **빌드를 두 단계로 실행한다.** 서비스 이름 순서에 의존하지 않는다.
기존 Jetson CUDA PyTorch를 유지하며 Flyvis 1.2.0을 별도 가상환경에 설치한다.

```bash
docker compose build actual-activate
docker compose build connectome360
```

추론 서비스만 먼저 열면 로봇으로 주행 명령을 보내지 않고 모델 로딩을 확인할 수 있다.
카메라·추론 서비스 자체에는 Sport 명령 발행기가 없다.

```bash
docker compose up -d connectome360
curl --fail http://127.0.0.1:8093/health
```

실제 작업을 받는 구성을 시작한다.

```bash
docker compose up -d --force-recreate actual-activate
docker compose logs --tail=100 connectome360 actual-activate
```

이 명령은 의존 서비스도 시작한다. Cobiz의 `LINE_TRACKING` 작업에
`selected_mask=1`을 주면 Road, `2`를 주면 Sidewalk다. 실제 로봇 실행은 이 작업을
수락한 동안에만 이뤄진다. `/line_tracking/swin_l/metrics`의 `inference_backend`가
Road에서 `connectome360`, Sidewalk에서 `legacy`인지 확인할 수 있다.
Road의 `camera_transport`는 `direct`, `camera_topic`은 `null`이다.
처음 전환할 때는 기존 `connectome360-camera` 컨테이너를 종료·제거해야 한다.

## 명령 척도와 오류 처리

기존 모델의 10 m/s는 합성 학습의 출력 척도다. 이를 임의로 0.5로 바꿔 로딩하면
학습된 속도와 회전의 비율이 달라지므로 모델은 원래 척도로 추론한다. 전달 전에
속도·회전 양쪽에 같은 축소 비율을 적용해 곡률을 보존한다. 어댑터 상한은 0.8 m/s,
최종 발행은 기존 `LINE_TRACKING_MAX_FORWARD_MPS`(기본 0.50 m/s)와 회전 상한을 따른다.
횡이동 명령은 0이다. 학습한 정지 선택은 속도·회전 모두 0을 만든다.

주행 명령은 같은 촬영 시각·요청 ID·모델 해시의 응답만 수용한다. 새 작업 이전의
대기 영상과 이전 작업의 늦은 응답은 사용하지 않는다. 영상 또는 추론이 오래되면
0 명령을 발행한다. 서비스 연결 실패·시간 초과·응답 검증 실패는 해당 Road 작업을
중단한다. 오래된 입력으로 인한 작업 종료 시점은 기존 작업 정책을 따른다.
Sidewalk로 임의 대체하지 않으며 이후 Sidewalk 작업은 계속 받을 수 있다.

## 확인 범위와 모델 한계

이 변경은 Jetson용 실행 경로를 제공한다. Mac에서 프로토콜·모델 무결성·Road/Sidewalk
전환·취소·오래된 입력 처리와 실제 모델의 CPU 추론을 확인한다. Jetson CUDA 이미지
빌드, GPU 메모리·처리 속도 측정, 실제 로봇 주행 성공을 확인한 결과는 아니다.

그림자 증강 이전 버전도 풀 오검출과 미검수 추정 정지 명령에 대한 낮은 일치도가
있다. 실행 가능하게 연결했다는 사실이 검출 정확도·정지 성능을 개선하지는 않는다.
`real_robot_validated=false` 등 원본 모델의 검증 기록은 유지한다.
