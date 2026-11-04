# Road 추론 서비스의 카메라 직접 입력

세 가지 Road Compose override 모두 `LINE_TRACKING_ROAD_DIRECT_INPUT=true`를
지정한다. 학습 가중치·카메라 투영·속도 제한·FCN 경로 생성 방식은 유지하고,
추론할 영상을 제어기에 보낸 뒤 다시 HTTP로 보내던 과정을 제거한다.

| 구성 | 카메라 소유 서비스 | 입력 | 제어기가 받는 결과 |
| --- | --- | --- | --- |
| 360° + FCN | `fcn360-road` | USB `/dev/video91` 직접 수집 | 네 방향 원래 분류 격자·촬영 시각 |
| 전면 + Flyvis | `front-connectome` | A2의 `SWIN_L_IMAGE_TOPIC` 직접 구독 | 학습한 전진·회전·정지 명령·촬영 시각 |
| 360° + Flyvis | `connectome360` | USB `/dev/video91` 직접 수집 | 학습한 전진·회전·정지 명령·촬영 시각 |

각 구성은 표의 서비스와 `actual-activate` 두 컨테이너를 사용한다.
`actual-activate`는 CoBiz 작업과 실제 로봇 명령을 담당한다. Sidewalk(2)는
기존 전면 카메라·인도 모델·제어기를 계속 사용한다. 원래 A2 전면 카메라의
ROS 발행과 CoBiz·A2 명령의 ROS 통신은 유지한다.

Road(1) 작업에서만 요청하며, 최신 프레임 하나만 처리한다. 새 작업보다 이전인
영상·반복 시각·미래 시각·오래된 입력은 거부한다. 추론 후에도 원본 촬영 시각으로
신선도를 다시 검사한다. 모델 해시와 요청·작업 ID를 확인하고, 취소된 작업의 늦은
성공·실패가 다음 작업의 명령이나 중단 상태에 영향을 주지 않게 한다.
입력이 없으면 기존 결과의 촬영 시각을 갱신하지 않으며, 제어기의 신선도 검사로
0 명령을 발행한다. 연결·추론·응답 검증 실패는 해당 Road 작업을 중단한다.

카메라/전면 구독은 첫 Road 요청에서 열며 요청 중단 후 유휴 2.5초에 해제한다.
이 시간은 카메라 프레임 간격이 아니라 **추론 요청이 없는 시간**이다.
Road 종료·Sidewalk 전환 때 장치나 구독을 닫을 뿐, 컨테이너·프로그램을 종료하지 않는다.
Road 요청은 계속되지만 영상만 없으면 입력을 기다리며, 기존 촬영 시각을 갱신하지 않아
제어기의 신선도 검사로 정지한다. 다음 Road 요청에서는 카메라를 다시 연다.
HTTP에는 영상·PNG가 없고, 명령 또는 작은 분류 격자와 메타데이터만 전달한다.
`camera_transport=direct`, `camera_topic=null`이 Road 지표에 표시된다.
Sidewalk 지표는 `camera_transport=ros`, `inference_backend=legacy`다.

## 360° Road의 하드웨어 디코딩과 CUDA 입력

360° Flyvis와 FCN의 Compose 직접 입력은 다음 경로를 사용한다.

```text
/dev/video91 (2880×1440 MJPEG)
 → jpegparse → nvv4l2decoder mjpeg=true (Jetson 하드웨어 디코딩)
 → nvvidconv (VIC, BGRx 변환)
 → 최신 디코딩 프레임 1개 → 시스템 메모리 staging → CUDA 업로드
 → CUDA에서 전방·좌측·후방·우측 투영 및 모델별 정규화
 → CUDA 모델 추론 → 작은 검출 격자 또는 주행 명령만 CPU/HTTP로 반환
```

이 구현은 **NVMM/EGL zero-copy가 아니다.** 디코딩된 BGRx 버퍼를 Python에서
시스템 메모리로 매핑한 뒤 CUDA로 업로드한다. JPEG 소프트웨어 디코딩과 매 프레임의
CPU 투영·정규화는 제거하지만, 이 staging 복사는 남는다. 버퍼의 행 stride·offset을
반영하고 업로드 완료 후 GStreamer 버퍼를 해제한다. 촬영 시각은 디코더 출력 PTS로부터
복원하므로 전처리·추론 완료 시각으로 갱신하지 않는다.

- Flyvis: 같은 카메라 프로필의 480×270 BGR 네 방향을 GPU에서 생성하고,
  grayscale·letterbox·BoxEye 정규화부터 회로 특징 보간·차도 헤드·주행 헤드까지
  CUDA 텐서를 유지한다. 주행 헤드의 122행→32행 AREA 축소도 같은 픽셀 겹침 가중치를 쓴다.
- FCN: 기존 512×256/1024×512 방향·화각과 1/32 픽셀 보간 좌표를 유지하고,
  GPU에서 BGR→RGB·0..1·ImageNet 평균/표준편차 정규화를 수행한다. TensorRT에
  CUDA NCHW 텐서를 직접 전달하고 네 방향의 원래 클래스 격자만 내려받는다.
  전방 고정 로봇 마스크와 이후 제어기는 유지한다.
- 같은 보간 좌표의 CPU 참조와 비교하면 반올림에 따라 입력 색상이 8비트 기준
  1단계 정도 차이 날 수 있다. 플랫폼별 OpenCV remap 구현이나 JPEG 하드웨어
  디코더의 색 범위까지 포함한 출력 동등성은 실기 영상으로 확인해야 한다.
- CUDA 또는 `jpegparse`, `nvv4l2decoder`, `nvvidconv` 플러그인이 없으면 시작 오류로
  알린다. CPU 디코딩·추론으로 자동 전환하지 않는다. 기본 이미지 빌드에
  `nvidia-l4t-gstreamer`, GStreamer bad 플러그인과 GstVideo introspection을 포함한다.

`/health`의 모델 정보는 `camera_decode=nvv4l2decoder-mjpeg`,
`camera_conversion=VIC-BGRx`, `camera_preprocess=cuda`,
`camera_memory=host-staging-to-cuda`를 표시한다. 상태 조회·시작 시 더미 워밍업은
USB 카메라를 열지 않는다. Road 작업에서만 실제 영상을 추론하고, 요청 유휴 2.5초 후
장치를 해제한다. 모델 가중치·해시·컨테이너 수는 바꾸지 않는다. 실전 전진 상한은
기존 `.env`의 `LINE_TRACKING_MAX_FORWARD_MPS=0.80`을 유지한다.

전면 Flyvis의 ROS 이미지 입력, Sidewalk, 오프라인 영상 검사 및 브라우저의 CPU 참조
경로는 그대로다. 실제 FPS·GPU/EMC·전력·온도와 장시간 동작은 Jetson 검증 대기다.
구현 근거: [NVIDIA MJPEG 가속 디코딩 문서](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/Multimedia/AcceleratedGstreamer.html).

## 기존 구성에서 전환

진행 중인 작업을 종료하고 **기존 Compose 구성으로** 컨테이너를 내린다.
이 명령은 모델 폴더와 TensorRT 캐시 볼륨을 삭제하지 않는다.

```bash
docker compose down --remove-orphans
```

코드를 갱신하고 `.env`에서 사용할 override 하나만 선택한다. 모델 경로·해시와
기존 Sidewalk·전면 카메라·DDS 값은 유지한다. 직접 입력은 override가 활성화하므로
별도의 `.env` 플래그 추가는 필요 없다. 제거된 `connectome360-camera`가 남아
카메라 장치를 점유하지 않도록 한다.

기본 이미지를 먼저 빌드하고 선택한 추론 서비스를 빌드한다.

```bash
docker compose config --quiet
docker compose build actual-activate
# 360° Flyvis: docker compose build connectome360
# 전면 Flyvis: docker compose build front-connectome
# FCN은 actual-activate와 같은 이미지를 사용한다.
```

선택한 추론 서비스만 먼저 실행하고 `/health`의 `ready`, 모델 해시와
`schema=line_tracking.direct_camera.v1`을 확인한다. 포트는 360° Flyvis 8093,
전면 Flyvis 8092, FCN 8094다. 상태 확인 자체는 카메라를 열거나 명령을 보내지 않는다.

```bash
# 아래는 360° Flyvis의 예다.
docker compose up -d connectome360
curl --fail http://127.0.0.1:8093/health
docker compose up -d --no-deps --force-recreate actual-activate
docker compose logs --tail=100 connectome360 actual-activate
```

360° 가속 입력으로 전환할 때도 위처럼 기본 이미지를 먼저 재빌드하고, Flyvis이면
`connectome360`까지 다시 빌드한다. `.env`에 별도 가속 플래그를 넣거나 모델을 다시
복사할 필요는 없다. 실제 Road 요청 전에 컨테이너 안에서 다음 항목을 확인할 수 있다.

```bash
# 360° Flyvis: CUDA와 디코딩 플러그인 확인. USB 카메라는 열지 않는다.
docker compose exec connectome360 /opt/connectome-venv/bin/python -c \
  'from tools.surround_capture import HardwareCamera; HardwareCamera.check_runtime(); print("CUDA / MJPEG plugins OK")'
curl --fail http://127.0.0.1:8093/health
# FCN이면 서비스 이름은 fcn360-road, Python은 /opt/venv/bin/python, 포트는 8094다.
```

`/health` 확인은 가속 경로가 구성됐다는 의미이며 실제 카메라의 디코딩 성공·FPS를
검증한 결과는 아니다. Jetson에서 첫 Road 요청의 오류, 촬영→명령 지연, `tegrastats`의
CPU·GR3D·EMC·RAM·온도를 함께 확인한다.

이 변경의 확인 범위는 로컬 HTTP·모의 카메라·ROS 작업 수명 검사다.
Jetson CUDA/USB/DDS 통합, 실제 FPS·CPU 부하 개선 및 현장 주행은 별도로 측정해야 한다.
기존 이미지 입력 CLI와 시뮬레이션은 유지하며 모델을 재학습하거나 교체하지 않는다.
