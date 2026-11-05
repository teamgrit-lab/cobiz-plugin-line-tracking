# Road 추론 서비스의 카메라 직접 입력

Road Compose override 모두 `LINE_TRACKING_ROAD_DIRECT_INPUT=true`를
지정한다. 학습 가중치·카메라 투영·속도 제한·FCN 경로 생성 방식은 유지하고,
추론할 영상을 제어기에 보낸 뒤 다시 HTTP로 보내던 과정을 제거한다.

| 구성 | 카메라 소유 서비스 | 입력 | 제어기가 받는 결과 |
| --- | --- | --- | --- |
| 360° + FCN | `fcn360-road` | USB `/dev/video91` 직접 수집 | 네 방향 원래 분류 격자·촬영 시각 |
| 전면 + Flyvis | `front-connectome` | A2의 `LINE_TRACKING_IMAGE_TOPIC` 직접 구독 | 학습한 전진·회전·정지 명령·촬영 시각 |
| 360° + Flyvis | `connectome360` | USB `/dev/video91` 직접 수집 | 학습한 전진·회전·정지 명령·촬영 시각 |
| 360° + R50 v7 + 기존 주행 모델 | `r50-360-policy` | USB `/dev/video91` 직접 수집 | 기존 주행 모델의 전진·회전·정지 명령·촬영 시각 |

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

카메라/전면 구독은 첫 Road 요청에서 열며 요청 중단 후 유휴 5초에 해제한다.
이 시간은 카메라 프레임 간격이 아니라 **추론 요청이 없는 시간**이다.
Road 종료·Sidewalk 전환 때 장치나 구독을 닫을 뿐, 컨테이너·프로그램을 종료하지 않는다.
Road 요청은 계속되지만 영상만 없으면 입력을 기다리며, 기존 촬영 시각을 갱신하지 않아
제어기의 신선도 검사로 정지한다. 다음 Road 요청에서는 카메라를 다시 연다.
HTTP에는 영상·PNG가 없고, 명령 또는 작은 분류 격자와 메타데이터만 전달한다.
`camera_transport=direct`, `camera_topic=null`이 Road 지표에 표시된다.
Sidewalk 지표는 `camera_transport=ros`, `inference_backend=r50`다.

Flyvis 이미지의 패키지 설치는 부모 이미지의 pip 인덱스·제약 환경변수와 설정 파일을
격리하고 PyPI에서 수행한다. 학습 환경에서 사용한 Hydra 1.3.7·OmegaConf 2.3.1·ANTLR
4.9.3 조합을 고정하며, CUDA PyTorch·torchvision·NumPy는 베이스 이미지의 정확한
버전 제약을 유지한다. 빌드 중 Flyvis/Hydra/OmegaConf import와 기존 CUDA PyTorch의
경로·버전·CUDA 버전 유지 여부를 확인한다. 이 설치 단계만 실패한 경우 기본 이미지가
이미 정상 빌드됐다면 `docker compose build connectome360`부터 다시 실행하면 된다.

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
전면 Flyvis 8092, FCN 8094다. R50 + 기존 주행 모델은 8093을 사용하므로 360° Flyvis와
동시에 실행하지 않는다. [R50 구성 사용법](r50-360-policy.md)을 참고한다.
상태 확인 자체는 카메라를 열거나 명령을 보내지 않는다.

```bash
# 아래는 360° Flyvis의 예다.
docker compose up -d connectome360
curl --fail http://127.0.0.1:8093/health
docker compose up -d --no-deps --force-recreate actual-activate
docker compose logs --tail=100 connectome360 actual-activate
```

이 변경의 확인 범위는 로컬 HTTP·모의 카메라·ROS 작업 수명 검사다.
Jetson CUDA/USB/DDS 통합, 실제 FPS·CPU 부하 개선 및 현장 주행은 별도로 측정해야 한다.
기존 이미지 입력 CLI와 시뮬레이션은 유지하며 모델을 재학습하거나 교체하지 않는다.
