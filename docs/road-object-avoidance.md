# Road 차량·사람 반대쪽 횡이동

Road 모드(`selected_mask=1`)에서 전면 영상의 사람·차량 검출을 LiDAR의 실제
3D 반사점과 대응시켜 위치 반대쪽으로 횡이동한다. 차도 추종은 기존 커넥톰이,
사람·차량 분류는 별도 TorchVision 검출기가, 횡이동은 로컬 회피 제어기가 담당한다.
커넥톰 가중치를 재학습하거나 새로운 학습된 횡이동 출력을 추가한 것은 아니다.

## 동작

- 로봇 기준 왼쪽 물체(`y>0`) → 오른쪽(`vy<0`), 오른쪽 물체 → 왼쪽.
- 정면 ±0.15 m는 직전 방향을 유지하고, 최초에는 기존 회피 선호 방향(기본 오른쪽)을 쓴다.
- 물체와의 거리가 가까울수록 **목표 횡속도 0.3~0.8 m/s**를 높인다.
  `0.3 + (0.8-0.3) × clamp(1 - 거리/6, 0, 1)`을 기본으로 사용한다.
- 가속도 제한은 기존 0.3 m/s²다. 따라서 출발·감속·정지 과정의 실제 명령은
  0.3 m/s보다 작을 수 있다. 반환 명령의 범위와 목표 속도 범위는 구별한다.
- 전진은 기존 명령 이하, 기본 최대 0.2 m/s로 제한한다. 커넥톰이 결정한
  회전율은 유지하고, 최종 속도 제한은 전진·횡이동·회전 비율을 함께 제한한다.
- 반대쪽 후보의 몸체 전체 이동 영역을 검사한다. 허용 가능한 0.3~0.8 m/s 후보가
  없으면 정지한다. 막혔다는 이유로 물체 쪽으로 방향을 바꾸지 않는다.
- 물체가 사라지면 기존 저속 회피·중앙 복귀 경로로 전환한다.
  차도 마스크, LiDAR, 카메라 결과의 유효 시간이 지나거나 깊이가 없으면 정지한다.
  커넥톰 정지, 작업 취소 및 종료는 회피 명령보다 우선한다.
- 이 기능은 물체의 이동 여부를 분류하지 않는다. 정차 차량도 대상이다.
  물체 속도 추적이나 장시간 움직임 예측은 수행하지 않는다.

COCO 분류 중 person, bicycle, car, motorcycle, bus, truck을 사용한다.
검출 상자 아래에 Road 마스크가 있고, 상자 안에 LiDAR 반사점이 2개 이상 있어야
횡이동 목표를 만든다. 밀집한 먼 배경보다 가까운 반사점 묶음을 우선한다.
기존 `SWIN_L_ROAD_ROI_EXCLUSION_POLYGON`의 카메라 고정 장치 가림 영역도
영상 검출에서 제외한다. 다른 카메라 장착에서는 이 가림 영역을 재확인해야 한다.

## 모델 및 적용

모델은 [TorchVision Faster R-CNN MobileNetV3 320 FPN COCO_V1](https://docs.pytorch.org/vision/stable/models/generated/torchvision.models.detection.fasterrcnn_mobilenet_v3_large_320_fpn.html)이다.
런타임은 로컬 파일과 SHA-256을 검사하며 자동 다운로드하지 않는다.

Mac에서는 다음 파일을 준비해 두었다.

- `models/road-objects-mobilenet-fpn.pth` — 검출 가중치, Git 제외
- `models/road-objects-mobilenet-fpn.json` — 출처와 SHA-256

다른 체크아웃에서는 `python tools/prepare_road_object_model.py`로 공식 가중치를
준비할 수 있다. SHA-256:

```text
907ea3f91ff92242bc1baea8049276a3e76bca48ce7560bd268cc029f37977b5
```

Jetson에서는 수정된 소스를 동기화하고 가중치를 다음 위치에 넣는다.

```text
~/dev/danjin-a2/cobiz-plugin-line-tracking/models/road-objects-mobilenet-fpn.pth
```

기존 전면 커넥톰 설정에 새 Compose 파일을 **마지막으로** 추가한다.
프로젝트 `.env`의 기존 모델 경로·해시는 유지한다.

```dotenv
COMPOSE_FILE=docker-compose.yml:docker-compose.front-connectome.yml:docker-compose.road-objects.yml
```

```bash
cd ~/dev/danjin-a2/cobiz-plugin-line-tracking
docker compose config --quiet
docker compose up -d --build
```

새 overlay는 LiDAR 회피 및 객체 검출을 활성화하고, 관측 격자를 반경 6 m,
360°, 좌우 4 m·후방 3 m로 구성한다. 센서에서 관측하지 않은 셀은 채우지 않는다.
기본 Compose만 실행하면 이 새 객체 회피 기능은 활성화되지 않는다.
`SWIN_L_ENGINE_DIR`를 변경했다면 위 가중치도 해당 `/models` 마운트 원본에 넣어야 한다.
소스 변경은 `actual-activate` 이미지 빌드에 포함된다. 실제 로봇의 실행·배포는 수행하지 않았다.

주요 설정은 `LINE_TRACKING_ROAD_OBJECT_MIN_LATERAL_MPS`,
`LINE_TRACKING_ROAD_OBJECT_MAX_LATERAL_MPS`, `...MIN_SCORE`, `...MAX_RANGE_M`이다.
최소·최대 횡속도는 0.3~0.8 m/s 밖으로 설정할 수 없다.
신뢰도 기본 0.55, 카메라 결과 최대 나이 0.8 s, LiDAR와 영상 시간 차이 최대 0.25 s다.
메트릭의 `road_object_avoidance`와 `lidar_obstacles.road_objects`,
`requested_lateral_mps`, `target_lateral_mps`, `vy`를 함께 확인한다.
Sport 변환은 기존 A2 보정에 따라 내부 `vy`를 `y=-vy`로 전송한다.

## 검증 범위

제어기·LiDAR 대응·ROS 작업 실행 및 Sport 출력 통합 검사 80개를 통과했다.
차량 크기 4.3×1.8 m, 0.35 m/s 접근을 사용하는 양방향 평면 운동 시험도 포함한다.
이는 모의 센서/물체 위치를 사용하는 제어 시험이며 실제 검출 정확도나
실제 로봇의 보행 안정성을 입증하지 않는다.

현장 추출 영상 110장에 실제 검출 가중치를 실행했다. 차량 검출을 확인했지만
일부 기둥·장비의 사람 오인도 남았다. 전수 정답 라벨이나 동기화된 LiDAR 대응 검증이
없으므로 정밀도·재현율 수치를 제공하지 않는다. Jetson 지연과 실물 횡보는 미검증이다.
기존 LiDAR 관측 범위 밖은 정지하며, 전면 영상 밖의 차도 의미는 새로 복원하지 않는다.
카메라 밖 공간은 기존 LiDAR 관측 가능 공간 정책을 사용하므로 측후방 지면 종류까지
보장하는 기능은 아니다.

브라우저의 이동 계산에는 `vy`와 횡이동 예상 궤적을 반영했다. **8773의 기존 전면
모델 서버는 이 ROS 객체 회피 파이프라인에 연결하지 않았다.** 그 서버로 실행하는
기존 화면의 완주 시험을 이번 기능의 종단 검증으로 간주하면 안 된다.

전체 Python 검사에서는 기존 AprilTag 거리 복원 실패 2건이 남아 있다.
별도 미추적 작업인 `test_road_corner.py`, `test_road_corner_ros.py`는
기존 `RoadPathResult` import 오류 때문에 전체 검사에서 제외했다.

```bash
.venv/bin/python -m pytest -q test/test_road_objects.py test/test_road_objects_ros.py \
  test/test_front_connectome.py test/test_local_avoidance.py \
  test/test_lidar_obstacles.py test/test_lidar_obstacles_ros.py
node --test test/test_dangjin_world.mjs test/test_dangjin_camera_sway.mjs
```
