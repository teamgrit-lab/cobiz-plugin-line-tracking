# Sidewalk의 출발·도착 AprilTag 직선 구간

COBIZ `LINE_TRACKING` payload에 `selected_mask: 2`, `start_tag_id`, `end_tag_id`를
함께 넣으면 태그 직선 구간을 실행한다. 두 ID가 없으면 기존 Sidewalk 주행이다.
하나만 있거나 같은 ID이거나 Road/혼합 모드이면 거절한다. payload는 기존처럼
객체와 JSON 문자열을 모두 받는다.

```json
{
  "type": "TASK_REGISTERED",
  "action_name": "LINE_TRACKING",
  "task_id": "sidewalk-tags-001",
  "payload": {
    "selected_mask": 2,
    "duration_sec": 300,
    "start_tag_id": 7,
    "end_tag_id": 8,
    "tag_size_m": 0.167
  }
}
```

`tag_size_m`는 검출 모서리 사이의 실제 한 변 길이(미터)이며 생략 시 0.167이다.
두 태그는 같은 크기를 사용한다. 흰 종이 전체의 길이를 넣지 않는다.

## 동작과 완료

1. 360 카메라와 자세 피드백을 최대 5초 기다린다.
2. 기존 Sidewalk 주행 중 출발 ID의 유효한 자세·거리를 확인하면 정지한다.
3. 3m 이내의 출발 태그에서 일관된 평행 방향을 새 촬영 프레임 3회로 확인한다.
4. 현재 전방에서 회전량이 작은 평행 방향으로 제자리 정렬한다. ±3도 이내의
   서로 다른 자세 피드백 3회를 확인하면 직선 구간으로 전환한다.
5. 정렬 완료 위치에 직선을 고정한다. 기본 0.30m/s로 이동하며 자세와 발 odometry로
   방향·횡방향 이탈을 보정한다. 태그를 향해 계속 조향하는 방식은 아니다.
6. 도착 ID가 카메라에서 3m 이내이며, 고정 직선 방향으로 로봇 앞뒤 ±0.5m 안에
   들어오면 정지하고 새 프레임 3회로 확인한다. 확인되면 StopMove/zero 뒤
   `TASK_COMPLETED`, `reason: tag_route_arrived:<ID>`를 보고한다.

직선의 위치는 정렬을 마친 로봇의 위치다. 태그와의 간격을 특정 값으로 옮기는
횡이동·주차 기능은 포함하지 않는다. 진행 방향은 평행한 두 후보 중 현재 전방에
가까운 쪽이다. 90도에 가까워 어느 쪽인지 불명확한 출발 자세는 확인하지 않는다.

도착 태그를 출발 전에 본 결과는 완료에 사용하지 않는다. 다른 ID도 무시한다.
한 ID가 서로 다른 위치에서 중복 검출되면 확인하지 않는다. 이 작업에서는 기존
stop-on-any-AprilTag 설정을 사용하지 않고 지정된 두 ID와 순서를 따른다.
태그 면의 자세가 모호하면 정렬을 시작하지 않는다. 도착은 면의 각도 정렬을
요구하지 않지만 유효한 거리·상대 위치 추정이 필요하다.

출발/도착 확인 제한은 각각 5초, 제자리 정렬 제한은 20초, 직선 이동 제한은
기본 120초다. payload `duration_sec`도 전체 작업의 상한으로 항상 적용하며
상한 도달은 `TASK_ABORTED`다. 일반 주행의 duration 완료 스위치로 도착 전에
성공 처리하지 않는다.

## 배포와 설정

태그 주행 설정과 360 장치 매핑은 기본 `docker-compose.yml`에 포함돼 있다.
추가 Compose 파일 없이 `actual-activate` 이미지를 재빌드하고 실행한다.
출발·도착 ID가 모두 있는 작업에서만 태그 모드를 실행하며, 두 ID가 없는 작업은
기존 Sidewalk/Road 주행을 유지한다.

```bash
docker compose -f docker-compose.yml build actual-activate
docker compose -f docker-compose.yml up -d actual-activate
```

기존 Road/Navigation override는 해당 기능을 사용할 때만 유지한다. 태그 기능을
위한 별도 override는 필요 없다. 이전 설정의 `COMPOSE_FILE`에 태그 전용 파일을
추가했다면 그 항목을 제거한다. 위 명령은 배포 예시이며 이번 변경 과정에서 실제
로봇에 배포하거나 주행하지 않았다.

태그 기능을 명시적으로 끄려면 `.env`에서 `LINE_TRACKING_TAG_ROUTE_ENABLED=false`로
설정한다. 이때 ID가 있는 작업은 `tag_route_disabled`로 거절하고 일반 작업은 받는다.
기본 Compose는 컨테이너 생성 시 호스트의 `TAG_ROUTE_VIDEO_DEVICE` 장치가 있어야 한다.
장치 등록과 별개로 실제 카메라 수집은 태그 작업을 시작할 때만 수행한다.

| 설정 | 기본값 | 의미 |
| --- | --- | --- |
| `LINE_TRACKING_TAG_ROUTE_ENABLED` | `true` | 기본 Compose의 태그 payload 처리 활성화 |
| `TAG_ROUTE_VIDEO_DEVICE` | `/dev/video91` | 호스트의 360 MJPEG 장치 |
| `LINE_TRACKING_TAG_ROUTE_STATE_TOPIC` | `/lf/sportmodestate` | `unitree_go/SportModeState` |
| `LINE_TRACKING_TAG_ROUTE_FRONT_CENTER_X` | `720` | 2880px 영상의 설치 전방, 잠정값 |
| `LINE_TRACKING_TAG_ROUTE_CAMERA_X_M` / `LINE_TRACKING_TAG_ROUTE_CAMERA_Y_M` | `0` / `0` | base 기준 카메라 장착 위치, 전방/좌측 양수 |
| `LINE_TRACKING_TAG_ROUTE_CAMERA_Z_M` | `0` | base 기준 카메라 장착 높이, 위쪽 양수 |
| `LINE_TRACKING_TAG_ROUTE_CAMERA_ROLL_DEG` / `LINE_TRACKING_TAG_ROUTE_CAMERA_PITCH_DEG` | `0` / `0` | 전방 축을 정한 카메라 좌표에서 body로의 장착 회전, 도 단위 |
| `LINE_TRACKING_TAG_ROUTE_SPEED_MPS` | `0.30` | 직선 속도, 기존 전진 상한 이하로 제한 |
| `LINE_TRACKING_TAG_ROUTE_MAX_RANGE_M` | `3.0` | 출발/도착 태그의 카메라 거리 상한 |
| `LINE_TRACKING_TAG_ROUTE_ARRIVAL_AHEAD_M` | `0.5` | 도착 태그의 진행축상 앞뒤 허용 거리 |
| `LINE_TRACKING_TAG_ROUTE_STRAIGHT_TIMEOUT_SEC` | `120` | 직선 구간 제한 시간 |

family는 `SWIN_L_APRILTAG_FAMILY`, 확인 횟수와 촬영 유효 시간은
`SWIN_L_APRILTAG_CONFIRM_MIN_HITS` / `SWIN_L_APRILTAG_MAX_AGE_SEC`를 사용한다.
회전 상한도 기존 주행 설정을 따른다. 후방은 전방에서 180도 반대 방향이다.

360 장치는 태그 payload를 가진 단독 Sidewalk 작업에서만 연다. Road 서비스가
이전 작업의 장치를 잠시 유지하면 5초 준비 시간 내에 재시도한다. 취소·완료 시
장치를 해제하고 이전 세션의 늦은 결과를 버린다. Navigation 구간에는 이 동작을
적용하지 않으며, navigation-worker의 단독 COBIZ 작업은 제어권 승인 뒤에만 연다.

## 피드백과 정지

`SportModeState.imu_state.quaternion`의 Unitree 순서 `[w,x,y,z]`와 `position`을
사용한다. `rpy`가 0으로 고정되는 경우에도 동작한다. 촬영 시각 근처의 자세를
조회/보간해 태그 방향을 세계 좌표의 고정 방향으로 바꾸며 roll/pitch도 반영한다.
SLAM은 필요 없다. 수평선 주위의 여섯 투시 영상에서 세워진 태그를 찾는다.

출발 태그가 사라져도 360 촬영·검출 heartbeat와 자세 피드백이 살아 있으면
직진을 유지한다. 촬영이 오래되거나 자세 피드백이 0.5초 이상 끊기면 정지·중단한다.
정렬 이후에도 기존 Sidewalk 영상·추론의 유효 시간을 검사하고, 설정된 경로
불가/낮은 신뢰도/과도한 측방 목표 및 branch 정지를 유지한다. 경로 소실의 회전
검색으로 태그 직선 모드를 덮어쓰지 않는다. 고정 직선에서 0.5m 이상 벗어나도
중단한다. 취소·shutdown·제어권 임대 만료는 기존 정지 경로를 사용한다.

`metrics.tag_route`에는 phase, 두 ID, 확인 횟수, 목표 yaw, 횡방향 오차,
중단 사유와 카메라 오류가 나온다. 장애물 회피 경로를 생성하는 기능은 아니다.
카메라 장착 회전/위치와 태그 크기를 실제 설치에 맞춰야 하며 스티칭의 위치·자세
추정 오차는 남는다. 현재 전방 720과 장착 위치 0은 측정된 보정값이 아니다.

기하·상태 전이·운동학 시뮬레이션과 ROS/model/camera mock 통합 검증을 수행한다.
Jetson CUDA 이미지 빌드 및 실제 태그를 이용한 로봇 주행 검증은 별도다.
