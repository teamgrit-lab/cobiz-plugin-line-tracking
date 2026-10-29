# Navigation 소유 구간 실행

`navigation-worker`는 한 모델로 기존 `LINE_TRACKING` Task와 Navigation의
`sidewalk`/`road` 구간을 모두 받는다. Navigation 구간의 완료·다음 모드 전환은
`teamgrit-navigation`이 담당한다. 별도의 `task-drive`와 ros2 검사 실행 방식도 유지한다.

NAVIGATION_3D는 `Nav2 시작 → Nav2 완료·종료 확인 → line-tracking 시작 →
waypoint 도착·line-tracking 종료 확인 → Nav2 시작 → Nav2 완료·종료 확인`으로 실행한다.
Sidewalk → Road도 STOPPED를 확인한 뒤 새 identity와 mask로 START한다. 구간 사이에
별도 heading controller를 실행하지 않으며, 새 주행기가 조향을 담당한다. SLAM은 계속
유지하고, 노면을 찾지 못했을 때의 기존 정지 동작도 유지한다. 모델은 메모리에 유지한다.

```bash
docker compose -f docker-compose.yml -f docker-compose.connectome.yml \
  -f docker-compose.navigation.yml up -d --build actual-activate connectome-planner
```

기존 R50/Swin 모델 pin, CameraInfo·TF·센서 토픽을 그대로 재사용한다.
overlay는 `SWIN_L_MODE`만 `navigation-worker`로 변경한다. Connectome을 쓰지 않는
배포는 중간 `docker-compose.connectome.yml`을 생략한다. 기존 task-drive 인스턴스를
교체하며 별도 GPU 모델 인스턴스를 추가하지 않는다.

Navigation 구간은 `/api/sport/request`와 `/task_state`에 직접 발행하지 않는다.
다음 reliable/volatile String JSON 토픽을 구간 제어에 사용한다.

| 방향 | 토픽 | 내용 |
| --- | --- | --- |
| 입력 | `/line_tracking/segment_command` | START / HEARTBEAT / STOP |
| 출력 | `/line_tracking/segment_status` | ACCEPTED / WAITING / READY / STOPPED / ERROR / REJECTED |
| 출력 | `/line_tracking/command` | 기존 최종 안전 검사를 거친 속도 명령 |

기존 `LINE_TRACKING` Task는 `/task_event`로 받으며 `selected_mask=0/1/2`, duration,
AprilTag와 자동 정지 스위치 등 기존 설정을 그대로 사용한다. `/line_tracking/task_request`로
제어권을 요청하고 A2 `/line_tracking/task_control`의 GRANTED를 받은 후에만
`TASK_STARTED`와 기존 Sport 출력을 시작한다. 단독 Task는 SLAM/지도 TF를 요구하지 않는다.
Navigation 준비·주행 중이거나 다른 소유자가 있으면 제어권을 거절한다.

요청은 `protocol:1`, 매 실행마다 새 `session_id`, 문자열 `task_id`, `stamp_sec`,
`command:ACQUIRE/HEARTBEAT/RELEASE`를 포함한다. 응답은 같은 identity에
`state:GRANTED/REJECTED/STOPPED/ERROR`를 포함하며 GRANTED의 `lease_sec`는 1초다.
worker는 승인을 최대 5초 기다리고, 승인 갱신이 1초 끊기면 기존 hard stop과
`TASK_ABORTED`를 보낸다. 시작 전 거절·시간 초과는 `TASK_REJECTED`다.
worker가 죽어서 스스로 정지할 수 없는 경우에는 A2의 별도 stop-only publisher가
임대 만료를 감지해 StopMove/zero를 보낸다. Navigation의 `enable_robot_driver=false`
검사 실행에서는 이 publisher와 단독 제어권 승인을 모두 비활성화한다.

단독 Task 종료 시 기존 StopMove/zero 출력을 수행하고 제어권을 반납한다. Navigation은
기존 Sport publisher의 1초 정지 출력이 끝나도록 2초간 기다린 후 다음 구간을 넘긴다.
시작 직후에도 2초 동안 이전 프로세스의 임대가 만료되기를 기다린다. 대기 중인 Navigation
goal의 취소·ingress lease 만료를 처리하며, 3.5초 안에 슬롯이 해제되지 않으면 거절한다.
동시에 두 Task가 주행하지 않는다. 새 worker와 A2 Navigation을 함께 재빌드해야 한다.

```json
{
  "protocol": 1,
  "session_id": "navigation-generated-unique-session",
  "segment_id": "1",
  "command": "START",
  "selected_mask": 2,
  "stamp_sec": 1792620000.0,
  "lease_sec": 1.0,
  "guidance": {
    "frame": "prior_map",
    "target_x": 10.0,
    "target_y": 4.0,
    "robot_x": 0.0,
    "robot_y": 0.0,
    "robot_yaw": 0.0,
    "pose_stamp_sec": 1792620000.0
  }
}
```

`selected_mask=2`는 인도, `1`은 차도다. 같은 identity의 START 재전송은 초기화하지 않는다.
동일 구간의 노면 변경이나 다른 owner의 START는 거절한다. 새 구간은 새 identity를 사용한다.
HEARTBEAT는 같은 identity·새 stamp·최대 1초 lease를 사용하며 모델을 재시작하지 않는다.
source stamp가 0.4초보다 오래되었거나 미래로 0.05초보다 앞서면 제어 요청을 무시한다.
호스트 시계는 동기화되어야 한다.

Navigation의 인도·차도 구간은 기본적으로 waypoint 방향을 사용한다. A2가 START와
HEARTBEAT마다 같은 지도 좌표의 목표와 최신 SLAM 자세를 보낸다. 목표와 frame은 구간 중
바꿀 수 없다. worker는 가장 최근에 전달받은 SLAM 자세로 목표를 로봇 좌표로 변환한다.
카메라 source stamp와 자세 시각을 맞추거나 보간하지 않으며, Jetson에서 자세의 0.5초 나이를
중복 검사하지 않는다. 송신 당시 유효했던 자세가 전달 지연만으로 `invalid_guidance`가 되는
경로를 제거했다. 오래된 순서의 자세는 최신 자세를 덮어쓰지 않는다. 잘못된 형식·비정상 좌표와
구간 중 목표/frame 변경은 계속 거절한다. A2의 localization 검사와 명령 stamp·lease는 유지한다.

감지된 노면을 로봇 폭과 여유 폭만큼 축소한 뒤 연결된 갈림길 후보를 추출한다. 같은 전방
거리에서 waypoint 방위에 가장 가까운 후보를 고르고 기존 branch 확인 횟수(기본 2프레임)를
채울 때까지 정지한다. 비슷한 후보의 방위 오차 차이가 3도 미만이면
`waypoint_branch_ambiguous`로 정지한다. 선택 후에는 목표 방위가 흔들려도 갈림길을 유지하며,
카메라 사이의 로봇 이동·회전을 SLAM 자세로 보정한다. 선택한 경로가 사라지면 반대편으로
갈아타지 않고 정지한다. 다음 구간의 새 generation에서 선택을 초기화한다.

인도는 기존 로컬 기하 엔진으로, 차도는 기존 Connectome 엔진으로 선택된 통로를 따라간다.
Connectome 요청에 선택된 `corridor_xy`와 로봇 좌표의 `target_xy`를 전달하고 서버·클라이언트
모두 최신 노면과 모든 경로 선분을 검사한다. 경로 EMA가 갈림길 사이를 가로지르지 않도록
검사된 경로를 그대로 사용한다. waypoint 방향은 통로 선택과 허용된 중앙 band 내 후보
점수에 반영하며, 노면 밖으로 직선 경로를 만들지 않는다. 지도 전체의 우회 경로 탐색은
수행하지 않으므로 갈림길 뒤에도 waypoint를 배치해 진행 방향을 구체적으로 지정한다.

새 A2는 worker READY/RUNNING의 `waypoint_guidance: true`를 확인한다. 구형 worker는
`surface_waypoint_guidance_unsupported`로 중단하고, waypoint 요청을 무시하는 구형
Connectome 서버도 경로를 허용하지 않는다. **A2 Navigation과 Jetson worker를 함께
업데이트하고, Connectome 사용 시 connectome-planner도 재빌드해야 한다.** 기존 단독
`LINE_TRACKING` Task와 guidance 없는 이전 구간 프로토콜은 기존 추종 방식을 유지한다.

STOP은 추론을 중지하고 경로/저장 명령을 비우며 zero command 뒤에 STOPPED를 보고한다.
이미 실행 중이던 추론은 완료될 수 있지만 generation 불일치로 결과를 폐기한다.
STOP이 START보다 먼저 도착해도 기록하여 뒤늦은 START가 구간을 되살리지 못한다.
lease가 만료되면 동일하게 정지하고 ERROR를 보고한다. 모델 가중치는 메모리에 유지한다.

상태와 속도 모두 `protocol`, `session_id`, `segment_id`, `stamp_sec`를 포함한다.
속도에는 증가하는 `sequence`, `vx`, `vy`, `yaw_rate`가 추가된다. `vy`는 기존
`drive_to_sport_move`의 부호 변환을 이미 적용한 driver/Sport 기준이므로 수신자가
다시 반전하지 않는다.

waypoint 도착·현재 실행기 종료 확인·다음 Mode 실행·최종 task 완료는 navigation 담당이다.
Navigation 구간의 주행 정지 검사는 단독 LINE_TRACKING과 같은 `.env`의
`LINE_TRACKING_STOP_ON_CAMERA_STALE/INFERENCE_STALE/CAMERA_TIMESTAMP_INVALID/`
`PATH_UNAVAILABLE/PATH_LOSS_LIMIT/LOW_CONFIDENCE/LATERAL_TARGET` 값을 사용한다.
이 값을 강제로 true로 바꾸지 않으며 `LINE_TRACKING_PATH_LOSS_RECOVERY_ENABLED`도 그대로
사용한다. 일반적인 waypoint 경로 소실도 같은 정지/유지/복구 정책을 따르고, 확인 중인 갈림길·
모호한 후보·선택한 갈림길 이탈 등의 제약은 유지한다. 차도의 Connectome 유효성 검사는
단독 실행과 동일하게 적용한다. AprilTag·duration 기반 별도 task 완료는 Navigation 소유권에
맞게 비활성화한다. 설정된 LiDAR 회피와 STOP·lease 만료 정지는 유지한다.

`metrics.navigation_worker`와 `metrics.navigation_segment`로 실행 방식·현재 구간을 확인한다.
`metrics.waypoint_guidance`에는 로봇 좌표의 목표, 갈림길 선택 상태와 정지 이유가 표시된다.
로컬 테스트는 실제 inference worker와 모델/ROS transport mock으로 두 노면의 재사용,
STOP/lease 만료, 이전 추론 결과 폐기와 Navigation 구간의 Sport/task 발행 부재를 검증한다.
단독 Task와 sidewalk → Nav2 → road 전환, 직접 접근 후 종료 및 단일 모델 재사용도 검증한다.
구형 직접 goal 발행자의 aligned_handoff 회전은 호환 경로로 별도 검증한다.
