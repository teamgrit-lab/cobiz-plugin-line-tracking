# Navigation 소유 구간 실행

`navigation-worker`는 한 모델로 기존 `LINE_TRACKING` Task와 Navigation의
`sidewalk`/`road` 구간을 모두 받는다. Navigation 구간의 완료·회전·다음 모드 전환은
`teamgrit-navigation`이 담당한다. 별도의 `task-drive`와 ros2 검사 실행 방식도 유지한다.

```bash
docker compose -f docker-compose.yml -f docker-compose.connectome.yml \
  -f docker-compose.navigation.yml up -d --build actual-activate connectome-planner
```

기존 R50/Swin 모델 pin, CameraInfo·TF·센서 토픽과 motion 설정을 그대로 재사용한다.
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
  "lease_sec": 1.0
}
```

`selected_mask=2`는 인도, `1`은 차도다. 같은 identity의 START 재전송은 초기화하지 않는다.
동일 구간의 노면 변경이나 다른 owner의 START는 거절한다. 새 구간은 새 identity를 사용한다.
HEARTBEAT는 같은 identity·새 stamp·최대 1초 lease를 사용하며 모델을 재시작하지 않는다.
source stamp가 0.4초보다 오래되었거나 미래로 0.05초보다 앞서면 제어 요청을 무시한다.
호스트 시계는 동기화되어야 한다.

STOP은 추론을 중지하고 경로/저장 명령을 비우며 zero command 뒤에 STOPPED를 보고한다.
이미 실행 중이던 추론은 완료될 수 있지만 generation 불일치로 결과를 폐기한다.
STOP이 START보다 먼저 도착해도 기록하여 뒤늦은 START가 구간을 되살리지 못한다.
lease가 만료되면 동일하게 정지하고 ERROR를 보고한다. 모델 가중치는 메모리에 유지한다.

상태와 속도 모두 `protocol`, `session_id`, `segment_id`, `stamp_sec`를 포함한다.
속도에는 증가하는 `sequence`, `vx`, `vy`, `yaw_rate`가 추가된다. `vy`는 기존
`drive_to_sport_move`의 부호 변환을 이미 적용한 driver/Sport 기준이므로 수신자가
다시 반전하지 않는다.

waypoint 도착·다음 방향 정렬·다음 Mode 실행·최종 task 완료는 navigation 담당이다.
Navigation 구간에서는 AprilTag·duration 기반 자동 task 완료와 경로 소실 시 직진 유지/회전 탐색을
비활성화하고, 카메라/추론 freshness·경로 없음·confidence 검사를 활성화한다.
설정된 LiDAR 회피와 motion 최종 검사는 유지한다. WAITING 중에는 zero 명령을 보낸다.

`metrics.navigation_worker`와 `metrics.navigation_segment`로 실행 방식·현재 구간을 확인한다.
로컬 테스트는 실제 inference worker와 모델/ROS transport mock으로 두 노면의 재사용,
STOP/lease 만료, 이전 추론 결과 폐기와 Navigation 구간의 Sport/task 발행 부재를 검증한다.
단독 Task → sidewalk → Spin → Nav2 → road → 단독 Task 전환과 단일 모델 재사용도 검증한다.
