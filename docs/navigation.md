# Navigation 소유 구간 실행

`navigation-worker`는 기존 task-drive의 모델·노면 경로·추종·LiDAR 회피·odom/IMU 보정을
재사용하되 task와 로봇 명령 소유권을 `teamgrit-navigation`에 맡기는 실행 방식이다.
기존 standalone `LINE_TRACKING` task-drive와 ros2 검사 실행 방식은 유지한다.

```bash
docker compose -f docker-compose.yml -f docker-compose.connectome.yml \
  -f docker-compose.navigation.yml up -d --build actual-activate connectome-planner
```

기존 R50/Swin 모델 pin, CameraInfo·TF·센서 토픽과 motion 설정을 그대로 재사용한다.
overlay는 `SWIN_L_MODE`만 `navigation-worker`로 변경한다. Connectome을 쓰지 않는
배포는 중간 `docker-compose.connectome.yml`을 생략한다. 기존 task-drive 인스턴스를
교체하며 별도 GPU 모델 인스턴스를 추가하지 않는다.

worker는 `/task_event`를 구독하지 않고 `/task_state`, `/api/sport/request`에 발행하지 않는다.
다음 reliable/volatile String JSON 토픽만 구간 제어에 사용한다.

| 방향 | 토픽 | 내용 |
| --- | --- | --- |
| 입력 | `/line_tracking/segment_command` | START / HEARTBEAT / STOP |
| 출력 | `/line_tracking/segment_status` | ACCEPTED / WAITING / READY / STOPPED / ERROR / REJECTED |
| 출력 | `/line_tracking/command` | 기존 최종 안전 검사를 거친 속도 명령 |

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
worker에서는 AprilTag·duration 기반 자동 task 완료와 경로 소실 시 직진 유지/회전 탐색을
비활성화하고, 카메라/추론 freshness·경로 없음·confidence 검사를 활성화한다.
설정된 LiDAR 회피와 motion 최종 검사는 유지한다. WAITING 중에는 zero 명령을 보낸다.

`metrics.navigation_worker`와 `metrics.navigation_segment`로 실행 방식·현재 구간을 확인한다.
로컬 테스트는 실제 inference worker와 모델/ROS transport mock으로 두 노면의 재사용,
STOP/lease 만료, 이전 추론 결과 폐기와 Sport/task 발행 부재를 검증한다.
