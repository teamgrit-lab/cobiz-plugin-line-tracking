# Odom/IMU로 촬영 이후 이동을 보정하는 경로 추종

`LINE_TRACKING_MOTION_ENABLED=true`는 connectome의 검증된 표면 경로를 현재
몸체 좌표로 보정하고 기본 20Hz로 상위 속도 명령을 다시 계산한다. Mode 0·1·2에
동일하게 적용한다. 새 영상/장애물 관측을 20Hz로 만드는 기능은 아니다.
기존 local planner 및 MCAP 모드에는 적용하지 않으며 기본값은 꺼짐이다.

## 입력과 좌표

- `/dog_odom`: `nav_msgs/Odometry`, 기본 `odom → robot_center`.
- `/dog_imu_raw`: `sensor_msgs/Imu`, 기본 `dog_imu_link`. 사용하는 값은 gyro이며
  가속도 이중 적분이나 독립 자세 센서로서의 중복 EKF 융합은 하지 않는다.
- 경로는 기존 `base_link`와 ROI 지면 변환을 사용한다. `robot_center` 또는
  `dog_imu_link`가 base_link와 같다고 자동으로 가정하지 않는다.
- 두 고정 변환은 TF에서 읽거나 `LINE_TRACKING_MOTION_BASE_FROM_CHILD_TRANSFORM`,
  `LINE_TRACKING_MOTION_BASE_FROM_IMU_TRANSFORM`에 row-major 4×4로 설정한다.
  각각 **base_link ← odom child**, **base_link ← IMU** 방향이다. 빈 값이면 TF가
  필요하고, 없을 때에는 `motion_*_transform_unavailable`로 정지한다.

설정 예시:

```dotenv
LINE_TRACKING_PATH_PLANNER=connectome
LINE_TRACKING_MOTION_ENABLED=true
LINE_TRACKING_MOTION_ODOM_TOPIC=/dog_odom
LINE_TRACKING_MOTION_IMU_TOPIC=/dog_imu_raw
LINE_TRACKING_MOTION_CONTROL_HZ=20.0
# 아래 값에는 해당 장비에서 확인한 실제 고정 변환을 넣는다. 빈 값은 TF 조회다.
LINE_TRACKING_MOTION_BASE_FROM_CHILD_TRANSFORM=
LINE_TRACKING_MOTION_BASE_FROM_IMU_TRANSFORM=
```

센서 frame 이름은 `LINE_TRACKING_MOTION_ODOM_FRAME`, `ODOM_CHILD_FRAME`,
`IMU_FRAME`으로 설정할 수 있다. 잘못된 frame·quaternion·비강체 변환은 거부한다.
IMU 공분산이 0인 것은 알려지지 않은 불확실성으로 취급하며, gyro covariance 첫
항목이 -1이면 값을 사용할 수 없으므로 정지한다. covariance가 0이라고 완벽한
측정으로 간주하거나 가중치를 만들지 않는다.

## 경로와 명령

촬영 시각 `t_image`의 odom을 선형 위치/최단 yaw 보간으로 구한다. 기본 4초의
유한 이력을 유지하며, 약 650Hz인 원격 odom·gyro는 최대 100Hz로 이력에 저장한다.
아직 odom이 도착하지 않은 최신 구간만 최대 0.05초까지 odom 속도와 body gyro로
전파한다. gyro는 **마지막 odom 이후 구간에만** 적용한다.

```text
p_now = inverse(T_odom_base(now)) × T_odom_base(t_image) × p_image
```

원래 꼭짓점을 모두 유지하면서 선분을 0.10m 이하 간격으로 나눈다. 스플라인으로
마스크 밖을 가로지르거나 미관측 구간을 연장하지 않는다. 원래 경로와 허용 영역에
같은 강체 변환을 적용하므로 초기 마스크 검사와 중앙 범위가 보존된다.
LiDAR 사용 시 스캔도 현재 몸체 좌표로 옮기고, 영상 허용 영역은 역변환으로 조회한다.
격자의 관측 여부는 주변 네 셀 모두 관측됐을 때만 인정하며 비용은 최댓값을 사용한다.
원래 영상/스캔 stamp와 수신 시각을 바꾸어 유효기간을 연장하지 않는다.

기존 정지 판정을 먼저 적용한 뒤 실제 유효 경로의 목표점을 이용한 pure pursuit로
`vx`, `yaw_rate`를 계산한다. 짧은 경로는 실제 끝점을 쓰며 가상의 4m 점을 만들지
않는다. yaw 한계를 넘는 곡률에는 속도를 낮춘다. 전진 속도 상한과 yaw 상한은 기존
설정을 그대로 사용한다. 각 발행 직전에 다음 제한과 LiDAR 최종 검사를 수행한다.

- 전진 가속 기본 0.30m/s², yaw 변화 기본 0.50rad/s².
- yaw를 바꾸는 동안 요청된 곡률을 낼 수 없으면 전진을 먼저 멈춘다.
- 명시적 정지와 필요한 감속은 가속 램프를 기다리지 않는다.
- 마지막 발행 이후 제어 주기 3배를 넘기면 `motion_control_gap`으로 정지한다.

고빈도 모드는 Joint/보행 제어기를 대체하지 않고 기존 Sport API 발행자 하나를
유지한다. `ros2` 검사 모드는 계산한 명령을 metrics에만 표시하며 Sport API를
발행하지 않는다. 실제 명령은 기존 task-drive의 작업 수락과 소유권 규칙을 따른다.

## 만료와 검증

센서 수신 나이와 source stamp 나이는 기본 0.20초, 연속 sample 간격은 0.10초,
외삽은 0.05초로 제한한다. 영상은 기존 connectome 나이 제한과 motion 제한 중
더 짧은 것을 유지하며 기본 1초다. 촬영 후 이동 0.80m 또는 회전 25°를 넘으면
새 영상을 기다린다. pose 점프·시간 역행·frame 변경·잘못된 값은 epoch를 바꾸어
이미 추론 중이던 이전 영상의 결과도 재사용하지 못하게 한다. 센서가 회복돼도
새 epoch에서 촬영한 유효 경로가 도착해야 다시 움직인다. 몸체 기울기 20° 초과는
평면 모델의 적용 범위를 벗어나므로 정지한다. 모든 검사는 legacy stop 스위치와
무관하며 연결 오류 시 직진 유지나 탐색 회전으로 대체하지 않는다.

metrics의 `motion_tracking`은 준비 상태/사유, source stamp, odom·IMU 나이,
촬영 후 이동·회전, 경로 점 수, 직전 명령을 제공한다. `sensor_reason`과
`fault_counts`에는 경로가 없는 동안에도 센서의 현재 준비 상태와 오류 횟수를 남긴다. 검사 모드의
`preview_command`는 계산 결과이며 실제 발행 명령이 아니다. 보정된 Path 메시지의
stamp는 현재 몸체 시각이고, 촬영 시각은 `motion_tracking.source_stamp_sec`에 남는다.

```bash
python -m pytest test/test_motion_tracking.py test/test_motion_tracking_ros.py -q
```

검사는 촬영 시각 보간, yaw wrap/quaternion 부호, 센서 고정 변환, gyro 짧은 전파,
이력/지연/점프 거부, 기존 꼭짓점 보존, 가속·yaw 변화와 긴 제어 공백 정지,
Mode 0·1·2의 추론 사이 명령 보정, 작업 취소, LiDAR 격자 및 영상 영역의 동일 좌표
보정을 포함한다. 실제 주행이나 코너 성공을 대신하는 검사는 아니다.
