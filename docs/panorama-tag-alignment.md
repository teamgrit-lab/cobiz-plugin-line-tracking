# 녹화된 360 영상의 태그 정렬 추정

`tools/panorama_tag_alignment.py`는 2:1 ERP 파노라마 영상에서 AprilTag를 찾아
전·후방 중심 오차와 태그 면에 대한 수평 방향 오차를 JSONL로 출력한다.
로봇을 제어하거나 기존 카메라 프로필·주행 모델을 변경하지 않는다.

설치 기준은 잠정적으로 2880px 영상의 전방 x=720, 후방 x=2160을 사용한다.
다른 해상도에서는 기본값이 각각 가로 폭의 1/4, 3/4이다. 실측 좌표는
`--front-center-x`, `--rear-center-x`로 지정한다. 이 값은 확인된 외부 보정값이
아니며, 출력에도 `axes_calibration: provisional`로 기록한다.

```bash
.venv/bin/python tools/panorama_tag_alignment.py \
  --video '/path/to/recording.mp4' \
  --front-center-x 720 --rear-center-x 2160 \
  --tolerance-deg 3 --confirm-frames 3 --sample-hz 5 \
  --output /tmp/tag-alignment.jsonl \
  --preview /tmp/tag-alignment.mp4
```

출력 파일은 새 경로여야 한다. `--output`을 생략하면 표준 출력으로 JSONL을
보낸다. `--preview`는 선택 사항이다. 특정 구간만 분석하려면
`--start-sec 80 --duration-sec 10`을 추가한다. 특정 태그만 보려면
`--tag-id 7` 또는 `--tag-id 7 8`을 추가한다. 기본 family는 기존 배포와 같은
`36h11`이며, ID를 지정하지 않으면 검출된 모든 태그를 보고한다.

첫 줄은 설정이며 이후 각 줄은 실제 디코딩한 샘플 프레임의 시각과 결과다.
소스 시각에 공백이 있으면 JSONL에 관측을 만들어 넣지 않는다. 표시 영상에는
해당 시간의 `NO SAMPLED SOURCE FRAME` 카드를 넣어 시간 공백을 유지한다.

| 필드 | 의미 |
| --- | --- |
| `panorama_x`, `panorama_y` | 원본 파노라마의 태그 중심 위치 |
| `front_error_deg`, `rear_error_deg` | 지정한 전·후방 중심까지의 최단 수평 각도 오차 |
| `horizontal_centered` | 전방 또는 후방 중심의 허용 오차 안에서 연속 확인됨 |
| `parallel_error_deg` | 태그 면의 수평 접선과 지정 전방 축 사이의 오차, -90..90도 |
| `parallel_aligned` | 유효한 자세 추정에서 평행 오차가 허용 범위 안으로 연속 확인됨 |
| `perpendicular_error_deg` | 태그 면의 수평 법선과 지정 전방 축 사이의 오차 |
| `pose_status` | 자세 추정의 유효성, 작은 태그·불확실한 해석 등의 상태 |

오차의 양수는 **파노라마 x가 증가하는 방향**이다. 로봇 회전 명령의 부호와
연결하지 않는다. 전·후방 중심에 있는 것과 태그 면에 평행한 것은 별도 판정이다.
태그를 옆에 두고 면과 평행하게 직진하려는 경우 `parallel_aligned`를 사용한다.
두 필드를 동시에 true로 요구하지 않는다. 평행 판정은 진행 가능한 두 방향을
구분하지 않으며, 어느 방향으로 이동할지는 별도 결정해야 한다.

태그 검출은 수평선을 중심으로 서로 겹치는 여섯 방향의 90도 투시 영상에서 수행한다.
천정·바닥 방향 전체를 검색하지는 않는다. 양 끝을
걸치는 태그도 검출할 수 있으며 같은 태그의 중복 투영은 한 관측으로 합친다.
자세 추정은 투시 영상의 가상 내부 파라미터와 PnP를 사용한다. 방향만 추정하므로
실제 태그 크기를 가정한 거리 값은 출력하지 않는다. 파노라마 수평 및 카메라
설치 방향을 기준으로 한 근사치이며, 스티칭 왜곡이나 카메라의 roll/pitch 오차를
교정하지 않는다. 측정한 태그 모서리와 평면 모델의 일치만으로 실제 정확도가
보장되지는 않는다.

태그가 없으면 `status: no_tag`, `detections: []`를 출력하고 확인 횟수를 초기화한다.
같은 ID의 태그가 서로 다른 위치에 동시에 있으면 `multiple_same_id`로 표시하고
정렬을 확정하지 않는다. 자세의 두 해가 비슷한 화상 오차로 서로 다른 방향을
설명하면 `ambiguous_pose`로 표시한다. 작은 태그나 자세 불확실성은 중심 방위각
추정을 막지 않지만 평행 정렬을 확정하지 못한다.

OpenCV의 aruco 모듈과 AprilTag dictionary가 필요하다. `ArucoDetector`와
Ubuntu 시스템 OpenCV의 이전 `detectMarkers` API를 모두 지원한다.
현재 작업 환경의 `.venv`에서 실행할 수 있다. 배포 컨테이너의 기존 ROS 검출기와
별도로 실행하는 오프라인 진단 명령이다. 태그 면의 방향, 카메라와 로봇의 횡방향
거리, 장애물·노면 및 경로 검증은 서로 다른 문제다. 이 결과는 실제 직진 경로나
주행 가능 여부를 결정하지 않는다.
