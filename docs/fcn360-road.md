# 360° FCN으로 Road 작업 실행

Jetson의 cobiz-plugin-line-tracking 저장소 기준이다. CoBiz LINE_TRACKING의 selected_mask=1은 FCN-ResNet18 512×256과 Insta 카메라를 사용한다. selected_mask=2는 기존 SWIN_L_PROFILE, 전면 ROS 이미지 토픽, 인도 제어기를 사용한다. 이 Compose 구성에서 모드 생략은 Road, 혼합 모드 0은 거절이다.

## 모델과 설정

Mac에서 학습 완료된 폴더를 Jetson으로 복사한다.

    src=/Volumes/MY_HDD/connectum_models/fcn512-road-runs/20261102-120344-okwqgd25
    dst=/home/teamgrit/dev/dangjin-a2/cobiz-plugin-line-tracking/models/fcn512-road-20261102
    ssh dangjin-a2 "mkdir -p '$dst'"
    rsync -av --exclude='._*' "$src/" "dangjin-a2:$dst/"

Jetson 저장소의 .env에서 아래 항목을 설정한다. FCN override와 front-connectome/기존 connectome 플래너 override는 각각 다른 Road 설정이므로 하나를 선택한다. 기존 Sidewalk용 SWIN_L_PROFILE, SWIN_L_IMAGE_TOPIC, 체크포인트 설정은 현재 사용하는 값으로 둔다.

    COMPOSE_FILE=docker-compose.yml:docker-compose.fcn360.yml
    FCN360_MODEL_DIR=./models/fcn512-road-20261102
    FCN360_MODEL_SHA256=6c0c39112cf2f39b65904c27f313cad4a07c2369ef464e953cb3df1e9ccf0671
    FCN360_VIDEO_DEVICE=/dev/video91
    FCN360_HEIGHT_M=1.6
    FCN360_MAX_AGE_SEC=1.0

새 모델을 사용할 때는 FCN360_MODEL_DIR과 FCN360_MODEL_SHA256을 함께 변경한다. SHA는 학습 결과 complete.json의 sha256.fcn_resnet18.onnx 값이다. FCN360_HEIGHT_M에는 평지에서 렌즈 중심까지의 실측 높이를 넣는다. 초기값 1.6 m, 수평 장착, 렌즈와 base_link의 수평 위치 일치는 추정이다.

## 검출만 먼저 보기

Jetson 저장소에서 이미지를 빌드하고 검출 서비스만 시작한다. 이 서비스에는 주행 명령 퍼블리셔가 없다.

    docker compose build actual-activate
    docker compose up -d fcn360-road
    docker compose logs -f fcn360-road

첫 실행은 ONNX 해시를 검증하고 Jetson에서 TensorRT 10 FP32 엔진을 만든다. 엔진은 전용 볼륨에 저장하며 모델·GPU·TensorRT·CUDA 조합이 달라지면 다시 만든다. ROS 2 뷰어에서 sensor_msgs/CompressedImage 토픽 /line_tracking/fcn360/overlay/compressed 를 best-effort QoS로 구독하면 앞·좌·후·우 순서의 2×2 화면이 보인다. 초록색이 차도다. 같은 ROS 도메인과 TeamGRIT DDS 설정을 사용한다. 카메라는 2880×1440 MJPEG를 제공해야 하며, 구독자가 있을 때 연다. 이미 다른 프로세스가 장치를 독점한다면 검출 서비스 로그에 장치 오류가 표시된다.

## CoBiz 작업에 연결

    cd /home/teamgrit/dev/dangjin-a2/cobiz-plugin-line-tracking
    docker compose up -d --build actual-activate fcn360-road
    docker compose ps

Road 작업에서 /line_tracking/swin_l/metrics의 inference_backend는 fcn512-360이다. fcn360.front_path_valid, center_status, correction_left_m, rear_offset_left_m을 확인한다. Sidewalk에서는 backend가 legacy이고 camera_topic이 기존 전면 토픽이다. Road 마스크는 하나의 원본 촬영 시각을 공유하는 4×8×16 클래스 격자이며 모델 해시가 포함된 전용 토픽으로 전달한다. 측면 경계와 후방 중심의 거리 단위는 평지 가정에 따른 추정치다.

- 전방의 가까운 차도에 이어진 영역에서 경로를 추출한다. 마스트 양옆의 같은 영상 줄이 모두 차도이면 가린 부분의 중앙을 추정한다. 로봇·풀은 차도 클래스로 재분류하지 않는다.
- 좌우에서 차도와 비차도의 경계를 모두 찾고 도로 폭이 안정적일 때만, 연속 3장 이상 같은 방향으로 확인된 치우침을 전방 목표에 조금씩 반영한다. 마당처럼 한쪽 경계가 사라지면 측면 보정은 0이다. 후방 가시성과 중심 치우침은 진단 수치다.
- 새 작업 전의 마스크, 순서가 뒤바뀐 마스크는 폐기한다. 전방 경로가 없거나 마지막으로 처리한 촬영 시각이 허용 시간을 넘으면 Road 명령은 정지다.
