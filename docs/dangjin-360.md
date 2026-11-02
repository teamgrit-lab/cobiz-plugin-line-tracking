# 최신 지도에서 360도 카메라 시험

```bash
cd /Users/kangminwoo/Documents/GitHub/cobiz-plugin-line-tracking
./tools/run_dangjin_360.sh
```

`Ready` 후 <http://127.0.0.1:8775/> → **로봇 실험 → 우사 전체 한 바퀴 → 추종 시작**.
8766에서 실행 중인 이전 기준선 서버와 별개다. 종료는 실행 터미널의 Ctrl+C.
다른 포트는 `./tools/run_dangjin_360.sh --port 8776`으로 지정한다.

전면·360도 시뮬레이션의 최대 전진 속도는 기본 **1.0 m/s**다.
`--max-speed 0.8`처럼 실행 옵션으로 낮출 수 있다. 모델의 정규화된 전진 출력을
이 상한에 맞춰 환산하며, 회전 명령과 학습된 정지 판단은 그대로 사용한다.
모델 가중치·Jetson 설정은 변경하지 않는다. 실행 중인 서버에는 재시작 후 반영되며,
브라우저도 새로고침해야 한다. 화면의 최대속도는 연결된 서버의 설정을 표시한다.

이 실행기는 `--front-model`을 사용하지 않는다. 로봇에 고정된 360도 카메라에서
앞·좌·뒤·우 RGB 네 장을 얻고, Flyvis 차도 출력층과 학습된 행동 출력층으로
속도·회전·정지를 결정한다. 지도·위치·목표점은 행동 모델에 전달되지 않는다.
현재 지도는 `2026-10-28-wider-perimeter-compost-apron`이다.

## 전면 모드와 공유하는 기능

- 확장한 12 m 외곽 도로와 퇴비사 옆 포장 구역, 중앙 통행 금지 구역.
- 랜덤 검출 구멍·순간 전체 누락, 강도 선택 및 동일 시드 재현.
- 보행 카메라 ±5°, 1.7 Hz 흔들림. 360도 장치 전체가 기울므로 앞·뒤는 반대 피치,
  좌·우는 롤이 생긴다. 추론 RGB와 파노라마에도 실제 반영한다.
- 로봇 뒤에서 따라보기, 로봇 화면 접기, 주행 기록 저장.
- 마주 오는 승용차 장면과 접촉·도로 이탈 기록.

흔들림은 기본 켜짐이다. 가상 시간 0.1초마다 추론하며, 끄면 기존 0.5초 간격이다.
실제 재생 속도는 네 방향 추론 시간에 따라 달라진다. 카메라 각도는
주행 모델 입력이 아니라 렌더링·센서 보정·시험 기록에 사용한다.

## 회피 제어 제거와 처리 비용 절감

전면·360도 **브라우저 시뮬레이션**에서 객체 검출 후 횡이동하는 별도 제어기를
제거했다. 객체 검출 모델을 로드하지 않으며, 매 프레임 720개 가상 LiDAR 광선을
발사하지 않는다. `--road-objects`, `--object-weights`, `--object-min-score` 옵션도
제거했다. 이전 실행 명령에 이 옵션이 있으면 빼야 한다.

학습된 차도·행동 모델이 속도·회전·정지를 결정한다. 차량 장면은 계속 사용할 수
있지만 별도 차량/사람 회피를 수행하지 않는다. 지도 기반 접촉·이탈 기록과 완주 후
종료는 유지한다. 이 변경의 대상은 로컬 시뮬레이션이며 ROS/Jetson 제어기는 별개다.

- 480×270 RGB 네 장을 960×540 PNG 한 장에 무손실로 묶는다. 위쪽은 앞·왼쪽,
  아래쪽은 뒤·오른쪽이다. GPU 읽기와 PNG 인코딩/디코딩을 각각 한 번 수행한다.
  서버는 원래 크기의 네 장으로 나눠 기존 배치 추론에 전달한다.
- 입력 해상도, 모델 가중치, 회로 적분 횟수, 흔들림 켰을 때 0.1초 추론 간격은 유지한다.
- 영상/마스크 미리보기는 실제 시간 기준 최대 5 Hz로 갱신한다. **로봇 화면 접기**를
  누르면 미리보기용 파노라마 렌더링·마스크 생성·전송을 생략하지만 입력 영상 추론은 계속한다.
- 미리보기 PNG 디코딩 완료를 기다리지 않고 모델 명령을 적용한다.
- 파노라마 투영 좌표와 픽셀 버퍼를 재사용하고, 중앙 오차 RMS는 누적으로 계산한다.
- 매 응답에서 학습 이력 전체를 전송하지 않는다. 전체 모델 정보는 `/api/status`와
  기록 내보내기에 남긴다. 패널에는 프레임 처리 시간, 차도/행동 추론 시간을 표시한다.

기존 서버를 종료한 뒤 새로 실행하고 브라우저를 새로고침한다. **가능한 빠르게**를
선택하고 로봇 화면을 접으면 표시 작업 부담이 줄어든다. 배속은 처리 가능한 범위의
상한이며, 추론 시간보다 빠르게 가상 시간을 건너뛰지 않는다.

이번 변경 후 자동 주행·성능 측정은 사용자 요청에 따라 실행하지 않았다.
아래 완주 기록은 최적화 이전 기록이다.

차량 그래픽은 [Khronos ToyCar](https://github.com/KhronosGroup/glTF-Sample-Assets/tree/main/Models/ToyCar)
CC0 모델을 사용하고 크기를 맞췄다. 원본·라이선스·SHA-256은 `environments/dangjin/assets/`에 있다.

## 전면 카메라 실행

전면 모드에도 같은 회피 제거와 미리보기 최적화가 적용된다. 전면 영상은 기존처럼
한 장으로 전송한다.

```bash
MPLBACKEND=Agg .cache/connectome-venv/bin/python tools/dangjin_sim_server.py \
  --front-model /Volumes/MY_HDD/connectum_models/front-road-v2-road-only-20261027 \
  --device cpu --road-device mps --port 8773
```

## 학습 저장 위치

- 로컬 실행용: `.cache/dangjin-360-20261028/road/`, `policy/`
- 볼륨 보관용: `/Volumes/MY_HDD/connectum_models/surround-road-20261028/`
- 신규 표본: `.cache/dangjin-360-20261028/dangjin-connectome-training/`, `dangjin-policy-training/`

차도 출력층은 최신 합성 RGB/정답 512장, 행동 출력층은 768개 위치·방향 변형의
4방향 RGB에서 학습했다. ±5° 범위 기울기를 포함하고, 행동 학습에는 검출 구멍과
전체 누락을 추가했다. 같은 위치 그룹의 변형은 학습/검증에 나눠 넣지 않았다.
Flyvis 시각 회로 가중치는 고정이다. 원래 전면 모델이나 이전 360도 모델은 보존했다.

차도 합성 검증 IoU는 0.9866, 행동 검증의 속도 MAE는 0.0038 m/s,
회전 MAE는 0.0190 rad/s다. 이는 같은 합성 부지의 보류 표본에 대한 수치이며
실제 우사 영상이나 새로운 부지에서의 성능을 의미하지 않는다.
학습 이력과 데이터 해시는 각 모델 폴더의 JSON에 기록했다.

볼륨의 모델로 실행하려면:

```bash
DANGJIN_360_MODEL_DIR=/Volumes/MY_HDD/connectum_models/surround-road-20261028 \
  ./tools/run_dangjin_360.sh
```

볼륨 폴더는 가중치·설정·소스 스냅샷이다. 실행에는 이 저장소와 Flyvis Python 환경,
Flyvis 인코더가 필요하다. 실행기는 모델 폴더 안의
`flyvis_encoder.pt`가 있으면 그것을 쓰고, 없으면 기존 `.cache/flyvis_encoder.pt`를 쓴다.

## 재학습

별도 수집 서버를 `--control baseline --training-root .cache/dangjin-360-20261028`로
시작한 뒤 인식 모델 개발의 두 수집 버튼으로 최신 영상을 저장한다. 다음 순서로 학습한다.

```bash
MPLBACKEND=Agg .cache/connectome-venv/bin/python tools/train_connectome_road.py \
  --data .cache/dangjin-360-20261028/dangjin-connectome-training \
  --output .cache/dangjin-360-20261028/road --epochs 60
MPLBACKEND=Agg .cache/connectome-venv/bin/python tools/train_connectome_policy.py \
  --data .cache/dangjin-360-20261028/dangjin-policy-training \
  --road-model .cache/dangjin-360-20261028/road \
  --output .cache/dangjin-360-20261028/policy --epochs 120
```

지도 버전 문자열만 바꿔 기존 모델을 통과시키지 않는다. 행동 학습기는 모든 표본의
지도 버전을 검사하고, 실행 시에는 차도 모델 해시와 학습 지도 버전을 검사한다.

## 남겨둔 시험 기록

추가 검증은 사용자의 요청으로 중단했다. 완료된 랜덤 누락 완주 기록은 카메라 흔들림·객체 횡이동을 끈 조건이며, 흔들림과 움직이는 차량 횡이동 시험은 일부 구간만 실행했다. 전체 교행 성공을 확인한 결과로 해석하지 않는다. 수집된 기록과 범위 설명은 볼륨 모델 폴더의 `validation/`에 보존했다.
