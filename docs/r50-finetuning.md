# R50 부분 라벨 파인튜닝과 Jetson 적용

`tools/train_r50_partial.py`는 v6 데이터셋으로 기존 MaskFormer ResNet-50의
65개 Mapillary 클래스를 유지하며 학습합니다. `255(ignore)`와 유효 마스크를
실제 손실에 적용하고, 검수 전 후보와 test는 학습·모델 선택에 사용하지 않습니다.
원본 데이터셋과 기존 R50 체크포인트는 덮어쓰지 않습니다.

명령은 별도 표시가 없으면 저장소 루트에서 실행합니다. 학습은 CUDA가 있는
Linux PC, 주행은 Jetson에서 실행하는 구성을 기준으로 합니다.

## 1. 학습 환경

Python 3.12와 NVIDIA 드라이버가 설치된 학습 PC에서 실행합니다.
아래는 [PyTorch 공식 버전 조합](https://pytorch.org/get-started/previous-versions/)의
CUDA 12.8 예입니다. 드라이버에 맞는 CUDA wheel을 선택하세요.
Jetson에서는 이 PC용 wheel로 컨테이너의 PyTorch를 교체하지 않습니다.

```bash
python3 -m venv .cache/r50-train-venv
source .cache/r50-train-venv/bin/activate
python -m pip install --upgrade pip
python -m pip install torch==2.8.0 torchvision==0.23.0 --index-url https://download.pytorch.org/whl/cu128
python -m pip install -r requirements-r50-training.txt
python -c "import torch; print(torch.__version__, torch.cuda.is_available())"
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
```

원본 고정 revision은 `pytorch_model.bin` 형식입니다. 최신 Transformers의
안전한 로딩 요구사항을 충족하는 PyTorch를 사용해야 하며, 위 조합은 이를
충족합니다. 최초 학습 시 Hugging Face에서 원본 가중치를 다운로드합니다.
이미 캐시가 준비된 경우 `--local-files-only`로 네트워크 접근 없이 시작할 수 있습니다.

## 2. 데이터와 학습 실행

기본 데이터셋은 `rosbag-results/dataset/r50-surface-v6-20260929`,
기본 출력은 `rosbag-results/models/r50-surface-v6`입니다.

먼저 모델 다운로드 없이 이미지·라벨·유효 마스크 정렬과 파일 해시를 확인합니다.
전체 train/val 파일을 읽기 때문에 저장장치 속도에 따라 몇 분 걸릴 수 있습니다.

```bash
python tools/train_r50_partial.py --dry-run
```

본 학습의 시작 예:

```bash
python tools/train_r50_partial.py \
  --dataset rosbag-results/dataset/r50-surface-v6-20260929 \
  --output-dir rosbag-results/models/r50-surface-v6 \
  --device cuda \
  --max-steps 1000 \
  --eval-every 100 \
  --batch-size 2 \
  --gradient-accumulation 4 \
  --lr 1e-5 \
  --seed 42
```

`max-steps`는 optimizer 갱신 횟수이며 epoch 수가 아닙니다. 메모리가 부족하면
batch size를 줄이고 gradient accumulation을 늘립니다. 기본값은 backbone을
고정하고 decoder와 예측 head를 학습합니다. `--unfreeze-last-stage`를 지정하면
ResNet의 마지막 stage도 `--backbone-lr 1e-6`으로 학습합니다.
FP16 초기 스케일 조정 중 `amp_overflow_skipped`가 나올 수 있습니다. 이때는
가중치를 갱신하지 않고 스케일을 낮춰 계속하며, 성공한 갱신만 step으로 셉니다.

학습 샘플은 기존 도로 전파 라벨과 새 시각 주석을 균형 있게 뽑습니다.
밝기·감마·부드러운 그림자 증강을 적용하며, `--no-augmentation`으로 끌 수 있습니다.
픽셀별 클래스 점수를 정규화한 부분 감독 NLL을 사용하고 기본 MaskFormer
Hungarian loss에는 부분 라벨을 전달하지 않습니다. 미주석 영역을 배경으로 채우지 않습니다.

원본 RGB는 운영과 같은 processor로 처리합니다. 목표 크기는 360×640이며
실제 입력은 processor의 `size_divisor=32`에 따라 384×640이 될 수 있습니다.
운영처럼 최종 점수를 360×640으로 맞추고, 부분 라벨·valid mask는 원본에서
nearest-neighbor로 같은 좌표계에 맞춥니다. 실제 tensor shape도 실행 기록에 남깁니다.

## 3. 저장본과 재개

```text
rosbag-results/models/r50-surface-v6/
├── best/
│   ├── model.safetensors
│   ├── config.json
│   ├── preprocessor_config.json
│   └── checkpoint-manifest.json
├── checkpoints/last/
├── baseline-evaluation.json
├── training.jsonl
└── run.json
```

`best/`는 검증 손실 기준으로 선정한 추론용 모델입니다.
`checkpoint-manifest.json`은 가중치·모델 설정·processor의 SHA-256,
데이터셋 출처와 학습·평가 기록을 묶습니다. 해시 검사는 모델의 주행 품질 인증이 아닙니다.
학습 재개에는 `best/` 대신 optimizer·난수 상태가 포함된 `checkpoints/last`를 사용합니다.
`training.jsonl`에는 학습 진행과 검증 지표가, `run.json`에는 실행 설정이 기록됩니다.
재개 가능한 저장본은 `--eval-every` 주기와 마지막 step에서 갱신됩니다.
새 추론 파일은 임시 폴더에서 저장·검증을 마친 뒤 `best/`에 반영합니다.
교체 중 프로세스가 강제 종료되어 `best/`가 없으면 `.best-previous/`에
이전 저장본이 남을 수 있습니다. manifest 검사 후 `best/`로 복구할 수 있습니다.

```bash
python tools/train_r50_partial.py \
  --dataset rosbag-results/dataset/r50-surface-v6-20260929 \
  --output-dir rosbag-results/models/r50-surface-v6 \
  --resume rosbag-results/models/r50-surface-v6/checkpoints/last \
  --device cuda --max-steps 1500
```

재개할 때는 원래 학습의 배치·증강·학습률 등 설정을 유지합니다. 다른 설정으로
별도 실험하려면 새 출력 폴더를 사용합니다. 완료된 본 학습 폴더에 검사용 실행을 섞지 마세요.

## 4. 추론 확인

학습 PC에서 먼저 저장본의 파일과 형식을 검사합니다.

```bash
python tools/r50_checkpoint.py --checkpoint rosbag-results/models/r50-surface-v6/best
```

실제 MP4에 운영 후처리를 적용한 오버레이를 생성할 수 있습니다.

```bash
python tools/segment_mapillary_full_video.py \
  --profile r50-finetuned-fp16-640x360 \
  --model-id "$(realpath rosbag-results/models/r50-surface-v6/best)" \
  --input rosbag-results/dataset/r50-surface-v6-20260929/source/Dangjin-A2_17_manual_20260915T065354Z_a2_front_camera_until_24m39s.mp4 \
  --output-dir rosbag-results/r50-finetuned-preview \
  --device cuda --max-frames 200
```

처음 200프레임은 로딩 확인용입니다. 그림자 구간과 인도 구간을 포함해 평가하려면
`--max-frames 0`으로 전체를 처리하거나 원본 rosbag의 `mcap` 재생 기능을 사용하세요.
동일 영상에 기존 `r50-fp16-640x360`을 적용한 결과와 비교합니다.

현재 val에는 Sidewalk 라벨이 없습니다. 기록되는 클래스별 재현율은 라벨이 있는
픽셀에 한정되며 전체 영상 IoU나 비도로 오검출률을 의미하지 않습니다.
인도 일반화와 연석 경계를 평가하려면 별도의 ROI 평가 라벨이 필요합니다.
새 프로파일은 v6 정의에 따라 Manhole을 Road에 포함하므로, 기존 프로파일과의
차이에는 가중치 변경과 이 그룹 규칙 변경이 함께 포함됩니다.

## 5. Jetson으로 복사

`best/`의 파일 네 개를 포함한 폴더 전체를 Jetson 프로젝트의
`models/r50-surface-v6/best/`에 복사합니다. `SWIN_L_ENGINE_DIR=./models`일 때
컨테이너에서는 `/models/r50-surface-v6/best`로 보입니다.

학습 PC와 Jetson이 같은 파일시스템이면 저장소 루트에서:

```bash
mkdir -p models/r50-surface-v6
cp -a rosbag-results/models/r50-surface-v6/best models/r50-surface-v6/
```

다른 컴퓨터이면 같은 목적지 구조로 `rsync` 또는 `scp`를 사용합니다.
추론용 네 파일이 함께 복사되어야 하며 기존 Hugging Face 캐시를 덮어쓰지 않습니다.

Jetson에서 검사하고 다음 명령의 첫 번째 값(64자리 해시)을 복사합니다.

```bash
python3 tools/r50_checkpoint.py --checkpoint models/r50-surface-v6/best
sha256sum models/r50-surface-v6/best/checkpoint-manifest.json
```

Jetson 프로젝트의 `.env`에서 아래 값을 설정합니다. `.env`가 없다면
`.env.example`을 복사한 뒤 현장 설정을 적용합니다.

```dotenv
SWIN_L_PROFILE=r50-finetuned-fp16-640x360
SWIN_L_BACKEND=pytorch
SWIN_L_DEVICE=cuda
SWIN_L_ENGINE_DIR=./models
SWIN_L_MODEL_ID=/models/r50-surface-v6/best
SWIN_L_MODEL_REVISION=
SWIN_L_CHECKPOINT_SHA256=여기에_checkpoint-manifest.json의_64자리_SHA256
SWIN_L_TRT_AUTO_BUILD=false
SWIN_L_ALLOW_BACKEND_FALLBACK=false
```

`SWIN_L_CHECKPOINT_SHA256`은 `model.safetensors` 자체가 아니라
**checkpoint-manifest.json 파일의 해시**입니다. 새 학습 모델로 교체하면
그 manifest의 해시도 갱신합니다. 모델 경로는 컨테이너 경로를 사용합니다.
ROI·마스크 선택·조향·정지 설정은 기존 현장 설정을 유지합니다.

## 6. 컨테이너 적용과 복귀

수정된 코드를 Jetson에도 반영한 뒤, 먼저 제어 없는 디버그 서비스에서 확인합니다.
이미 실행 중인 다른 추론 서비스는 검증 시간 동안 정지해 GPU 경쟁을 피합니다.

```bash
docker compose stop actual-activate
docker compose up -d --build --force-recreate debugging-swin-l
docker compose logs -f debugging-swin-l
```

학습 모델·경로 출력을 확인한 뒤 실제 주행 서비스로 전환합니다.

```bash
docker compose stop debugging-swin-l
docker compose up -d --build --force-recreate actual-activate
docker compose logs -f actual-activate
```

`actual-activate`는 manifest 해시가 지정된 로컬 모델만 허용하고, 가중치·설정의
파일 해시와 R50/65클래스/전처리 계약을 검사합니다. 누락되거나 다른 파일이면
시작에 실패하며 다른 모델로 자동 전환하지 않습니다. 모델 로딩 시 누락된
파라미터를 임의 초기화하는 것도 거부합니다. 로그와 메트릭에서 새 profile,
model_id와 checkpoint SHA를 확인하세요.

기존 R50으로 복귀할 때는 다음 네 항목을 바꾸고 서비스를 재생성합니다.

```dotenv
SWIN_L_PROFILE=r50-fp16-640x360
SWIN_L_MODEL_ID=
SWIN_L_MODEL_REVISION=
SWIN_L_CHECKPOINT_SHA256=
```

`SWIN_L_BACKEND=pytorch`는 유지합니다. 모델 학습·저장 자체가 로봇을 실행하거나
현재 `.env`를 변경하지는 않습니다.

## 구현 검증 기록 — 2026-09-29

- Python 3.12 / torch 2.8.0+cu128 / Transformers 5.16.1 / RTX A6000에서 확인했습니다.
- v6의 train 627장·val 145장 전체 파일 해시, 라벨/valid/origin 정렬 검사가 통과했습니다.
- 실제 고정 R50 가중치로 train 4장·val 2장을 제한 사용해 optimizer 2회 갱신하고,
  `checkpoints/last`에서 3번째 갱신까지 재개했습니다. 샘플러 진행 횟수도 이어졌습니다.
- 기본 배치 2·누적 4·증강 설정에서도 초기 AMP overflow 후 스케일이 조정되어
  실제 optimizer 갱신과 저장이 완료되는 것을 확인했습니다. 이 저장본에서
  2번째 갱신까지 재개하고 기존 `best/`를 새 모델로 교체하는 것도 통과했습니다.
- 저장된 safetensors를 주행용 `BestSoFarSegmenter`의 새 프로파일로 읽고,
  원본 MP4 2프레임에서 CUDA FP16 추론 및 360×640 결과를 확인했습니다.
  실제 저장본을 사용한 task-drive 사전 검증도 통과했습니다.
- 전체 테스트 결과는 658 passed / 16 skipped / 4 failed였습니다.
  실패 4개는 변경 전에도 존재한 yaw 부호 기대값 불일치로,
  `test_apriltag_task_stop_ros.py`의 회전 테스트 2개와
  `test_unitree_sport_api.py`의 부호·요청 변환 테스트 2개입니다.
- Compose 설정 해석과 entrypoint 셸 구문 검사를 통과했습니다.

검사용 모델은 `rosbag-results/models/r50-surface-v6-*smoke*-20260929/`에 저장했습니다.
이 짧은 학습 모델들은 동작 확인용이며 본 학습 결과가 아닙니다. `.env` 변경,
Jetson 컨테이너 빌드/기동, 실제 차량 주행은 실행하지 않았습니다.
