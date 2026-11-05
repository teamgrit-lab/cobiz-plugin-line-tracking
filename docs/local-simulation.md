# Road 로컬 시뮬레이션 관리

브라우저 환경 `environments/dangjin/`, 시뮬레이션 전용 서버·수집기·실행기,
시험 문서와 `test/local_simulation/`은 로컬 전용이다. `.gitignore`와
`.dockerignore`에 같은 범위를 명시했다. 이미 추적되던 파일은 로컬에 보존하고
Git 인덱스에서 제외했다. Git 이력과 실제 센서 샘플 `datasets/`는 유지한다.

새 시뮬레이션 도구와 실험 파일은 제외되는 `simulation/` 아래에 둔다.

새 체크아웃에는 시뮬레이터가 없어도 R50 Sidewalk, front Connectome,
Connectome360, FCN Road의 import·설정·실기 이미지 빌드 경로가 동작한다.
모델 가중치와 로봇용 베이스 이미지는 기존과 같이 별도 준비한다.

추적되는 공용 파일은 다음과 같다.

- `config/camera360-profile.json`: 원본과 내용·프로필 해시가 같은 카메라 입력 규약
- `tools/panorama_mask.py`, `surround_camera.py`, `surround_codec.py`: 투영·입력 검증
- `tools/fcn_replay.py`: 기록 영상에도 사용하는 FCN 검출·제어 재생
- `tools/sensor_faults.py`: 학습과 재생에서 공유하는 시드 기반 검출 손상
- `tools/camera_trajectory.py`, `training_geometry.py`: 오프라인 학습용 명령 제안

모델 패키지의 `simulation-training.json`, schema, 학습 출처와 가중치 해시는
기존 모델을 검증하는 계약이므로 유지한다. 시뮬레이터를 제거했다고 학습 출처를
실제 주행 검증으로 바꾸지 않는다.

전면 모델의 실데이터 전용 학습에는 시뮬레이터가 필요하지 않다. 기존 합성 표본을
섞으려면 `train_front_connectome.py --simulation-dir environments/dangjin`을
명시한다. 합성 정책 교사가 필요한 `train_connectome360.py`에도
`--simulation-dir`과 기존 합성 데이터 경로를 지정한다. 누락된 자산은 설명과 함께
오류를 내며, 추론 서비스는 이 학습 경로를 import하지 않는다.

기존 로컬 시뮬레이터의 파일 경로는 보존했다. 기본 pytest 수집은 브라우저 시험을
제외하며, 필요하면 `test/local_simulation/`의 시험 파일을 명시해 실행한다.
원래 구현을 별도로 재현하려면 이 변경 전 커밋을 독립 worktree로 확인할 수 있다.

```bash
git worktree add --detach .cache/dangjin-simulator 33dd30d
```

이 worktree는 당시 코드·환경·모델을 재현하는 용도다. 현재 실기 코드에 과거 파일을
덮어쓰지 않는다. 새 로컬 파일을 만들 때 `git check-ignore -v <경로>`로 규칙을
확인할 수 있다. 제외 대상에 `git add -f`를 사용하면 다시 추적될 수 있다.
