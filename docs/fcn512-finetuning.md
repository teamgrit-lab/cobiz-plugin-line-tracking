# 당진 360° 차도 검출기 파인튜닝

외장 볼륨을 연결하고 저장소 루트에서 실행한다. uv가 Python 3.12와 스크립트에 명시된 패키지를 준비한다. 첫 실행에서 설치 캐시에 없는 패키지는 인터넷에서 내려받는다.

    ./tools/finetune_fcn512.py

볼륨에 복사된 코드로도 어느 디렉터리에서나 실행할 수 있다.

    uv run --script /Volumes/MY_HDD/connectum_models/fcn512-training/finetune_fcn512.py

기본 데이터는 볼륨의 connectum_data/real360/fcn512-road-dataset-v1, 초기 가중치는 connectum_models/fcn512-training/base/fcn_resnet18.onnx에 둔다. 기본 모델의 SHA-256과 복원한 PyTorch 모델의 출력을 검사한 뒤 학습한다. 다른 경로에서는 --volume, --dataset, --base-onnx, --output-root로 지정할 수 있다. --help에서 전체 옵션을 확인한다.

- 기본값은 8에폭, 배치 4다. CUDA, Apple MPS, CPU 순서로 사용 가능한 장치를 선택한다. 처음 두 에폭은 분류 헤드, 이후에는 ResNet의 마지막 블록도 헤드 학습률의 1/10로 학습한다.
- 각 분할의 검수용 부분 라벨 목록과 이진 라벨 목록을 이미지 ID로 결합한다. 의미 마스크에서 1은 로봇, 3은 차도이고 이진 마스크에서 1은 차도다. 두 마스크 모두 255인 픽셀은 각 손실에서 제외한다. 검증 녹화와 학습 녹화의 중복, ID482의 25분 이후 샘플은 입력 단계에서 차단한다.
- 각 에폭에 원래 FCN 영상 비교와 동일한 판정 방식으로 검증한다. 검증 이진 마스크의 차도 IoU가 가장 높은 학습 에폭을 보관하고 동점이면 부분 의미 라벨의 차도 IoU를 사용한다. 기준 모델의 성능, 라벨이 있는 풀·본체 픽셀에서의 차도 오검출률도 metrics.json에 기록한다. 검증 픽셀이 없는 클래스는 null이다.

결과는 매번 connectum_models/fcn512-road-runs/에 새 디렉터리로 저장된다. best.pt는 선택된 에폭의 PyTorch 가중치이고 fcn_resnet18.onnx는 같은 가중치의 고정 입력 모델이다. RGB ImageNet 정규화, 입력 [1,3,256,512], 출력 [1,21,8,16], 클래스 목록은 config.json과 classes.txt에서 확인할 수 있다. 내보낸 ONNX와 다시 읽은 best.pt를 실제 입력 네 장으로 대조하면 complete.json에 파일 해시와 완료 시각을 기록한다.

Jetson Road 작업의 모델 설치, 검출 미리보기와 실행 방법은 [360° FCN Road 적용](fcn360-road.md)에 있다.
