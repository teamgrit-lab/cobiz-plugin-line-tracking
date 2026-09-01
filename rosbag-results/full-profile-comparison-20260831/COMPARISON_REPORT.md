# R50 현재 설정·후보 005·Swin-L 전체 영상 비교 보고서

- 완료 시각: 2026-09-01 00:25 KST
- 입력: `/Users/kangminwoo/Documents/GitHub/cobiz-plugin-line-tracking/rosbag-results/full`
- 대상: 8개 영상, 출력 기준 54,992프레임
- 결론: **현재 유지 설정 권장**

## 비교 설정

| 설정 | 모델 | 입력/정밀도 | temporal alpha | hysteresis margin |
| --- | --- | --- | ---: | ---: |
| 현재 유지 설정 | MaskFormer R50 | 640×360, FP16 | 0.62 | 0.07 |
| 후보 005 | MaskFormer R50 | 640×360, FP16 | 0.50 | 0.30 |
| 기준 | Swin-L best-so-far | 384×384, FP32 | 0.62 | 0.07 |

세 `rssp` 원본은 손상된 타임베이스를 피하기 위해 이전에 검증된 복구 입력을 사용했다. 마지막 `rssp` 입력은 OpenCV 디코더가 꼬리 2프레임을 읽지 못해 두 R50 출력과 비교 영상에서 마지막 정상 프레임을 2회 복제했다. 원본 파일은 변경하지 않았다.

## 전체 영상 처리 결과

두 R50 설정 모두 8개 전체 영상을 생성했고, 모든 최종 출력과 비교 영상을 전체 디코드 검증했다.

| 지표 | 현재 R50 | 후보 005 | 해석 |
| --- | ---: | ---: | --- |
| 8개 집계 처리 FPS | 12.23 | 12.52 | 현재의 `062352`는 기존 측정값을 재사용해 실행 시점 차이가 있음 |
| 같은 실행 구간 7개 처리 FPS | 12.55 | 12.46 | 후보가 0.67% 느림 |
| Swin 비교 표본 처리 FPS | 14.23 | 14.22 | 사실상 동일 |
| selected-label change | 0.01602 | 0.00359 | 후보가 77.6% 감소 |
| Road adjacent IoU | 0.8324 | 0.9645 | 후보가 더 안정적 |
| Sidewalk adjacent IoU | 0.9024 | 0.9794 | 후보가 더 안정적 |

후보 005가 짧은 대표 구간에서 기록했던 14.46 FPS는 전체 영상에서는 재현되지 않았다. 8개 전체 영상 집계는 12.52 FPS였고, 동일한 Swin 비교 표본에서는 14.22 FPS였다. 그래도 두 값 모두 10 FPS 기준을 넘는다.

## Swin-L 유사도 검증

8개 영상마다 시작·중간·종료 구간을 선택해 각 R50 설정을 Swin-L과 직접 재추론했다. 설정별 24개 클립, 총 1,197개 디코드 프레임을 비교했다. Ground truth가 없으므로 아래 수치는 실제 정확도가 아니라 Swin-L best-so-far와의 유사도이다.

| 지표 | 현재 R50 vs Swin-L | 후보 005 vs Swin-L | 우세 |
| --- | ---: | ---: | --- |
| selected-label agreement | 0.9132 | 0.9099 | 현재 R50 |
| Road inter-profile IoU | 0.4530 | 0.4359 | 현재 R50 |
| Sidewalk inter-profile IoU | 0.6661 | 0.6618 | 현재 R50 |
| Road adjacent IoU | 0.8811 | 0.9663 | 후보 005 |
| Sidewalk adjacent IoU | 0.9270 | 0.9764 | 후보 005 |

후보 005는 프레임 간 안정성이 크게 좋아졌지만, Swin-L과의 agreement와 Road/Sidewalk IoU는 모두 소폭 낮아졌다. 높은 hysteresis가 한 번 선택된 클래스를 오래 유지하면서 흔들림은 줄였지만 잘못 선택된 Road/Sidewalk도 더 오래 남긴 것으로 해석된다.

## 시각 검증

- 후보 005는 경계의 깜빡임과 작은 마스크 조각의 출현·소멸을 확실히 줄였다.
- 일부 보행로 장면에서 후보 005가 Swin-L에 없는 Road(초록) 조각을 더 오래 유지했다.
- 현재 설정은 후보보다 프레임 간 변화가 크지만, 전체 표본 평균에서는 Swin-L 의미 분류에 더 가깝다.
- 따라서 시간 안정성만 보면 후보 005가 우수하지만, 품질과 Swin-L 유사도를 함께 고려하면 현재 설정이 더 안전하다.

## 비교 영상 패널 순서

- 4패널: `원본 → 현재 R50 → 후보 005 → Swin-L`
- 3패널: `원본 → 현재 R50 → 후보 005`

Swin-L 전체 결과가 기존에 존재하는 4개 영상은 4패널 전체 길이 비교로 만들었다. 나머지 4개는 3패널 전체 길이 비교를 만들고, Swin-L 비교는 시작·중간·종료 샘플 영상과 JSON으로 보완했다.

## 주요 산출물

- 현재 R50 전체 결과: `/Users/kangminwoo/Documents/GitHub/cobiz-plugin-line-tracking/rosbag-results/full-profile-comparison-20260831/r50-current`
- 후보 005 전체 결과: `/Users/kangminwoo/Documents/GitHub/cobiz-plugin-line-tracking/rosbag-results/full-profile-comparison-20260831/r50-candidate-005`
- Swin-L 전체 참조: `/Users/kangminwoo/Documents/GitHub/cobiz-plugin-line-tracking/rosbag-results/full-profile-comparison-20260831/swin-l-reference`
- 전체 비교 영상: `/Users/kangminwoo/Documents/GitHub/cobiz-plugin-line-tracking/rosbag-results/full-profile-comparison-20260831/comparison`
- Swin-L 표본 비교: `/Users/kangminwoo/Documents/GitHub/cobiz-plugin-line-tracking/rosbag-results/full-profile-comparison-20260831/sampled-validation`
- 수치 요약: `/Users/kangminwoo/Documents/GitHub/cobiz-plugin-line-tracking/rosbag-results/full-profile-comparison-20260831/comparison-summary.json`
- 전체 비교 contact sheet: `/Users/kangminwoo/Documents/GitHub/cobiz-plugin-line-tracking/rosbag-results/full-profile-comparison-20260831/comparison/full-comparison-contact-sheet.jpg`
- Swin 표본 contact sheet: `/Users/kangminwoo/Documents/GitHub/cobiz-plugin-line-tracking/rosbag-results/full-profile-comparison-20260831/sampled-validation/sampled-swin-comparison-contact-sheet.jpg`

## 권장 사항

현재 유지 설정(alpha 0.62, hysteresis 0.07)을 기본값으로 유지하는 것이 좋다. 후보 005는 후속 실험에서 그대로 채택하기보다, 클래스별 hysteresis를 분리하거나 Road에 더 낮은 hold 조건을 적용해 의미 오류가 장시간 고착되지 않도록 조정하는 편이 타당하다.
