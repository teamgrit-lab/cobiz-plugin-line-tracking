# [202608] 당진낙농축협 프로젝트 사후분석

- Document ID: 1mRME7Q3F8xQq_NjXVHEOPi5K5niuPv1Bo9t-TgSa39k
- Revision ID: ANLCKQnco_Cz81JTR4tQbkGv9PsKn3XvK1EnKqFxD2rSFOChFYXyx7Kd78N7RFOILgFpg4G0Ayk8d7e-C8aQ7lVkZ210Pc8tPH68bLJJIao
- Selected tab: all
- Protected controls: 210
- Opaque controls: 210
- Authoritative dropdowns: 0

Protected-control annotations are preservation instructions. Do not insert their displayed placeholder text to recreate a native control.

## 당진낙농축협 프로젝트 포스트모템 - Overview (t.0)

[P00001 | 1:30 | HEADING_1]
당진낙농축협 프로젝트 포스트모템 - Overview

[P00002 | 30:50 | NORMAL_TEXT | LIST id=kix.753ve4328ni8 level=0]
Created: 2026.08.21

[P00003 | 50:72 | NORMAL_TEXT | LIST id=kix.753ve4328ni8 level=0]
Author: SeungJong Kim

[P00004 | 72:75 | HEADING_2]
요약

[P00005 | 75:163 | NORMAL_TEXT | LIST id=kix.753ve4328ni8 level=0]
자율주행 워크플로우 자체는 동작했으나, 이를 안정적으로 반복하기 위한 전제인 "지도 위에서 로봇 위치를 1 m 이내로 유지"가 당진 환경에서 보장되지 않았음

[P00006 | 163:271 | NORMAL_TEXT | LIST id=kix.753ve4328ni8 level=0]
첫 번째 문제는 지도 자체의 정확성임. SLAM으로 만든 지도는 내부에 기울기와 변형을 가진 채 생성되며, 당진처럼 넓은 부지에서는 이 작은 오차가 수십 cm에서 m 단위의 위치 오차로 증폭됨

[P00007 | 271:445 | NORMAL_TEXT | LIST id=kix.753ve4328ni8 level=0]
두 번째 문제는 localization과 지도의 종속 관계임. AprilTag + scan-to-scan 보정과 GPS(RTK) 보정 모두 정확한 지도를 전제로 동작하는데, 지도 내부 오차가 남아 있으면 RTK가 아무리 정확해도 그 정확도를 지도 전체로 가져올 수 없어 보정 수단들이 설계된 성능을 내지 못했음

[P00008 | 445:590 | NORMAL_TEXT | LIST id=kix.753ve4328ni8 level=0]
세 번째 문제는 SLAM 자체의 드리프트임. 축사 복도처럼 한 방향으로 뻗은 환경에서는 LiDAR 정합이 진행 방향 오차를 감지하지 못해 오차가 조용히 누적되며, 이것이 알고리즘의 한계인지 센서(내장 IMU, JT128)의 한계인지는 판정하지 못한 채 종료됨

[P00009 | 590:810 | NORMAL_TEXT | LIST id=kix.753ve4328ni8 level=0]
개선 전략을 다음과 같이 제안함. 고성능 IMU, Livox Mid360, heading GPS 등 센서 조합을 A2에 장착하고 성남자율주행테스트베드 등에서 동일 주행 조건의 위치 추정 성능을 수치로 판정하는 벤치마크를 구축한 뒤, 검증된 최적 조합으로 현장에 다시 도전함. 또한 통신 음영과 localization 상실 등 실패 시나리오의 fallback을 CoBiz와 함께 정의하고 검증함

[P00010 | 810:829 | HEADING_2]
소프트웨어 검수는 왜 실패했는가?

[P00011 | 829:891 | NORMAL_TEXT | LIST id=kix.753ve4328ni8 level=0]
당진낙농축협 프로젝트의 핵심 사안인 전체 축사동 로봇 자율주행을 안정적으로 수행할 만한 완성도에 이르지 못했음

[P00012 | 891:956 | NORMAL_TEXT | LIST id=kix.753ve4328ni8 level=0]
더 근본적으로, 이 워크플로우가 성립하려면 다음 두 전제조건이 필요한데 당진 환경에서는 어느 쪽도 보장할 수 없었음

[P00013 | 956:996 | NORMAL_TEXT | LIST id=kix.agt96cnwchsj level=0]
로봇이 3D 지도 상에서 스스로의 위치를 1 m 이내로 지속적으로 추정

[P00014 | 996:1042 | NORMAL_TEXT | LIST id=kix.agt96cnwchsj level=0]
주행 중 발생할 수 있는 여러 실패 시나리오에 대한 최소한의 fallback 계획

[P00015 | 1042:1056 | HEADING_2]
어떤 가설로 접근했는가?

[P00016 | 1056:1245 | NORMAL_TEXT]
시스템의 설계 가설은 prior-map 정합 localization이다. 사전에 정확하게 만든 지도를 기준 좌표계로 두고, 로봇이 주행 중 만드는 local map을 이 지도에 맞춰 자신의 위치를 파악한다. 지도 출처에 제약이 없고, 여러 로봇이 하나의 지도를 공유하면 multi-robot으로 확장하기 쉽다는 것이 이 구조의 장점이다.

[P00017 | 1245:1246 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00018 | 1246:1386 | NORMAL_TEXT]
이 가설이 성립하려면 두 가지가 필요하다. 지도 자체가 정확할 것, 그리고 로봇과 지도 사이의 위치 관계가 주행 내내 정확하게 유지될 것. 지도가 넓을수록 같은 각도 오차가 더 큰 거리 오차로 증폭되므로, 요구 정밀도는 부지 크기에 비례해 올라간다.

[P00019 | 1386:1387 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00020 | 1387:1677 | NORMAL_TEXT]
당진에서의 결과는 다음과 같다. 지도의 상대적인 정확도, 즉 지도 안에서 구조물끼리의 위치 관계는 확보했다. 그러나 RTK 정합을 현장에서 확보했음에도, SLAM 지도에 내부 기울기와 변형이 남아 있으면 기준점 몇 개의 절대 좌표는 그 주변에서만 유효하고 지도 전체로 퍼지지 않는다. 넓은 구간의 최후 보루로 설계한 GPS(RTK) 보정 역시 정확도 상한이 지도 정확도에 묶여 있다. 결국 "검증된 지도"가 다른 모든 것에 앞서는 선결 조건이었고, 절대 정확도의 병목은 측량이 아니라 SLAM 지도의 내부 정합도였다.

[P00021 | 1677:1693 | HEADING_2]
영역별로 무엇이 문제였는가?

[P00022 | 1693:1701 | HEADING_3]
Mapping

[P00023 | 1701:1759 | NORMAL_TEXT | LIST id=kix.ucmpxe6kt3oi level=0]
로봇 LiDAR와 드론 사진측량 등 여러 소스를 조합해 당시 만들 수 있는 가장 정확한 지도를 채택했음

[P00024 | 1759:1861 | NORMAL_TEXT | LIST id=kix.ucmpxe6kt3oi level=0]
그러나 독립 소스 두 개의 불일치가 위치 오차 예산과 같은 수준이라 서로를 검증하는 용도로 쓸 수 없었고, SLAM 지도는 부지를 약간 기울어진 좌표계로 생성해 사후 보정이 필요했음

[P00025 | 1861:1863 | NORMAL_TEXT]
[INLINE_OBJECT kix.6iw4c2n22arh]

[P00026 | 1863:1864 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00027 | 1864:1961 | NORMAL_TEXT]
드론 지도와 로봇 LiDAR 지도를 구역 단위로 비교한 불일치 분포. 두 지도가 일치한다고 볼 수 있는 구역이 소수였다. (상세 수치는 Technical Detail 1장)

[P00028 | 1961:1974 | HEADING_3]
Localization

[P00029 | 1974:2079 | NORMAL_TEXT | LIST id=kix.ucmpxe6kt3oi level=0]
위치 보정 수단들(AprilTag + scan-to-scan 정합, GPS)은 모두 정확한 지도를 전제로 동작함. 지도 내부 오차가 남은 상태에서는 어느 수단도 설계된 성능이 나오지 않음

[P00030 | 2079:2181 | NORMAL_TEXT | LIST id=kix.ucmpxe6kt3oi level=0]
SLAM 자체의 드리프트가 그 아래에 있음. 축사 복도 환경에서는 LiDAR 정합이 진행 방향 오차를 원리적으로 감지하지 못하며, 이 상태가 특정 지점이 아니라 긴 구간 내내 이어짐

[P00031 | 2181:2182 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00032 | 2182:2184 | NORMAL_TEXT]
[INLINE_OBJECT kix.2rfeuak1ga5f]

[P00033 | 2184:2185 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00034 | 2185:2297 | NORMAL_TEXT]
주행 중 수집한 scan마다 "어느 방향의 위치를 잴 수 있는가"를 계산한 결과. 복도형 구간에서는 진행 방향을 잴 수 없는 상태가 길게 이어진다. (상세 분석은 Technical Detail 2장)

[P00035 | 2297:2322 | HEADING_3]
CoBiz: fallback과 네트워크 음영

[P00036 | 2322:2426 | NORMAL_TEXT | LIST id=kix.ucmpxe6kt3oi level=0]
당진의 네트워크 음영은 특정 구역에 들어가면 발생하는 공간적 성질이라, 이벤트 재전송을 늘리는 방식으로는 해결되지 않음. 태스크의 시작이나 종료가 음영 구역에 걸리면 워크플로우가 중단됨

[P00037 | 2426:2511 | NORMAL_TEXT | LIST id=kix.ucmpxe6kt3oi level=0]
단기 완화는 적용했음: 서버 retry 시간 연장으로 주행 중간의 짧은 음영은 커버되었고, 태스크 경계가 음영 구역에 걸리지 않도록 워크플로우를 배치함

[P00038 | 2511:2618 | NORMAL_TEXT | LIST id=kix.ucmpxe6kt3oi level=0]
구조적 해법으로 두 가지 대안이 논의됨: 워크플로우 전체를 로봇에 미리 전달하고 중지 절차를 분리하는 방안, 그리고 장기적으로 워크플로우 수행 책임을 로봇이 소유하고 서버는 조회만 하는 방안

[P00039 | 2618:2633 | HEADING_2]
앞으로 어떻게 할 것인가?

[P00040 | 2633:2731 | NORMAL_TEXT]
이번에 답하지 못한 질문을 검증 가능한 가설로 바꾸고, 가설마다 검증 방법과 판정 기준을 붙인다. 검증 방법과 판정 기준의 상세는 Technical Detail 4장에 있다.

[P00041 | 2731:2732 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00042 | 2732:2925 | NORMAL_TEXT | LIST id=kix.ucmpxe6kt3oi level=0]
H1. 센서 가설: 드리프트의 지배 요인은 센서 HW다. 측량된 테스트베드에서 JT128, Livox Mid360, 고성능 IMU, heading GPS를 동시 기록하고 조합별 오차를 현실에서 측량한 Ground Truth 대비 비교해, 센서 한계인지 알고리즘 한계인지 판정한다. 이후 가장 성능이 좋은 센서 조합을 당진 환경에 다시 적용한다.

[P00043 | 2925:3061 | NORMAL_TEXT | LIST id=kix.ucmpxe6kt3oi level=0]
H2. 환경 가설: 당진의 실패는 우리 테스트 환경에 없던 환경 특성에서 왔다. 언덕, 주행 거리, 복도형 구간, 음영 분포를 변수로 정리해 당진 이상의 험난한 조건을 테스트 환경에 상시 보유하고, 신규 현장을 계약 전에 체크리스트로 판정한다

[P00044 | 3061:3153 | NORMAL_TEXT | LIST id=kix.ucmpxe6kt3oi level=0]
H3. 절대 기준 가설: 벤치마크 부지 한 곳에 측량급 기준을 적용해 지도 내부 오차를 정량화한다. 지도 내부 정확도가 개선되면 당진 현장에 해당 방안을 적용한다.

[P00045 | 3153:3310 | NORMAL_TEXT | LIST id=kix.ucmpxe6kt3oi level=0]
H4. CoBiz Fallback 시험: 워크플로우 소유권을 로봇으로 옮기면 음영의 영향은 "제어 불가"에서 "보고 지연"으로 줄어든다. CoBiz와 함께 prototype을 만들어 음영 재현 환경에서 검증하고, 실패 시나리오별 로봇 동작과 서버 처리를 한 벌의 계약으로 정의한다

[P00046 | 3310:3311 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## 당진낙농축협 프로젝트 포스트모템 - Technical Detail (t.pr6f9xybkfzg)

[P00047 | 1:38 | HEADING_1]
당진낙농축협 프로젝트 포스트모템 - Technical Detail

[P00048 | 38:58 | NORMAL_TEXT | LIST id=kix.l0j20dls9hqm level=0]
Created: 2026.08.21

[P00049 | 58:80 | NORMAL_TEXT | LIST id=kix.l0j20dls9hqm level=0]
Author: SeungJong Kim

[P00050 | 80:97 | HEADING_2]
0. 설계 가설의 상세와 결과

[P00051 | 97:264 | NORMAL_TEXT | LIST id=kix.l0j20dls9hqm level=0]
가설: prior-map 정합 localization. 성립 전제는 (i) prior-map 자체의 정확성, (ii) prior-map과 robot local map 사이 변환의 지속 정확성. 같은 각도/스케일 오차도 지도가 넓을수록 큰 거리 오차로 증폭되므로 요구 정밀도는 지도 크기에 비례한다.

[P00052 | 264:400 | NORMAL_TEXT | LIST id=kix.l0j20dls9hqm level=0]
전제 i의 확보 계획: scanner SLAM의 자체 추정 위치를 GNSS(RTK)로 보정하고, 국토지리정보원 정사영상 overlay, 사전에 파악한 현장 landmark와 3D 지도의 차이 측정을 교차해 배포 전에 지도 정확도를 끌어올린다.

[P00053 | 400:503 | NORMAL_TEXT | LIST id=kix.l0j20dls9hqm level=0]
전제 ii의 확보 계획: AprilTag + scan-to-scan(ICP) 정합이 기본이고, 이 조합에 내재한 오차가 문제되는 넓은 지도에서는 GNSS(RTK)까지 보정에 동원한다.

[P00054 | 503:733 | NORMAL_TEXT | LIST id=kix.l0j20dls9hqm level=0]
결과, 전제 i: 상대 기하는 확보했다(이종 LiDAR 간 0.138 m RMSE). 그러나 절대 기준을 지도에 이식하는 단계가 설계상 막혔다. RTK 정합은 현장에서 확보했지만, SLAM 지도에 내부 오차(frame 기울기 2.0~2.3도, 두 계보가 360 m baseline을 0.82 m 다르게 보는 변형)가 남아 있으면 기준점 몇 개의 절대 좌표는 그 주변에서만 유효하고 지도 전체로 확장되지 않는다.

[P00055 | 733:835 | NORMAL_TEXT | LIST id=kix.l0j20dls9hqm level=0]
결과, 전제 ii: SLAM frontend는 축사 복도 환경에서 진행 방향 registration이 구속되지 않아(degenerate) drift를 스스로 감지하지 못한다(2장).

[P00056 | 835:999 | NORMAL_TEXT | LIST id=kix.l0j20dls9hqm level=0]
종속 구조: 최후 보루인 GNSS(RTK) 보정은 정확도 상한이 지도 georeference 정확도에 묶여 있다. 전제 i의 절대 기준 부재가 전제 ii의 fallback까지 함께 무력화했으므로, "georeference가 검증된 prior-map"이 다른 모든 것에 앞서는 선결 조건이었다.

[P00057 | 999:1010 | HEADING_2]
1. Mapping

[P00058 | 1010:1028 | NORMAL_TEXT]
당진 지도는 세 소스에서 왔다.

[P00059 | 1028:1029 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00060 | 1032:1035 | NORMAL_TEXT | TABLE row=0 col=0]
소스

[P00061 | 1036:1039 | NORMAL_TEXT | TABLE row=0 col=1]
강점

[P00062 | 1040:1043 | NORMAL_TEXT | TABLE row=0 col=2]
약점

[P00063 | 1045:1077 | NORMAL_TEXT | TABLE row=1 col=0]
로봇 LiDAR 세션 14개 (2026-08-08~12)

[P00064 | 1078:1129 | NORMAL_TEXT | TABLE row=1 col=1]
상대 기하 최상. 이종 LiDAR 상호 재현 0.138 m RMSE, 세션 간 잔차 mm급

[P00065 | 1130:1156 | NORMAL_TEXT | TABLE row=1 col=2]
GPS 전 구간 NO_FIX. 절대 기준 없음

[P00066 | 1158:1179 | NORMAL_TEXT | TABLE row=2 col=0]
드론 사진측량 (2026-06-30)

[P00067 | 1180:1233 | NORMAL_TEXT | TABLE row=2 col=1]
개활지 수평의 canonical. 정사영상 부합, 스케일 오차 0.06%, 커버리지 1.31배

[P00068 | 1234:1276 | NORMAL_TEXT | TABLE row=2 col=2]
축사 사이 통로 지면이 5~7 m 가라앉은 팬텀. 자체 수평 오차 0.6도

[P00069 | 1278:1295 | NORMAL_TEXT | TABLE row=3 col=0]
final4 (구 병합 지도)

[P00070 | 1296:1306 | NORMAL_TEXT | TABLE row=3 col=1]
국소 기하 mm급

[P00071 | 1307:1348 | NORMAL_TEXT | TABLE row=3 col=2]
datum 자세 오차 2.30도 이월, 장거리 드리프트, stale 필드

[P00072 | 1349:1368 | HEADING_3]
1.1 어느 지도가 가장 정확한가

[P00073 | 1368:1576 | NORMAL_TEXT]
최종 map2/map3(remerge)가 단일 소스 어느 것보다 정확하다. 소스별 강점만 취하도록 역할을 분리했기 때문이다: 본체 수평은 드론 프라이어(XY와 yaw만), 팔 구역은 두 LiDAR 합의, 수직과 수평도는 전 구역 LiDAR 소유. 배포 채택본은 여기에 드론 점을 병합한 tau_100(779624ad)이며, 좌표계 자기일관성 검증을 통과한 유일한 계보다.

[P00074 | 1576:1577 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00075 | 1577:1913 | NORMAL_TEXT]
다만 "정확"은 상대 기하에 한정된다. 두 계보가 360 m 기선을 0.82 m 다르게 보고, 어느 쪽도 cm급 절대 기준을 갖지 못했다. 이것은 측량으로 풀리는 문제가 아니었다. RTK 기준점은 어느 계보가 맞는지 판정하는 수단일 뿐이고, 판정이 끝나도 지도에 내부 왜곡(비강체 변형 0.82 m, tilt)이 남아 있는 한 기준점의 절대 정확도는 그 주변에서만 유효하며 지도 전체로 확장되지 않는다. RTK 정합의 정확도를 global map에 이식하려면 지도가 먼저 내부적으로 강체에 가깝고 왜곡이 기준 이하여야 하며, 이것이 prior-map 정합 가설에서 절대 기준 확보의 실제 병목이었다.

[P00076 | 1913:1914 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00077 | 1914:1916 | NORMAL_TEXT]
[INLINE_OBJECT kix.i3qoq8kuxso]

[P00078 | 1916:1917 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00079 | 1917:2308 | NORMAL_TEXT]
그림 1. 드론 지도와 LiDAR 지도를 30 m tile 단위 ICP로 정합한 misalignment의 누적 분포(판정 가능 146 tile). 두 지도가 0.3 m 이내로 일치하는 tile은 5%, 중앙값 0.83 m, 1.0 m 이내가 62%다. navigation error budget 1 m인 시스템에서 map source 간 불일치가 budget과 같은 자릿수라는 것이 이 그림의 요점이다. 단, 병합은 tau 게이트를 통과한 tile의 드론 점만 채택했으므로 이 불일치가 배포 지도에 그대로 들어간 것은 아니고, 주행 구역 대부분은 LiDAR 검증 영역이다. 이 수치가 말하는 것은 배포 지도의 결함이라기보다, 독립 소스 두 개를 교차 검증에 쓸 수 있는 수준이 아니었다는 사실이다.

[P00080 | 2308:2309 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00081 | 2309:2311 | NORMAL_TEXT]
[INLINE_OBJECT kix.edpfhlpb3fd8]

[P00082 | 2311:2312 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00083 | 2312:2620 | NORMAL_TEXT]
그림 2. graph optimization 전, 드론 지도 registration이 각 LiDAR submap(542개)에 요구한 horizontal correction의 크기(중앙값 1.94 m, p90 7.04 m). 큰 correction이 남측 구역과 pad 진입부에 공간적으로 응집되어 있다. 불일치가 random noise가 아니라 지역별로 방향이 일정한 non-rigid deformation이라는 증거이며, 이런 변형은 global rigid alignment로 제거되지 않으므로 역할 분리 병합(1.1)이 필요했던 이유이기도 하다.

[P00084 | 2620:2630 | HEADING_3]
1.2 z축 문제

[P00085 | 2630:2655 | NORMAL_TEXT]
z축 문제는 층위가 다른 세 가지로 나뉜다.

[P00086 | 2655:2656 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00087 | 2656:2878 | NORMAL_TEXT | LIST id=kix.jbfdud6mun2m level=0]
SLAM 프레임 자체의 기울기. 원시 SLAM 지도는 지배 지면이 map 프레임에서 2.0~2.3도 기울어져 생성된다(5ad8f837 실측 2.2993도). georeference 파이프라인은 롤/피치를 추정한 적이 없고 0으로 가정하므로 datum이 이 기울기를 감지하지 못한다. 기울어진 지도 위에서 yaw-only 앵커로 주행하면 고도 변화량에 비례하는 수평 오차가 방향 고정으로 발생한다.

[P00088 | 2878:2997 | NORMAL_TEXT | LIST id=kix.jbfdud6mun2m level=0]
라이브 z 드리프트. 지도가 수평이어도 주행 중 z 오차가 누적된다. 434.9 m 주행에서 +7.5 m 실측. 앵커가 yaw-only고 GPS 보정이 x/y/yaw만 다루므로 태그 시야 밖에서 z는 무보정이다.

[P00089 | 2997:2998 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00090 | 2998:3000 | NORMAL_TEXT]
[INLINE_OBJECT kix.f604xkucfm3u]

[P00091 | 3000:3001 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00092 | 3001:3345 | NORMAL_TEXT]
그림 3. 지도 세대별 gravity 기준 tilt / attitude error. SLAM은 부지를 약 2.3도 기울어진 frame으로 생성했고(1번 항목), 이 attitude error는 final4 datum으로 그대로 이월되었다. leveling은 이를 0.09도까지 잡는 사후 보정이며(2번 항목), 채택 지도의 잔여 tilt 0.74도도 운용 무시 가능 기준 0.3도(재작업 판정 규칙 R-2)를 여전히 넘는다. yaw-only anchor에서는 이 tilt가 고도 변화에 비례하는 방향 고정 수평 오차가 된다: 2.3도는 부지 고도차 17.6 m 기준 약 0.71 m, 0.74도는 약 0.23 m.

[P00093 | 3345:3346 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00094 | 3346:3348 | NORMAL_TEXT]
[INLINE_OBJECT kix.j2rktu1v3t29]

[P00095 | 3348:3349 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00096 | 3349:3690 | NORMAL_TEXT]
그림 4. z error 대 주행 거리. 파란 실선은 매핑 세션 8개에서 front-end odometry와 graph-optimized pose의 z 격차(최대 0.99 m), 주황 점선은 라이브 A2 localization의 z drift(+7.5 m / 434.9 m)를 기울기로 표시한 것. front-end는 매핑에서도 z 오차를 만들지만 offline graph optimization이 제거한다. 라이브에는 이 correction 경로가 없어서 같은 front-end의 z 오차가 그대로 누적된다(3번 항목). 라이브 z 문제는 drift의 크기보다 map constraint가 없다는 데 있다.

[P00097 | 3690:3706 | HEADING_2]
2. Localization

[P00098 | 3706:3738 | HEADING_3]
2.1 GPS: 커버리지와 georeference 종속성

[P00099 | 3738:3910 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
커버리지: 실측 주행 377초 중 RTK status 2는 t+277 이후에만 존재. fix 2213개 중 1884개가 min_fix_status 기각. 수신된 고정해의 정밀도 자체는 수평 sigma 0.014, 6초 fix 공백을 반복해 그때마다 GPS 앵커가 물러났다 재취득하며 좌표 엣지를 다시 쓴다.

[P00100 | 3910:4244 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
종속성: GPS 보정은 ECEF 좌표를 T_ecef_map으로 지도 좌표에 사상한다. 따라서 GPS 정확도의 상한은 지도 georeference 정확도이고, 이 상한을 결정하는 것은 지도 쪽이다. RTK 정합이 아무리 정확해도 지도에 내부 왜곡(1장의 z축 문제, 비강체 변형)이 남아 있으면 그 정확도를 T_ecef_map 하나로 지도 전체에 이식할 수 없기 때문이다. 당진 지도의 datum은 구 계보를 승계한 값으로 수 m급 절대 불확실성을 가진 상태였고, "georeference가 검증된 local 3D SLAM 지도"라는 선결 조건에 도달하려면 지도 내부 정합도 확보가 먼저 필요했다.

[P00101 | 4244:4269 | HEADING_3]
2.2 SLAM frontend와 JT128

[P00102 | 4269:4329 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
증상 1, 드리프트: 약간의 정합 오차에도 누적이 빠르다. z +7.5 m / 434.9 m가 대표 실측.

[P00103 | 4329:4586 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
증상 2, 정지 기울기: 평지에 로봇을 두어도 자세를 기울어진 대로 추정한다. 원인 규명 결과, 초기화 창이 평균한 기울기가 세계 프레임에 고정되고, 자세 공분산 하한이 0으로 설정되어 있어 LiDAR 업데이트 후 공분산이 수축하면서 중력 방향 보정 게인이 0에 수렴하는 구조였다. 하한을 0.2도로 되살리는 수정을 배포했다(PR #129). LiDAR point-to-plane 관측만으로는 지도와 공유하는 기울기를 관측할 수 없다는 구조적 한계는 남는다.

[P00104 | 4586:4670 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
시도한 보강: 후방 JT128을 SLAM 점군에 병합해 yaw 관측성을 높이는 실험(전방 반구만 쓰던 것을 양쪽으로), 태그 앵커 측량 게이트 강화.

[P00105 | 4670:4820 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
미확인: 잔여 기울기와 드리프트가 SLAM 알고리즘(ESKF + point-to-plane) 한계인지, HW(A2 내장 IMU 품질, JT128 특성, 보행 진동) 문제인지 분리하지 못했다. 분리하려면 동일 환경에서 센서 조합을 바꾼 벤치마크가 필요하다(4장 H1).

[P00106 | 4820:4821 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00107 | 4821:4823 | NORMAL_TEXT]
[INLINE_OBJECT kix.s0pw4jtopc2h]

[P00108 | 4823:4824 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00109 | 4824:5786 | NORMAL_TEXT]
그림 5. point-to-plane registration의 translation constraint matrix H = mean(n nᵀ)를 keyframe scan 1,069개에 대해 계산한 최소/최대 eigenvalue 비 λmin/λmax. 왼쪽은 전 세션 CDF, 오른쪽은 최장 세션 4b4ff22c(660 m)의 궤적 진행에 따른 값이다. 이 비가 낮을수록 특정 방향의 translation이 scan matching으로 구속되지 않는다. 중앙값 0.088, scan의 26%가 0.05 미만이며, weakest axis는 수직이 아니라 수평 방향이다. 축사 복도처럼 한 방향으로 뻗은 환경에서 진행 방향 translation이 degenerate하다는 뜻이고, 오른쪽 panel에서 보듯 이 상태가 수백 m 구간 내내 이어진다. 이것이 scan matching 설정 자체에 내재한 ambiguity이며, "약간의 오차에도 쉽게 drift가 생긴다"(증상 1)의 기하적 배경이다. degenerate 방향의 오차는 residual에 나타나지 않으므로 표준 point-to-plane 구성의 front-end는 이를 감지하지 못한다. 환경의 성질이므로 알고리즘을 바꿔도 degeneracy 자체는 남지만, degeneracy를 감지해 해당 방향 갱신을 제한하는 기법이나 추가 constraint(IMU 품질, 두 번째 LiDAR, heading GPS)로 영향은 줄일 수 있다. 이 수치는 scan 단독 기준 하한이라 sliding map에 정합하는 실제 front-end의 순간 constraint는 이보다 낫지만, degenerate 방향 오차가 map으로 흡수되며 누적되는 구조는 같다. AprilTag은 이 구조에서 GICP refiner의 position seed 역할이므로 태그 단독 정확도는 판정 대상이 아니며, 판정 대상은 seed 이후의 registration이 무엇을 구속할 수 있는가다.

[P00110 | 5786:5811 | HEADING_2]
3. CoBiz: 워크플로우와 네트워크 음영

[P00111 | 5811:5821 | HEADING_3]
3.1 문제 구조

[P00112 | 5821:6066 | NORMAL_TEXT]
당진 현장의 네트워크 음영은 시간적(일시 단절)이 아니라 공간적(특정 구역 진입 시 상실)이다. 현행 Task-Workflow 구조는 서버가 step task 단위로 이벤트를 내려보내고 로봇이 응답하는 방식이므로, 음영 구간에 태스크 경계가 걸리면 register 이벤트를 몇 번을 재발행하든 로봇이 받을 수 없다. 여기에 업링크 포화가 겹쳐 음영이 아닌 곳에서도 API 왕복 중앙값 8.7초로 서버 10초 창을 넘기는 실패가 발생했다.

[P00113 | 6066:6085 | HEADING_3]
3.2 설계 이력과 논의된 선택지

[P00114 | 6085:6216 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
초기 설계에서 워크플로우 전체 선전송을 검토했으나 배제됨: 전체를 로봇에 주면 stop해야 하는 순간에 task abort 명령을 못 받아 워크플로우 중단이 안 된다. 현행은 진행 중인 step task까지만 수행하고 중단하는 절충.

[P00115 | 6216:6337 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
선택지 1: 워크플로우 전체 전송 + stop request와 stop의 분리 + stop 확인까지 서버가 이벤트를 반복 발행. cobiz-core에서 워크플로우를 분해해 태스크를 실행하는 부분의 신규 구현이 필요.

[P00116 | 6337:6422 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
선택지 2 (장기): 워크플로우 수행 책임을 로봇이 소유하고 CoBiz는 read만 한다. 음영의 영향이 "제어 불가"에서 "보고 지연"으로 줄어든다.

[P00117 | 6422:6497 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
적용된 완화: CoBiz retry 시간 연장(주행 중간 음영 커버 확인), 태스크 종료/시작을 음영지역에서 피하는 워크플로우 배치.

[P00118 | 6497:6516 | HEADING_3]
3.3 남은 fallback 공백

[P00119 | 6516:6669 | NORMAL_TEXT]
localization lost(드리프트, 동적 환경 변화, 앵커 전환 실패의 구분 불가), network down 중 태스크 경계, 목표 도달 실패 시 부분 완료 보고. 세 시나리오 모두 로봇 동작과 서버 처리, 사용자 표시 상태가 한 벌의 계약으로 정의되어 있지 않다.

[P00120 | 6669:6681 | HEADING_2]
4. 가설 검증 상세

[P00121 | 6681:6740 | HEADING_3]
H1. 센서 가설: 잔여 기울기와 drift의 지배 요인은 센서 HW(JT128 + A2 내장 IMU)다

[P00122 | 6740:6940 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
검증: 정확히 측량된 테스트베드 공간에서 A2 위에 JT128, Livox Mid360, 1,000 Hz급 외장 IMU, heading을 제공하는 GPS를 독립적으로 mount하고, 동일한 주행에서 전 센서를 동시 기록한다. 같은 궤적에 대해 센서 조합별 SLAM localization 오차를 참값 대비 정량 비교하므로 주행 조건 차이가 변수에서 빠진다.

[P00123 | 6940:7107 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
확장: 테스트베드에서 얻은 오차율을 km 단위 주행으로 확장 측정하고, 내리막 / 오르막 / 계단 환경에서 오차율이 유지되는지까지 내부 수치로 확보한다. 거리와 지형별 오차 프로파일이 있어야 "당진 규모 현장에서 어느 센서 구성이면 error budget을 만족하는가"를 계약 전에 답할 수 있다.

[P00124 | 7107:7206 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
판정: 센서를 바꿨을 때 오차가 유의하게 줄면 HW 지배, 어떤 조합에서도 유사하면 알고리즘(ESKF + point-to-plane) 한계로 판정하고 알고리즘 교체를 검토한다.

[P00125 | 7206:7250 | HEADING_3]
H2. 환경 가설: 당진의 실패는 우리 테스트 환경에 없던 환경 특성에서 왔다

[P00126 | 7250:7382 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
검증: 고객 환경과 우리 환경의 차이를 변수 단위로 분리한다. 언덕의 높이와 기울기, 전체 주행 거리, 복도형 degenerate 구간의 길이(그림 5의 구속 이방성을 재현하는 구간), 네트워크 음영의 공간 분포가 당장 확인된 변수다.

[P00127 | 7382:7534 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
action: 이 변수들을 기준으로 당진 수준 이상의 험난한 조건을 테스트 환경에 상시 보유한다. 신규 현장 계약 전에 "그 현장의 변수 조합이 우리 테스트 환경의 검증 범위 안인가"를 체크리스트로 판정할 수 있게 한다. 현장에서 처음 만나는 조건을 없애는 것이 목표다.

[P00128 | 7534:7604 | HEADING_3]
H3. (선택적) Mapping 절대 기준 가설: 벤치마크 부지 한 곳에 측량급 기준을 적용해 지도 내부 오차를 정량화한다.

[P00129 | 7604:7754 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
검증: 당진처럼 km 단위 거리를 가진 현장에 대해, 한 번은 실제 토지측량에서 쓰는 정확도(측량 기준점, total station 또는 VRS 기반)로 절대 좌표를 확보하고 SLAM 지도의 내부 변형(계보 간 0.82 m 차이 같은)을 절대 기준으로 정량화해 본다.

[P00130 | 7754:7824 | HEADING_3]
H4. CoBiz Fallback시험: 워크플로우 소유권을 옮기면 음영의 영향은 "제어 불가"에서 "보고 지연"으로 줄어든다

[P00131 | 7824:8025 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
action: CoBiz와의 지속적인 논의와 integration으로 진행한다. 논의된 선택지는 워크플로우 소유권(서버 분배 vs 로봇 소유 + 서버 read), stop request/stop 분리, 음영 구간 태스크 경계 회피이며, 선택지 1(전체 전송 + stop 분리)의 prototype을 만들어 음영 재현 환경(H2)에서 검증하는 것이 첫 단계다.

[P00132 | 8025:8219 | NORMAL_TEXT | LIST id=kix.59el7os8r96o level=0]
병행: network down / localization lost / 목표 도달 실패 세 시나리오에 대해 로봇 동작, 서버 처리, 사용자 표시 상태를 한 벌의 계약으로 정의한다. localization lost는 원인 구분(drift, 동적 환경 변화, anchor 전환 실패)이 사후에 가능하도록 nav bag 상시 기록을 선결 조건으로 둔다.

[P00133 | 8219:8220 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## 2026.08.25 당진 개발 계획 (t.e10a589kzez8)

[P00134 | 1:21 | TITLE]
2026.08.25 당진 개발 계획

[P00135 | 21:22 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00136 | 22:42 | NORMAL_TEXT | LIST id=kix.w1cidtoxmmck level=0]
Created: 2026.08.25

[P00137 | 42:52 | NORMAL_TEXT | LIST id=kix.w1cidtoxmmck level=0]
Author: @JaeWon Lee <jwlee@teamgrit.kr>

[P00138 | 52:55 | HEADING_2]
문제

[P00139 | 55:58 | HEADING_3]
HW

[P00140 | 58:82 | NORMAL_TEXT | LIST id=kix.isgoem6b1ed7 level=0]
IMU가 4g 이상의 값을 받지 못 한다.

[P00141 | 82:101 | NORMAL_TEXT | LIST id=kix.isgoem6b1ed7 level=1]
[\[202608\] 당진낙농축협 프로젝트 사후분석](https://docs.google.com/document/d/1mRME7Q3F8xQq_NjXVHEOPi5K5niuPv1Bo9t-TgSa39k/edit?tab=t.ilk4oa9h4k6x) 16g로 설정 변경 후 테스트

[P00142 | 101:139 | NORMAL_TEXT | LIST id=kix.isgoem6b1ed7 level=0]
A2의 Lidar가 야외 환경에서 point cloud 범위가 좁다

[P00143 | 139:142 | HEADING_3]
SW

[P00144 | 142:153 | NORMAL_TEXT | LIST id=kix.cnz32kt8g55x level=0]
이상 감지 후 대응

[P00145 | 153:170 | NORMAL_TEXT | LIST id=kix.cnz32kt8g55x level=0]
3D Map이 정확하지 않다.

[P00146 | 170:171 | HEADING_2]
⟦EMPTY PARAGRAPH⟧

[P00147 | 171:176 | HEADING_2]
Goal

[P00148 | 176:198 | NORMAL_TEXT | LIST id=kix.2ael4tj58f18 level=0]
오피스에서 시작해서 3동까지 자율 주행

[P00149 | 198:200 | NORMAL_TEXT]
[INLINE_OBJECT kix.g45h6xceps5g]

[P00150 | 200:232 | NORMAL_TEXT | LIST id=kix.2ael4tj58f18 level=1]
빨간 구역(1,3,5): 기존의 3D SLAM & NAV

[P00151 | 232:275 | NORMAL_TEXT | LIST id=kix.2ael4tj58f18 level=2]
전체 지도를 사용하지 않고 해당 구역의 지도만을 이용해서 최대한 오차를 줄임

[P00152 | 275:301 | NORMAL_TEXT | LIST id=kix.2ael4tj58f18 level=1]
파란 구역(2,4): Line Tracking

[P00153 | 301:331 | NORMAL_TEXT | LIST id=kix.2ael4tj58f18 level=2]
기본적으로 Line 따라 이동 다음과 같은 경우 정지

[P00154 | 331:370 | NORMAL_TEXT | LIST id=kix.2ael4tj58f18 level=3]
Tag가 인식된 경우 -> 2구역의 끝인지 판단하고 아니면 다시 진행

[P00155 | 370:444 | NORMAL_TEXT | LIST id=kix.2ael4tj58f18 level=3]
Line이 인식되지 않은 경우(장애물이 있거나, Line의 끝점이거나) -> 30초(수정 가능하게 설정) 동안 재시도후 없으면 정지

[P00156 | 444:513 | NORMAL_TEXT | LIST id=kix.2ael4tj58f18 level=1]
1구역 -> 2구역: 2구역 시작점에 로봇의 위치 정보를 로봇 내부로부터 얻어서 최대한 해당 위치로 이동할 수 있도록 구성

[P00157 | 513:573 | NORMAL_TEXT | LIST id=kix.2ael4tj58f18 level=1]
2구역 -> 3구역: 3구역 내의 건물에 Tag를 부착하여 Tag가 인식되면 3D SLAM & NAV 실행

[P00158 | 573:642 | NORMAL_TEXT | LIST id=kix.2ael4tj58f18 level=1]
3구역 -> 4구역: 4구역 시작점에 로봇의 위치 정보를 로봇 내부로부터 얻어서 최대한 해당 위치로 이동할 수 있도록 구성

[P00159 | 642:709 | NORMAL_TEXT | LIST id=kix.2ael4tj58f18 level=1]
4구역 -> 5구역: Line이 끝나는 위치를 5구역 지도상에서 미리 지정해두고 그점부터 3D SLAM & NAV 실행

[P00160 | 709:738 | NORMAL_TEXT | LIST id=kix.2ael4tj58f18 level=1]
5구역 이후: 기존의 3D SLAM & NAV 사용

[P00161 | 738:764 | NORMAL_TEXT | LIST id=kix.2ael4tj58f18 level=0]
3동 중앙 통로를 끝에서부터 끝까지 자율 주행

[P00162 | 764:765 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00163 | 765:772 | HEADING_2]
체크리스트 

[P00164 | 772:776 | HEADING_3]
하우징

[P00165 | 776:782 | NORMAL_TEXT | LIST id=kix.wshtjdf0zfmx level=0]
담당: @JongHyeon Park <jhpark@teamgrit.kr>

[P00166 | 782:785 | NORMAL_TEXT | LIST id=kix.wshtjdf0zfmx level=0]
확인

[P00167 | 785:812 | NORMAL_TEXT | LIST id=kix.wshtjdf0zfmx level=1]
Livox Lidar 추가 장착 -> 하루 소요

[P00168 | 812:837 | NORMAL_TEXT | LIST id=kix.wshtjdf0zfmx level=1]
외장 IMU 사용 가능 여부 -> 3일 소요

[P00169 | 837:840 | NORMAL_TEXT | LIST id=kix.wshtjdf0zfmx level=0]
작업

[P00170 | 840:856 | NORMAL_TEXT | LIST id=kix.wshtjdf0zfmx level=1]
Livox Lidar 하우징

[P00171 | 856:862 | HEADING_3]
CoBiz

[P00172 | 862:868 | NORMAL_TEXT | LIST id=kix.hcd3ohv5c9kg level=0]
담당: @SeungYun Baek <seungyun@teamgrit.kr>

[P00173 | 868:872 | HEADING_3]
로봇팔

[P00174 | 872:878 | NORMAL_TEXT | LIST id=kix.omd11r4vptu level=0]
담당: @JaeHoon Jang <jhjang@teamgrit.kr>

[P00175 | 878:879 | NORMAL_TEXT | LIST id=kix.omd11r4vptu level=0]
⟦EMPTY PARAGRAPH⟧

[P00176 | 879:896 | HEADING_3]
자율주행 - 오피스 to 3동

[P00177 | 896:902 | NORMAL_TEXT | LIST id=kix.8dj1mhr9vt2v level=0]
담당: @MinWoo Kang <mwkang@teamgrit.kr>

[P00178 | 902:905 | NORMAL_TEXT | LIST id=kix.8dj1mhr9vt2v level=0]
확인

[P00179 | 905:928 | NORMAL_TEXT | LIST id=kix.8dj1mhr9vt2v level=1]
지금까지의 SLAM & NAV 구조 확인

[P00180 | 928:931 | NORMAL_TEXT | LIST id=kix.8dj1mhr9vt2v level=0]
작업

[P00181 | 931:946 | NORMAL_TEXT | LIST id=kix.8dj1mhr9vt2v level=1]
Line Tracking 

[P00182 | 946:959 | HEADING_3]
자율주행 - 3동 내부

[P00183 | 959:965 | NORMAL_TEXT | LIST id=kix.8dj1mhr9vt2v level=0]
담당: @Tai Thai (태성) <taithai@teamgrit.kr>

[P00184 | 965:968 | NORMAL_TEXT | LIST id=kix.8dj1mhr9vt2v level=0]
확인

[P00185 | 968:988 | NORMAL_TEXT | LIST id=kix.8dj1mhr9vt2v level=1]
A2 front lidar imu 

[P00186 | 988:1011 | NORMAL_TEXT | LIST id=kix.8dj1mhr9vt2v level=1]
A2 lidar + A2 body IMU

[P00187 | 1011:1036 | NORMAL_TEXT | LIST id=kix.8dj1mhr9vt2v level=1]
Livox lidar + livox IMU 

[P00188 | 1036:1072 | NORMAL_TEXT | LIST id=kix.8dj1mhr9vt2v level=1]
LVIO(기존의 LIO + 360 or front camera)

[P00189 | 1072:1075 | NORMAL_TEXT | LIST id=kix.8dj1mhr9vt2v level=0]
작업

[P00190 | 1075:1092 | NORMAL_TEXT | LIST id=kix.8dj1mhr9vt2v level=1]
LVIO implemented

[P00191 | 1092:1093 | NORMAL_TEXT | LIST id=kix.8dj1mhr9vt2v level=1]
⟦EMPTY PARAGRAPH⟧

[P00192 | 1093:1094 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00193 | 1094:1095 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## 9월 한달 일정 (t.riw4sdui7elp)

[P00194 | 1:10 | TITLE]
9월 한달 일정

[P00195 | 10:11 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00196 | 11:31 | NORMAL_TEXT | LIST id=kix.rq2a24i2fkfl level=0]
Created: 2026.08.31

[P00197 | 31:41 | NORMAL_TEXT | LIST id=kix.rq2a24i2fkfl level=0]
Author: @JaeWon Lee <jwlee@teamgrit.kr>

[P00198 | 44:47 | NORMAL_TEXT | TABLE row=0 col=0]
분류

[P00199 | 48:54 | NORMAL_TEXT | TABLE row=0 col=1]
8/31주

[P00200 | 55:70 | NORMAL_TEXT | TABLE row=0 col=2]
9/7주, 당진 출장 1회

[P00201 | 71:87 | NORMAL_TEXT | TABLE row=0 col=3]
9/14주, 당진 출장 1회

[P00202 | 88:94 | NORMAL_TEXT | TABLE row=0 col=4]
9/21주

[P00203 | 96:102 | NORMAL_TEXT | TABLE row=1 col=0]
Cobiz

[P00204 | 103:116 | NORMAL_TEXT | TABLE row=1 col=1]
음영지역 대응방안 모색

[P00205 | 117:136 | NORMAL_TEXT | TABLE row=1 col=2]
음영지역 대응방안 개발 및 테스트

[P00206 | 137:138 | NORMAL_TEXT | TABLE row=1 col=3]
⟦EMPTY PARAGRAPH⟧

[P00207 | 139:140 | NORMAL_TEXT | TABLE row=1 col=4]
⟦EMPTY PARAGRAPH⟧

[P00208 | 142:152 | NORMAL_TEXT | TABLE row=2 col=0]
하우징(박종현)

[P00209 | 153:177 | NORMAL_TEXT | TABLE row=2 col=1]
Livox 테스트 하우징 설계안테나 고정

[P00210 | 178:202 | NORMAL_TEXT | TABLE row=2 col=2]
(필요시) Livox lidar 배선 작업

[P00211 | 203:204 | NORMAL_TEXT | TABLE row=2 col=3]
⟦EMPTY PARAGRAPH⟧

[P00212 | 205:206 | NORMAL_TEXT | TABLE row=2 col=4]
⟦EMPTY PARAGRAPH⟧

[P00213 | 208:218 | NORMAL_TEXT | TABLE row=3 col=0]
SLAM(태성)

[P00214 | 219:236 | NORMAL_TEXT | TABLE row=3 col=1]
Livox Lidar test

[P00215 | 236:246 | NORMAL_TEXT | TABLE row=3 col=1]
LIVO test

[P00216 | 246:274 | NORMAL_TEXT | TABLE row=3 col=1]
-> 신기교 및 공사장 환경 극복 후 최종 fix

[P00217 | 275:308 | NORMAL_TEXT | TABLE row=3 col=2]
당진 환경 데이터 녹화 및 테스트(3구역 ~ 축사 내부)

[P00218 | 309:342 | NORMAL_TEXT | TABLE row=3 col=3]
당진 환경 데이터 녹화 및 테스트(1구역 ~ 축사 내부)

[P00219 | 343:349 | NORMAL_TEXT | TABLE row=3 col=4]
이슈 대응

[P00220 | 351:364 | NORMAL_TEXT | TABLE row=4 col=0]
Vision(강민우)

[P00221 | 365:384 | NORMAL_TEXT | TABLE row=4 col=1]
Road Tracking test

[P00222 | 384:410 | NORMAL_TEXT | TABLE row=4 col=1]
Road Tracking 기반 제어 기능 개발

[P00223 | 411:445 | NORMAL_TEXT | TABLE row=4 col=2]
당진 환경에서 도로, 인도 데이터 수집 및 테스트 (2구역)

[P00224 | 446:467 | NORMAL_TEXT | TABLE row=4 col=3]
기존 NAV 시스템 통합 및 테스트

[P00225 | 468:474 | NORMAL_TEXT | TABLE row=4 col=4]
이슈 대응

[P00226 | 475:489 | NORMAL_TEXT | LIST id=kix.bk4p0acyxrda level=0]
신기교 및 공사장 환경[INLINE_OBJECT kix.sbczrl47hk3]

[P00227 | 489:517 | NORMAL_TEXT | LIST id=kix.bk4p0acyxrda level=1]
신기교 : 오르막 내리막, 계단, 반복되는 환경[INLINE_OBJECT kix.4jjvry8h8t6t]

[P00228 | 517:539 | NORMAL_TEXT | LIST id=kix.bk4p0acyxrda level=1]
공사장: 특징점이 적고, 반복되는 환경

[P00229 | 539:541 | NORMAL_TEXT]
[INLINE_OBJECT kix.g4a7qwnhojwe]

[P00230 | 541:542 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00231 | 542:549 | NORMAL_TEXT | LIST id=kix.bk4p0acyxrda level=0]
당진 환경[INLINE_OBJECT kix.t9jkta3v0syl]

[P00232 | 549:553 | NORMAL_TEXT | LIST id=kix.bk4p0acyxrda level=1]
1구역

[P00233 | 553:557 | NORMAL_TEXT | LIST id=kix.bk4p0acyxrda level=1]
2구역

[P00234 | 557:561 | NORMAL_TEXT | LIST id=kix.bk4p0acyxrda level=1]
3구역

[P00235 | 561:565 | NORMAL_TEXT | LIST id=kix.bk4p0acyxrda level=1]
4구역

[P00236 | 565:569 | NORMAL_TEXT | LIST id=kix.bk4p0acyxrda level=1]
5구역

[P00237 | 569:575 | NORMAL_TEXT | LIST id=kix.bk4p0acyxrda level=1]
축사 내부

[P00238 | 575:576 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## ROSBAG 정리 (t.1g6jy235dfjg)

[P00239 | 1:12 | NORMAL_TEXT]
전체 파일 :  [A2-정자-데이터셋](https://drive.google.com/drive/folders/1--ipsIPWDrEFdZGLyc7bP9Nz32ZzdNBT)

[P00240 | 15:18 | NORMAL_TEXT | TABLE row=0 col=0]
번호

[P00241 | 19:22 | NORMAL_TEXT | TABLE row=0 col=1]
일시

[P00242 | 23:26 | NORMAL_TEXT | TABLE row=0 col=2]
장소

[P00243 | 27:31 | NORMAL_TEXT | TABLE row=0 col=3]
파일 

[P00244 | 32:35 | NORMAL_TEXT | TABLE row=0 col=4]
메모

[P00245 | 36:45 | NORMAL_TEXT | TABLE row=0 col=5]
촬영 당시 영상

[P00246 | 47:49 | NORMAL_TEXT | TABLE row=1 col=0]
1

[P00247 | 50:52 | NORMAL_TEXT | TABLE row=1 col=1]
2026년 8월 27일

[P00248 | 53:57 | NORMAL_TEXT | TABLE row=1 col=2]
신기교

[P00249 | 58:60 | NORMAL_TEXT | TABLE row=1 col=3]
[20260827_070619_teamgrit_rosbag](https://drive.google.com/drive/folders/1xD-bXAc1e1A016wGCzwdXpbEf1Y5hxcH?usp=sharing)

[P00250 | 61:71 | NORMAL_TEXT | TABLE row=1 col=4]
보행자 통로 주행

[P00251 | 72:74 | NORMAL_TEXT | TABLE row=1 col=5]
[20260827_161829.mp4](https://drive.google.com/file/d/1TBqP4oNSuRMu_PJNdYFx36OP0EhB_DNx/view?usp=drive_link)

[P00252 | 76:78 | NORMAL_TEXT | TABLE row=2 col=0]
2

[P00253 | 79:81 | NORMAL_TEXT | TABLE row=2 col=1]
2026년 8월 27일

[P00254 | 82:86 | NORMAL_TEXT | TABLE row=2 col=2]
신기교

[P00255 | 87:89 | NORMAL_TEXT | TABLE row=2 col=3]
[20260827_070327_teamgrit_rosbag](https://drive.google.com/drive/folders/1_TUay_ZM4cCtXP48ePPk9cBWAuHvcWYO?usp=drive_link)

[P00256 | 90:111 | NORMAL_TEXT | TABLE row=2 col=4]
계단 주행, IMU 데이터 확인 필요

[P00257 | 112:113 | NORMAL_TEXT | TABLE row=2 col=5]
⟦EMPTY PARAGRAPH⟧

[P00258 | 115:117 | NORMAL_TEXT | TABLE row=3 col=0]
3

[P00259 | 118:120 | NORMAL_TEXT | TABLE row=3 col=1]
2026년 8월 27일

[P00260 | 121:128 | NORMAL_TEXT | TABLE row=3 col=2]
공사장 환경

[P00261 | 129:131 | NORMAL_TEXT | TABLE row=3 col=3]
[20260827_062352_teamgrit_rosbag](https://drive.google.com/drive/folders/1KJ-k16Xn-rU2te3XBpvMIgfv6KW4MSsf?usp=drive_link)

[P00262 | 132:150 | NORMAL_TEXT | TABLE row=3 col=4]
어린이 공원 <-> 대정로 왕복

[P00263 | 151:153 | NORMAL_TEXT | TABLE row=3 col=5]
[20260827_153843.mp4](https://drive.google.com/file/d/1wZfHdkAhNLLSk7tLjGNMIqg21t72XZG1/view?usp=sharing)

[P00264 | 155:157 | NORMAL_TEXT | TABLE row=4 col=0]
4

[P00265 | 158:160 | NORMAL_TEXT | TABLE row=4 col=1]
2026년 8월 27일

[P00266 | 161:168 | NORMAL_TEXT | TABLE row=4 col=2]
공사장 환경

[P00267 | 169:171 | NORMAL_TEXT | TABLE row=4 col=3]
[20260827_063215_teamgrit_rosbag](https://drive.google.com/drive/folders/1sphgib5YsjU1RNENextAsOQpyPTMRQPi?usp=drive_link)

[P00268 | 172:218 | NORMAL_TEXT | TABLE row=4 col=4]
느티마을 어린이 공원부터 동쪽으로 한바퀴, 앞뒤 Lidar Pcd 데이터가 합쳐짐

[P00269 | 219:224 | NORMAL_TEXT | TABLE row=4 col=5]
[20260827_155155.mp4](https://drive.google.com/file/d/1yp361aaAq2PC--R6ZUGCtRa2hMPBM1UE/view?usp=drive_link), [20260827_154706.mp4](https://drive.google.com/file/d/1I3c4Mm4dOQ__hlf16l7YmtDkrs3mKe9K/view?usp=drive_link)

[P00270 | 226:228 | NORMAL_TEXT | TABLE row=5 col=0]
5

[P00271 | 229:231 | NORMAL_TEXT | TABLE row=5 col=1]
2026년 8월 27일

[P00272 | 232:239 | NORMAL_TEXT | TABLE row=5 col=2]
공사장 환경

[P00273 | 240:242 | NORMAL_TEXT | TABLE row=5 col=3]
[20260827_064730_teamgrit_rosbag](https://drive.google.com/drive/folders/1kccie_uBFfyCdwfjT9b3u1gg23sa-jA9?usp=drive_link)

[P00274 | 243:266 | NORMAL_TEXT | TABLE row=5 col=4]
느티마을 어린이 공원부터 동쪽으로 한바퀴

[P00275 | 267:268 | NORMAL_TEXT | TABLE row=5 col=5]
⟦EMPTY PARAGRAPH⟧

[P00276 | 270:272 | NORMAL_TEXT | TABLE row=6 col=0]
6

[P00277 | 273:275 | NORMAL_TEXT | TABLE row=6 col=1]
2026년 8월 31일

[P00278 | 276:283 | NORMAL_TEXT | TABLE row=6 col=2]
오피스 내부

[P00279 | 284:286 | NORMAL_TEXT | TABLE row=6 col=3]
[20260831_041812_teamgrit_rosbag](https://drive.google.com/drive/folders/1fyu7VceBcvltWDw015wi7uhflTNvaeoc)

[P00280 | 287:309 | NORMAL_TEXT | TABLE row=6 col=4]
Livox-Lidar 센서 데이터 추가

[P00281 | 310:312 | NORMAL_TEXT | TABLE row=6 col=5]
[화면 기록 2026-08-31 오후 1.44.52.mov](https://drive.google.com/file/d/1fvzR7fpr5mfEZrfsxTKIBUZLhgv0OFWJ/view?usp=drive_link)

[P00282 | 314:316 | NORMAL_TEXT | TABLE row=7 col=0]
7

[P00283 | 317:319 | NORMAL_TEXT | TABLE row=7 col=1]
⟦PROTECTED: opaqueTemplateControl/copy-only@317:318 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00284 | 320:321 | NORMAL_TEXT | TABLE row=7 col=2]
⟦EMPTY PARAGRAPH⟧

[P00285 | 322:324 | NORMAL_TEXT | TABLE row=7 col=3]
⟦PROTECTED: opaqueTemplateControl/copy-only@322:323 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00286 | 325:326 | NORMAL_TEXT | TABLE row=7 col=4]
⟦EMPTY PARAGRAPH⟧

[P00287 | 327:328 | NORMAL_TEXT | TABLE row=7 col=5]
⟦EMPTY PARAGRAPH⟧

[P00288 | 330:332 | NORMAL_TEXT | TABLE row=8 col=0]
8

[P00289 | 333:335 | NORMAL_TEXT | TABLE row=8 col=1]
⟦PROTECTED: opaqueTemplateControl/copy-only@333:334 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00290 | 336:337 | NORMAL_TEXT | TABLE row=8 col=2]
⟦EMPTY PARAGRAPH⟧

[P00291 | 338:340 | NORMAL_TEXT | TABLE row=8 col=3]
⟦PROTECTED: opaqueTemplateControl/copy-only@338:339 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00292 | 341:342 | NORMAL_TEXT | TABLE row=8 col=4]
⟦EMPTY PARAGRAPH⟧

[P00293 | 343:344 | NORMAL_TEXT | TABLE row=8 col=5]
⟦EMPTY PARAGRAPH⟧

[P00294 | 346:348 | NORMAL_TEXT | TABLE row=9 col=0]
9

[P00295 | 349:351 | NORMAL_TEXT | TABLE row=9 col=1]
⟦PROTECTED: opaqueTemplateControl/copy-only@349:350 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00296 | 352:353 | NORMAL_TEXT | TABLE row=9 col=2]
⟦EMPTY PARAGRAPH⟧

[P00297 | 354:356 | NORMAL_TEXT | TABLE row=9 col=3]
⟦PROTECTED: opaqueTemplateControl/copy-only@354:355 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00298 | 357:358 | NORMAL_TEXT | TABLE row=9 col=4]
⟦EMPTY PARAGRAPH⟧

[P00299 | 359:360 | NORMAL_TEXT | TABLE row=9 col=5]
⟦EMPTY PARAGRAPH⟧

[P00300 | 362:365 | NORMAL_TEXT | TABLE row=10 col=0]
10

[P00301 | 366:368 | NORMAL_TEXT | TABLE row=10 col=1]
⟦PROTECTED: opaqueTemplateControl/copy-only@366:367 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00302 | 369:370 | NORMAL_TEXT | TABLE row=10 col=2]
⟦EMPTY PARAGRAPH⟧

[P00303 | 371:373 | NORMAL_TEXT | TABLE row=10 col=3]
⟦PROTECTED: opaqueTemplateControl/copy-only@371:372 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00304 | 374:375 | NORMAL_TEXT | TABLE row=10 col=4]
⟦EMPTY PARAGRAPH⟧

[P00305 | 376:377 | NORMAL_TEXT | TABLE row=10 col=5]
⟦EMPTY PARAGRAPH⟧

[P00306 | 379:380 | NORMAL_TEXT | TABLE row=11 col=0]
⟦EMPTY PARAGRAPH⟧

[P00307 | 381:383 | NORMAL_TEXT | TABLE row=11 col=1]
⟦PROTECTED: opaqueTemplateControl/copy-only@381:382 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00308 | 384:385 | NORMAL_TEXT | TABLE row=11 col=2]
⟦EMPTY PARAGRAPH⟧

[P00309 | 386:388 | NORMAL_TEXT | TABLE row=11 col=3]
⟦PROTECTED: opaqueTemplateControl/copy-only@386:387 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00310 | 389:390 | NORMAL_TEXT | TABLE row=11 col=4]
⟦EMPTY PARAGRAPH⟧

[P00311 | 391:392 | NORMAL_TEXT | TABLE row=11 col=5]
⟦EMPTY PARAGRAPH⟧

[P00312 | 394:395 | NORMAL_TEXT | TABLE row=12 col=0]
⟦EMPTY PARAGRAPH⟧

[P00313 | 396:397 | NORMAL_TEXT | TABLE row=12 col=1]
⟦EMPTY PARAGRAPH⟧

[P00314 | 398:399 | NORMAL_TEXT | TABLE row=12 col=2]
⟦EMPTY PARAGRAPH⟧

[P00315 | 400:401 | NORMAL_TEXT | TABLE row=12 col=3]
⟦EMPTY PARAGRAPH⟧

[P00316 | 402:403 | NORMAL_TEXT | TABLE row=12 col=4]
⟦EMPTY PARAGRAPH⟧

[P00317 | 404:405 | NORMAL_TEXT | TABLE row=12 col=5]
⟦EMPTY PARAGRAPH⟧

[P00318 | 406:407 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## 분석 (t.ncqbbaz4cxl5)

[P00319 | 1:37 | NORMAL_TEXT]
 2: 20260827_070327_teamgrit_rosbag

[P00320 | 37:67 | NORMAL_TEXT | LIST id=kix.da5od0di42n0 level=0]
계단을 올라가는 0~30구간이 가장 충격을 많이 받음

[P00321 | 67:139 | NORMAL_TEXT | LIST id=kix.da5od0di42n0 level=0]
두 Unitree IMU 중 가장 큰 vector peak는 IMU1의 T+21.316s, 197.11m/s², 20.099g임

[P00322 | 139:202 | NORMAL_TEXT | LIST id=kix.da5od0di42n0 level=0]
IMU1 Y축 absolute peak는 156.816m/s², 15.991g이며 정확히 −16g 레일에 도달함

[P00323 | 202:251 | NORMAL_TEXT | LIST id=kix.da5od0di42n0 level=0]
IMU2 vector peak는 T+8.738s의 169.49m/s², 17.283g임

[P00324 | 251:310 | NORMAL_TEXT | LIST id=kix.da5od0di42n0 level=0]
IMU2 Y축 absolute peak도 156.811m/s², 15.990g으로 +16g 레일에 도달함

[P00325 | 310:387 | NORMAL_TEXT | LIST id=kix.da5od0di42n0 level=0]
/dog_imu_raw의 topic 구간은 약 169.469초이며 최대 timestamp gap은 약 0.002630초(2.630ms)임

[P00326 | 387:389 | NORMAL_TEXT]
[INLINE_OBJECT kix.i9vs9xtyj2t]

[P00327 | 389:390 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00328 | 390:391 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00329 | 391:426 | HEADING_3]
4: 20260827_063215_teamgrit_rosbag

[P00330 | 426:477 | NORMAL_TEXT | LIST id=kix.5unu5ex7ef9v level=0]
IMU1 vector peak는 T+595.350s의 156.64m/s², 15.973g임

[P00331 | 477:521 | NORMAL_TEXT | LIST id=kix.5unu5ex7ef9v level=0]
IMU1 Y축 absolute peak는 149.19m/s², 15.213g임

[P00332 | 521:584 | NORMAL_TEXT | LIST id=kix.5unu5ex7ef9v level=0]
IMU1의 ±16g 포화 샘플은 0개이며 최대 timestamp gap은 약 0.004218초(4.218ms)임

[P00333 | 584:618 | NORMAL_TEXT | LIST id=kix.5unu5ex7ef9v level=0]
전체 기록 구간은 metadata 기준 약 734.748초임

[P00334 | 618:619 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00335 | 619:620 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00336 | 620:621 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## 2026.07.08 ROS2 Topic Record Script (t.1uj637uml90e)

[P00337 | 1:37 | TITLE]
2026.07.08 ROS2 Topic Record Script

[P00338 | 37:38 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00339 | 38:58 | NORMAL_TEXT | LIST id=kix.e1i9ji39ut3h level=0]
Created: 2026.07.08

[P00340 | 58:83 | NORMAL_TEXT | LIST id=kix.e1i9ji39ut3h level=0]
Last Updated: 2026.08.27

[P00341 | 83:105 | NORMAL_TEXT | LIST id=kix.e1i9ji39ut3h level=0]
Author: SeungJong Kim

[P00342 | 105:106 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00343 | 106:108 | NORMAL_TEXT]
[HORIZONTAL_RULE]

[P00344 | 108:130 | HEADING_1]
Jetson 에서 rosbag 녹화시 

[P00345 | 130:153 | HEADING_2]
Docker Container 내부 접근

[P00346 | 153:181 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@153:154 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧docker start rosbag_record

[P00347 | 181:183 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@181:182 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00348 | 183:219 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@183:184 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧docker exec -it rosbag_record bash

[P00349 | 219:221 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@219:220 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00350 | 221:315 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@221:222 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧source /opt/ros/${ROS_DISTRO}/setup.bash && source /opt/ros/teamgrit/dds/teamgrit_dds_env.sh

[P00351 | 315:317 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@315:316 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00352 | 317:338 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@317:318 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧cd /recorded_rosbag

[P00353 | 338:340 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@338:339 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00354 | 340:405 | NORMAL_TEXT | LIST id=kix.50axeunmkdcl level=0]
/bags 폴더 내부에 rosbag 저장해야 host 와 mount 된 volume 에 rosbag 파일이 작성됨.

[P00355 | 405:407 | NORMAL_TEXT]
[HORIZONTAL_RULE]

[P00356 | 407:427 | HEADING_1]
A2 내부에서 rosbag 녹화시 

[P00357 | 427:428 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00358 | 428:456 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@428:429 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧# Jetson 내부에서 A2 내부 컴퓨터 접속

[P00359 | 456:463 | NORMAL_TEXT]
ssh A2

[P00360 | 463:464 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00361 | 464:471 | NORMAL_TEXT]
# 또는 

[P00362 | 471:533 | NORMAL_TEXT]
ssh unitree@192.168.123.162Password : Unitree#24226 (대문자 포함)

[P00363 | 533:535 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@533:534 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00364 | 535:571 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@535:536 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧docker exec -it teamgrit-slam bash

[P00365 | 571:573 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@571:572 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00366 | 573:601 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@573:574 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧#teamgrit-slam 도커 컨테이너 내부 

[P00367 | 601:643 | NORMAL_TEXT]
source /opt/ros/${ROS_DISTRO}/setup.bash 

[P00368 | 643:651 | NORMAL_TEXT]
cd slam

[P00369 | 651:677 | NORMAL_TEXT]
source install/setup.bash

[P00370 | 677:680 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@677:678 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧[HORIZONTAL_RULE]

[P00371 | 680:719 | HEADING_2]
운영에서 사용되는 모든 토픽 (Topics for Operation)

[P00372 | 719:720 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00373 | 720:722 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@720:721 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00374 | 722:1419 | NORMAL_TEXT]
BAG=$(date +%Y%m%d_%H%M%S)_teamgrit_rosbag && ros2 bag record -s mcap -o $BAG --max-cache-size 1073741824   /unitree/slam_lidar/points1 /unitree/slam_lidar/imu1 /unitree/slam_lidar/imu2   /grit_slam/odometry /grit_slam/optimized_odom /grit_slam/optimized_path   /grit_slam/cloud_registered_body /grit_slam/global_map_compressed /grit_slam/prior_map /grit_slam/pose_graph_markers   /tf /tf_static   /a2/front_camera/image_raw /a2/front_camera/camera_info   /cobiz/thermal_camera/image_raw/compressed /cobiz/thermal_camera/status  /ublox_gps_node/fix /navheading   /cobiz/env_sensor/co2 /cobiz/env_sensor/humidity /cobiz/env_sensor/temperature   /device_info /task_state /task_event /piper_control 

[P00375 | 1419:1440 | HEADING_2]
⟦PROTECTED: opaqueTemplateControl/copy-only@1419:1420 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧Topics for LIO-SLAM

[P00376 | 1440:1441 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00377 | 1441:1700 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@1441:1442 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧BAG=$(date +%Y%m%d_%H%M%S)_teamgrit_rosbag && ros2 bag record -s mcap -o $BAG --max-cache-size 1073741824    /unitree/slam_lidar/points1 /unitree/slam_lidar/imu1 /unitree/slam_lidar/imu2  /tf /tf_static /ublox_gps_node/fix /navheading /ublox_gps_node/navpvt

[P00378 | 1700:1735 | HEADING_2]
⟦PROTECTED: opaqueTemplateControl/copy-only@1700:1701 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧Topics for Sensor_data_collecting

[P00379 | 1735:2107 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@1735:1736 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧BAG=$(date +%Y%m%d_%H%M%S)_teamgrit_rosbag && ros2 bag record -s mcap -o $BAG --max-cache-size 1073741824  /unitree/slam_lidar/points1 /unitree/slam_lidar/points2  /unitree/slam_lidar/imu1 /unitree/slam_lidar/imu2 /tf /tf_static /ublox_gps_node/fix /navheading /ublox_gps_node/navpvt /a2/front_camera/res_360p/camera_info /a2/front_camera/res_360p/image_raw /dog_imu_raw

[P00380 | 2107:2159 | HEADING_2]
⟦PROTECTED: opaqueTemplateControl/copy-only@2107:2108 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧Topics for Sensor_data_collecting_with_livox_lidar

[P00381 | 2159:2555 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@2159:2160 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧BAG=$(date +%Y%m%d_%H%M%S)_teamgrit_rosbag && ros2 bag record -s mcap -o $BAG --max-cache-size 1073741824  /unitree/slam_lidar/points1 /unitree/slam_lidar/points2  /unitree/slam_lidar/imu1 /unitree/slam_lidar/imu2 /tf /tf_static /ublox_gps_node/fix /navheading /ublox_gps_node/navpvt /a2/front_camera/res_360p/camera_info /a2/front_camera/res_360p/image_raw /dog_imu_raw /livox/lidar /livox/imu

[P00382 | 2555:2607 | HEADING_2]
⟦PROTECTED: opaqueTemplateControl/copy-only@2555:2556 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧Topics for SLAM & Nav (Z-axis Problem, 2026-08-05)

[P00383 | 2607:2608 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00384 | 2608:2610 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@2608:2609 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00385 | 2610:3743 | NORMAL_TEXT]
BAG=$(date +%Y%m%d_%H%M%S)_teamgrit_rosbag && ros2 bag record -o "$BAG" --max-cache-size 1073741824 /unitree/slam_lidar/imu1 /tf /tf_static /api/sport/request /path_follower/request /path_follower/cmd_vel_nav2 /path_follower/goal /path_follower/status /path_follower/cancel /path_follower/nav2_path /grit_slam/localization/command /grit_slam/localization/status /grit_slam/mapping/status /task_state /task_event /safety/status /grit_slam/odometry /grit_slam/odometry_gps /grit_slam/localized_odom /grit_slam/optimized_odom /grit_slam/localized_path /grit_slam/optimized_path /grit_slam/pose_graph_markers /grit_slam/semantic_graph_nodes /grit_slam/semantic_graph_edges /tag_map_anchor/pose /tag_map_anchor/status /tag_map_anchor/path /gps_map_anchor/pose /gps_map_anchor/status /landmark_refiner/anchor /landmark_refiner/score /initialpose /fiducial_localizer/pose /fiducial_localizer/tag_point /detections /lowcmd /lowstate /lowstate_raw /navclock /navcov /navheading /navrelposned /navstate /navstatus /navsvin /navtimegps /navtimeutc /nmea /rtcm /rtk_status /ublox_gps_node/fix /ublox_gps_node/fix_velocity /ublox_gps_node/navpvt

[P00386 | 3743:3745 | HEADING_2]
⟦PROTECTED: opaqueTemplateControl/copy-only@3743:3744 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00387 | 3745:3767 | HEADING_2]
Topics for Navigation

[P00388 | 3767:3768 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00389 | 3768:3769 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00390 | 3769:3771 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@3769:3770 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00391 | 3771:3879 | NORMAL_TEXT]
BAG=$(date +%Y%m%d_%H%M%S)_nav_postmortem && ros2 bag record -s mcap --max-cache-size 1073741824  -o $BAG \

[P00392 | 3879:3912 | NORMAL_TEXT]
/grit_slam/cloud_registered_body

[P00393 | 3912:3934 | NORMAL_TEXT]
/grit_slam/global_map

[P00394 | 3934:3967 | NORMAL_TEXT]
/grit_slam/global_map_compressed

[P00395 | 3967:3993 | NORMAL_TEXT]
/grit_slam/localized_odom

[P00396 | 3993:4019 | NORMAL_TEXT]
/grit_slam/localized_path

[P00397 | 4019:4046 | NORMAL_TEXT]
/grit_slam/loop_scan_local

[P00398 | 4046:4075 | NORMAL_TEXT]
/grit_slam/loop_submap_local

[P00399 | 4075:4095 | NORMAL_TEXT]
/grit_slam/odometry

[P00400 | 4095:4119 | NORMAL_TEXT]
/grit_slam/odometry_gps

[P00401 | 4119:4145 | NORMAL_TEXT]
/grit_slam/optimized_odom

[P00402 | 4145:4171 | NORMAL_TEXT]
/grit_slam/optimized_path

[P00403 | 4171:4201 | NORMAL_TEXT]
/grit_slam/pose_graph_markers

[P00404 | 4201:4222 | NORMAL_TEXT]
/grit_slam/prior_map

[P00405 | 4222:4254 | NORMAL_TEXT]
/grit_slam/semantic_graph_edges

[P00406 | 4254:4286 | NORMAL_TEXT]
/grit_slam/semantic_graph_nodes

[P00407 | 4286:4309 | NORMAL_TEXT]
/local_costmap/costmap

[P00408 | 4309:4336 | NORMAL_TEXT]
/local_costmap/costmap_raw

[P00409 | 4336:4371 | NORMAL_TEXT]
/local_costmap/costmap_raw_updates

[P00410 | 4371:4402 | NORMAL_TEXT]
/local_costmap/costmap_updates

[P00411 | 4402:4427 | NORMAL_TEXT]
/local_costmap/footprint

[P00412 | 4427:4473 | NORMAL_TEXT]
/local_costmap/local_costmap/transition_event

[P00413 | 4473:4508 | NORMAL_TEXT]
/local_costmap/published_footprint

[P00414 | 4508:4534 | NORMAL_TEXT]
/local_costmap/stvl_layer

[P00415 | 4534:4564 | NORMAL_TEXT]
/local_costmap/stvl_layer_raw

[P00416 | 4564:4602 | NORMAL_TEXT]
/local_costmap/stvl_layer_raw_updates

[P00417 | 4602:4636 | NORMAL_TEXT]
/local_costmap/stvl_layer_updates

[P00418 | 4636:4644 | NORMAL_TEXT]
/lowcmd

[P00419 | 4644:4654 | NORMAL_TEXT]
/lowstate

[P00420 | 4654:4668 | NORMAL_TEXT]
/lowstate_raw

[P00421 | 4668:4678 | NORMAL_TEXT]
/navclock

[P00422 | 4678:4686 | NORMAL_TEXT]
/navcov

[P00423 | 4686:4698 | NORMAL_TEXT]
/navheading

[P00424 | 4698:4712 | NORMAL_TEXT]
/navrelposned

[P00425 | 4712:4722 | NORMAL_TEXT]
/navstate

[P00426 | 4722:4733 | NORMAL_TEXT]
/navstatus

[P00427 | 4733:4742 | NORMAL_TEXT]
/navsvin

[P00428 | 4742:4754 | NORMAL_TEXT]
/navtimegps

[P00429 | 4754:4766 | NORMAL_TEXT]
/navtimeutc

[P00430 | 4766:4772 | NORMAL_TEXT]
/nmea

[P00431 | 4772:4794 | NORMAL_TEXT]
/path_follower/cancel

[P00432 | 4794:4822 | NORMAL_TEXT]
/path_follower/cmd_vel_nav2

[P00433 | 4822:4842 | NORMAL_TEXT]
/path_follower/goal

[P00434 | 4842:4867 | NORMAL_TEXT]
/path_follower/nav2_path

[P00435 | 4867:4890 | NORMAL_TEXT]
/path_follower/request

[P00436 | 4890:4912 | NORMAL_TEXT]
/path_follower/status

[P00437 | 4912:4918 | NORMAL_TEXT]
/rtcm

[P00438 | 4918:4930 | NORMAL_TEXT]
/rtk_status

[P00439 | 4930:4942 | NORMAL_TEXT]
/task_event

[P00440 | 4942:4954 | NORMAL_TEXT]
/task_state

[P00441 | 4954:4978 | NORMAL_TEXT]
/terrain_cost/occupancy

[P00442 | 4978:4982 | NORMAL_TEXT]
/tf

[P00443 | 4982:4993 | NORMAL_TEXT]
/tf_static

[P00444 | 4993:5013 | NORMAL_TEXT]
/ublox_gps_node/fix

[P00445 | 5013:5042 | NORMAL_TEXT]
/ublox_gps_node/fix_velocity

[P00446 | 5042:5065 | NORMAL_TEXT]
/ublox_gps_node/navpvt

[P00447 | 5065:5066 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00448 | 5066:5068 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5066:5067 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00449 | 5068:5070 | HEADING_2]
[INLINE_OBJECT kix.ynrxi2x5n7du]

[P00450 | 5070:5089 | HEADING_2]
Front Camera Only 

[P00451 | 5089:5091 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5089:5090 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00452 | 5091:5253 | NORMAL_TEXT]
BAG=$(date +%Y%m%d_%H%M%S)_teamgrit_rosbag && ros2 bag record -s mcap -o $BAG --max-cache-size 1073741824 /a2/front_camera/image_raw /a2/front_camera/camera_info

[P00453 | 5253:5279 | HEADING_2]
⟦PROTECTED: opaqueTemplateControl/copy-only@5253:5254 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧Host 에서 저장된 rosbag 파일 확인

[P00454 | 5279:5280 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00455 | 5280:5312 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5280:5281 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧cd ~/dev/bears/recorded_rosbag

[P00456 | 5312:5314 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5312:5313 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00457 | 5314:5382 | NORMAL_TEXT | LIST id=kix.jq8cguvqtmki level=0]
위 폴더 내에서 저장된 시각으로 생성된 YYYYmmdd_HHMMSS_teamgrit_rosbag 폴더 및 파일 확인 가능

[P00458 | 5382:5383 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00459 | 5383:5384 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00460 | 5384:5385 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00461 | 5385:5386 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00462 | 5386:5387 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## Technical Notes (t.b8ro0fodelvs)

[P00463 | 1:2 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## 2026.08.25 JT128 IMU Rail Issue (t.c5bq044gyptb)

[P00464 | 1:2 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## 2026.08.25 JT128 IMU 4g Limit 도달 관찰 - 1 (t.wwbv0tn9kmtr)

[P00465 | 1:41 | HEADING_1]
2026.08.25 JT128 IMU 4g Limit 도달 관찰 - 1

[P00466 | 41:61 | NORMAL_TEXT | LIST id=kix.ey4cthoewqyw level=0]
Created: 2026.08.25

[P00467 | 61:82 | NORMAL_TEXT | LIST id=kix.ey4cthoewqyw level=0]
Author: GPT 5.6 Luna

[P00468 | 82:112 | NORMAL_TEXT | LIST id=kix.ey4cthoewqyw level=0]
Human Reviewer: SeungJong Kim

[P00469 | 112:121 | HEADING_2]
사용한 데이터셋

[P00470 | 121:157 | NORMAL_TEXT | LIST id=kix.fer9uck3k742 level=0]
[20260713_041620_teamgrit_rosbag.zip](https://drive.google.com/file/d/1GH6dztvqvhik56yQeMByRDHNvnRJqVIO/view?usp=sharing)

[P00471 | 157:158 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00472 | 158:161 | HEADING_2]
결론

[P00473 | 161:207 | NORMAL_TEXT | LIST id=kix.ey4cthoewqyw level=0]
이 rosbag에서 IMU 가속도는 각 축의 ±4g 측정 한계에 반복적으로 도달함

[P00474 | 207:303 | NORMAL_TEXT | LIST id=kix.ey4cthoewqyw level=0]
4g는 9.80665m/s² × 4 = 39.2266m/s²이며, 실제 raw rail 값은 양의 방향 +39.2028m/s², 음의 방향 -39.2040m/s²로 기록됨

[P00475 | 303:322 | NORMAL_TEXT | LIST id=kix.ey4cthoewqyw level=0]
도달은 중력 방향인 Y축에 집중됨

[P00476 | 322:367 | NORMAL_TEXT | LIST id=kix.ey4cthoewqyw level=0]
이는 로봇의 충격 입력이 IMU의 축별 측정 범위에 도달했거나 초과했음을 의미함

[P00477 | 367:413 | NORMAL_TEXT | LIST id=kix.ey4cthoewqyw level=0]
Rail에서 잘린 값만으로 실제 충격이 4g를 얼마나 초과했는지는 확인할 수 없음

[P00478 | 413:429 | HEADING_2]
Rosbag metadata

[P00479 | 432:435 | NORMAL_TEXT | TABLE row=0 col=0]
항목

[P00480 | 436:438 | NORMAL_TEXT | TABLE row=0 col=1]
값

[P00481 | 440:447 | NORMAL_TEXT | TABLE row=1 col=0]
Rosbag

[P00482 | 448:487 | NORMAL_TEXT | TABLE row=1 col=1]
20260713_041620_teamgrit_rosbag_0.mcap

[P00483 | 489:495 | NORMAL_TEXT | TABLE row=2 col=0]
기록 시작

[P00484 | 496:527 | NORMAL_TEXT | TABLE row=2 col=1]
2026-07-13T04:16:21.471025060Z

[P00485 | 529:535 | NORMAL_TEXT | TABLE row=3 col=0]
기록 종료

[P00486 | 536:567 | NORMAL_TEXT | TABLE row=3 col=1]
2026-07-13T04:27:50.640249397Z

[P00487 | 569:575 | NORMAL_TEXT | TABLE row=4 col=0]
재생 시간

[P00488 | 576:591 | NORMAL_TEXT | TABLE row=4 col=1]
689.169224337s

[P00489 | 593:600 | NORMAL_TEXT | TABLE row=5 col=0]
전체 메시지

[P00490 | 601:609 | NORMAL_TEXT | TABLE row=5 col=1]
320,989

[P00491 | 611:620 | NORMAL_TEXT | TABLE row=6 col=0]
전체 topic

[P00492 | 621:624 | NORMAL_TEXT | TABLE row=6 col=1]
13

[P00493 | 626:632 | NORMAL_TEXT | TABLE row=7 col=0]
시간 기준

[P00494 | 633:657 | NORMAL_TEXT | TABLE row=7 col=1]
Rosbag 시작 시점을 T+0으로 사용함

[P00495 | 658:659 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00496 | 659:677 | NORMAL_TEXT]
주요 topic은 다음과 같음.

[P00497 | 677:678 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00498 | 681:687 | NORMAL_TEXT | TABLE row=0 col=0]
Topic

[P00499 | 688:693 | NORMAL_TEXT | TABLE row=0 col=1]
Type

[P00500 | 694:700 | NORMAL_TEXT | TABLE row=0 col=2]
메시지 수

[P00501 | 701:707 | NORMAL_TEXT | TABLE row=0 col=3]
평균 주기

[P00502 | 709:734 | NORMAL_TEXT | TABLE row=1 col=0]
/unitree/slam_lidar/imu1

[P00503 | 735:755 | NORMAL_TEXT | TABLE row=1 col=1]
sensor_msgs/msg/Imu

[P00504 | 756:764 | NORMAL_TEXT | TABLE row=1 col=2]
136,296

[P00505 | 765:774 | NORMAL_TEXT | TABLE row=1 col=3]
197.77Hz

[P00506 | 776:801 | NORMAL_TEXT | TABLE row=2 col=0]
/unitree/slam_lidar/imu2

[P00507 | 802:822 | NORMAL_TEXT | TABLE row=2 col=1]
sensor_msgs/msg/Imu

[P00508 | 823:831 | NORMAL_TEXT | TABLE row=2 col=2]
136,281

[P00509 | 832:841 | NORMAL_TEXT | TABLE row=2 col=3]
197.75Hz

[P00510 | 843:876 | NORMAL_TEXT | TABLE row=3 col=0]
/unitree/slam_lidar/points_local

[P00511 | 877:905 | NORMAL_TEXT | TABLE row=3 col=1]
sensor_msgs/msg/PointCloud2

[P00512 | 906:912 | NORMAL_TEXT | TABLE row=3 col=2]
6,884

[P00513 | 913:920 | NORMAL_TEXT | TABLE row=3 col=3]
9.99Hz

[P00514 | 922:942 | NORMAL_TEXT | TABLE row=4 col=0]
/grit_slam/odometry

[P00515 | 943:965 | NORMAL_TEXT | TABLE row=4 col=1]
nav_msgs/msg/Odometry

[P00516 | 966:972 | NORMAL_TEXT | TABLE row=4 col=2]
3,472

[P00517 | 973:980 | NORMAL_TEXT | TABLE row=4 col=3]
5.04Hz

[P00518 | 981:987 | HEADING_2]
관찰 플롯

[P00519 | 987:989 | NORMAL_TEXT]
[INLINE_OBJECT kix.izv5jj5vrpe7]

[P00520 | 989:990 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00521 | 990:1190 | NORMAL_TEXT]
플롯의 X, Y, Z 패널은 각 축의 0.25초 구간 signed 최솟값, 최댓값 범위와 평균을 m/s²로 표시함. 따라서 양의 방향과 음의 방향 rail 도달을 모두 확인할 수 있음. Y축이 rail에 자주 닿아 보이는 것은 해당 구간에 rail sample이 반복된다는 뜻이며, 모든 raw sample이 계속 ±39.2m/s²에 고정됐다는 뜻은 아님.

[P00522 | 1190:1197 | HEADING_2]
축별 관찰값

[P00523 | 1200:1202 | NORMAL_TEXT | TABLE row=0 col=0]
축

[P00524 | 1203:1216 | NORMAL_TEXT | TABLE row=0 col=1]
IMU1 rail 도달

[P00525 | 1217:1230 | NORMAL_TEXT | TABLE row=0 col=2]
IMU2 rail 도달

[P00526 | 1231:1234 | NORMAL_TEXT | TABLE row=0 col=3]
관찰

[P00527 | 1236:1238 | NORMAL_TEXT | TABLE row=1 col=0]
X

[P00528 | 1239:1242 | NORMAL_TEXT | TABLE row=1 col=1]
13

[P00529 | 1243:1246 | NORMAL_TEXT | TABLE row=1 col=2]
39

[P00530 | 1247:1253 | NORMAL_TEXT | TABLE row=1 col=3]
드문 도달

[P00531 | 1255:1264 | NORMAL_TEXT | TABLE row=2 col=0]
Y, 중력 방향

[P00532 | 1265:1271 | NORMAL_TEXT | TABLE row=2 col=1]
1,760

[P00533 | 1272:1276 | NORMAL_TEXT | TABLE row=2 col=2]
885

[P00534 | 1277:1288 | NORMAL_TEXT | TABLE row=2 col=3]
반복 도달이 집중됨

[P00535 | 1290:1292 | NORMAL_TEXT | TABLE row=3 col=0]
Z

[P00536 | 1293:1295 | NORMAL_TEXT | TABLE row=3 col=1]
0

[P00537 | 1296:1298 | NORMAL_TEXT | TABLE row=3 col=2]
1

[P00538 | 1299:1309 | NORMAL_TEXT | TABLE row=3 col=3]
사실상 도달 없음

[P00539 | 1310:1311 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00540 | 1311:1431 | NORMAL_TEXT]
전체 raw sample에서 단일 축이 4g를 초과한 값은 확인되지 않음. 값이 rail에서 더 올라가지 않는 구조이므로, 이 결과는 “4g를 넘지 않았다”가 아니라 “측정값이 축별 범위 끝에 도달했다”로 해석함.

[P00541 | 1431:1432 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00542 | 1432:1468 | NORMAL_TEXT]
T+285~304초 구간에서 Y축 rail 도달은 다음과 같음.

[P00543 | 1468:1469 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00544 | 1472:1475 | NORMAL_TEXT | TABLE row=0 col=0]
센서

[P00545 | 1476:1488 | NORMAL_TEXT | TABLE row=0 col=1]
rail sample

[P00546 | 1489:1499 | NORMAL_TEXT | TABLE row=0 col=2]
전체 sample

[P00547 | 1500:1503 | NORMAL_TEXT | TABLE row=0 col=3]
비율

[P00548 | 1505:1510 | NORMAL_TEXT | TABLE row=1 col=0]
IMU1

[P00549 | 1511:1515 | NORMAL_TEXT | TABLE row=1 col=1]
103

[P00550 | 1516:1522 | NORMAL_TEXT | TABLE row=1 col=2]
3,792

[P00551 | 1523:1530 | NORMAL_TEXT | TABLE row=1 col=3]
2.716%

[P00552 | 1532:1537 | NORMAL_TEXT | TABLE row=2 col=0]
IMU2

[P00553 | 1538:1541 | NORMAL_TEXT | TABLE row=2 col=1]
60

[P00554 | 1542:1548 | NORMAL_TEXT | TABLE row=2 col=2]
3,792

[P00555 | 1549:1556 | NORMAL_TEXT | TABLE row=2 col=3]
1.582%

[P00556 | 1557:1563 | HEADING_2]
관찰 범위

[P00557 | 1563:1639 | NORMAL_TEXT | LIST id=kix.ey4cthoewqyw level=0]
대상: /unitree/slam_lidar/imu1, /unitree/slam_lidar/imu2의 linear_acceleration

[P00558 | 1639:1683 | NORMAL_TEXT | LIST id=kix.ey4cthoewqyw level=0]
필드: sensor_msgs/msg/Imu.linear_acceleration

[P00559 | 1683:1720 | NORMAL_TEXT | LIST id=kix.ey4cthoewqyw level=0]
단위: signed linear_acceleration의 m/s²

[P00560 | 1720:1749 | NORMAL_TEXT | LIST id=kix.ey4cthoewqyw level=0]
기준: 각 축의 ±4g, 약 ±39.2m/s² 범위

[P00561 | 1749:1810 | NORMAL_TEXT | LIST id=kix.ey4cthoewqyw level=0]
해석 제한: 센서가 rail에서 clipping되므로 실제 충격 크기의 상한은 이 기록만으로 추정할 수 없음

[P00562 | 1810:1813 | HEADING_2]
참고

[P00563 | 1813:1857 | NORMAL_TEXT | LIST id=kix.ey4cthoewqyw level=0]
[JT128 사용자 매뉴얼](https://www.hesaitech.com/hesaitech/pdf/JT128_%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C_J01-zh-260330.pdf?v=3): 기본 가속도 축 범위 ±4g, 해상도 0.122mg

[P00564 | 1857:1858 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00565 | 1858:1859 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## 2026.08.25 JT128 IMU 4g Limit 도달 관찰 - 2 (t.ocsrddlt2ooq)

[P00566 | 1:41 | HEADING_1]
2026.08.25 JT128 IMU 4g Limit 도달 관찰 - 2

[P00567 | 41:61 | NORMAL_TEXT | LIST id=kix.txckjhj95vw5 level=0]
Created: 2026.08.25

[P00568 | 61:82 | NORMAL_TEXT | LIST id=kix.txckjhj95vw5 level=0]
Author: GPT 5.6 Luna

[P00569 | 82:112 | NORMAL_TEXT | LIST id=kix.txckjhj95vw5 level=0]
Human Reviewer: SeungJong Kim

[P00570 | 112:115 | HEADING_2]
요약

[P00571 | 115:195 | NORMAL_TEXT]
첨부된 20260713 LiDAR, IMU, Odometry 현황 문서의 IMU 계산 기준을 적용해 A2 의 다른 rosbag 도 분석하였음.

[P00572 | 195:196 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00573 | 196:243 | NORMAL_TEXT | LIST id=kix.txckjhj95vw5 level=0]
계산 대상: sensor_msgs/msg/Imu.linear_acceleration

[P00574 | 243:342 | NORMAL_TEXT | LIST id=kix.txckjhj95vw5 level=0]
그래프: 0.25초 구간별 signed linear_acceleration의 축별 min-max 범위와 mean, acceleration vector norm peak를 표시함

[P00575 | 342:402 | NORMAL_TEXT | LIST id=kix.txckjhj95vw5 level=0]
peak table: raw sample 전체에서의 축별 peak와 vector norm peak를 기록함

[P00576 | 402:485 | NORMAL_TEXT | LIST id=kix.txckjhj95vw5 level=0]
표의 축별 peak는 비교를 위해 absolute peak로 유지하고, 플롯의 축별 값은 signed linear_acceleration으로 표시함

[P00577 | 485:545 | NORMAL_TEXT | LIST id=kix.txckjhj95vw5 level=0]
가속도 단위는 m/s²로 표기하고, vector norm은 참고용으로 g = 9.80665m/s²도 병기함

[P00578 | 545:596 | NORMAL_TEXT | LIST id=kix.txckjhj95vw5 level=0]
중력 성분은 제거하지 않음. 정지 상태에서도 vector norm이 약 1g가 될 수 있음

[P00579 | 596:681 | NORMAL_TEXT | LIST id=kix.txckjhj95vw5 level=0]
/unitree/slam_lidar/imu*에는 첨부 문서의 JT128 quantized rail 39.2028m/s², 39.2040m/s²를 적용함

[P00580 | 681:687 | HEADING_2]
분석 대상

[P00581 | 690:693 | NORMAL_TEXT | TABLE row=0 col=0]
ID

[P00582 | 694:701 | NORMAL_TEXT | TABLE row=0 col=1]
rosbag

[P00583 | 702:716 | NORMAL_TEXT | TABLE row=0 col=2]
Raw IMU topic

[P00584 | 717:725 | NORMAL_TEXT | TABLE row=0 col=3]
topic 수

[P00585 | 726:732 | NORMAL_TEXT | TABLE row=0 col=4]
재생 시간

[P00586 | 734:738 | NORMAL_TEXT | TABLE row=1 col=0]
B05

[P00587 | 739:754 | NORMAL_TEXT | TABLE row=1 col=1]
[recovered.mcap](https://drive.google.com/file/d/14punE7RXf1klVTPd6LlRngasknNTNuYR/view?usp=drivesdk)

[P00588 | 755:768 | NORMAL_TEXT | TABLE row=1 col=2]
/dog_imu_raw

[P00589 | 769:775 | NORMAL_TEXT | TABLE row=1 col=3]
6,869

[P00590 | 776:783 | NORMAL_TEXT | TABLE row=1 col=4]
6.709s

[P00591 | 785:789 | NORMAL_TEXT | TABLE row=2 col=0]
D03

[P00592 | 790:826 | NORMAL_TEXT | TABLE row=2 col=1]
[20260713_041620_teamgrit_rosbag.zip](https://drive.google.com/file/d/1GH6dztvqvhik56yQeMByRDHNvnRJqVIO/view?usp=drivesdk)

[P00593 | 827:878 | NORMAL_TEXT | TABLE row=2 col=2]
/unitree/slam_lidar/imu1, /unitree/slam_lidar/imu2

[P00594 | 879:896 | NORMAL_TEXT | TABLE row=2 col=3]
136,296, 136,281

[P00595 | 897:904 | NORMAL_TEXT | TABLE row=2 col=4]
689.2s

[P00596 | 906:910 | NORMAL_TEXT | TABLE row=3 col=0]
D06

[P00597 | 911:959 | NORMAL_TEXT | TABLE row=3 col=1]
[20260805_040824_nav_postmortem_no_points_0.mcap](https://drive.google.com/file/d/1nzFE6yOWBTQt2JkLpLvdVYeBG6BaDf3V/view?usp=drivesdk)

[P00598 | 960:985 | NORMAL_TEXT | TABLE row=3 col=2]
/unitree/slam_lidar/imu1

[P00599 | 986:993 | NORMAL_TEXT | TABLE row=3 col=3]
42,437

[P00600 | 994:1001 | NORMAL_TEXT | TABLE row=3 col=4]
212.3s

[P00601 | 1003:1007 | NORMAL_TEXT | TABLE row=4 col=0]
D07

[P00602 | 1008:1047 | NORMAL_TEXT | TABLE row=4 col=1]
[20260805_080420_teamgrit_rosbag_0.mcap](https://drive.google.com/file/d/12W3VpFExWzP9pO7tbf9Kw8GOH4472onj/view?usp=drivesdk)

[P00603 | 1048:1073 | NORMAL_TEXT | TABLE row=4 col=2]
/unitree/slam_lidar/imu1

[P00604 | 1074:1081 | NORMAL_TEXT | TABLE row=4 col=3]
36,113

[P00605 | 1082:1089 | NORMAL_TEXT | TABLE row=4 col=4]
180.7s

[P00606 | 1091:1095 | NORMAL_TEXT | TABLE row=5 col=0]
D08

[P00607 | 1096:1135 | NORMAL_TEXT | TABLE row=5 col=1]
[20260805_082617_teamgrit_rosbag_0.mcap](https://drive.google.com/file/d/1eEMo9PxHHhDAhU4BejcWe2IrdwBrlllQ/view?usp=drivesdk)

[P00608 | 1136:1161 | NORMAL_TEXT | TABLE row=5 col=2]
/unitree/slam_lidar/imu1

[P00609 | 1162:1170 | NORMAL_TEXT | TABLE row=5 col=3]
209,346

[P00610 | 1171:1180 | NORMAL_TEXT | TABLE row=5 col=4]
1,047.4s

[P00611 | 1181:1197 | HEADING_2]
Raw IMU peak 결과

[P00612 | 1197:1359 | NORMAL_TEXT]
JT128 rail hit은 해당 topic의 한 sample에서 X, Y, Z 중 하나라도 양 또는 음의 quantized rail 근처에 도달한 횟수임. rail hit은 센서가 실제 물리 가속도를 rail 이상으로 측정했다는 의미가 아니라, 저장된 값이 측정 범위에 도달했음을 뜻함.

[P00613 | 1359:1360 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00614 | 1363:1374 | NORMAL_TEXT | TABLE row=0 col=0]
ID / topic

[P00615 | 1375:1383 | NORMAL_TEXT | TABLE row=0 col=1]
samples

[P00616 | 1384:1390 | NORMAL_TEXT | TABLE row=0 col=2]
평균 Hz

[P00617 | 1391:1430 | NORMAL_TEXT | TABLE row=0 col=3]
axis absolute peak (ax, ay, az) [m/s²]

[P00618 | 1431:1450 | NORMAL_TEXT | TABLE row=0 col=4]
vector peak [m/s²]

[P00619 | 1451:1467 | NORMAL_TEXT | TABLE row=0 col=5]
vector peak [g]

[P00620 | 1468:1480 | NORMAL_TEXT | TABLE row=0 col=6]
peak T+ [s]

[P00621 | 1481:1492 | NORMAL_TEXT | TABLE row=0 col=7]
JT128 rail

[P00622 | 1494:1511 | NORMAL_TEXT | TABLE row=1 col=0]
B05 /dog_imu_raw

[P00623 | 1512:1518 | NORMAL_TEXT | TABLE row=1 col=1]
6,869

[P00624 | 1519:1527 | NORMAL_TEXT | TABLE row=1 col=2]
1,023.7

[P00625 | 1528:1546 | NORMAL_TEXT | TABLE row=1 col=3]
8.05, 8.00, 23.84

[P00626 | 1547:1553 | NORMAL_TEXT | TABLE row=1 col=4]
25.55

[P00627 | 1554:1560 | NORMAL_TEXT | TABLE row=1 col=5]
2.606

[P00628 | 1561:1567 | NORMAL_TEXT | TABLE row=1 col=6]
6.692

[P00629 | 1568:1575 | NORMAL_TEXT | TABLE row=1 col=7]
적용 안 함

[P00630 | 1577:1606 | NORMAL_TEXT | TABLE row=2 col=0]
D03 /unitree/slam_lidar/imu1

[P00631 | 1607:1615 | NORMAL_TEXT | TABLE row=2 col=1]
136,296

[P00632 | 1616:1622 | NORMAL_TEXT | TABLE row=2 col=2]
197.8

[P00633 | 1623:1643 | NORMAL_TEXT | TABLE row=2 col=3]
39.20, 39.20, 39.06

[P00634 | 1644:1650 | NORMAL_TEXT | TABLE row=2 col=4]
60.32

[P00635 | 1651:1657 | NORMAL_TEXT | TABLE row=2 col=5]
6.151

[P00636 | 1658:1665 | NORMAL_TEXT | TABLE row=2 col=6]
93.261

[P00637 | 1666:1680 | NORMAL_TEXT | TABLE row=2 col=7]
1,773, 1.301%

[P00638 | 1682:1711 | NORMAL_TEXT | TABLE row=3 col=0]
D03 /unitree/slam_lidar/imu2

[P00639 | 1712:1720 | NORMAL_TEXT | TABLE row=3 col=1]
136,281

[P00640 | 1721:1727 | NORMAL_TEXT | TABLE row=3 col=2]
197.7

[P00641 | 1728:1748 | NORMAL_TEXT | TABLE row=3 col=3]
39.20, 39.20, 39.20

[P00642 | 1749:1755 | NORMAL_TEXT | TABLE row=3 col=4]
55.45

[P00643 | 1756:1762 | NORMAL_TEXT | TABLE row=3 col=5]
5.654

[P00644 | 1763:1770 | NORMAL_TEXT | TABLE row=3 col=6]
93.256

[P00645 | 1771:1783 | NORMAL_TEXT | TABLE row=3 col=7]
924, 0.678%

[P00646 | 1785:1814 | NORMAL_TEXT | TABLE row=4 col=0]
D06 /unitree/slam_lidar/imu1

[P00647 | 1815:1822 | NORMAL_TEXT | TABLE row=4 col=1]
42,437

[P00648 | 1823:1829 | NORMAL_TEXT | TABLE row=4 col=2]
199.9

[P00649 | 1830:1850 | NORMAL_TEXT | TABLE row=4 col=3]
21.50, 39.20, 10.40

[P00650 | 1851:1857 | NORMAL_TEXT | TABLE row=4 col=4]
43.80

[P00651 | 1858:1864 | NORMAL_TEXT | TABLE row=4 col=5]
4.466

[P00652 | 1865:1872 | NORMAL_TEXT | TABLE row=4 col=6]
12.839

[P00653 | 1873:1885 | NORMAL_TEXT | TABLE row=4 col=7]
170, 0.401%

[P00654 | 1887:1916 | NORMAL_TEXT | TABLE row=5 col=0]
D07 /unitree/slam_lidar/imu1

[P00655 | 1917:1924 | NORMAL_TEXT | TABLE row=5 col=1]
36,113

[P00656 | 1925:1931 | NORMAL_TEXT | TABLE row=5 col=2]
199.9

[P00657 | 1932:1952 | NORMAL_TEXT | TABLE row=5 col=3]
39.20, 39.20, 21.50

[P00658 | 1953:1959 | NORMAL_TEXT | TABLE row=5 col=4]
49.00

[P00659 | 1960:1966 | NORMAL_TEXT | TABLE row=5 col=5]
4.997

[P00660 | 1967:1974 | NORMAL_TEXT | TABLE row=5 col=6]
72.065

[P00661 | 1975:1987 | NORMAL_TEXT | TABLE row=5 col=7]
269, 0.745%

[P00662 | 1989:2018 | NORMAL_TEXT | TABLE row=6 col=0]
D08 /unitree/slam_lidar/imu1

[P00663 | 2019:2027 | NORMAL_TEXT | TABLE row=6 col=1]
209,346

[P00664 | 2028:2034 | NORMAL_TEXT | TABLE row=6 col=2]
199.9

[P00665 | 2035:2055 | NORMAL_TEXT | TABLE row=6 col=3]
39.20, 39.20, 23.11

[P00666 | 2056:2062 | NORMAL_TEXT | TABLE row=6 col=4]
50.65

[P00667 | 2063:2069 | NORMAL_TEXT | TABLE row=6 col=5]
5.165

[P00668 | 2070:2078 | NORMAL_TEXT | TABLE row=6 col=6]
730.134

[P00669 | 2079:2093 | NORMAL_TEXT | TABLE row=6 col=7]
1,346, 0.643%

[P00670 | 2094:2100 | HEADING_3]
주요 관측

[P00671 | 2100:2213 | NORMAL_TEXT | LIST id=kix.txckjhj95vw5 level=0]
가장 큰 vector norm peak는 D03 /unitree/slam_lidar/imu1의 60.32m/s², 6.151g임. 같은 bag의 IMU2도 55.45m/s², 5.654g까지 도달했음.

[P00672 | 2213:2330 | NORMAL_TEXT | LIST id=kix.txckjhj95vw5 level=0]
Unitree IMU topic은 여러 bag에서 하나 이상의 축이 약 39.2m/s² rail에 도달했음. 이는 개별 축의 저장 범위 도달이며, vector norm peak를 4g로 제한하는 기준은 아님.

[P00673 | 2330:2444 | NORMAL_TEXT | LIST id=kix.txckjhj95vw5 level=0]
B05 /dog_imu_raw는 최대 vector norm이 2.606g이고 Z축 peak가 23.84m/s²임. JT128 rail을 적용하지 않아 Unitree IMU와 같은 기준으로 비교하지 않음.

[P00674 | 2444:2543 | NORMAL_TEXT | LIST id=kix.txckjhj95vw5 level=0]
D03의 결과는 기존 분석 문서에 기록된 IMU1 6.151g, IMU2 5.654g 결과와 일치함. 이번 문서에서는 다른 raw IMU bag도 같은 방식으로 추가 비교했음.

[P00675 | 2543:2555 | HEADING_2]
rosbag별 시각화

[P00676 | 2555:2591 | NORMAL_TEXT]
각 항목에 실제 rosbag 파일명과 기록 특성을 함께 표기함.

[P00677 | 2591:2746 | NORMAL_TEXT]
그래프 왼쪽에는 0.25초 bin의 signed linear_acceleration min-max 범위와 mean을 표시함. 오른쪽에는 같은 bin의 vector norm peak를 표시함. 분홍색 점은 raw sample 전체의 global vector norm peak임.

[P00678 | 2746:2815 | HEADING_3]
[recovered.mcap](https://drive.google.com/file/d/14punE7RXf1klVTPd6LlRngasknNTNuYR/view?usp=drivesdk) (B05) | 2026-07-13 nav postmortem, /dog_imu_raw 단기 기록

[P00679 | 2815:2901 | NORMAL_TEXT]
/dog_imu_raw의 peak는 bag 후반 T+6.692s에 발생했음. Z축은 23.84m/s²까지 증가했음. JT128 rail은 적용하지 않음.

[P00680 | 2901:2903 | NORMAL_TEXT]
[INLINE_OBJECT kix.u9ujgjoatxne]

[P00681 | 2903:2904 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00682 | 2904:2981 | HEADING_3]
[20260713_041620_teamgrit_rosbag.zip](https://drive.google.com/file/d/1GH6dztvqvhik56yQeMByRDHNvnRJqVIO/view?usp=drivesdk) (D03) | 2026-07-13 현장 혼합, 듀얼 IMU, 장시간 기록

[P00683 | 2981:3084 | NORMAL_TEXT]
D03은 2026-07-13 현장 혼합 기록임. 두 IMU의 최대 vector peak는 T+93.26s 부근에 발생했음. IMU1은 6.151g, IMU2는 5.654g를 기록했음.

[P00684 | 3084:3086 | NORMAL_TEXT]
[INLINE_OBJECT kix.4j07mrxm090y]

[P00685 | 3086:3087 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00686 | 3087:3174 | HEADING_3]
[20260805_040824_nav_postmortem_no_points_0.mcap](https://drive.google.com/file/d/1nzFE6yOWBTQt2JkLpLvdVYeBG6BaDf3V/view?usp=drivesdk) (D06) | LiDAR point 없음, IMU와 output 기록

[P00687 | 3174:3266 | NORMAL_TEXT]
D06은 LiDAR point topic 없이 IMU와 output을 포함한 기록임. IMU1의 vector peak는 T+12.839s에 4.466g를 기록했음.

[P00688 | 3266:3268 | NORMAL_TEXT]
[INLINE_OBJECT kix.2xsrqixm4m4k]

[P00689 | 3268:3269 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00690 | 3269:3351 | HEADING_3]
[20260805_080420_teamgrit_rosbag_0.mcap](https://drive.google.com/file/d/12W3VpFExWzP9pO7tbf9Kw8GOH4472onj/view?usp=drivesdk) (D07) | TeamGRIT Office 데이터, IMU1 단일 topic

[P00691 | 3351:3413 | NORMAL_TEXT]
D07은 IMU1의 X축이 rail에 도달한 T+72.065s에 4.997g vector peak를 기록했음.

[P00692 | 3413:3414 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00693 | 3414:3416 | NORMAL_TEXT]
[INLINE_OBJECT kix.mw1mp11qcuy3]

[P00694 | 3416:3497 | HEADING_3]
[20260805_082617_teamgrit_rosbag_0.mcap](https://drive.google.com/file/d/1eEMo9PxHHhDAhU4BejcWe2IrdwBrlllQ/view?usp=drivesdk) (D08) | 성남테스트베드 기록, IMU1 단일 topic, 장시간 기록

[P00695 | 3497:3563 | NORMAL_TEXT]
D08은 분석 대상 중 가장 긴 IMU 기록이며, T+730.134s에 5.165g vector peak를 기록했음.

[P00696 | 3563:3564 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00697 | 3564:3566 | NORMAL_TEXT]
[INLINE_OBJECT kix.rf4h2ncfc3lq]

[P00698 | 3566:3567 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## 2026.08.25 JT128 IMU 측정범위 수정 진행사항 (펌웨어) (t.ilk4oa9h4k6x)

[P00699 | 1:41 | HEADING_1]
2026.08.25 JT128 IMU 측정범위 수정 진행사항 (펌웨어)

[P00700 | 41:61 | NORMAL_TEXT | LIST id=kix.6erzxwp8cexy level=0]
Created: 2026.08.25

[P00701 | 61:83 | NORMAL_TEXT | LIST id=kix.6erzxwp8cexy level=0]
Author: GPT 5.6 Luna 

[P00702 | 83:113 | NORMAL_TEXT | LIST id=kix.6erzxwp8cexy level=0]
Human Reviewer: SeungJong Kim

[P00703 | 113:116 | HEADING_2]
요약

[P00704 | 116:167 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
JT128 ROS 2 publisher는 A2 PC의 호스트 영역에서 실행 중임을 확인함.

[P00705 | 167:255 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
A2에서 두 JT128 장치의 PTC TCP 연결을 확인함. 대상 주소는 192.168.124.20, 192.168.124.21, PTC 포트는 9347임.

[P00706 | 255:328 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
JT128 공식 매뉴얼에서 가속도 범위 선택지 ±2g, ±4g, ±8g, ±16g와 0xFF, 0x0000012E 명령을 확인함.

[P00707 | 328:421 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
공개 자료에는 0x0000012E 뒤에 붙는 16g 선택 필드와 전체 Payload가 공개되어 있지 않음.  → 16g 수정 성공, 문서 6.2 & 7 항목 확인. 

[P00708 | 421:466 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
16g 수정 이후 A2 주행 시 JT128 IMU 값의 확실한 개선을 확인하였음

[P00709 | 466:468 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=1]
[INLINE_OBJECT kix.13fei4hiy7tk]

[P00710 | 468:544 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=1]
4g (~=40m/s^2) 를 넘는 값이 office 내부 A2 주행에서도 관찰되며, 4g 기준을 넘는 값이 rosbag 에 저장되었음

[P00711 | 544:557 | HEADING_2]
1. A2 런타임 확인

[P00712 | 557:581 | HEADING_3]
1.1 publisher 위치와 실행 경로

[P00713 | 581:607 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
최종 접속 호스트: unitree-a2-pc2

[P00714 | 607:634 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
A2 호스트 주소: 192.168.123.162

[P00715 | 634:671 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
LiDAR 전용 네트워크 인터페이스: 192.168.124.162

[P00716 | 671:715 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
ROS 2 publisher 프로세스: hesai_ros_driver_node

[P00717 | 715:743 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
실행 경로: unitree_slam.service

[P00718 | 743:776 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
실행 위치: Docker 컨테이너가 아닌 A2 호스트 영역

[P00719 | 776:853 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
활성 설정 파일: /home/unitree/graph_pid_ws/config_files/hs_lidar_jt128/config.yaml

[P00720 | 853:863 | HEADING_3]
1.2 연결 상태

[P00721 | 863:907 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
JT128 장치 주소: 192.168.124.20, 192.168.124.21

[P00722 | 907:939 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
두 드라이버 모두 is_use_ptc: true로 설정됨

[P00723 | 939:976 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
두 JT128 장치로 향하는 PTC TCP 9347 연결을 확인함

[P00724 | 976:1009 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
포인트클라우드 UDP 포트 2368, 2369를 사용 중임

[P00725 | 1009:1065 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
현재 설정 파일에는 g-force 또는 acceleration full-scale 지정 항목이 없음

[P00726 | 1065:1066 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00727 | 1066:1175 | NORMAL_TEXT]
위 항목은 A2에 직접 접속해 확인한 런타임 사실임. 실제 ROS topic 이름과 IMU 메시지의 최종 스케일은 16g 변경 전후의 topic과 raw packet을 비교해 별도 확인해야 함.

[P00728 | 1175:1197 | HEADING_2]
2. JT128 공식 매뉴얼 확인 내용

[P00729 | 1197:1239 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
IMU Acceleration Unit 현재 값: 0x7A, 십진수 122

[P00730 | 1239:1275 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
매뉴얼의 단위식: 0.001 mg × 122 = 0.122 mg

[P00731 | 1275:1307 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
가속도 범위 선택지: ±2g, ±4g, ±8g, ±16g

[P00732 | 1307:1322 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
기본 가속도 범위: ±4g

[P00733 | 1322:1332 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
범위 설정 명령:

[P00734 | 1332:1355 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=1]
PTC command code: 0xFF

[P00735 | 1355:1384 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=1]
extended command: 0x0000012E

[P00736 | 1384:1451 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
같은 명령 영역에서 자이로 범위도 다루므로, Payload는 가속도 범위와 자이로 범위를 구분하는 구조일 가능성이 높음

[P00737 | 1451:1452 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00738 | 1452:1622 | NORMAL_TEXT]
매뉴얼은 명령 코드와 extended command까지만 공개함. 16g 선택에 필요한 최종 파라미터 바이트, 저장 방식, 재부팅 필요 여부는 TCP API Reference Manual을 확인하도록 안내함. 해당 문서는 공개 다운로드 문서가 아니라 Hesai 지원 채널에서 받아야 하는 자료로 확인됨.

[P00739 | 1622:1641 | HEADING_2]
3. PTC와 16g 명령 프레임

[P00740 | 1641:1653 | HEADING_3]
3.1 PTC의 의미

[P00741 | 1653:1784 | NORMAL_TEXT]
Hesai 자료에서 PTC는 LiDAR 제어 명령을 TCP로 교환하는 명령 프레임 및 API 계열을 가리킴. 공개 문서에서 약어의 풀네임은 일관되게 정의되어 있지 않음. 이 문서에서는 PTC를 Hesai TCP 제어 명령으로 표현함.

[P00742 | 1784:1807 | HEADING_3]
3.2 공개 자료에서 확인한 프레임 구조

[P00743 | 1807:1848 | NORMAL_TEXT]
공개 TCP API와 SDK에서 확인한 공통 프레임 구조는 다음과 같음.

[P00744 | 1848:1849 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00745 | 1849:1950 | NORMAL_TEXT]
47 74 [command code: 1 byte] [return/reserved: 1 byte] [payload length: uint32 big-endian] [payload]

[P00746 | 1950:1951 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00747 | 1951:1996 | NORMAL_TEXT]
JT128 extended command를 적용하면 다음 부분까지 확인 가능함.

[P00748 | 1996:1997 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00749 | 1997:2045 | NORMAL_TEXT]
47 74 FF 00 [Payload length: uint32 big-endian]

[P00750 | 2045:2046 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00751 | 2046:2089 | NORMAL_TEXT]
00 00 01 2E [가속도/자이로 범위 파라미터: 공개 자료에서 미확정]

[P00752 | 2089:2090 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00753 | 2090:2124 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
00 00 01 2E는 extended command 자체임

[P00754 | 2124:2153 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
00 00 01 2E만으로 16g가 선택되지는 않음

[P00755 | 2153:2230 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
Hesai 공식 Python 예제는 command code와 Payload를 hex 문자열로 조립해 PTC TCP 포트 9347로 전송함

[P00756 | 2230:2298 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
SDK 문서도 extended command의 경우 dataIn에 extended command 자체를 포함한다고 설명함

[P00757 | 2298:2325 | HEADING_3]
3.3 사용하지 않아야 하는 추정 Payload

[P00758 | 2325:2411 | NORMAL_TEXT]
현재 공개 자료로 재현 가능한 범위는 프레임 외곽 구조와 extended command까지임. 16g 선택을 위한 완성된 송신 바이트열은 확정하지 못함.

[P00759 | 2411:2412 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00760 | 2412:2485 | NORMAL_TEXT]
다음 값은 공식 JT128 API Reference Manual 또는 공식 도구의 송신 캡처로 검증되지 않았으므로 사용하지 않음.

[P00761 | 2485:2486 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00762 | 2486:2501 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
00 00 01 2E 10

[P00763 | 2501:2516 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
00 00 01 2E 03

[P00764 | 2516:2531 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
00 00 01 2E 04

[P00765 | 2531:2559 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
가속도 필드만 바꾸고 자이로 필드를 생략하는 형식

[P00766 | 2559:2582 | HEADING_2]
4. 공식 SDK와 도구에서 확인한 범위

[P00767 | 2582:2605 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
PTC 1.0 식별자: 0x47 0x74

[P00768 | 2605:2661 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
Payload length: network byte order, 즉 big-endian uint32

[P00769 | 2661:2691 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
extended command marker: 0xFF

[P00770 | 2691:2754 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
SDK는 uint32 형태의 sub-command를 지원하고 여러 PTC 명령을 big-endian으로 직렬화함

[P00771 | 2754:2808 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
공식 LidarUtilities에는 JT128 지원과 Send PTC Command 기능이 있음

[P00772 | 2808:2920 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
Ubuntu 22용 LidarUtilities 바이너리에는 일반적인 IMU 관련 PTC 명령 상수가 보이지만, JT128의 0x0000012E 안에서 16g를 선택하는 파라미터 인코딩은 노출되지 않음

[P00773 | 2920:2921 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00774 | 2921:3043 | NORMAL_TEXT]
SDK에 일반 IMU 명령 상수가 있다는 사실만으로 JT128 16g 명령을 만들 수는 없음. 일반 IMU 설정 명령 0xB7을 JT128 extended command 0x0000012E의 대체값으로 사용하지 않음.

[P00775 | 3043:3061 | HEADING_2]
5. 16g 변경 시 예상 영향

[P00776 | 3061:3071 | HEADING_3]
5.1 기대 효과

[P00777 | 3071:3098 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
더 큰 충격과 진동을 포화 없이 측정할 수 있음

[P00778 | 3098:3109 | HEADING_3]
5.2 예상 부작용

[P00779 | 3109:3207 | NORMAL_TEXT]
동일한 ADC 또는 동일한 비트 수를 전제로 하면, ±4g에서 ±16g로 범위를 넓힐 때 1 count당 물리량이 커져 저가속도 구간의 분해능이 대략 4배 거칠어질 수 있음.

[P00780 | 3207:3208 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00781 | 3208:3234 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
작은 진동과 저주파 움직임의 양자화 오차 증가

[P00782 | 3234:3291 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
IMU 기반 자세 보정 또는 LiDAR-inertial odometry의 저가속도 민감도 저하 가능성

[P00783 | 3291:3352 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
드라이버가 IMU Acceleration Unit 필드를 반영하지 않으면 가속도 스케일을 잘못 해석할 가능성

[P00784 | 3352:3410 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
필터의 IMU noise 및 covariance가 기존 4g 기준으로 고정되어 있으면 튜닝 재검토 필요

[P00785 | 3410:3443 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
저장형 명령이면 재부팅 뒤에도 설정이 유지되는지 확인 필요

[P00786 | 3443:3444 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00787 | 3444:3529 | NORMAL_TEXT]
다른 Hesai 16g 모델의 매뉴얼에서 0.488 mg 단위를 확인할 수 있으나, JT128 공식 설정값은 아니므로 JT128 결과로 사용하지 않음.

[P00788 | 3529:3544 | HEADING_3]
5.3 변경 후 검증 항목

[P00789 | 3544:3592 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
JT128 UDP IMU packet의 Acceleration Unit 값 변경 여부

[P00790 | 3592:3648 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
ROS IMU 메시지의 linear_acceleration 값과 raw packet 값의 일치 여부

[P00791 | 3648:3676 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
정지 상태에서 중력 크기와 자세 추정의 정상 여부

[P00792 | 3676:3714 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
작은 진동과 큰 충격을 각각 기록한 뒤 포화 여부와 noise 비교

[P00793 | 3714:3765 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
LiDAR-inertial odometry에서 기존보다 drift가 악화되지 않았는지 여부

[P00794 | 3765:3794 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
두 JT128 장치에 같은 설정이 적용되었는지 여부

[P00795 | 3794:3814 | HEADING_2]
6. 16g 확정을 위한 다음 단계

[P00796 | 3814:3848 | HEADING_3]
6.1 Hesai API Reference Manual 요청

[P00797 | 3848:3905 | NORMAL_TEXT]
Hesai에 JT128 TCP API Reference Manual을 요청하고, 다음 항목을 문의함.

[P00798 | 3905:3906 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00799 | 3906:3923 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
PTC command 0xFF

[P00800 | 3923:3951 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
extended command 0x0000012E

[P00801 | 3951:4017 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
±2g, ±4g, ±8g, ±16g에 대한 acceleration range enum 또는 field encoding

[P00802 | 4017:4042 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
gyro range field의 순서와 길이

[P00803 | 4042:4062 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
command response 형식

[P00804 | 4062:4095 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
persistent save 여부와 reboot 필요 여부

[P00805 | 4095:4111 | HEADING_3]
6.2 공식 도구 패킷 캡처

[P00806 | 4111:4264 | NORMAL_TEXT]
테스트용 JT128 한 대에서 공식 LidarUtilities의 Send PTC Command를 사용해 4g와 16g 설정을 각각 적용함. TCP 9347 송신 패킷을 캡처해 두 Payload를 비교함. 이 방법으로 공개 문서에 없는 최종 Payload를 확인할 수 있음.

[P00807 | 4264:4265 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00808 | 4265:4351 | NORMAL_TEXT]
(2026.08.25) 직접 A2 내에서 LidarUtilities 사용, 관련 설정 확인 불가 → Hesai 또는 Unitree 측에 문의 필요할 듯.

[P00809 | 4351:4353 | NORMAL_TEXT]
[INLINE_OBJECT kix.oa95cs1seip]

[P00810 | 4353:4354 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00811 | 4354:4382 | NORMAL_TEXT]
(2026.08.25) 아래 방법으로 하면 된다!

[P00812 | 4382:4383 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00813 | 4383:4450 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@4383:4384 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧cd /home/unitree/pandarview218/PandarView2 && ./LidarUtilities.sh

[P00814 | 4450:4452 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@4450:4451 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P00815 | 4452:4454 | NORMAL_TEXT]
[INLINE_OBJECT kix.78qf1x2407y6]

[P00816 | 4454:4455 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00817 | 4455:4465 | HEADING_2]
7. 수정 이후 

[P00818 | 4465:4466 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00819 | 4466:4468 | NORMAL_TEXT]
[INLINE_OBJECT kix.o5wcivb10rd]

[P00820 | 4468:4469 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00821 | 4469:4480 | NORMAL_TEXT | LIST id=kix.4n2id22hf9vi level=0]
확실한 개선 확인됨

[P00822 | 4480:4482 | NORMAL_TEXT | LIST id=kix.4n2id22hf9vi level=0]
[20260825-office-after-jt128-16g-setting](https://drive.google.com/drive/folders/1CWR98ctoySsSPUtUOgNa5ZJuh7WdMgrt)

[P00823 | 4482:4485 | HEADING_2]
참고

[P00824 | 4485:4509 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
[Hesai JT128 User Manual](https://www.hesaitech.com/wp-content/uploads/2026/04/JT128_User_Manual_J01-en-260330.pdf)

[P00825 | 4509:4527 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
[Hesai Use TCP API](https://hesaitechnology.github.io/dev/docs/how_to_guides/TCP_api/)

[P00826 | 4527:4554 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
[Hesai LidarUtilities Guide](https://hesaitechnology.github.io/dev/docs/how_to_guides/LidarUtilities/)

[P00827 | 4554:4573 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
[Hesai Get IMU Data](https://hesaitechnology.github.io/dev/docs/how_to_guides/Get_IMU_data/)

[P00828 | 4573:4613 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
[Hesai Cyber Security PTC Python example](https://github.com/HesaiTechnology/Cyber_Security/blob/master/ptc_cgi_sample/ptc/py/ptc.py)

[P00829 | 4613:4642 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
[Hesai SDK custom command API](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0/blob/master/docs/invoke_sdk_api_command_interface.md)

[P00830 | 4642:4674 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
[Hesai SDK PTC 1.0 parser header](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0/blob/master/libhesai/PtcParser/include/ptc_1_0_parser.h)

[P00831 | 4674:4714 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
[Hesai SDK PTC 1.0 parser implementation](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0/blob/master/libhesai/PtcParser/src/ptc_1_0_parser.cc)

[P00832 | 4714:4742 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
[Hesai SDK PTC client header](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0/blob/master/libhesai/PtcClient/include/ptc_client.h)

[P00833 | 4742:4778 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
[Hesai SDK PTC client implementation](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0/blob/master/libhesai/PtcClient/src/ptc_client.cc)

[P00834 | 4778:4834 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
[Hesai Pandar128E3X User Manual, non-JT128 corroboration](https://www.hesaitech.com/wp-content/uploads/Pandar128E3X_v4p5_User_Manual_128-en-240810.pdf)

[P00835 | 4834:4864 | NORMAL_TEXT | LIST id=kix.7qb14zkzb6vg level=0]
[Hesai official downloads page](https://www.hesaitech.com/downloads/)

[P00836 | 4864:4865 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## 2026.08.26 JT128 IMU 16g 수정 이후 탄천 테스트 (t.3qal6yuaxu5)

[P00837 | 1:39 | HEADING_1]
2026.08.26 JT128 IMU 16g 수정 이후 탄천 테스트

[P00838 | 39:40 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00839 | 40:60 | NORMAL_TEXT | LIST id=kix.ln1w9tipvhfo level=0]
Created: 2026.08.26

[P00840 | 60:81 | NORMAL_TEXT | LIST id=kix.ln1w9tipvhfo level=0]
Author: GPT 5.6 Luna

[P00841 | 81:111 | NORMAL_TEXT | LIST id=kix.ln1w9tipvhfo level=0]
Human Reviewer: SeungJong Kim

[P00842 | 111:112 | NORMAL_TEXT | LIST id=kix.ln1w9tipvhfo level=0]
⟦EMPTY PARAGRAPH⟧

[P00843 | 112:115 | HEADING_2]
요약

[P00844 | 115:162 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
세 rosbag은 모두 MCAP, ROS 2 Jazzy, 9개 topic으로 기록됨

[P00845 | 162:263 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
/unitree/slam_lidar/imu1, imu2는 약 500Hz로 기록되며, 모든 축의 ROS 출력값 최소 간격은 0.0047856452m/s², 즉 0.488mg로 동일함

[P00846 | 263:319 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
기존 ±4g 기준을 넘는 샘플은 세 rosbag 모두에서 반복되며, 도달은 Y축에 가장 많이 집중됨

[P00847 | 319:396 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
추정 ±16g 축 끝점에 도달한 sample은 확인되지 않음. 다만 vector norm은 여러 축의 합성값이므로 16g를 넘을 수 있음

[P00848 | 396:518 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
/dog_imu_raw는 별도 IMU topic으로 JT128 rail 기준을 적용하지 않음. /navheading은 세 rosbag 모두 linear acceleration이 0이어서 peak 분석과 플롯에서 제외함

[P00849 | 518:535 | HEADING_2]
분석 대상 및 metadata

[P00850 | 535:542 | HEADING_3]
Rosbag

[P00851 | 545:548 | NORMAL_TEXT | TABLE row=0 col=0]
ID

[P00852 | 549:555 | NORMAL_TEXT | TABLE row=0 col=1]
기록 시작

[P00853 | 556:562 | NORMAL_TEXT | TABLE row=0 col=2]
기록 종료

[P00854 | 563:569 | NORMAL_TEXT | TABLE row=0 col=3]
재생 시간

[P00855 | 570:577 | NORMAL_TEXT | TABLE row=0 col=4]
전체 메시지

[P00856 | 578:587 | NORMAL_TEXT | TABLE row=0 col=5]
전체 topic

[P00857 | 589:592 | NORMAL_TEXT | TABLE row=1 col=0]
R1

[P00858 | 593:624 | NORMAL_TEXT | TABLE row=1 col=1]
2026-08-26T07:36:46.485887035Z

[P00859 | 625:656 | NORMAL_TEXT | TABLE row=1 col=2]
2026-08-26T07:37:37.152559522Z

[P00860 | 657:671 | NORMAL_TEXT | TABLE row=1 col=3]
50.666672487s

[P00861 | 672:679 | NORMAL_TEXT | TABLE row=1 col=4]
61,432

[P00862 | 680:682 | NORMAL_TEXT | TABLE row=1 col=5]
9

[P00863 | 684:687 | NORMAL_TEXT | TABLE row=2 col=0]
R2

[P00864 | 688:719 | NORMAL_TEXT | TABLE row=2 col=1]
2026-08-26T07:38:18.667284762Z

[P00865 | 720:751 | NORMAL_TEXT | TABLE row=2 col=2]
2026-08-26T07:40:58.104678386Z

[P00866 | 752:767 | NORMAL_TEXT | TABLE row=2 col=3]
159.437393624s

[P00867 | 768:776 | NORMAL_TEXT | TABLE row=2 col=4]
199,500

[P00868 | 777:779 | NORMAL_TEXT | TABLE row=2 col=5]
9

[P00869 | 781:784 | NORMAL_TEXT | TABLE row=3 col=0]
R3

[P00870 | 785:816 | NORMAL_TEXT | TABLE row=3 col=1]
2026-08-26T07:44:24.482963188Z

[P00871 | 817:848 | NORMAL_TEXT | TABLE row=3 col=2]
2026-08-26T07:46:14.969344442Z

[P00872 | 849:864 | NORMAL_TEXT | TABLE row=3 col=3]
110.486381254s

[P00873 | 865:873 | NORMAL_TEXT | TABLE row=3 col=4]
129,740

[P00874 | 874:876 | NORMAL_TEXT | TABLE row=3 col=5]
9

[P00875 | 877:878 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00876 | 878:943 | NORMAL_TEXT]
모든 파일의 storage identifier는 mcap이며, metadata의 ROS distro는 jazzy임.

[P00877 | 943:958 | HEADING_3]
주요 topic 메시지 수

[P00878 | 961:967 | NORMAL_TEXT | TABLE row=0 col=0]
Topic

[P00879 | 968:971 | NORMAL_TEXT | TABLE row=0 col=1]
R1

[P00880 | 972:975 | NORMAL_TEXT | TABLE row=0 col=2]
R2

[P00881 | 976:979 | NORMAL_TEXT | TABLE row=0 col=3]
R3

[P00882 | 981:1006 | NORMAL_TEXT | TABLE row=1 col=0]
/unitree/slam_lidar/imu1

[P00883 | 1007:1014 | NORMAL_TEXT | TABLE row=1 col=1]
25,314

[P00884 | 1015:1022 | NORMAL_TEXT | TABLE row=1 col=2]
79,641

[P00885 | 1023:1030 | NORMAL_TEXT | TABLE row=1 col=3]
55,183

[P00886 | 1032:1057 | NORMAL_TEXT | TABLE row=2 col=0]
/unitree/slam_lidar/imu2

[P00887 | 1058:1065 | NORMAL_TEXT | TABLE row=2 col=1]
25,321

[P00888 | 1066:1073 | NORMAL_TEXT | TABLE row=2 col=2]
79,656

[P00889 | 1074:1081 | NORMAL_TEXT | TABLE row=2 col=3]
55,208

[P00890 | 1083:1096 | NORMAL_TEXT | TABLE row=3 col=0]
/dog_imu_raw

[P00891 | 1097:1103 | NORMAL_TEXT | TABLE row=3 col=1]
7,966

[P00892 | 1104:1111 | NORMAL_TEXT | TABLE row=3 col=2]
30,967

[P00893 | 1112:1119 | NORMAL_TEXT | TABLE row=3 col=3]
13,101

[P00894 | 1121:1133 | NORMAL_TEXT | TABLE row=4 col=0]
/navheading

[P00895 | 1134:1138 | NORMAL_TEXT | TABLE row=4 col=1]
406

[P00896 | 1139:1145 | NORMAL_TEXT | TABLE row=4 col=2]
1,275

[P00897 | 1146:1150 | NORMAL_TEXT | TABLE row=4 col=3]
884

[P00898 | 1152:1180 | NORMAL_TEXT | TABLE row=5 col=0]
/unitree/slam_lidar/points1

[P00899 | 1181:1185 | NORMAL_TEXT | TABLE row=5 col=1]
507

[P00900 | 1186:1192 | NORMAL_TEXT | TABLE row=5 col=2]
1,594

[P00901 | 1193:1199 | NORMAL_TEXT | TABLE row=5 col=3]
1,105

[P00902 | 1201:1229 | NORMAL_TEXT | TABLE row=6 col=0]
/unitree/slam_lidar/points2

[P00903 | 1230:1234 | NORMAL_TEXT | TABLE row=6 col=1]
507

[P00904 | 1235:1241 | NORMAL_TEXT | TABLE row=6 col=2]
1,594

[P00905 | 1242:1248 | NORMAL_TEXT | TABLE row=6 col=3]
1,105

[P00906 | 1249:1255 | HEADING_2]
분석 기준

[P00907 | 1255:1302 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
대상 필드: sensor_msgs/msg/Imu.linear_acceleration

[P00908 | 1302:1318 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
단위: signed m/s²

[P00909 | 1318:1349 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
시간 기준: rosbag 시작 시점을 T+0으로 사용함

[P00910 | 1349:1412 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
시각화: 0.25초 구간별 축별 signed min-max 범위와 mean, vector norm peak 표시

[P00911 | 1412:1461 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
vector norm: sqrt(x² + y² + z²)이며 중력 성분을 제거하지 않음

[P00912 | 1461:1501 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
축별 peak: raw sample 전체에서의 absolute peak

[P00913 | 1501:1573 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
/unitree/slam_lidar/imu1, imu2: 기존 문서의 JT128 quantized ±4g reference 적용

[P00914 | 1573:1612 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=1]
positive reference: +39.2028090671m/s²

[P00915 | 1612:1651 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=1]
negative reference: -39.2040054784m/s²

[P00916 | 1651:1714 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
/dog_imu_raw: 별도 IMU topic으로 분류하고 JT128 ±4g reference를 적용하지 않음

[P00917 | 1714:1739 | HEADING_2]
Quantization 및 16g 끝점 관찰

[P00918 | 1739:1757 | HEADING_3]
JT128 IMU1 / IMU2

[P00919 | 1757:1838 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
R1, R2, R3의 IMU1과 IMU2에서 X, Y, Z 모든 축의 최소 unique value 간격이 0.0047856452m/s²로 동일함

[P00920 | 1838:1867 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
이 간격은 0.488mg × 9.80665와 일치함

[P00921 | 1867:1948 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
기존 0.122mg 단위의 count index로 환산하면 모든 값의 나머지가 0이며, 기존 1, 2, 3 count 위치의 값은 관찰되지 않음

[P00922 | 1948:2009 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
따라서 현재 ROS 출력의 유효 양자화 간격은 이전 0.122mg 기준보다 4배 거친 0.488mg로 확인됨

[P00923 | 2009:2109 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
이 값은 ROS message의 출력 lattice에 대한 관찰이며, raw JT128 packet 내부에서 센서가 직접 해당 단위를 사용했는지까지는 이 문서만으로 확정하지 않음

[P00924 | 2109:2131 | HEADING_3]
축별 범위와 vector norm 구분

[P00925 | 2131:2219 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
R1 IMU1의 vector norm peak는 16.035g이나, 축별 absolute peak는 X 10.265g, Y 15.468g, Z 4.701g임

[P00926 | 2219:2307 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
R3 IMU2의 vector norm peak는 16.312g이나, 축별 absolute peak는 X 11.100g, Y 15.830g, Z 4.488g임

[P00927 | 2307:2369 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
vector norm이 16g를 넘었다는 사실만으로 개별 축이 16g에서 clipping되었다고 해석하지 않음

[P00928 | 2369:2439 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
추정 ±16g quantized endpoint 근처 sample은 여섯 개 Unitree topic 모두에서 확인되지 않음

[P00929 | 2439:2451 | HEADING_2]
Rosbag별 시각화

[P00930 | 2451:2639 | NORMAL_TEXT]
각 플롯의 왼쪽 패널은 0.25초 bin의 signed linear_acceleration 축별 min-max와 mean임. 오른쪽 패널은 같은 bin의 vector norm peak임. JT128 topic에는 기존 ±4g reference를 보조선으로 표시함. /dog_imu_raw는 JT128 reference를 표시하지 않음.

[P00931 | 2639:2675 | HEADING_3]
R1: 20260826_073646_teamgrit_rosbag

[P00932 | 2675:2725 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
IMU1 vector peak는 T+28.424s의 157.25m/s², 16.035g임

[P00933 | 2725:2775 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
IMU2 vector peak는 T+17.793s의 136.92m/s², 13.962g임

[P00934 | 2775:2826 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
두 Unitree IMU 모두 기존 4g reference crossing이 Y축에 집중됨

[P00935 | 2826:2891 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
/dog_imu_raw는 topic 구간이 약 36.083초이며 최대 timestamp gap은 약 24.858초임

[P00936 | 2891:2892 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00937 | 2892:2894 | NORMAL_TEXT]
[INLINE_OBJECT kix.qsnh3uyu2dfm]

[P00938 | 2894:2930 | HEADING_3]
R2: 20260826_073818_teamgrit_rosbag

[P00939 | 2930:2981 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
IMU1 vector peak는 T+133.445s의 147.04m/s², 14.994g임

[P00940 | 2981:3031 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
IMU2 vector peak는 T+87.967s의 127.37m/s², 12.988g임

[P00941 | 3031:3077 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
기존 4g reference crossing은 두 IMU 모두 Y축에서 가장 많음

[P00942 | 3077:3120 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
/dog_imu_raw의 최대 timestamp gap은 약 25.127초임

[P00943 | 3120:3121 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00944 | 3121:3123 | NORMAL_TEXT]
[INLINE_OBJECT kix.mu6ag9izdnwi]

[P00945 | 3123:3159 | HEADING_3]
R3: 20260826_074424_teamgrit_rosbag

[P00946 | 3159:3229 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
세 파일 중 가장 큰 Unitree vector peak는 IMU2의 T+64.697s 159.96m/s², 16.312g임

[P00947 | 3229:3273 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
IMU2 Y축 absolute peak는 155.24m/s², 15.830g임

[P00948 | 3273:3323 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
IMU1 vector peak는 T+36.313s의 131.43m/s², 13.402g임

[P00949 | 3323:3388 | NORMAL_TEXT | LIST id=kix.nc8isvfrwi71 level=0]
/dog_imu_raw의 topic 구간은 약 95.612초이며 최대 timestamp gap은 약 26.467초임

[P00950 | 3388:3389 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00951 | 3389:3391 | NORMAL_TEXT]
[INLINE_OBJECT kix.1y60i8c0ecql]

[P00952 | 3391:3392 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00953 | 3392:3428 | HEADING_3]
R4: 20260827_070619_teamgrit_rosbag

[P00954 | 3428:3500 | NORMAL_TEXT | LIST id=kix.tlg63hbkwtln level=0]
두 Unitree IMU 중 가장 큰 vector peak는 IMU1의 T+48.069s, 156.75m/s², 15.984g임

[P00955 | 3500:3544 | NORMAL_TEXT | LIST id=kix.tlg63hbkwtln level=0]
IMU1 Y축 absolute peak는 152.43m/s², 15.543g임

[P00956 | 3544:3594 | NORMAL_TEXT | LIST id=kix.tlg63hbkwtln level=0]
IMU2 vector peak는 T+63.091s의 140.95m/s², 14.373g임

[P00957 | 3594:3670 | NORMAL_TEXT | LIST id=kix.tlg63hbkwtln level=0]
/dog_imu_raw의 topic 구간은 약 87.385초이며 최대 timestamp gap은 약 0.002891초(2.891ms)임

[P00958 | 3670:3671 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00959 | 3671:3673 | NORMAL_TEXT]
[INLINE_OBJECT kix.77xxiygk63w]

[P00960 | 3673:3674 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00961 | 3674:3710 | HEADING_3]
R5: 20260827_070327_teamgrit_rosbag

[P00962 | 3710:3782 | NORMAL_TEXT | LIST id=kix.tlg63hbkwtln level=0]
두 Unitree IMU 중 가장 큰 vector peak는 IMU1의 T+21.316s, 197.11m/s², 20.099g임

[P00963 | 3782:3845 | NORMAL_TEXT | LIST id=kix.tlg63hbkwtln level=0]
IMU1 Y축 absolute peak는 156.816m/s², 15.991g이며 정확히 −16g 레일에 도달함

[P00964 | 3845:3894 | NORMAL_TEXT | LIST id=kix.tlg63hbkwtln level=0]
IMU2 vector peak는 T+8.738s의 169.49m/s², 17.283g임

[P00965 | 3894:3953 | NORMAL_TEXT | LIST id=kix.tlg63hbkwtln level=0]
IMU2 Y축 absolute peak도 156.811m/s², 15.990g으로 +16g 레일에 도달함

[P00966 | 3953:4030 | NORMAL_TEXT | LIST id=kix.tlg63hbkwtln level=0]
/dog_imu_raw의 topic 구간은 약 169.469초이며 최대 timestamp gap은 약 0.002630초(2.630ms)임

[P00967 | 4030:4032 | NORMAL_TEXT]
[INLINE_OBJECT kix.part6vedhqvz]

## 2026.08.25 당진낙농축협 3D Map & Map Merger 인수인계 (t.in0h1y31cqj9)

[P00968 | 1:44 | HEADING_1]
2026.08.25 당진낙농축협 3D Map & Map Merger 인수인계

[P00969 | 44:64 | NORMAL_TEXT | LIST id=kix.21a471xgk0q level=0]
Created: 2026.08.25

[P00970 | 64:85 | NORMAL_TEXT | LIST id=kix.21a471xgk0q level=0]
Author: GPT 5.6 Luna

[P00971 | 85:116 | NORMAL_TEXT | LIST id=kix.21a471xgk0q level=0]
Human Reviewer: SeungJong Kim

[P00972 | 116:123 | HEADING_2]
원본 데이터

[P00973 | 123:141 | NORMAL_TEXT | LIST id=kix.vnblc6uo6u6j level=0]
[3D Map & Map Merger](https://drive.google.com/drive/folders/1vXuhPgdnEEhkXsx2zd1_mBrBE9gVWSaw) 에서 아래 데이터 확인 가능

[P00974 | 141:220 | NORMAL_TEXT | LIST id=kix.vnblc6uo6u6j level=1]
Map Source Session (Scout Mini + Livox Mid360 으로 당진낙농축협 현장을 Partial Scan 한 결과)

[P00975 | 220:314 | NORMAL_TEXT | LIST id=kix.vnblc6uo6u6j level=1]
Result Map (Drone Map, Scout Mini + Livox Mid360, A2 LiDAR, Satellite Map 을 모두 사용한 최종 결과 후보) 

[P00976 | 314:337 | NORMAL_TEXT | LIST id=kix.vnblc6uo6u6j level=1]
아래 Map Merge 를 위한 소스코드

[P00977 | 337:350 | HEADING_2]
Map Merge 원리

[P00978 | 350:466 | NORMAL_TEXT]
현재 14개 source와 결과 map은 아래 과정을 거쳐 생성됨. 서로 다른 session의 pose를 하나의 map frame에 맞춘 뒤 원본 scan을 다시 투영하고 voxel 단위로 합치는 과정임.

[P00979 | 466:481 | NORMAL_TEXT | LIST id=kix.d1nenzvjuub9 level=0]
입력과 기준 좌표를 정함.

[P00980 | 481:727 | NORMAL_TEXT]
bootstrap.py는 14개 source session을 순회하면서 사용할 cloud를 준비함. session에 global map PCD가 있으면 그 파일을 읽고, PCD가 없고 raw scan과 pose만 있으면 scan을 pose에 따라 임시 cloud로 렌더링함. 이때 final4 compressed map을 anchor로 사용함. anchor는 아직 최종 map이 아니라, 각 source를 처음 배치하기 위한 공통 기준임.

[P00981 | 727:964 | NORMAL_TEXT]
각 mapping session은 서로 다른 local frame을 사용하므로 공통 기준 좌표가 필요함. 같은 건물 모서리가 한 session에서는 (10, 4, 2)에 있고 다른 session에서는 (−7, 12, 2)에 있을 수 있음. 이 상태에서 점을 합치면 실제로는 같은 구조가 서로 다른 위치에 두 번 나타남. 따라서 merge 전에 “moving cloud를 어느 frame으로 옮길 것인가”를 먼저 정해야 함.

[P00982 | 964:1131 | NORMAL_TEXT]
bootstrap.py는 각 source에 대해 초기 변환 T_f4_ses와 정합 품질값 tmse_m2, overlap, icp_rmse를 map/2_remerge/work/bootstrap.json에 기록함. 이 파일은 다음 단계가 source를 graph에 넣을 수 있는지 판단하는 입력이 됨.

[P00983 | 1131:1132 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00984 | 1132:1163 | NORMAL_TEXT | LIST id=kix.d1nenzvjuub9 level=0]
초기 정합으로 각 source의 대략적인 위치를 찾음.

[P00985 | 1163:1453 | NORMAL_TEXT]
global_align.py는 먼저 cloud를 voxel downsample함. 수백만 개의 점을 그대로 비교하면 feature 계산과 후보 탐색이 지나치게 느려지고, 센서 noise와 동일 표면의 점 밀도 차이가 correspondence를 불안정하게 만듦. downsample된 cloud에서 surface normal과 FPFH feature를 계산함. FPFH는 한 점 주변의 기하학적 모양을 요약하므로, 두 cloud가 아직 다른 위치에 있어도 모서리나 구조물처럼 비슷한 모양을 가진 점을 찾는 데 사용함.

[P00986 | 1453:1784 | NORMAL_TEXT]
그 다음 global_align.py는 한쪽 점의 FPFH 최근접 이웃만 믿지 않고 양쪽에서 서로 최근접인 mutual correspondence만 남김. 한 방향만 사용하면 구조가 실제로 대응하지 않아도 moving cloud의 모든 점이 anchor 안에서 가장 비슷한 점 하나를 얻게 됨. mutual 조건은 이런 우연한 매칭을 줄임. 남은 correspondence 중에서도 점 사이 거리와 z 차이가 서로 일관되는지 검사함. 실제 rigid transform으로 대응되는 점들은 다른 대응점과의 거리 관계를 함께 보존하지만, 잘못 연결된 점들은 이 관계를 반복해서 만족하기 어려움.

[P00987 | 1784:2084 | NORMAL_TEXT]
이 구현은 두 map이 gravity-aligned라는 전제에서 x, y, z, yaw의 4-DoF 변환을 찾음. 두 map 모두 z-up 또는 ENU 기준으로 만들어져 roll과 pitch는 자유롭게 허용하지 않음. 이 전제를 사용하면 yaw 후보를 두 점으로 계산할 수 있어, full 6-DoF보다 더 적은 correspondence로 후보를 만들고 불필요한 기울기 해를 줄일 수 있음. 2-point RANSAC은 여러 후보를 만들고, 실제 geometry에 대한 truncated MSE와 overlap이 좋은 후보를 선택함.

[P00988 | 2084:2346 | NORMAL_TEXT]
icp.py는 global_align.py가 만든 초기 변환을 받아 point-to-plane ICP를 두 해상도에서 실행함. coarse voxel에서는 큰 오차를 먼저 줄이고, fine voxel에서는 더 작은 구조 차이를 조정함. point-to-plane 방식은 대응점 사이의 3차원 거리 전체보다 target surface의 normal 방향 오차를 줄이므로, 벽이나 바닥처럼 평면 구조가 많은 map에서 표면이 미끄러지듯 맞춰지는 현상을 줄이는 데 유리함.

[P00989 | 2346:2687 | NORMAL_TEXT]
ICP가 수렴했다는 이유만으로 결과를 사용하지는 않음. icp.py는 일정 거리 안에서 실제로 대응된 점의 비율인 overlap과 대응점 오차의 제곱 평균인 truncated MSE를 계산함. 전체 map이 서로 겹치지 않아도 일부 영역만 겹칠 수 있으므로, 단순 fitness 하나만 보면 겹치는 양과 정합 정확도를 구분하기 어려움. truncated MSE는 일정 거리 밖의 correspondence를 제외하고 겹치는 영역의 오차만 측정하며, overlap은 그 결과를 지지하는 점이 충분한지 확인함. 이 두 값을 함께 써야 “작은 영역에 우연히 맞은 결과”를 정합 결과로 채택하는 일을 줄일 수 있음.

[P00990 | 2687:2688 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00991 | 2688:2722 | NORMAL_TEXT | LIST id=kix.d1nenzvjuub9 level=0]
연속 keyframe을 submap으로 묶어 계산량을 줄임.

[P00992 | 2722:2950 | NORMAL_TEXT]
submaps.py는 한 session의 연속 keyframe을 일정 개수씩 묶음. 기본은 15개 keyframe이고, bootstrap residual이 큰 session은 10개씩 더 짧게 나눔. 각 scan은 묶음의 중앙 keyframe frame으로 옮긴 뒤 80 m range crop과 0.10 m voxel reduction을 적용함. 결과는 sm_0000.pcd와 같은 submap PCD임.

[P00993 | 2950:3220 | NORMAL_TEXT]
submap은 수백만 개의 raw point를 graph node로 직접 비교하지 않도록 점 수를 줄인 계산 단위임. 인접 scan의 local 구조는 유지하며, session 내부에서 pose가 크게 흔들리는 구간은 더 작은 submap으로 나누어 서로 다른 오차가 한 submap에 섞이지 않게 함. 최종 map은 최적화가 끝난 뒤 raw scan으로 다시 렌더링하므로 submap PCD를 직접 사용하지 않음. submap은 정합용 중간 표현이며 최종 geometry가 아님.

[P00994 | 3220:3433 | NORMAL_TEXT]
graph geometry 입력이 없는 rigid_only session은 여기서 제외함. 이 session은 저장된 rigid map과 tag 정보를 갖지만 raw scan trajectory가 없으므로, 일반 session처럼 submap과 odometry chain을 만들 수 없음. 대신 후속 metadata.py에서 tag anchor를 map2에 재정합하는 데 사용함.

[P00995 | 3433:3434 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P00996 | 3434:3457 | NORMAL_TEXT | LIST id=kix.d1nenzvjuub9 level=0]
source 사이의 상대 제약을 측정함.

[P00997 | 3457:3723 | NORMAL_TEXT]
measure_edges.py는 submap 하나를 graph node 하나로 보고, node 사이의 상대 pose를 edge로 계산함. 먼저 같은 session에서 인접 submap을 연결하는 odometry edge를 만듦. 이 edge는 각 session의 원래 pose trajectory에서 계산되므로, 짧은 구간의 연속성을 보존함. 다만 odometry edge만 연결하면 session 내부는 이어져도 서로 다른 session 사이의 위치 관계는 보장되지 않음.

[P00998 | 3723:3935 | NORMAL_TEXT]
measure_edges.py는 submap 중심 위치를 KD-tree로 검색해 서로 가까운 후보를 만듦. 같은 session에서 일정 간격 이상 떨어진 후보는 intra-session loop 후보로, 다른 session 사이의 후보는 inter-session 후보로 분류함. 공간적으로 가까운 pair만 검사해 계산량을 제한하고 실제로 겹칠 가능성이 있는 pair에 집중함.

[P00999 | 3935:4226 | NORMAL_TEXT]
후보 pair마다 point-to-plane ICP를 실행하고, transform, information matrix, fitness, RMSE, source point 수를 저장함. information matrix는 해당 edge가 어느 방향의 pose를 얼마나 강하게 지지하는지를 graph optimizer에 전달함. 결과는 edges.npz와 node 목록인 nodes.json으로 저장됨. 약 2,500개의 ICP 측정을 별도 cache해 이후 gate 기준만 바꾸어 graph 최적화를 반복할 수 있게 함.

[P01000 | 4226:4227 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01001 | 4227:4259 | NORMAL_TEXT | LIST id=kix.d1nenzvjuub9 level=0]
pose graph를 최적화해 전체 drift를 분산함.

[P01002 | 4259:4605 | NORMAL_TEXT]
optimize.py는 먼저 edges.npz의 edge를 gate함. 현재 기준은 odometry edge는 유지하고, 그 외 edge는 RMSE 0.13 m 이하, fitness 0.15 이상, correspondence support 1,500점 이상, bootstrap seed에서의 translation deviation 1.0 m 이하를 요구함. 반복적인 barn bay나 대칭 구조에서는 ICP가 실제 위치가 아닌 다른 구조에 수렴할 수 있음. 이런 잘못된 edge는 숫자상으로는 서로 잘 맞는 것처럼 보일 수 있어, RMSE와 support만이 아니라 초기 위치에서 지나치게 멀리 이동했는지도 함께 봄.

[P01003 | 4605:4912 | NORMAL_TEXT]
gate 후에도 graph가 모든 session을 연결하지 못할 수 있음. optimize.py는 session 사이의 가장 가까운 node를 골라 spanning tree 형태의 virtual edge를 추가함. virtual edge는 정밀한 측정값이 아니라 bootstrap placement를 약하게 연결하는 역할임. 이 연결로 고립된 graph component를 하나의 전역 map 안에서 최적화할 수 있음. virtual edge의 weight를 낮게 두어 실제 ICP edge가 있는 곳에서는 측정된 geometry가 우선하도록 함.

[P01004 | 4912:5187 | NORMAL_TEXT]
그 다음 Open3D의 pose graph global optimization이 모든 node pose를 동시에 조정함. pair별 ICP 결과를 실행 순서대로 누적하면 한 구간의 작은 오차가 다음 구간으로 계속 전달되어 drift가 커짐. pose graph는 odometry, intra-session loop, inter-session loop를 한꺼번에 사용해 서로 충돌하는 제약의 오차를 전체 graph에 분산함. 결과는 최적화된 submap pose를 담은 graph.json임.

[P01005 | 5187:5188 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01006 | 5188:5218 | NORMAL_TEXT | LIST id=kix.d1nenzvjuub9 level=0]
최적화된 pose로 raw scan을 다시 렌더링함.

[P01007 | 5218:5516 | NORMAL_TEXT]
render.py는 graph.json의 submap pose와 원래 keyframe pose를 결합해 각 raw scan을 map frame으로 옮김. 이때 range crop을 적용하고, submap에서 이미 줄어든 점을 다시 사용하는 대신 raw scan을 읽음. submap을 만들 때의 0.10 m reduction과 local grouping은 최종 geometry에 누적하지 않음. submap은 pose를 계산하기 위한 중간 표현이며, 최종 PCD는 최적화된 pose로 raw observation을 다시 쌓아 만듦.

[P01008 | 5516:5749 | NORMAL_TEXT]
render.py는 두 종류의 geometry set을 만듦. map2는 graph에 들어간 모든 session을 사용해 coverage를 우선함. map1은 inter-session evidence가 부족하거나 내부 drift를 graph가 해결하지 못한 session을 제외해 정합 신뢰도를 우선함. 따라서 map1과 map2는 같은 optimizer를 사용하지만 포함하는 source 범위와 선택 목적이 다름.

[P01009 | 5749:5897 | NORMAL_TEXT]
이 단계에서 map2 전체를 사용해 dominant plane을 한 번 추정하고, 필요한 경우 map1과 map2에 적용할 초기 leveling transform도 계산함. 다만 이 한 번의 추정은 다음 단계 fix_leveling.py에서 다시 정밀하게 보정함.

[P01010 | 5897:5898 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01011 | 5898:5929 | NORMAL_TEXT | LIST id=kix.d1nenzvjuub9 level=0]
두 PCD를 같은 voxel grid에서 merge함.

[P01012 | 5929:6198 | NORMAL_TEXT]
두 PCD A와 B를 merge한다고 하면, 먼저 B의 모든 점에 정합 transform T_B→A를 적용해 A의 frame으로 옮김. 그 다음 A와 변환된 B의 점을 하나의 배열로 합쳐 동일한 voxel grid에 넣음. voxel index는 각 좌표를 voxel 크기로 나눈 뒤 floor한 정수 좌표로 계산함. 같은 voxel에 들어온 점들의 x, y, z를 평균해 대표점 하나를 만들므로, 겹치는 영역의 중복 점은 줄고 서로 다른 점이 같은 공간을 채우는 구조는 유지됨.

[P01013 | 6198:6468 | NORMAL_TEXT]
두 PCD를 같은 voxel grid에서 처리하면 동일 표면의 중복 점을 줄이고 session마다 다른 점 밀도와 noise의 영향을 완화할 수 있음. centroid는 같은 voxel 안의 여러 관측을 하나의 위치로 요약해 점 밀도를 일정하게 만들고, 큰 파일을 관리 가능한 크기로 줄임. intensity가 있으면 같은 voxel 안의 intensity도 평균하며, 색상이나 tag overlay처럼 평균하면 안 되는 데이터는 voxel_reduce의 별도 처리 규칙을 사용함.

[P01014 | 6468:6776 | NORMAL_TEXT]
현재 remerge render.py와 drone_merge.py는 여러 cloud를 하나로 쌓은 뒤 voxel_reduce를 직접 호출함. 두 cloud를 이미 같은 frame에 둔 뒤 centroid merge를 수행하는 일반 구현은 voxel_merge에 정리되어 있음. remerge의 원본 merge voxel은 0.05 m, compressed variant는 다시 0.15 m voxel로 줄임. 따라서 compressed PCD는 새로운 정합 결과가 아니라 같은 pose와 geometry를 더 큰 voxel로 표현한 variant임.

[P01015 | 6776:6777 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01016 | 6777:6820 | NORMAL_TEXT | LIST id=kix.d1nenzvjuub9 level=0]
leveling, metadata, drone 결합을 후속 단계에서 처리함.

[P01017 | 6820:6907 | NORMAL_TEXT]
이 단계는 하나의 스크립트가 아니라, LiDAR 결과를 안정적인 map product로 만들고 drone data를 필요한 영역에만 추가하는 후속 과정임.

[P01018 | 6907:7136 | NORMAL_TEXT | LIST id=kix.d1nenzvjuub9 level=1]
fix_leveling.py는 왜 plane을 찾고 보정하는가: render.py에서 만든 map에는 session별 작은 roll 또는 pitch 오차가 남을 수 있음. map이 수백 m까지 넓으면 0.1도 정도의 기울기도 거리에 따라 수십 cm의 높이 차이로 커짐. 바닥이나 지붕처럼 여러 session에서 공통으로 관측되는 넓은 평면을 기준으로 삼으면, 이 작은 기울기를 map 전체에 적용할 수 있음.

[P01019 | 7136:7518 | NORMAL_TEXT]
먼저 RANSAC이 무작위로 세 점을 골라 plane n·p + d = 0 후보를 만들고, 일정 거리 안에 들어오는 점이 많은 후보를 선택함. 실제 map에는 나무, 벽, 차량, 내부 구조물 같은 outlier가 섞여 있으므로 모든 점을 한 번에 평면 fitting하면 dominant plane이 아닌 구조물 쪽으로 결과가 끌릴 수 있음. RANSAC으로 inlier를 찾은 뒤에는 SVD로 그 inlier들의 중심을 기준으로 가장 평평한 plane의 normal을 다시 계산함. SVD는 inlier point cloud의 분산이 가장 작은 방향을 찾아 plane normal로 사용하므로, 무작위 세 점으로 만든 RANSAC 후보보다 안정적인 평균 plane을 얻을 수 있음.

[P01020 | 7518:7708 | NORMAL_TEXT]
마지막으로 plane normal을 z축으로 회전하고 plane이 z=0에 오도록 이동 transform을 만듦. 이 transform을 map1과 map2 양쪽에 똑같이 적용해 두 결과가 서로 다른 높이 frame을 갖지 않게 함. fix_leveling.py는 이 보정 후의 plane을 다시 측정해 residual tilt도 기록함.

[P01021 | 7708:7709 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01022 | 7709:7981 | NORMAL_TEXT | LIST id=kix.d1nenzvjuub9 level=1]
metadata.py는 왜 필요한가: PCD의 점만으로는 이 map이 어떤 session에서 만들어졌는지, point count와 bounds가 무엇인지, tag가 어디에 있는지, georeference가 실제 측량인지 inherited transform인지 알 수 없음. metadata.py는 map1, map2, map3의 YAML에 PCD 파일명, origin, bounds, point count, tag anchor, remerge source와 quality 값을 기록함.

[P01023 | 7981:8314 | NORMAL_TEXT]
tag anchor는 optimized keyframe pose와 tag observation을 결합해 map frame의 tag pose로 다시 계산함. raw scan이 없는 rigid_only session의 tag는 map2 PCD와 별도 ICP로 재정합한 뒤 사용함. map3은 map2 geometry에 T_f4_new와 final4의 ECEF datum을 이어 붙여 georeferenced metadata를 만드는 variant임. map3의 georeference는 독립 survey가 아니라 final4 datum을 map-to-map registration으로 전달한 값임.

[P01024 | 8314:8481 | NORMAL_TEXT]
geometry를 다시 만들지 않고 metadata의 자세만 수정해야 할 때는 fix_datum_attitude.py가 map3 config의 datum attitude를 보정함. 이 스크립트는 --config를 입력으로 받고 선택적으로 --out을 사용해 결과 config를 별도 파일로 저장함.

[P01025 | 8481:8482 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01026 | 8482:8878 | NORMAL_TEXT | LIST id=kix.d1nenzvjuub9 level=1]
drone_check2.py는 왜 별도 seed가 필요한가: LiDAR와 photogrammetry cloud는 점 밀도와 표면 표현이 다르므로 LiDAR용 FPFH correspondence를 그대로 적용하기 어려움. 이 스크립트는 두 cloud를 4 m coarse occupancy grid로 바꾸고, drone footprint를 5도 간격으로 회전시키며 FFT correlation으로 XY 위치와 yaw 후보를 찾음. 이후 2 m cell의 top-surface 높이를 사용해 z offset을 계산함. occupancy는 높이와 점 density의 세부 차이를 버리고 “어디에 구조가 있는가”만 비교하므로 서로 다른 sensor modality 사이에서도 coarse seed를 만들 수 있음.

[P01027 | 8878:9145 | NORMAL_TEXT]
상위 후보에 대해서는 cell별 top surface 높이 차이로 z offset을 계산하고, multiscale ICP로 transform을 정밀화함. 마지막에는 point-to-point ICP with scaling으로 photogrammetry scale 차이도 확인한 뒤, 필요한 경우 scale을 반영하고 rigid ICP를 다시 실행함. 결과 work/drone_T.json은 drone map을 LiDAR map2 frame으로 옮기는 전역 transform임.

[P01028 | 9145:9268 | NORMAL_TEXT]
같은 디렉터리의 drone_check.py는 이 transform을 저장하는 본 처리보다 앞서 drone과 LiDAR의 feasibility, global alignment, ICP 결과를 확인하는 진단용 스크립트임.

[P01029 | 9268:9671 | NORMAL_TEXT | LIST id=kix.d1nenzvjuub9 level=1]
drone_constraints.py와 optimize_with_priors.py는 왜 필요한가: 전역 transform 하나만 적용하면 drone map과 LiDAR map 사이의 지역별 drift를 해결할 수 없음. drone_constraints.py는 vegetation을 excess-green filter로 제거한 뒤 각 LiDAR submap 주변의 drone structure를 찾아 local ICP correction을 계산함. 나무와 잎은 LiDAR 구조와 대응하지 않아 ICP를 잘못된 방향으로 끌 수 있음. support point 수, fitness, correction 크기, vertical correction 크기를 검사해 신뢰할 수 없는 correction은 prior로 사용하지 않음.

[P01030 | 9671:10036 | NORMAL_TEXT]
optimize_with_priors.py는 이 correction들을 pose graph의 prior edge로 넣어 graph를 다시 최적화함. 이때 drone prior는 주로 XY와 yaw를 지지하고, z와 roll/pitch는 LiDAR solution을 유지하도록 축별 weight를 나눔. drone은 수평 footprint를 넓게 제공하지만 photogrammetry의 높이와 기울기는 식생, phantom floor, reconstruction noise에 영향을 받을 수 있기 때문임. 결과 graph는 기존 LiDAR graph의 local rigidity를 유지하면서 필요한 수평 drift만 drone shape 쪽으로 보정함.

[P01031 | 10036:10037 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01032 | 10037:10306 | NORMAL_TEXT | LIST id=kix.d1nenzvjuub9 level=1]
drone_thresholds.py는 왜 tile별로 검증하는가: drone과 LiDAR의 정합 오차는 map 전체에서 일정하다고 가정할 수 없음. 그래서 overlap 영역을 30 m tile로 나누고 각 tile에서 local ICP를 실행함. 충분한 상호 구조가 있고 local misalignment가 측정된 tile만 verified로 분류함. 구조가 부족한 tile은 unverifiable, LiDAR가 없는 drone-only 영역은 frontier로 따로 기록함.

[P01033 | 10306:10495 | NORMAL_TEXT]
검증할 LiDAR 구조가 없는 영역은 “정합 오차가 작다”고 판단할 수 없음. tau는 verified tile의 local misalignment 허용값임. 예를 들어 tau_030은 0.30 m 이하로 확인된 tile만 사용하고, tau_100은 1.00 m 이하까지 허용함. 결과는 work/drone_tiles.json에 저장됨.

[P01034 | 10495:10496 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01035 | 10496:10864 | NORMAL_TEXT | LIST id=kix.d1nenzvjuub9 level=1]
drone_merge.py는 왜 LiDAR를 base로 두고 fill만 추가하는가: map2 LiDAR geometry를 기준으로 유지하고, drone point는 LiDAR point가 0.40 m 안에 없는 공간에만 추가함. 같은 표면에 두 sensor의 점을 모두 쌓으면 표면이 이중으로 생기거나 localization에 불필요한 두께가 생길 수 있음. verified tile은 local ICP correction을 적용한 뒤 fill로 사용하고, LiDAR가 없는 frontier는 별도 조건으로 추가함. drone reconstruction의 phantom floor처럼 local LiDAR ground보다 2.0 m 이상 낮은 점은 제거함.

[P01036 | 10864:11009 | NORMAL_TEXT]
drone_merge.py는 이 정책을 tau_030, tau_050, tau_100 각각에 적용해 세 결과를 만듦. threshold가 커질수록 더 많은 tile이 들어갈 수 있지만, point count 증가가 전체 map 정확도의 증가를 뜻하지는 않음.

[P01037 | 11009:11156 | NORMAL_TEXT]
colorize_merge.py는 이 geometry에 색상을 입힌 representation variant를 만들 뿐, 새로운 pose 정합이나 merge 판단을 수행하지 않음. 실제 위치: scripts/georef_align/colorize_merge.py

[P01038 | 11156:11157 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01039 | 11157:11427 | NORMAL_TEXT | LIST id=kix.d1nenzvjuub9 level=1]
interior_transplant.py와 rigidified 결과: interior_transplant.py는 tau_100 뒤에 LiDAR가 충분히 관측하지 못한 interior 영역을 보강하는 후속 결과를 만듦. 이후 inventory의 rigidified는 이 결과를 바탕으로 저장된 후속 map product임. 현재 merge 실행 경로에서 interior 보강을 수행하는 실제 스크립트는 scripts/georef_align/interior_transplant.py임.

[P01040 | 11427:11433 | HEADING_2]
핵심 요약

[P01041 | 11433:11498 | NORMAL_TEXT | LIST id=kix.8509kypu1bzk level=0]
현재 canonical map은 source session 14개와 complete product 10개로 분류됨.

[P01042 | 11498:11615 | NORMAL_TEXT | LIST id=kix.8509kypu1bzk level=0]
source session 중 3개는 partial scan only이고, 1개는 rigid map only임. 이 4개는 complete operational map으로 세지 않고 계보의 입력으로만 관리함.

[P01043 | 11615:11704 | NORMAL_TEXT | LIST id=kix.8509kypu1bzk level=0]
remerge map1과 map2는 현재 LiDAR pose-graph 재렌더 결과이며, map3는 georeferenced metadata variant임.

[P01044 | 11704:11783 | NORMAL_TEXT | LIST id=kix.8509kypu1bzk level=0]
tau 030, 050, 100은 map2 LiDAR에 drone fill을 overlap threshold별로 추가한 composite임.

[P01045 | 11783:11873 | NORMAL_TEXT | LIST id=kix.8509kypu1bzk level=0]
2026-08-18 interior-transplant와 rigidified는 tau_100 이후의 interior 보강 및 rigidification 결과임.

[P01046 | 11873:12018 | NORMAL_TEXT | LIST id=kix.8509kypu1bzk level=0]
현재 즉시 확인해야 할 불일치는 map/2_remerge/map3_original.pcd의 실제 PCD point count 27,842,980과 map3_config.yaml의 metadata point count 22,481,984가 서로 다르다는 점임.

[P01047 | 12018:12029 | HEADING_2]
1. 인벤토리 기준

[P01048 | 12029:12039 | HEADING_3]
Valid의 정의

[P01049 | 12039:12085 | NORMAL_TEXT]
이 문서에서 valid map은 다음 기준을 만족하는 canonical PCD임.

[P01050 | 12085:12086 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01051 | 12086:12111 | NORMAL_TEXT | LIST id=kix.zdaechxvvkqs level=0]
현재 worktree의 실제 파일이 존재함.

[P01052 | 12111:12159 | NORMAL_TEXT | LIST id=kix.zdaechxvvkqs level=0]
PCD header가 해석 가능하고, POINTS, FIELDS, DATA가 확인됨.

[P01053 | 12159:12200 | NORMAL_TEXT | LIST id=kix.zdaechxvvkqs level=0]
complete product는 대응 metadata YAML이 존재함.

[P01054 | 12200:12279 | NORMAL_TEXT | LIST id=kix.zdaechxvvkqs level=0]
동일 map의 compressed, colored, tag-overlay 파일은 별도 map으로 중복 집계하지 않고 variant로 기록함.

[P01055 | 12279:12374 | NORMAL_TEXT | LIST id=kix.zdaechxvvkqs level=0]
partial scan only와 rigid only는 파일 자체가 유효하더라도 complete operational map이 아닌 lineage input으로 분류함.

[P01056 | 12374:12375 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01057 | 12375:12530 | NORMAL_TEXT]
세 raw scan only session은 metadata YAML이 없어 PCD filename과 session directory UUID로 식별하였음. 따라서 이 세 항목에는 metadata 기반 georeference 또는 point-count 검증을 적용할 수 없음.

[P01058 | 12530:12539 | HEADING_3]
수치 계산 방법

[P01059 | 12539:12585 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
대상 파일은 canonical full-resolution binary PCD임.

[P01060 | 12585:12638 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
point count는 PCD header의 POINTS와 payload 순회 결과를 사용함.

[P01061 | 12638:12684 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
XYZ bounds는 binary payload를 chunk 단위로 읽어 계산함.

[P01062 | 12684:12744 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
XY bbox area는 XY 전체 bounding box 면적이며 실제 점유 면적이나 지형 면적이 아님.

[P01063 | 12744:12842 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
z p01, p50, p99는 전체 payload에서 결정적인 stride로 추출한 finite Z sample의 백분위임. 높이 정확도나 datum 정확도를 의미하지 않음.

[P01064 | 12842:12871 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
원본을 수정하지 않고 SHA-256을 함께 기록함.

[P01065 | 12871:12872 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01066 | 12872:12888 | NORMAL_TEXT]
재생성 명령은 다음과 같음.

[P01067 | 12888:12889 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01068 | 12889:12901 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@12889:12890 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧cd backend

[P01069 | 12901:12975 | NORMAL_TEXT]
PYTHONPATH=$PWD uv run python scripts/build_map_handoff.py --repo-root ..

[P01070 | 12975:12977 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@12975:12976 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P01071 | 12977:13117 | NORMAL_TEXT]
옵션은 --out-dir PATH, --no-measure임. --no-measure는 payload를 읽지 않고 header와 provenance만 다시 작성할 때 사용함. 생성기 실제 위치는 [scripts/build_map_handoff.py](https://drive.google.com/file/d/1OB6b4lxpO7HZU55yw_8w4y3OB4cHBQk4/view?usp=drivesdk)임.

[P01072 | 13117:13126 | HEADING_2]
2. 계보 개요

[P01073 | 13126:13128 | NORMAL_TEXT]
[INLINE_OBJECT kix.10df572t11h3]

[P01074 | 13128:13129 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01075 | 13129:13279 | NORMAL_TEXT]
이 플롯은 map의 성능 순위가 아니라 생성 순서를 보여줌. 상자는 canonical map product, 화살표는 부모 map에서 파생된 관계임. 왼쪽에서 오른쪽으로 source와 bootstrap, remerge, drone fill, 후속 보강 결과가 이어짐.

[P01076 | 13279:13280 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01077 | 13280:13307 | NORMAL_TEXT]
계보를 실행 경로 관점에서 읽으면 다음과 같음.

[P01078 | 13307:13308 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01079 | 13308:13347 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@13308:13309 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧14 source sessions + final4 bootstrap

[P01080 | 13347:13357 | NORMAL_TEXT]
        |

[P01081 | 13357:13401 | NORMAL_TEXT]
        +--> remerge map1, precision subset

[P01082 | 13401:13411 | NORMAL_TEXT]
        |

[P01083 | 13411:13467 | NORMAL_TEXT]
        +--> remerge map2, maximum raw-session coverage

[P01084 | 13467:13534 | NORMAL_TEXT]
        |       +--> tau 030 / tau 050 / tau 100, drone gated fill

[P01085 | 13534:13544 | NORMAL_TEXT]
        |

[P01086 | 13544:13602 | NORMAL_TEXT]
        +--> remerge map3, georeferenced metadata variant

[P01087 | 13602:13651 | NORMAL_TEXT]
                +--> tau 030 / tau 050 / tau 100

[P01088 | 13651:13652 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01089 | 13652:13699 | NORMAL_TEXT]
tau 100 --> interior-transplant --> rigidified

[P01090 | 13699:13701 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@13699:13700 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P01091 | 13701:13886 | NORMAL_TEXT]
final4는 remerge의 초기 anchor와 datum reference임. 최종 map의 point content가 final4에서 단순 복사된다는 뜻은 아님. drone-20260630은 LiDAR map과 별개의 photogrammetry reference이며 tau products에서 fill source로 사용됨.

[P01092 | 13886:13907 | HEADING_2]
3. Source session 목록

[P01093 | 13907:14085 | NORMAL_TEXT]
complete_source는 기록된 session map으로서 complete lineage의 후보임. partial_scan_only는 raw scan과 pose source만 있는 항목임. rigid_only는 tag anchor에 사용되는 rigid map으로, graph geometry 입력으로 넣지 않음.

[P01094 | 14085:14086 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01095 | 14086:14088 | NORMAL_TEXT]
[INLINE_OBJECT kix.woeljymms6jt]

[P01096 | 14088:14089 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01097 | 14089:14134 | NORMAL_TEXT]
이 플롯은 14개 source session의 저장 규모와 높이 범위를 비교함.

[P01098 | 14134:14135 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01099 | 14135:14208 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
Point count (million): PCD에 저장된 점 개수임. 저장 규모와 관측량을 보는 값이며 정합 품질을 뜻하지 않음.

[P01100 | 14208:14298 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
XY bounding-box area (ha): XY 전체 범위를 직사각형으로 감싼 면적임. 실제 점유 면적이 아니라 session footprint의 상한임.

[P01101 | 14298:14435 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
Z extent, p01 to p99 (m): 1 percentile부터 99 percentile까지의 Z 범위임. 선은 범위, 검은 점은 중앙값임. session 간 frame이나 관측 구역이 다르므로 절대고도 정확도 비교에는 사용하지 않음.

[P01102 | 14435:14436 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01103 | 14439:14442 | NORMAL_TEXT | TABLE row=0 col=0]
ID

[P01104 | 14443:14446 | NORMAL_TEXT | TABLE row=0 col=1]
상태

[P01105 | 14447:14467 | NORMAL_TEXT | TABLE row=0 col=2]
실제 canonical PCD 위치

[P01106 | 14468:14475 | NORMAL_TEXT | TABLE row=0 col=3]
points

[P01107 | 14476:14487 | NORMAL_TEXT | TABLE row=0 col=4]
XY bbox ha

[P01108 | 14488:14508 | NORMAL_TEXT | TABLE row=0 col=5]
Z p01 / p50 / p99 m

[P01109 | 14509:14512 | NORMAL_TEXT | TABLE row=0 col=6]
역할

[P01110 | 14514:14530 | NORMAL_TEXT | TABLE row=1 col=0]
source-d52c7fd6

[P01111 | 14531:14547 | NORMAL_TEXT | TABLE row=1 col=1]
complete_source

[P01112 | 14548:14679 | NORMAL_TEXT | TABLE row=1 col=2]
map/0_source-sessions/2026-08-08-05-12-42-d52c7fd6-3778-4c7d-b8e1-e85419a75eb5/global_map_d52c7fd6-3778-4c7d-b8e1-e85419a75eb5.pcd

[P01113 | 14680:14690 | NORMAL_TEXT | TABLE row=1 col=3]
1,511,617

[P01114 | 14691:14697 | NORMAL_TEXT | TABLE row=1 col=4]
2.316

[P01115 | 14698:14723 | NORMAL_TEXT | TABLE row=1 col=5]
-8.009 / -0.895 / 10.286

[P01116 | 14724:14742 | NORMAL_TEXT | TABLE row=1 col=6]
기록 session source

[P01117 | 14744:14760 | NORMAL_TEXT | TABLE row=2 col=0]
source-22d059d5

[P01118 | 14761:14777 | NORMAL_TEXT | TABLE row=2 col=1]
complete_source

[P01119 | 14778:14909 | NORMAL_TEXT | TABLE row=2 col=2]
map/0_source-sessions/2026-08-08-07-55-40-22d059d5-1e1b-4b2e-bc31-dff7a131e56d/global_map_22d059d5-1e1b-4b2e-bc31-dff7a131e56d.pcd

[P01120 | 14910:14920 | NORMAL_TEXT | TABLE row=2 col=3]
5,127,250

[P01121 | 14921:14927 | NORMAL_TEXT | TABLE row=2 col=4]
1.759

[P01122 | 14928:14953 | NORMAL_TEXT | TABLE row=2 col=5]
-0.403 / 10.256 / 15.247

[P01123 | 14954:14972 | NORMAL_TEXT | TABLE row=2 col=6]
기록 session source

[P01124 | 14974:14990 | NORMAL_TEXT | TABLE row=3 col=0]
source-3d41c468

[P01125 | 14991:15007 | NORMAL_TEXT | TABLE row=3 col=1]
complete_source

[P01126 | 15008:15139 | NORMAL_TEXT | TABLE row=3 col=2]
map/0_source-sessions/2026-08-08-08-08-08-3d41c468-8975-420a-bf8f-5eca83280348/global_map_3d41c468-8975-420a-bf8f-5eca83280348.pcd

[P01127 | 15140:15150 | NORMAL_TEXT | TABLE row=3 col=3]
3,178,423

[P01128 | 15151:15157 | NORMAL_TEXT | TABLE row=3 col=4]
2.111

[P01129 | 15158:15182 | NORMAL_TEXT | TABLE row=3 col=5]
-0.576 / 9.076 / 14.582

[P01130 | 15183:15201 | NORMAL_TEXT | TABLE row=3 col=6]
기록 session source

[P01131 | 15203:15219 | NORMAL_TEXT | TABLE row=4 col=0]
source-8d5094de

[P01132 | 15220:15236 | NORMAL_TEXT | TABLE row=4 col=1]
complete_source

[P01133 | 15237:15368 | NORMAL_TEXT | TABLE row=4 col=2]
map/0_source-sessions/2026-08-08-08-34-30-8d5094de-725f-4103-a59f-a9c0f126a64d/global_map_8d5094de-725f-4103-a59f-a9c0f126a64d.pcd

[P01134 | 15369:15379 | NORMAL_TEXT | TABLE row=4 col=3]
2,624,207

[P01135 | 15380:15386 | NORMAL_TEXT | TABLE row=4 col=4]
1.893

[P01136 | 15387:15411 | NORMAL_TEXT | TABLE row=4 col=5]
-0.583 / 9.128 / 14.551

[P01137 | 15412:15430 | NORMAL_TEXT | TABLE row=4 col=6]
기록 session source

[P01138 | 15432:15448 | NORMAL_TEXT | TABLE row=5 col=0]
source-2faf6b7c

[P01139 | 15449:15465 | NORMAL_TEXT | TABLE row=5 col=1]
complete_source

[P01140 | 15466:15597 | NORMAL_TEXT | TABLE row=5 col=2]
map/0_source-sessions/2026-08-08-08-41-00-2faf6b7c-3e88-45fa-9120-2f1355ab1384/global_map_2faf6b7c-3e88-45fa-9120-2f1355ab1384.pcd

[P01141 | 15598:15608 | NORMAL_TEXT | TABLE row=5 col=3]
1,595,180

[P01142 | 15609:15615 | NORMAL_TEXT | TABLE row=5 col=4]
3.890

[P01143 | 15616:15640 | NORMAL_TEXT | TABLE row=5 col=5]
-0.371 / 4.251 / 18.605

[P01144 | 15641:15659 | NORMAL_TEXT | TABLE row=5 col=6]
기록 session source

[P01145 | 15661:15677 | NORMAL_TEXT | TABLE row=6 col=0]
source-2328a659

[P01146 | 15678:15694 | NORMAL_TEXT | TABLE row=6 col=1]
complete_source

[P01147 | 15695:15826 | NORMAL_TEXT | TABLE row=6 col=2]
map/0_source-sessions/2026-08-08-08-51-13-2328a659-f7c4-4ee0-aad8-62d0d968796b/global_map_2328a659-f7c4-4ee0-aad8-62d0d968796b.pcd

[P01148 | 15827:15835 | NORMAL_TEXT | TABLE row=6 col=3]
832,254

[P01149 | 15836:15842 | NORMAL_TEXT | TABLE row=6 col=4]
2.586

[P01150 | 15843:15867 | NORMAL_TEXT | TABLE row=6 col=5]
-0.977 / 1.667 / 12.477

[P01151 | 15868:15886 | NORMAL_TEXT | TABLE row=6 col=6]
기록 session source

[P01152 | 15888:15904 | NORMAL_TEXT | TABLE row=7 col=0]
source-602633da

[P01153 | 15905:15923 | NORMAL_TEXT | TABLE row=7 col=1]
partial_scan_only

[P01154 | 15924:16055 | NORMAL_TEXT | TABLE row=7 col=2]
map/0_source-sessions/2026-08-08-08-55-02-602633da-1139-4c37-9219-b35a49416b1b/global_map_602633da-1139-4c37-9219-b35a49416b1b.pcd

[P01155 | 16056:16066 | NORMAL_TEXT | TABLE row=7 col=3]
1,606,840

[P01156 | 16067:16073 | NORMAL_TEXT | TABLE row=7 col=4]
4.709

[P01157 | 16074:16098 | NORMAL_TEXT | TABLE row=7 col=5]
-1.782 / 2.266 / 13.319

[P01158 | 16099:16137 | NORMAL_TEXT | TABLE row=7 col=6]
raw scan and pose source, metadata 없음

[P01159 | 16139:16155 | NORMAL_TEXT | TABLE row=8 col=0]
source-ba4cf16c

[P01160 | 16156:16172 | NORMAL_TEXT | TABLE row=8 col=1]
complete_source

[P01161 | 16173:16304 | NORMAL_TEXT | TABLE row=8 col=2]
map/0_source-sessions/2026-08-08-09-03-10-ba4cf16c-11e8-4a3a-84e2-3b5bb4426e01/global_map_ba4cf16c-11e8-4a3a-84e2-3b5bb4426e01.pcd

[P01162 | 16305:16315 | NORMAL_TEXT | TABLE row=8 col=3]
1,575,605

[P01163 | 16316:16322 | NORMAL_TEXT | TABLE row=8 col=4]
3.424

[P01164 | 16323:16347 | NORMAL_TEXT | TABLE row=8 col=5]
-0.636 / 1.901 / 14.630

[P01165 | 16348:16366 | NORMAL_TEXT | TABLE row=8 col=6]
기록 session source

[P01166 | 16368:16384 | NORMAL_TEXT | TABLE row=9 col=0]
source-69a6c83f

[P01167 | 16385:16401 | NORMAL_TEXT | TABLE row=9 col=1]
complete_source

[P01168 | 16402:16533 | NORMAL_TEXT | TABLE row=9 col=2]
map/0_source-sessions/2026-08-08-09-10-24-69a6c83f-451a-44ce-b438-3a6cddd8060c/global_map_69a6c83f-451a-44ce-b438-3a6cddd8060c.pcd

[P01169 | 16534:16544 | NORMAL_TEXT | TABLE row=9 col=3]
1,222,506

[P01170 | 16545:16551 | NORMAL_TEXT | TABLE row=9 col=4]
4.507

[P01171 | 16552:16576 | NORMAL_TEXT | TABLE row=9 col=5]
-0.578 / 1.682 / 15.384

[P01172 | 16577:16595 | NORMAL_TEXT | TABLE row=9 col=6]
기록 session source

[P01173 | 16597:16613 | NORMAL_TEXT | TABLE row=10 col=0]
source-8788afd5

[P01174 | 16614:16632 | NORMAL_TEXT | TABLE row=10 col=1]
partial_scan_only

[P01175 | 16633:16764 | NORMAL_TEXT | TABLE row=10 col=2]
map/0_source-sessions/2026-08-08-09-16-10-8788afd5-e5bd-421f-a6f7-4d0612c8702f/global_map_8788afd5-e5bd-421f-a6f7-4d0612c8702f.pcd

[P01176 | 16765:16775 | NORMAL_TEXT | TABLE row=10 col=3]
1,736,467

[P01177 | 16776:16782 | NORMAL_TEXT | TABLE row=10 col=4]
7.701

[P01178 | 16783:16807 | NORMAL_TEXT | TABLE row=10 col=5]
-0.407 / 2.342 / 14.977

[P01179 | 16808:16846 | NORMAL_TEXT | TABLE row=10 col=6]
raw scan and pose source, metadata 없음

[P01180 | 16848:16864 | NORMAL_TEXT | TABLE row=11 col=0]
source-4b4ff22c

[P01181 | 16865:16881 | NORMAL_TEXT | TABLE row=11 col=1]
complete_source

[P01182 | 16882:17013 | NORMAL_TEXT | TABLE row=11 col=2]
map/0_source-sessions/2026-08-08-09-27-53-4b4ff22c-12d7-4fad-a119-e93d597a097b/global_map_4b4ff22c-12d7-4fad-a119-e93d597a097b.pcd

[P01183 | 17014:17024 | NORMAL_TEXT | TABLE row=11 col=3]
2,175,907

[P01184 | 17025:17032 | NORMAL_TEXT | TABLE row=11 col=4]
11.096

[P01185 | 17033:17058 | NORMAL_TEXT | TABLE row=11 col=5]
-0.140 / 10.200 / 26.847

[P01186 | 17059:17081 | NORMAL_TEXT | TABLE row=11 col=6]
가장 큰 source footprint

[P01187 | 17083:17099 | NORMAL_TEXT | TABLE row=12 col=0]
source-97dc233f

[P01188 | 17100:17111 | NORMAL_TEXT | TABLE row=12 col=1]
rigid_only

[P01189 | 17112:17243 | NORMAL_TEXT | TABLE row=12 col=2]
map/0_source-sessions/2026-08-09-02-10-21-97dc233f-c93c-4eb2-ba6f-a106d17cef75/global_map_97dc233f-c93c-4eb2-ba6f-a106d17cef75.pcd

[P01190 | 17244:17254 | NORMAL_TEXT | TABLE row=12 col=3]
1,221,832

[P01191 | 17255:17261 | NORMAL_TEXT | TABLE row=12 col=4]
1.384

[P01192 | 17262:17286 | NORMAL_TEXT | TABLE row=12 col=5]
-4.169 / -0.511 / 9.130

[P01193 | 17287:17323 | NORMAL_TEXT | TABLE row=12 col=6]
rigid/tag anchor, graph geometry 제외

[P01194 | 17325:17341 | NORMAL_TEXT | TABLE row=13 col=0]
source-74b187be

[P01195 | 17342:17360 | NORMAL_TEXT | TABLE row=13 col=1]
partial_scan_only

[P01196 | 17361:17492 | NORMAL_TEXT | TABLE row=13 col=2]
map/0_source-sessions/2026-08-09-23-26-51-74b187be-18b4-4ed9-9cf4-3414369b8119/global_map_74b187be-18b4-4ed9-9cf4-3414369b8119.pcd

[P01197 | 17493:17500 | NORMAL_TEXT | TABLE row=13 col=3]
55,529

[P01198 | 17501:17507 | NORMAL_TEXT | TABLE row=13 col=4]
1.551

[P01199 | 17508:17531 | NORMAL_TEXT | TABLE row=13 col=5]
-1.351 / 1.800 / 9.785

[P01200 | 17532:17570 | NORMAL_TEXT | TABLE row=13 col=6]
raw scan and pose source, metadata 없음

[P01201 | 17572:17588 | NORMAL_TEXT | TABLE row=14 col=0]
source-e0ee95c1

[P01202 | 17589:17605 | NORMAL_TEXT | TABLE row=14 col=1]
complete_source

[P01203 | 17606:17737 | NORMAL_TEXT | TABLE row=14 col=2]
map/0_source-sessions/2026-08-09-23-53-25-e0ee95c1-f82b-4214-a574-59c60e088800/global_map_e0ee95c1-f82b-4214-a574-59c60e088800.pcd

[P01204 | 17738:17746 | NORMAL_TEXT | TABLE row=14 col=3]
601,802

[P01205 | 17747:17753 | NORMAL_TEXT | TABLE row=14 col=4]
1.261

[P01206 | 17754:17778 | NORMAL_TEXT | TABLE row=14 col=5]
-0.453 / 3.511 / 12.362

[P01207 | 17779:17797 | NORMAL_TEXT | TABLE row=14 col=6]
기록 session source

[P01208 | 17798:17799 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01209 | 17799:17807 | NORMAL_TEXT]
대표값 해석:

[P01210 | 17807:17808 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01211 | 17808:17891 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
source-4b4ff22c는 source 중 XY bbox가 11.096 ha로 가장 크지만, bbox가 실제 유효 점유 영역을 보장하지는 않음.

[P01212 | 17891:17970 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
source-74b187be는 55,529 points로 가장 작음. 파일이 읽힌다는 사실만으로 complete map으로 승격하면 안 됨.

[P01213 | 17970:18052 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
source별 Z 중앙값은 서로 다른 session frame, 관측 범위, 지형 구성의 영향을 받으므로 절대고도 정확도 비교로 사용하면 안 됨.

[P01214 | 18052:18075 | HEADING_2]
4. Complete product 목록

[P01215 | 18075:18077 | NORMAL_TEXT]
[INLINE_OBJECT kix.2fg9yzgs4ze3]

[P01216 | 18077:18078 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01217 | 18078:18255 | NORMAL_TEXT]
이 플롯은 complete product 간의 저장 규모, XY 범위, Z 분포를 비교함. source plot과 같은 세 panel을 사용하므로 map의 생성 단계와 좌표 frame을 함께 확인해야 함. point count가 많거나 bbox가 넓다고 정합 또는 georeference가 더 정확하다는 뜻은 아님.

[P01218 | 18255:18256 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01219 | 18259:18262 | NORMAL_TEXT | TABLE row=0 col=0]
ID

[P01220 | 18263:18271 | NORMAL_TEXT | TABLE row=0 col=1]
상태 및 용도

[P01221 | 18272:18292 | NORMAL_TEXT | TABLE row=0 col=2]
실제 canonical PCD 위치

[P01222 | 18293:18300 | NORMAL_TEXT | TABLE row=0 col=3]
points

[P01223 | 18301:18312 | NORMAL_TEXT | TABLE row=0 col=4]
XY bbox ha

[P01224 | 18313:18333 | NORMAL_TEXT | TABLE row=0 col=5]
Z p01 / p50 / p99 m

[P01225 | 18334:18346 | NORMAL_TEXT | TABLE row=0 col=6]
metadata 상태

[P01226 | 18348:18355 | NORMAL_TEXT | TABLE row=1 col=0]
final4

[P01227 | 18356:18400 | NORMAL_TEXT | TABLE row=1 col=1]
remerge bootstrap and georeference baseline

[P01228 | 18401:18444 | NORMAL_TEXT | TABLE row=1 col=2]
map/1_final4-bootstrap/final4_original.pcd

[P01229 | 18445:18456 | NORMAL_TEXT | TABLE row=1 col=3]
11,103,601

[P01230 | 18457:18464 | NORMAL_TEXT | TABLE row=1 col=4]
25.289

[P01231 | 18465:18489 | NORMAL_TEXT | TABLE row=1 col=5]
-0.777 / 6.705 / 24.935

[P01232 | 18490:18493 | NORMAL_TEXT | TABLE row=1 col=6]
일관

[P01233 | 18495:18510 | NORMAL_TEXT | TABLE row=2 col=0]
drone-20260630

[P01234 | 18511:18542 | NORMAL_TEXT | TABLE row=2 col=1]
drone photogrammetry reference

[P01235 | 18543:18588 | NORMAL_TEXT | TABLE row=2 col=2]
map/3_drone-canonical/20260630_drone_map.pcd

[P01236 | 18589:18600 | NORMAL_TEXT | TABLE row=2 col=3]
36,150,351

[P01237 | 18601:18608 | NORMAL_TEXT | TABLE row=2 col=4]
28.296

[P01238 | 18609:18634 | NORMAL_TEXT | TABLE row=2 col=5]
12.728 / 21.764 / 36.948

[P01239 | 18635:18651 | NORMAL_TEXT | TABLE row=2 col=6]
일관, LiDAR 대체 아님

[P01240 | 18653:18666 | NORMAL_TEXT | TABLE row=3 col=0]
remerge-map1

[P01241 | 18667:18706 | NORMAL_TEXT | TABLE row=3 col=1]
precision subset operational local map

[P01242 | 18707:18739 | NORMAL_TEXT | TABLE row=3 col=2]
map/2_remerge/map1_original.pcd

[P01243 | 18740:18751 | NORMAL_TEXT | TABLE row=3 col=3]
22,430,799

[P01244 | 18752:18759 | NORMAL_TEXT | TABLE row=3 col=4]
29.480

[P01245 | 18760:18784 | NORMAL_TEXT | TABLE row=3 col=5]
-1.770 / 5.782 / 22.773

[P01246 | 18785:18788 | NORMAL_TEXT | TABLE row=3 col=6]
일관

[P01247 | 18790:18803 | NORMAL_TEXT | TABLE row=4 col=0]
remerge-map2

[P01248 | 18804:18855 | NORMAL_TEXT | TABLE row=4 col=1]
maximum raw-session coverage operational local map

[P01249 | 18856:18888 | NORMAL_TEXT | TABLE row=4 col=2]
map/2_remerge/map2_original.pcd

[P01250 | 18889:18900 | NORMAL_TEXT | TABLE row=4 col=3]
22,481,984

[P01251 | 18901:18908 | NORMAL_TEXT | TABLE row=4 col=4]
29.480

[P01252 | 18909:18933 | NORMAL_TEXT | TABLE row=4 col=5]
-1.792 / 5.726 / 22.910

[P01253 | 18934:18937 | NORMAL_TEXT | TABLE row=4 col=6]
일관

[P01254 | 18939:18952 | NORMAL_TEXT | TABLE row=5 col=0]
remerge-map3

[P01255 | 18953:18993 | NORMAL_TEXT | TABLE row=5 col=1]
estimated georeference metadata variant

[P01256 | 18994:19026 | NORMAL_TEXT | TABLE row=5 col=2]
map/2_remerge/map3_original.pcd

[P01257 | 19027:19038 | NORMAL_TEXT | TABLE row=5 col=3]
27,842,980

[P01258 | 19039:19046 | NORMAL_TEXT | TABLE row=5 col=4]
29.577

[P01259 | 19047:19071 | NORMAL_TEXT | TABLE row=5 col=5]
-0.499 / 6.945 / 22.562

[P01260 | 19072:19095 | NORMAL_TEXT | TABLE row=5 col=6]
불일치, config 22,481,984

[P01261 | 19097:19105 | NORMAL_TEXT | TABLE row=6 col=0]
tau-030

[P01262 | 19106:19134 | NORMAL_TEXT | TABLE row=6 col=1]
LiDAR plus gated drone fill

[P01263 | 19135:19271 | NORMAL_TEXT | TABLE row=6 col=2]
map/4_drone-merged/tau_030/2026-08-14-00-51-52-4a08d792-a010-4ee3-955a-0155d275c6ac/global_map_4a08d792-a010-4ee3-955a-0155d275c6ac.pcd

[P01264 | 19272:19283 | NORMAL_TEXT | TABLE row=6 col=3]
23,437,560

[P01265 | 19284:19291 | NORMAL_TEXT | TABLE row=6 col=4]
29.967

[P01266 | 19292:19316 | NORMAL_TEXT | TABLE row=6 col=5]
-2.206 / 4.794 / 23.169

[P01267 | 19317:19320 | NORMAL_TEXT | TABLE row=6 col=6]
일관

[P01268 | 19322:19330 | NORMAL_TEXT | TABLE row=7 col=0]
tau-050

[P01269 | 19331:19359 | NORMAL_TEXT | TABLE row=7 col=1]
LiDAR plus gated drone fill

[P01270 | 19360:19496 | NORMAL_TEXT | TABLE row=7 col=2]
map/4_drone-merged/tau_050/2026-08-14-00-51-52-88e44663-a472-46a3-acb0-5df1ec139470/global_map_88e44663-a472-46a3-acb0-5df1ec139470.pcd

[P01271 | 19497:19508 | NORMAL_TEXT | TABLE row=7 col=3]
24,696,836

[P01272 | 19509:19516 | NORMAL_TEXT | TABLE row=7 col=4]
29.967

[P01273 | 19517:19541 | NORMAL_TEXT | TABLE row=7 col=5]
-2.337 / 4.415 / 22.777

[P01274 | 19542:19545 | NORMAL_TEXT | TABLE row=7 col=6]
일관

[P01275 | 19547:19555 | NORMAL_TEXT | TABLE row=8 col=0]
tau-100

[P01276 | 19556:19584 | NORMAL_TEXT | TABLE row=8 col=1]
LiDAR plus gated drone fill

[P01277 | 19585:19721 | NORMAL_TEXT | TABLE row=8 col=2]
map/4_drone-merged/tau_100/2026-08-14-00-51-52-779624ad-9c1a-4a82-8660-777d76748c7e/global_map_779624ad-9c1a-4a82-8660-777d76748c7e.pcd

[P01278 | 19722:19733 | NORMAL_TEXT | TABLE row=8 col=3]
26,681,113

[P01279 | 19734:19741 | NORMAL_TEXT | TABLE row=8 col=4]
29.967

[P01280 | 19742:19766 | NORMAL_TEXT | TABLE row=8 col=5]
-2.402 / 4.394 / 22.942

[P01281 | 19767:19770 | NORMAL_TEXT | TABLE row=8 col=6]
일관

[P01282 | 19772:19792 | NORMAL_TEXT | TABLE row=9 col=0]
interior-transplant

[P01283 | 19793:19819 | NORMAL_TEXT | TABLE row=9 col=1]
tau_100 이후 interior 보강 결과

[P01284 | 19820:19955 | NORMAL_TEXT | TABLE row=9 col=2]
map/5_interior-transplant/2026-08-18-22-56-09-2bcf4e2c-02fd-4c00-adba-575a320a9abf/global_map_2bcf4e2c-02fd-4c00-adba-575a320a9abf.pcd

[P01285 | 19956:19967 | NORMAL_TEXT | TABLE row=9 col=3]
31,296,042

[P01286 | 19968:19975 | NORMAL_TEXT | TABLE row=9 col=4]
29.967

[P01287 | 19976:20000 | NORMAL_TEXT | TABLE row=9 col=5]
-2.285 / 3.218 / 22.267

[P01288 | 20001:20004 | NORMAL_TEXT | TABLE row=9 col=6]
일관

[P01289 | 20006:20017 | NORMAL_TEXT | TABLE row=10 col=0]
rigidified

[P01290 | 20018:20059 | NORMAL_TEXT | TABLE row=10 col=1]
interior-transplant 이후 rigidification 결과

[P01291 | 20060:20186 | NORMAL_TEXT | TABLE row=10 col=2]
map/6_rigidified/2026-08-18-23-41-11-56687561-4259-4f1e-b835-29c43af77a68/global_map_56687561-4259-4f1e-b835-29c43af77a68.pcd

[P01292 | 20187:20198 | NORMAL_TEXT | TABLE row=10 col=3]
31,315,242

[P01293 | 20199:20206 | NORMAL_TEXT | TABLE row=10 col=4]
29.737

[P01294 | 20207:20231 | NORMAL_TEXT | TABLE row=10 col=5]
-2.225 / 3.231 / 22.127

[P01295 | 20232:20235 | NORMAL_TEXT | TABLE row=10 col=6]
일관

[P01296 | 20236:20237 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01297 | 20237:20297 | NORMAL_TEXT]
정확한 metadata path, PCD header, SHA-256은 [maps.json](https://drive.google.com/file/d/1IqkYNW00eQJ-HdiHvwTnlIa8K2IVe-WI/view?usp=drivesdk)에 기록되어 있음.

[P01298 | 20297:20298 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01299 | 20298:20306 | NORMAL_TEXT]
대표값 해석:

[P01300 | 20306:20307 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01301 | 20307:20457 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
remerge map1과 map2는 각각 약 22.4M points, 29.480 ha bbox로 같은 LiDAR frame 규모를 보임. map1은 precision subset, map2는 maximum raw-session coverage라는 목적 차이가 있음.

[P01302 | 20457:20579 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
map3는 point count가 map2보다 약 5.36M 많지만 config의 point count는 map2와 같음. map3 생성 경로 또는 metadata 기록을 먼저 정리해야 후속 merge 판단이 가능함.

[P01303 | 20579:20712 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
tau threshold가 030에서 100으로 올라갈수록 point count가 23.44M에서 26.68M으로 증가함. 이는 accepted drone fill 범위가 넓어진 결과이지 map 전체 정확도의 단조 증가를 의미하지 않음.

[P01304 | 20712:20842 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
drone canonical의 Z percentile이 LiDAR 계열과 크게 다름. 이는 photogrammetry footprint와 frame 특성의 차이를 보여주는 대표값일 뿐, 어느 쪽이 더 정확한지 판정하는 수치가 아님.

[P01305 | 20842:20995 | NORMAL_TEXT | LIST id=kix.ycs8a6e5nae3 level=0]
interior-transplant는 tau_100 대비 point count가 약 4.61M 늘어난 후속 결과임. rigidified는 이 결과에 rigidification을 적용한 최신 단계이며 point count는 31.32M, XY bbox는 29.737 ha임.

[P01306 | 20995:21016 | HEADING_2]
5. Merge 소스코드와 실행 경로

[P01307 | 21016:21045 | HEADING_3]
5.1 Dangjin remerge pipeline

[P01308 | 21045:21292 | NORMAL_TEXT]
공통 구현은 [scripts/remerge/common.py](https://drive.google.com/file/d/1TccA92s9X0WNN8CoqXZoWbmYLRyCkDek/view?usp=drivesdk)에 있고, 모든 script는 원본 repository의 backend/에서 실행하도록 작성되어 있음. positional CLI argument는 없으며 원본 실행 환경의 remerge-output와 remerge-output/work를 고정 경로로 읽고 씀. 이 archive에서는 해당 산출물을 각각 map/2_remerge/와 map/2_remerge/work/에 보관하였음.

[P01309 | 21292:21293 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01310 | 21293:21311 | NORMAL_TEXT]
실제 실행 순서는 다음과 같음.

[P01311 | 21311:21312 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01312 | 21312:21324 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@21312:21313 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧cd backend

[P01313 | 21324:21383 | NORMAL_TEXT]
PYTHONPATH=$PWD uv run python scripts/remerge/bootstrap.py

[P01314 | 21383:21440 | NORMAL_TEXT]
PYTHONPATH=$PWD uv run python scripts/remerge/submaps.py

[P01315 | 21440:21503 | NORMAL_TEXT]
PYTHONPATH=$PWD uv run python scripts/remerge/measure_edges.py

[P01316 | 21503:21561 | NORMAL_TEXT]
PYTHONPATH=$PWD uv run python scripts/remerge/optimize.py

[P01317 | 21561:21617 | NORMAL_TEXT]
PYTHONPATH=$PWD uv run python scripts/remerge/render.py

[P01318 | 21617:21679 | NORMAL_TEXT]
PYTHONPATH=$PWD uv run python scripts/remerge/fix_leveling.py

[P01319 | 21679:21737 | NORMAL_TEXT]
PYTHONPATH=$PWD uv run python scripts/remerge/metadata.py

[P01320 | 21737:21739 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@21737:21738 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P01321 | 21742:21748 | NORMAL_TEXT | TABLE row=0 col=0]
실제 위치

[P01322 | 21749:21752 | NORMAL_TEXT | TABLE row=0 col=1]
역할

[P01323 | 21753:21763 | NORMAL_TEXT | TABLE row=0 col=2]
주요 입력과 출력

[P01324 | 21765:21794 | NORMAL_TEXT | TABLE row=1 col=0]
scripts/remerge/bootstrap.py

[P01325 | 21795:21882 | NORMAL_TEXT | TABLE row=1 col=1]
final4 compressed를 bootstrap anchor로 사용하고 source session을 global align plus ICP로 초기 정합

[P01326 | 21883:21917 | NORMAL_TEXT | TABLE row=1 col=2]
map/2_remerge/work/bootstrap.json

[P01327 | 21919:21946 | NORMAL_TEXT | TABLE row=2 col=0]
scripts/remerge/submaps.py

[P01328 | 21947:21993 | NORMAL_TEXT | TABLE row=2 col=1]
consecutive keyframe을 submap으로 렌더링하고 voxelize

[P01329 | 21994:22036 | NORMAL_TEXT | TABLE row=2 col=2]
map/2_remerge/work/submaps/, submaps.json

[P01330 | 22038:22071 | NORMAL_TEXT | TABLE row=3 col=0]
scripts/remerge/measure_edges.py

[P01331 | 22072:22127 | NORMAL_TEXT | TABLE row=3 col=1]
odom, intra, inter candidate edge를 normal plus ICP로 측정

[P01332 | 22128:22150 | NORMAL_TEXT | TABLE row=3 col=2]
edges.npz, nodes.json

[P01333 | 22152:22180 | NORMAL_TEXT | TABLE row=4 col=0]
scripts/remerge/optimize.py

[P01334 | 22181:22244 | NORMAL_TEXT | TABLE row=4 col=1]
edge gate, virtual link, Open3D pose graph global optimization

[P01335 | 22245:22256 | NORMAL_TEXT | TABLE row=4 col=2]
graph.json

[P01336 | 22258:22284 | NORMAL_TEXT | TABLE row=5 col=0]
scripts/remerge/render.py

[P01337 | 22285:22368 | NORMAL_TEXT | TABLE row=5 col=1]
optimized pose로 raw scan을 다시 렌더링하고 map1 precision subset, map2 maximum coverage 생성

[P01338 | 22369:22429 | NORMAL_TEXT | TABLE row=5 col=2]
map1_original.pcd, map2_original.pcd, render.json, T_f4_new

[P01339 | 22431:22463 | NORMAL_TEXT | TABLE row=6 col=0]
scripts/remerge/fix_leveling.py

[P01340 | 22464:22511 | NORMAL_TEXT | TABLE row=6 col=1]
map1/map2에 shared plane RANSAC/SVD leveling 적용

[P01341 | 22512:22537 | NORMAL_TEXT | TABLE row=6 col=2]
render.json과 map pose 보정

[P01342 | 22539:22567 | NORMAL_TEXT | TABLE row=7 col=0]
scripts/remerge/metadata.py

[P01343 | 22568:22631 | NORMAL_TEXT | TABLE row=7 col=1]
map1/map2/map3 metadata, tag anchor, map3 georef provenance 작성

[P01344 | 22632:22659 | NORMAL_TEXT | TABLE row=7 col=2]
map*_config.yaml, metadata

[P01345 | 22661:22695 | NORMAL_TEXT | TABLE row=8 col=0]
scripts/remerge/strip_outliers.py

[P01346 | 22696:22750 | NORMAL_TEXT | TABLE row=8 col=1]
map1/map2/map3 PCD와 config를 in-place 갱신하는 optional 단계

[P01347 | 22751:22780 | NORMAL_TEXT | TABLE row=8 col=2]
기존 산출물 overwrite, 실행 전 백업 필요

[P01348 | 22782:22810 | NORMAL_TEXT | TABLE row=9 col=0]
scripts/remerge/validate.py

[P01349 | 22811:22817 | NORMAL_TEXT | TABLE row=9 col=1]
결과 진단

[P01350 | 22818:22857 | NORMAL_TEXT | TABLE row=9 col=2]
positional args 없음, fixed output 경로 검사

[P01351 | 22859:22888 | NORMAL_TEXT | TABLE row=10 col=0]
scripts/remerge/check_opt.py

[P01352 | 22889:22908 | NORMAL_TEXT | TABLE row=10 col=1]
optimization 상태 진단

[P01353 | 22909:22961 | NORMAL_TEXT | TABLE row=10 col=2]
positional args 없음, import 시 실행되는 diagnostic script

[P01354 | 22962:22963 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01355 | 22963:23123 | NORMAL_TEXT]
strip_outliers.py는 기본 pipeline에 포함하지 말고 결과 보존 여부를 확인한 뒤 선택적으로 실행해야 함. remerge script들은 실제 산출물 경로를 고정으로 사용하므로 다른 작업 결과가 있는 상태에서 실행하면 output collision이 발생할 수 있음.

[P01356 | 23123:23162 | HEADING_3]
5.2 Georeference와 drone merge pipeline

[P01357 | 23165:23171 | NORMAL_TEXT | TABLE row=0 col=0]
실제 위치

[P01358 | 23172:23175 | NORMAL_TEXT | TABLE row=0 col=1]
역할

[P01359 | 23176:23187 | NORMAL_TEXT | TABLE row=0 col=2]
실제 실행 args

[P01360 | 23189:23225 | NORMAL_TEXT | TABLE row=1 col=0]
scripts/georef_align/drone_check.py

[P01361 | 23226:23299 | NORMAL_TEXT | TABLE row=1 col=1]
drone map과 LiDAR map의 feasibility, global align, ICP를 확인하고 transform을 출력

[P01362 | 23300:23365 | NORMAL_TEXT | TABLE row=1 col=2]
positional args 없음. 실행 시 fixed worktree data를 사용하며 drone_T 저장 없음

[P01363 | 23367:23404 | NORMAL_TEXT | TABLE row=2 col=0]
scripts/georef_align/drone_check2.py

[P01364 | 23405:23455 | NORMAL_TEXT | TABLE row=2 col=1]
occupancy/yaw sweep와 ICP를 수행하고 drone transform 저장

[P01365 | 23456:23497 | NORMAL_TEXT | TABLE row=2 col=2]
positional args 없음, work/drone_T.json 생성

[P01366 | 23499:23541 | NORMAL_TEXT | TABLE row=3 col=0]
scripts/georef_align/drone_constraints.py

[P01367 | 23542:23579 | NORMAL_TEXT | TABLE row=3 col=1]
저장된 drone transform으로 graph prior 생성

[P01368 | 23580:23621 | NORMAL_TEXT | TABLE row=3 col=2]
positional args 없음, work/drone_T.json 필요

[P01369 | 23623:23668 | NORMAL_TEXT | TABLE row=4 col=0]
scripts/georef_align/optimize_with_priors.py

[P01370 | 23669:23717 | NORMAL_TEXT | TABLE row=4 col=1]
graph와 drone prior를 이용한 pose graph optimization

[P01371 | 23718:23774 | NORMAL_TEXT | TABLE row=4 col=2]
positional args 없음, 기존 graph를 graph_pre_drone.json으로 보존

[P01372 | 23776:23817 | NORMAL_TEXT | TABLE row=5 col=0]
scripts/georef_align/drone_thresholds.py

[P01373 | 23818:23877 | NORMAL_TEXT | TABLE row=5 col=1]
threshold [0.10, 0.20, 0.30, 0.50, 1.00]별 accepted tile 계산

[P01374 | 23878:23918 | NORMAL_TEXT | TABLE row=5 col=2]
positional args 없음, drone_tiles.json 생성

[P01375 | 23920:23956 | NORMAL_TEXT | TABLE row=6 col=0]
scripts/georef_align/drone_merge.py

[P01376 | 23957:24010 | NORMAL_TEXT | TABLE row=6 col=1]
map2, map3, drone transform와 tile 결과로 tau product 생성

[P01377 | 24011:24059 | NORMAL_TEXT | TABLE row=6 col=2]
positional args 없음, map/4_drone-merged/tau_* 생성

[P01378 | 24061:24100 | NORMAL_TEXT | TABLE row=7 col=0]
scripts/georef_align/colorize_merge.py

[P01379 | 24101:24128 | NORMAL_TEXT | TABLE row=7 col=1]
tau map colored variant 생성

[P01380 | 24129:24148 | NORMAL_TEXT | TABLE row=7 col=2]
positional args 없음

[P01381 | 24150:24194 | NORMAL_TEXT | TABLE row=8 col=0]
scripts/georef_align/interior_transplant.py

[P01382 | 24195:24224 | NORMAL_TEXT | TABLE row=8 col=1]
tau_100 이후 interior 보강 결과 생성

[P01383 | 24225:24299 | NORMAL_TEXT | TABLE row=8 col=2]
PYTHONPATH=$PWD uv run python scripts/georef_align/interior_transplant.py

[P01384 | 24301:24344 | NORMAL_TEXT | TABLE row=9 col=0]
scripts/georef_align/fix_datum_attitude.py

[P01385 | 24345:24378 | NORMAL_TEXT | TABLE row=9 col=1]
map3 metadata의 datum attitude 보정

[P01386 | 24379:24496 | NORMAL_TEXT | TABLE row=9 col=2]
PYTHONPATH=$PWD uv run python scripts/georef_align/fix_datum_attitude.py --config ../remerge-output/map3_config.yaml

[P01387 | 24497:24498 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01388 | 24498:24676 | NORMAL_TEXT]
fix_datum_attitude.py는 --out PATH를 추가할 수 있고, 기본은 config in-place rewrite임. drone check부터 rerender, leveling, metadata 생성까지는 fixed work directory를 공유하므로 중간 결과를 확인하면서 순서대로 실행해야 함.

[P01389 | 24676:24677 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01390 | 24677:24875 | NORMAL_TEXT]
위 표의 실행 args는 원본 repository의 backend/를 current directory로 둔 실제 실행 기준임. 예를 들어 --config ../remerge-output/map3_config.yaml은 원본 실행 경로를 나타내며, 같은 map config의 archive 위치는 map/2_remerge/map3_config.yaml임.

[P01391 | 24875:24876 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01392 | 24876:24904 | NORMAL_TEXT]
추가 utility의 실제 CLI는 다음과 같음.

[P01393 | 24904:24905 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01394 | 24905:24917 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@24905:24906 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧cd backend

[P01395 | 24917:24918 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01396 | 24918:24987 | NORMAL_TEXT]
PYTHONPATH=$PWD uv run --with rasterio --with pyproj --with pillow \

[P01397 | 24987:25030 | NORMAL_TEXT]
  python scripts/georef_align/prepare.py \

[P01398 | 25030:25087 | NORMAL_TEXT]
  --pcd PATH --config PATH --tif PATH --sheet 36603012 \

[P01399 | 25087:25147 | NORMAL_TEXT]
  --tfw PATH --site-e 158860.0 --site-n 484990.0 --out PATH

[P01400 | 25147:25148 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01401 | 25148:25235 | NORMAL_TEXT]
PYTHONPATH=$PWD uv run --with pyproj python scripts/georef_align/apply_correction.py \

[P01402 | 25235:25323 | NORMAL_TEXT]
  --correction correction.json --config map3_config.yaml --out map3_config_aligned.yaml

[P01403 | 25323:25324 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01404 | 25324:25409 | NORMAL_TEXT]
PYTHONPATH=$PWD uv run --with pyproj python scripts/georef_align/waypoint_remap.py \

[P01405 | 25409:25464 | NORMAL_TEXT]
  --in template35.json --out template35_on_tau100.json

[P01406 | 25464:25465 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01407 | 25465:25525 | NORMAL_TEXT]
PYTHONPATH=$PWD uv run --with pyproj --with pillow python \

[P01408 | 25525:25588 | NORMAL_TEXT]
  scripts/georef_align/route_on_map.py --route workflow.json \

[P01409 | 25588:25626 | NORMAL_TEXT]
  --session SESSION_DIR --out OUT_DIR

[P01410 | 25626:25628 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@25626:25627 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P01411 | 25628:25845 | NORMAL_TEXT]
prepare.py, apply_correction.py, waypoint_remap.py, route_on_map.py는 위처럼 named arguments를 사용함. drone_check*.py, constraints, optimize, thresholds, merge script는 현재 positional argument를 받지 않고 fixed worktree path를 사용함.

[P01412 | 25845:25846 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## 2026.08.25 A2 내부 PC 에 GUI 로 접근하는 법 (t.egqdiu90n7cj)

[P01413 | 1:36 | TITLE]
2026.08.25 A2 내부 PC 에 GUI 로 접근하는 법

[P01414 | 36:56 | NORMAL_TEXT | LIST id=kix.mwk5tn8k2dta level=0]
Created: 2026.08.25

[P01415 | 56:78 | NORMAL_TEXT | LIST id=kix.mwk5tn8k2dta level=0]
Author: SeungJong Kim

[P01416 | 78:79 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01417 | 79:151 | NORMAL_TEXT]
LiDAR 관련 펌웨어 업데이트를 진행하면서 찾은 가이드이며, 추후 A2 내부 GUI 접근이 필요한 경우 이 방법대로 접근 가능

[P01418 | 151:152 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01419 | 152:173 | NORMAL_TEXT]
reference: [Unitree宇树](https://serviceconsole.unitree.com/#/help/032301)

[P01420 | 173:175 | NORMAL_TEXT]
[INLINE_OBJECT kix.42zxsgskzjpj]

[P01421 | 175:176 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01422 | 176:179 | HEADING_2]
방법

[P01423 | 179:239 | NORMAL_TEXT | LIST id=kix.f51nizij16vt level=0]
install nomachine(https://www.nomachine.com/) in host macOS

[P01424 | 239:283 | NORMAL_TEXT | LIST id=kix.f51nizij16vt level=0]
connect to A2 using ssh and run this script

[P01425 | 283:284 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01426 | 284:326 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@284:285 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧unitree@unitree-a2-pc2:~$ ./nomachine.sh

[P01427 | 326:338 | NORMAL_TEXT]
start......

[P01428 | 338:367 | NORMAL_TEXT]
[sudo] password for unitree:

[P01429 | 367:398 | NORMAL_TEXT]
NX> 162 Disabled service: nxd.

[P01430 | 398:434 | NORMAL_TEXT]
NX> 162 Disabled service: nxserver.

[P01431 | 434:476 | NORMAL_TEXT]
NX> 162 Service: nxnode already disabled.

[P01432 | 476:533 | NORMAL_TEXT]
NX> 111 New connections to NoMachine server are enabled.

[P01433 | 533:568 | NORMAL_TEXT]
NX> 161 Enabled service: nxserver.

[P01434 | 568:632 | NORMAL_TEXT]
NX> 162 WARNING: Cannot find X servers running on this machine.

[P01435 | 632:698 | NORMAL_TEXT]
NX> 162 WARNING: A new virtual display will be created on demand.

[P01436 | 698:728 | NORMAL_TEXT]
NX> 161 Enabled service: nxd.

[P01437 | 728:730 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@728:729 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P01438 | 730:773 | NORMAL_TEXT]
3. add this to host machines ~/.ssh/config

[P01439 | 773:799 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@773:774 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧Host dangjin_a2_internal

[P01440 | 799:828 | NORMAL_TEXT]
    HostName 192.168.123.162

[P01441 | 828:845 | NORMAL_TEXT]
    User unitree

[P01442 | 845:870 | NORMAL_TEXT]
    ProxyJump dangjin_a2

[P01443 | 870:918 | NORMAL_TEXT]
    LocalForward 127.0.0.1:14000 127.0.0.1:4000

[P01444 | 918:947 | NORMAL_TEXT]
    ExitOnForwardFailure yes

[P01445 | 947:948 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01446 | 948:976 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@948:949 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧4. run ssh -N  (tunneling)

[P01447 | 976:1031 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@976:977 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧-> % ssh -N dangjin_a2_internal -o IdentitiesOnly=yes

[P01448 | 1031:1087 | NORMAL_TEXT]
Setting up xterm-ghostty terminfo on 192.168.123.162...

[P01449 | 1087:1126 | NORMAL_TEXT]
teamgrit@bears-go2x-jetson's password:

[P01450 | 1126:1162 | NORMAL_TEXT]
unitree@192.168.123.162's password:

[P01451 | 1162:1221 | NORMAL_TEXT]
Welcome to Ubuntu 22.04.4 LTS (GNU/Linux 6.5.2-rt8 x86_64)

[P01452 | 1221:1222 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01453 | 1222:1265 | NORMAL_TEXT]
 * Documentation:  https://help.ubuntu.com

[P01454 | 1265:1316 | NORMAL_TEXT]
 * Management:     https://landscape.canonical.com

[P01455 | 1316:1320 | NORMAL_TEXT]
...

[P01456 | 1320:1379 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@1320:1321 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧5. in host nomachine, attach to A2 using local tunnel ip.

[P01457 | 1379:1381 | NORMAL_TEXT]
[INLINE_OBJECT kix.6xwm7dzhv5mx]

## 2026.08.25 Navigation 인터페이스 및 메커니즘 (t.4hjicyf4aasa)

[P01458 | 1:36 | HEADING_1]
2026.08.25 Navigation 인터페이스 및 메커니즘

[P01459 | 36:56 | NORMAL_TEXT | LIST id=kix.nx3e5nm1ddgs level=0]
Created: 2026.08.25

[P01460 | 56:77 | NORMAL_TEXT | LIST id=kix.nx3e5nm1ddgs level=0]
Author: GPT 5.6 Luna

[P01461 | 77:107 | NORMAL_TEXT | LIST id=kix.nx3e5nm1ddgs level=0]
Human Reviewer: SeungJong Kim

[P01462 | 107:113 | HEADING_2]
코드베이스

[P01463 | 113:186 | NORMAL_TEXT | LIST id=kix.3wnzjxvl3qek level=0]
[https://github.com/teamgrit-lab/dangjin-a2/tree/main/teamgrit-navigation](https://github.com/teamgrit-lab/dangjin-a2/tree/main/teamgrit-navigation)

[P01464 | 186:189 | HEADING_2]
요약

[P01465 | 189:307 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
teamgrit-navigation의 주 실행 경로는 nav2_native이며, 외부 waypoint를 JSON ROS 2 topic으로 받아 Nav2 planner와 controller action으로 변환함

[P01466 | 307:407 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
전역 계획은 map, 로컬 costmap과 로컬 제어는 odom, 로봇 기준 프레임은 body를 사용하며, TF는 teamgrit-slam이 소유하고 Navigation은 소비함

[P01467 | 407:511 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
SLAM의 등록 point cloud는 EMC와 STVL로 병렬 분기되며, EMC terrain cost는 bridge를 거쳐 local costmap의 StaticLayer 입력이 됨

[P01468 | 511:625 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
Nav2의 FollowPath가 생성한 cmd_vel은 Unitree Request로 변환되고, 별도 emergency-stop 컨테이너가 안전 상태에 따라 로봇 API로 전달하거나 정지 명령으로 대체함

[P01469 | 625:745 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
task 입력은 cobiz_plugin_node에서 waypoint와 mode로 분해되고, waypoint_publisher_node와 nav2_path_bridge_node를 거쳐 Nav2 action으로 전달됨

[P01470 | 745:754 | HEADING_2]
1. 동작 기준

[P01471 | 754:858 | NORMAL_TEXT]
동작은 ./teamgrit-navigation의 launch, source, YAML, Docker Compose에 정의되어 있음. 설명 범위는 NAV_MODE=nav2_native임.

[P01472 | 858:859 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01473 | 859:877 | NORMAL_TEXT]
실행 진입점은 다음 세 파일임.

[P01474 | 877:878 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01475 | 878:904 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
deploy/docker-compose.yml

[P01476 | 904:929 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
deploy/entrypoint.nav.sh

[P01477 | 929:979 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
nav/src/mode/nav2_native/launch/bringup.launch.py

[P01478 | 979:1005 | HEADING_2]
2. 전체 경계: 외부 입력에서 로봇 명령까지

[P01479 | 1005:1044 | NORMAL_TEXT]
nav2_native의 외부 입력은 다음 경로로 내부 처리에 들어감.

[P01480 | 1044:1046 | NORMAL_TEXT]
[INLINE_OBJECT kix.q80whagaof88]

[P01481 | 1046:1049 | NORMAL_TEXT]
[INLINE_OBJECT kix.r9tvrbjw1wd5][INLINE_OBJECT kix.mc83y8owhbap]

[P01482 | 1049:1063 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@1049:1050 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧flowchart LR

[P01483 | 1063:1088 | NORMAL_TEXT]
    C[CoBiz 또는 operator]

[P01484 | 1088:1125 | NORMAL_TEXT]
    T[/task_event 또는 waypoint JSON/]

[P01485 | 1125:1151 | NORMAL_TEXT]
    CP[cobiz_plugin_node]

[P01486 | 1151:1183 | NORMAL_TEXT]
    WP[waypoint_publisher_node]

[P01487 | 1183:1211 | NORMAL_TEXT]
    B[/path_follower/goal/]

[P01488 | 1211:1241 | NORMAL_TEXT]
    PB[nav2_path_bridge_node]

[P01489 | 1241:1274 | NORMAL_TEXT]
    P[planner_server\nThetaStar]

[P01490 | 1274:1316 | NORMAL_TEXT]
    F[controller_server\nMPPI FollowPath]

[P01491 | 1316:1352 | NORMAL_TEXT]
    V[/path_follower/cmd_vel_nav2/]

[P01492 | 1352:1382 | NORMAL_TEXT]
    D[unitree_cmd_vel_driver]

[P01493 | 1382:1413 | NORMAL_TEXT]
    R[/path_follower/request/]

[P01494 | 1413:1444 | NORMAL_TEXT]
    E[emergency-stop\n별도 컨테이너]

[P01495 | 1444:1471 | NORMAL_TEXT]
    U[/api/sport/request/]

[P01496 | 1471:1496 | NORMAL_TEXT]
    Robot[Unitree robot]

[P01497 | 1496:1497 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01498 | 1497:1509 | NORMAL_TEXT]
    C --> T

[P01499 | 1509:1522 | NORMAL_TEXT]
    T --> CP

[P01500 | 1522:1568 | NORMAL_TEXT]
    CP -->|/path_follower/multi_waypoints| WP

[P01501 | 1568:1581 | NORMAL_TEXT]
    WP --> B

[P01502 | 1581:1594 | NORMAL_TEXT]
    B --> PB

[P01503 | 1594:1632 | NORMAL_TEXT]
    PB -->|ComputePathThroughPoses| P

[P01504 | 1632:1660 | NORMAL_TEXT]
    P -->|nav_msgs/Path| PB

[P01505 | 1660:1685 | NORMAL_TEXT]
    PB -->|FollowPath| F

[P01506 | 1685:1697 | NORMAL_TEXT]
    F --> V

[P01507 | 1697:1709 | NORMAL_TEXT]
    V --> D

[P01508 | 1709:1721 | NORMAL_TEXT]
    D --> R

[P01509 | 1721:1733 | NORMAL_TEXT]
    R --> E

[P01510 | 1733:1745 | NORMAL_TEXT]
    E --> U

[P01511 | 1745:1761 | NORMAL_TEXT]
    U --> Robot

[P01512 | 1761:1763 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@1761:1762 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P01513 | 1763:1884 | NORMAL_TEXT]
외부 waypoint는 바로 로봇 명령으로 사용되지 않음. 먼저 map 좌표의 경로 목표로 정규화되고, planner가 costmap을 이용해 경로를 만든 뒤, controller가 경로 추종용 속도 명령을 계산함.

[P01514 | 1884:1885 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01515 | 1885:1926 | NORMAL_TEXT]
지각 데이터와 localization 데이터는 별도 입력 경계로 들어옴.

[P01516 | 1926:1928 | NORMAL_TEXT]
[INLINE_OBJECT kix.yc0atf84old5]

[P01517 | 1928:1942 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@1928:1929 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧flowchart TD

[P01518 | 1942:1963 | NORMAL_TEXT]
    S[teamgrit-slam]

[P01519 | 1963:1991 | NORMAL_TEXT]
    O[/grit_slam/odometry/]

[P01520 | 1991:2046 | NORMAL_TEXT]
    TF[/tf, /tf_static\nearth -> map -> odom -> body/]

[P01521 | 2046:2087 | NORMAL_TEXT]
    C[/grit_slam/cloud_registered_body/]

[P01522 | 2087:2129 | NORMAL_TEXT]
    EMC[emc 컨테이너\nelevation_mapping_cupy]

[P01523 | 2129:2180 | NORMAL_TEXT]
    GM[/elevation_mapping_node/elevation_map_raw/]

[P01524 | 2180:2211 | NORMAL_TEXT]
    TB[terrain_cost_bridge.py]

[P01525 | 2211:2244 | NORMAL_TEXT]
    TC[/terrain_cost/occupancy/]

[P01526 | 2244:2297 | NORMAL_TEXT]
    L[local costmap\nStaticLayer + STVL + Inflation]

[P01527 | 2297:2336 | NORMAL_TEXT]
    G[global costmap\nmap + Inflation]

[P01528 | 2336:2359 | NORMAL_TEXT]
    F[MPPI controller]

[P01529 | 2359:2360 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01530 | 2360:2372 | NORMAL_TEXT]
    S --> O

[P01531 | 2372:2385 | NORMAL_TEXT]
    S --> TF

[P01532 | 2385:2397 | NORMAL_TEXT]
    S --> C

[P01533 | 2397:2438 | NORMAL_TEXT]
    C --> EMC --> GM --> TB --> TC --> L

[P01534 | 2438:2466 | NORMAL_TEXT]
    C -->|직접 PointCloud2| L

[P01535 | 2466:2479 | NORMAL_TEXT]
    TF --> L

[P01536 | 2479:2492 | NORMAL_TEXT]
    TF --> G

[P01537 | 2492:2504 | NORMAL_TEXT]
    O --> F

[P01538 | 2504:2516 | NORMAL_TEXT]
    L --> F

[P01539 | 2516:2547 | NORMAL_TEXT]
    G --> P[ThetaStar planner]

[P01540 | 2547:2559 | NORMAL_TEXT]
    P --> F

[P01541 | 2559:2575 | HEADING_2]
⟦PROTECTED: opaqueTemplateControl/copy-only@2559:2560 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧3. 외부 인터페이스 계약

[P01542 | 2575:2617 | HEADING_3]
3.1 teamgrit-slam에서 Navigation으로 들어오는 데이터

[P01543 | 2620:2626 | NORMAL_TEXT | TABLE row=0 col=0]
인터페이스

[P01544 | 2627:2630 | NORMAL_TEXT | TABLE row=0 col=1]
타입

[P01545 | 2631:2638 | NORMAL_TEXT | TABLE row=0 col=2]
내부 소비자

[P01546 | 2639:2642 | NORMAL_TEXT | TABLE row=0 col=3]
의미

[P01547 | 2644:2664 | NORMAL_TEXT | TABLE row=1 col=0]
/grit_slam/odometry

[P01548 | 2665:2687 | NORMAL_TEXT | TABLE row=1 col=1]
nav_msgs/msg/Odometry

[P01549 | 2688:2734 | NORMAL_TEXT | TABLE row=1 col=2]
Nav2 controller, teamgrit_native, safety node

[P01550 | 2735:2789 | NORMAL_TEXT | TABLE row=1 col=3]
로봇의 연속적인 주행 추정값이며 ODOM_TOPIC으로 controller 입력을 바꿀 수 있음

[P01551 | 2791:2824 | NORMAL_TEXT | TABLE row=2 col=0]
/grit_slam/cloud_registered_body

[P01552 | 2825:2853 | NORMAL_TEXT | TABLE row=2 col=1]
sensor_msgs/msg/PointCloud2

[P01553 | 2854:2870 | NORMAL_TEXT | TABLE row=2 col=2]
EMC, local STVL

[P01554 | 2871:2931 | NORMAL_TEXT | TABLE row=2 col=3]
body 프레임의 등록 point cloud이며 terrain과 3D obstacle 입력으로 함께 사용됨

[P01555 | 2933:2949 | NORMAL_TEXT | TABLE row=3 col=0]
/tf, /tf_static

[P01556 | 2950:2973 | NORMAL_TEXT | TABLE row=3 col=1]
tf2_msgs/msg/TFMessage

[P01557 | 2974:3007 | NORMAL_TEXT | TABLE row=3 col=2]
costmap, path bridge, controller

[P01558 | 3008:3043 | NORMAL_TEXT | TABLE row=3 col=3]
earth -> map -> odom -> body 변환 제공

[P01559 | 3045:3071 | NORMAL_TEXT | TABLE row=4 col=0]
/grit_slam/optimized_path

[P01560 | 3072:3090 | NORMAL_TEXT | TABLE row=4 col=1]
nav_msgs/msg/Path

[P01561 | 3091:3133 | NORMAL_TEXT | TABLE row=4 col=2]
선택적 path_simplifier_node, teamgrit_native

[P01562 | 3134:3169 | NORMAL_TEXT | TABLE row=4 col=3]
최적화 경로를 waypoint로 변환할 때 사용되는 선택 입력

[P01563 | 3170:3171 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01564 | 3171:3267 | NORMAL_TEXT]
Navigation은 원시 LiDAR topic을 직접 소비하지 않음. 로컬 지각 입력은 SLAM이 제공하는 /grit_slam/cloud_registered_body임.

[P01565 | 3267:3313 | HEADING_3]
3.2 CoBiz와 Navigation 사이의 task·waypoint 인터페이스

[P01566 | 3313:3479 | NORMAL_TEXT]
ENABLE_COBIZ=true이면 nav2_native와 teamgrit_native launch가 grit_nav_cobiz/cobiz_plugin_node를 시작함. 이 node는 ROS message type을 별도로 만들지 않고 std_msgs/msg/String 안에 JSON을 담음.

[P01567 | 3479:3480 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01568 | 3483:3486 | NORMAL_TEXT | TABLE row=0 col=0]
방향

[P01569 | 3487:3493 | NORMAL_TEXT | TABLE row=0 col=1]
Topic

[P01570 | 3494:3497 | NORMAL_TEXT | TABLE row=0 col=2]
타입

[P01571 | 3498:3501 | NORMAL_TEXT | TABLE row=0 col=3]
역할

[P01572 | 3503:3522 | NORMAL_TEXT | TABLE row=1 col=0]
CoBiz → Navigation

[P01573 | 3523:3535 | NORMAL_TEXT | TABLE row=1 col=1]
/task_event

[P01574 | 3536:3556 | NORMAL_TEXT | TABLE row=1 col=2]
std_msgs/msg/String

[P01575 | 3557:3597 | NORMAL_TEXT | TABLE row=1 col=3]
TASK_REGISTERED, TASK_ABORTED JSON을 수신함

[P01576 | 3599:3618 | NORMAL_TEXT | TABLE row=2 col=0]
Navigation → CoBiz

[P01577 | 3619:3631 | NORMAL_TEXT | TABLE row=2 col=1]
/task_state

[P01578 | 3632:3652 | NORMAL_TEXT | TABLE row=2 col=2]
std_msgs/msg/String

[P01579 | 3653:3685 | NORMAL_TEXT | TABLE row=2 col=3]
task 시작, 완료, 거부 또는 실패 JSON을 발행함

[P01580 | 3687:3712 | NORMAL_TEXT | TABLE row=3 col=0]
plugin → waypoint router

[P01581 | 3713:3744 | NORMAL_TEXT | TABLE row=3 col=1]
/path_follower/multi_waypoints

[P01582 | 3745:3765 | NORMAL_TEXT | TABLE row=3 col=2]
std_msgs/msg/String

[P01583 | 3766:3817 | NORMAL_TEXT | TABLE row=3 col=3]
task payload에서 추출한 waypoint 배열과 req_id, frame을 전달함

[P01584 | 3819:3840 | NORMAL_TEXT | TABLE row=4 col=0]
plugin → path bridge

[P01585 | 3841:3863 | NORMAL_TEXT | TABLE row=4 col=1]
/path_follower/cancel

[P01586 | 3864:3884 | NORMAL_TEXT | TABLE row=4 col=2]
std_msgs/msg/String

[P01587 | 3885:3909 | NORMAL_TEXT | TABLE row=4 col=3]
active task의 경로 취소를 요청함

[P01588 | 3911:3940 | NORMAL_TEXT | TABLE row=5 col=0]
plugin → localization 계열 소비자

[P01589 | 3941:3954 | NORMAL_TEXT | TABLE row=5 col=1]
/initialpose

[P01590 | 3955:3999 | NORMAL_TEXT | TABLE row=5 col=2]
geometry_msgs/msg/PoseWithCovarianceStamped

[P01591 | 4000:4045 | NORMAL_TEXT | TABLE row=5 col=3]
task에 initial_pose가 있으면 map frame 초기 자세를 발행함

[P01592 | 4047:4068 | NORMAL_TEXT | TABLE row=6 col=0]
path bridge → plugin

[P01593 | 4069:4091 | NORMAL_TEXT | TABLE row=6 col=1]
/path_follower/status

[P01594 | 4092:4112 | NORMAL_TEXT | TABLE row=6 col=2]
std_msgs/msg/String

[P01595 | 4113:4159 | NORMAL_TEXT | TABLE row=6 col=3]
MOVING, REACHED, FAILED, ABORTED JSON 상태를 전달함

[P01596 | 4160:4161 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01597 | 4161:4299 | NORMAL_TEXT]
cobiz_plugin_node는 action_name == NAVIGATION_3D 여부를 별도로 검사하지 않음. TASK_REGISTERED에서 유효한 waypoint가 추출되면 task를 처리하고, task 식별자는 상태 JSON에 보존함.

[P01598 | 4299:4336 | HEADING_3]
3.3 Navigation 내부의 waypoint 입력 인터페이스

[P01599 | 4336:4409 | NORMAL_TEXT]
waypoint_publisher_node가 서로 다른 입력 방식을 하나의 /path_follower/goal 형식으로 정규화함.

[P01600 | 4409:4410 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01601 | 4413:4416 | NORMAL_TEXT | TABLE row=0 col=0]
입력

[P01602 | 4417:4420 | NORMAL_TEXT | TABLE row=0 col=1]
타입

[P01603 | 4421:4424 | NORMAL_TEXT | TABLE row=0 col=2]
처리

[P01604 | 4426:4457 | NORMAL_TEXT | TABLE row=1 col=0]
/path_follower/multi_waypoints

[P01605 | 4458:4490 | NORMAL_TEXT | TABLE row=1 col=1]
std_msgs/msg/String JSON object

[P01606 | 4491:4541 | NORMAL_TEXT | TABLE row=1 col=2]
waypoints[] 또는 waypoint[]를 읽고 각 항목의 x, y, z를 정규화함

[P01607 | 4543:4579 | NORMAL_TEXT | TABLE row=2 col=0]
/path_follower/simplified_waypoints

[P01608 | 4580:4621 | NORMAL_TEXT | TABLE row=2 col=1]
std_msgs/msg/String JSON array 또는 object

[P01609 | 4622:4657 | NORMAL_TEXT | TABLE row=2 col=2]
path simplifier가 만든 waypoint를 정규화함

[P01610 | 4659:4684 | NORMAL_TEXT | TABLE row=3 col=0]
/path_follower/load_file

[P01611 | 4685:4708 | NORMAL_TEXT | TABLE row=3 col=1]
std_msgs/msg/String 경로

[P01612 | 4709:4742 | NORMAL_TEXT | TABLE row=3 col=2]
CSV 또는 YAML 파일을 읽어 waypoint로 변환함

[P01613 | 4744:4765 | NORMAL_TEXT | TABLE row=4 col=0]
launch waypoint_file

[P01614 | 4766:4776 | NORMAL_TEXT | TABLE row=4 col=1]
parameter

[P01615 | 4777:4806 | NORMAL_TEXT | TABLE row=4 col=2]
node 시작 시 CSV 또는 YAML 파일을 읽음

[P01616 | 4807:4808 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01617 | 4808:4832 | NORMAL_TEXT]
router 출력 JSON은 아래 구조임.

[P01618 | 4832:4833 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01619 | 4833:4836 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@4833:4834 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧{

[P01620 | 4836:4854 | NORMAL_TEXT]
  "frame": "map",

[P01621 | 4854:4871 | NORMAL_TEXT]
  "waypoints": [

[P01622 | 4871:4906 | NORMAL_TEXT]
    {"x": 1.0, "y": 2.0, "z": 0.0}

[P01623 | 4906:4911 | NORMAL_TEXT]
  ],

[P01624 | 4911:4925 | NORMAL_TEXT]
  "req_id": 7

[P01625 | 4925:4927 | NORMAL_TEXT]
}

[P01626 | 4927:4929 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@4927:4928 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P01627 | 4929:5008 | NORMAL_TEXT]
nav2_path_bridge_node는 이 JSON을 다시 읽어 waypoint를 map frame의 PoseStamped 목표로 변환함.

[P01628 | 5008:5044 | HEADING_2]
4. 프레임과 소유권: 외부 데이터가 내부 계산에 들어가는 방식

[P01629 | 5044:5054 | HEADING_3]
4.1 TF 체인

[P01630 | 5054:5084 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5054:5055 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧earth -> map -> odom -> body

[P01631 | 5084:5086 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5084:5085 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P01632 | 5089:5093 | NORMAL_TEXT | TABLE row=0 col=0]
프레임

[P01633 | 5094:5097 | NORMAL_TEXT | TABLE row=0 col=1]
용도

[P01634 | 5098:5102 | NORMAL_TEXT | TABLE row=0 col=2]
소유자

[P01635 | 5103:5120 | NORMAL_TEXT | TABLE row=0 col=3]
Navigation 내부 사용

[P01636 | 5122:5128 | NORMAL_TEXT | TABLE row=1 col=0]
earth

[P01637 | 5129:5147 | NORMAL_TEXT | TABLE row=1 col=1]
전역 기준 및 다중 map 정합

[P01638 | 5148:5162 | NORMAL_TEXT | TABLE row=1 col=2]
teamgrit-slam

[P01639 | 5163:5174 | NORMAL_TEXT | TABLE row=1 col=3]
직접 계산하지 않음

[P01640 | 5176:5180 | NORMAL_TEXT | TABLE row=2 col=0]
map

[P01641 | 5181:5201 | NORMAL_TEXT | TABLE row=2 col=1]
전역 계획 및 waypoint 목표

[P01642 | 5202:5216 | NORMAL_TEXT | TABLE row=2 col=2]
teamgrit-slam

[P01643 | 5217:5254 | NORMAL_TEXT | TABLE row=2 col=3]
global costmap, planner, path bridge

[P01644 | 5256:5261 | NORMAL_TEXT | TABLE row=3 col=0]
odom

[P01645 | 5262:5284 | NORMAL_TEXT | TABLE row=3 col=1]
점프가 없는 연속 local frame

[P01646 | 5285:5299 | NORMAL_TEXT | TABLE row=3 col=2]
teamgrit-slam

[P01647 | 5300:5325 | NORMAL_TEXT | TABLE row=3 col=3]
local costmap, EMC, STVL

[P01648 | 5327:5332 | NORMAL_TEXT | TABLE row=4 col=0]
body

[P01649 | 5333:5347 | NORMAL_TEXT | TABLE row=4 col=1]
로봇 base frame

[P01650 | 5348:5362 | NORMAL_TEXT | TABLE row=4 col=2]
teamgrit-slam

[P01651 | 5363:5402 | NORMAL_TEXT | TABLE row=4 col=3]
robot_base_frame, point cloud 입력 frame

[P01652 | 5403:5404 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01653 | 5404:5631 | NORMAL_TEXT]
Navigation은 이 TF를 다시 발행하지 않음. nav2_path_bridge_node는 lookupTransform(map, body, TimePointZero)로 로봇 pose를 읽고, 목표 JSON의 frame이 map과 다르면 goal을 거부함. 빈 frame도 허용하지 않음. 따라서 path bridge가 임의로 odom 또는 다른 frame의 목표를 map으로 변환해 주는 구조는 아님.

[P01654 | 5631:5648 | HEADING_3]
4.2 frame별 계산 경계

[P01655 | 5648:5700 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
전역 costmap: global_frame=map, robot_base_frame=body

[P01656 | 5700:5769 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
로컬 costmap: global_frame=odom, robot_base_frame=body, rolling window

[P01657 | 5769:5812 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
planner 입력 경로: map frame의 시작 pose와 목표 pose

[P01658 | 5812:5861 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
controller 입력 경로: planner가 반환한 nav_msgs/msg/Path

[P01659 | 5861:5906 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
EMC: map_frame=odom, base_frame=body 설정을 사용함

[P01660 | 5906:5967 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
Cobiz initial_pose: plugin parameter 기본값에 따라 map frame으로 발행됨

[P01661 | 5967:5968 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01662 | 5968:6092 | NORMAL_TEXT]
ODOM_TOPIC은 nav2_native launch에서 controller_server의 parameter로 전달됨. path bridge의 pose 조회는 odometry topic을 직접 읽지 않고 TF를 사용함.

[P01663 | 6092:6125 | HEADING_2]
5. Task와 waypoint가 내부 경로로 바뀌는 과정

[P01664 | 6125:6143 | HEADING_3]
5.1 CoBiz task 입력

[P01665 | 6143:6167 | NORMAL_TEXT]
CoBiz task는 아래 순서로 전달됨.

[P01666 | 6167:6169 | NORMAL_TEXT]
[INLINE_OBJECT kix.nngzcsmt7x78]

[P01667 | 6169:6186 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@6169:6170 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧sequenceDiagram

[P01668 | 6186:6223 | NORMAL_TEXT]
    box rgb(235, 245, 255) CoBiz Web

[P01669 | 6223:6260 | NORMAL_TEXT]
        participant WEB as CoBiz Web

[P01670 | 6260:6297 | NORMAL_TEXT]
        participant API as CoBiz API

[P01671 | 6297:6305 | NORMAL_TEXT]
    end

[P01672 | 6305:6343 | NORMAL_TEXT]
    box rgb(245, 245, 245) cobiz-core

[P01673 | 6343:6386 | NORMAL_TEXT]
        participant H as health_check_node

[P01674 | 6386:6427 | NORMAL_TEXT]
        participant R as request_manager

[P01675 | 6427:6435 | NORMAL_TEXT]
    end

[P01676 | 6435:6482 | NORMAL_TEXT]
    box rgb(235, 255, 240) teamgrit-navigation

[P01677 | 6482:6525 | NORMAL_TEXT]
        participant X as cobiz_plugin_node

[P01678 | 6525:6574 | NORMAL_TEXT]
        participant W as waypoint_publisher_node

[P01679 | 6574:6621 | NORMAL_TEXT]
        participant B as nav2_path_bridge_node

[P01680 | 6621:6651 | NORMAL_TEXT]
        participant N as Nav2

[P01681 | 6651:6659 | NORMAL_TEXT]
    end

[P01682 | 6659:6660 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01683 | 6660:6694 | NORMAL_TEXT]
    WEB->>API: navigation task 생성

[P01684 | 6694:6734 | NORMAL_TEXT]
    API-->>H: WebSocket TASK_REGISTERED

[P01685 | 6734:6773 | NORMAL_TEXT]
    H->>X: /task_event TASK_REGISTERED

[P01686 | 6773:6826 | NORMAL_TEXT]
    X->>X: payload, initial_pose, mode, waypoints 파싱

[P01687 | 6826:6862 | NORMAL_TEXT]
    X->>R: /task_state TASK_STARTED

[P01688 | 6862:6891 | NORMAL_TEXT]
    R->>API: POST task state

[P01689 | 6891:6933 | NORMAL_TEXT]
    X->>W: /path_follower/multi_waypoints

[P01690 | 6933:6964 | NORMAL_TEXT]
    W->>B: /path_follower/goal

[P01691 | 6964:6999 | NORMAL_TEXT]
    B->>N: ComputePathThroughPoses

[P01692 | 6999:7016 | NORMAL_TEXT]
    N-->>B: Path

[P01693 | 7016:7038 | NORMAL_TEXT]
    B->>N: FollowPath

[P01694 | 7038:7074 | NORMAL_TEXT]
    N-->>B: ACCEPTED 또는 terminal 결과

[P01695 | 7074:7108 | NORMAL_TEXT]
    B-->>X: /path_follower/status

[P01696 | 7108:7161 | NORMAL_TEXT]
    X->>R: /task_state TASK_COMPLETED 또는 TASK_FAILED

[P01697 | 7161:7190 | NORMAL_TEXT]
    R->>API: POST task state

[P01698 | 7190:7217 | NORMAL_TEXT]
    API-->>WEB: task 상태 반영

[P01699 | 7217:7219 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@7217:7218 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P01700 | 7219:7249 | NORMAL_TEXT]
plugin은 payload를 아래 규칙으로 처리함.

[P01701 | 7249:7250 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01702 | 7250:7302 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
payload가 object이면 그대로 사용하고, JSON string이면 다시 parse함

[P01703 | 7302:7362 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
waypoint는 event 본문 또는 payload 안의 waypoints 또는 waypoint에서 찾음

[P01704 | 7362:7403 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
각 waypoint에 x, y가 있어야 하며 z가 없으면 0.0을 사용함

[P01705 | 7403:7482 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
mode와 walk_mode를 정규화하며, 여러 waypoint mode가 섞이면 첫 mode를 /path_follower/mode에 적용함

[P01706 | 7482:7547 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
initial_pose가 있으면 위치를 검증하고 /initialpose에 orientation.w=1.0으로 발행함

[P01707 | 7547:7598 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
map_id는 로그에 남길 뿐, map 파일 탐색이나 SLAM map 전환에 사용하지 않음

[P01708 | 7598:7646 | NORMAL_TEXT | LIST id=kix.khuqn3uqz9l5 level=0]
이미 다른 task가 active이면 새 task를 TASK_REJECTED로 보고함

[P01709 | 7646:7647 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01710 | 7647:7827 | NORMAL_TEXT]
waypoint가 유효하면 plugin은 active_task_id를 기록하고 /task_state에 TASK_STARTED를 발행한 다음 mode와 waypoint를 내부 topic으로 전달함. TASK_STARTED는 localization READY 또는 Nav2 FollowPath goal 수락을 기다리지 않음.

[P01711 | 7827:7841 | HEADING_3]
5.2 취소와 종료 상태

[P01712 | 7841:7867 | NORMAL_TEXT]
TASK_ABORTED는 아래 순서로 처리됨.

[P01713 | 7867:7868 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01714 | 7868:7898 | NORMAL_TEXT | LIST id=kix.j0zz81ldno2m level=0]
같은 task_id의 TASK_ABORTED를 수신함

[P01715 | 7898:7925 | NORMAL_TEXT | LIST id=kix.j0zz81ldno2m level=0]
/path_follower/cancel을 발행함

[P01716 | 7925:7960 | NORMAL_TEXT | LIST id=kix.j0zz81ldno2m level=0]
곧바로 /task_state에 TASK_ABORTED를 발행함

[P01717 | 7960:7976 | NORMAL_TEXT | LIST id=kix.j0zz81ldno2m level=0]
active task를 지움

[P01718 | 7976:7977 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01719 | 7977:8153 | NORMAL_TEXT]
path bridge는 cancel topic을 수신하면 내부 waypoint와 계획을 비우고, active planner/controller action에 비동기 cancel을 요청함. plugin은 Nav2 action의 cancel terminal을 기다리지 않고 /task_state에 abort를 보고함.

[P01720 | 8153:8185 | HEADING_2]
6. nav2_path_bridge_node의 내부 동작

[P01721 | 8185:8336 | NORMAL_TEXT]
nav2_path_bridge_node는 외부의 단순 JSON waypoint interface와 Nav2 action interface 사이의 adapter임. 직접 cmd_vel을 계산하지 않으며, planner와 controller action을 순서대로 호출함.

[P01722 | 8336:8361 | HEADING_3]
6.1 입력 수신과 generation 관리

[P01723 | 8361:8397 | NORMAL_TEXT]
goal 또는 clicked point 입력 시 아래를 처리함.

[P01724 | 8397:8398 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01725 | 8398:8450 | NORMAL_TEXT | LIST id=kix.hvmgcgpneiyh level=0]
JSON에서 waypoint를 읽거나 PointStamped를 단일 waypoint로 변환함

[P01726 | 8450:8489 | NORMAL_TEXT | LIST id=kix.hvmgcgpneiyh level=0]
입력 frame이 global_frame인 map과 일치하는지 확인함

[P01727 | 8489:8532 | NORMAL_TEXT | LIST id=kix.hvmgcgpneiyh level=0]
current_waypoints와 current_goal_index를 갱신함

[P01728 | 8532:8605 | NORMAL_TEXT | LIST id=kix.hvmgcgpneiyh level=0]
path_generation을 증가시켜 이전 planner/controller 결과를 stale result로 무시할 수 있게 함

[P01729 | 8605:8673 | NORMAL_TEXT | LIST id=kix.hvmgcgpneiyh level=0]
기존 FollowPath 또는 planner action이 있으면 cancel을 요청하고 새 generation을 준비함

[P01730 | 8673:8674 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01731 | 8674:8776 | NORMAL_TEXT]
generation은 늦게 도착한 비동기 action callback을 차단하는 역할을 함. 이전 generation의 result는 새 waypoint batch에 적용하지 않음.

[P01732 | 8776:8804 | HEADING_3]
6.2 로봇 pose와 남은 waypoint 계산

[P01733 | 8804:8844 | NORMAL_TEXT]
평가 timer는 기본 10 Hz로 실행되며, 매 주기 아래를 수행함.

[P01734 | 8844:8845 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01735 | 8845:8880 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
TF buffer에서 최신 map -> body 변환을 조회함

[P01736 | 8880:8917 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
TF가 없으면 planning을 진행하지 않고 다음 주기를 기다림

[P01737 | 8917:8946 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
로봇 pose에서 순서대로 waypoint를 검사함

[P01738 | 8946:9008 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
waypoint_reached_threshold_m 기본값 0.1 m보다 가까운 앞쪽 waypoint는 제거함

[P01739 | 9008:9066 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
남은 waypoint가 없고 실행 중인 action도 없으면 REACHED를 발행하고 goal을 정리함

[P01740 | 9066:9067 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01741 | 9067:9177 | NORMAL_TEXT]
별도 progress index를 사용하지 않고 로봇 pose와 waypoint 거리로 이미 지난 목표를 판단함. z가 거의 0이면 2D 거리, waypoint z가 유효하면 3D 거리를 사용함.

[P01742 | 9177:9199 | HEADING_3]
6.3 planner action 호출

[P01743 | 9199:9264 | NORMAL_TEXT]
남은 waypoint가 있고 유효한 계획이 없으면 ComputePathThroughPoses action을 호출함.

[P01744 | 9264:9265 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01745 | 9265:9301 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
action: /compute_path_through_poses

[P01746 | 9301:9323 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
planner id: GridBased

[P01747 | 9323:9381 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
planner plugin: nav2_theta_star_planner::ThetaStarPlanner

[P01748 | 9381:9396 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
use_start=true

[P01749 | 9396:9425 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
start: TF로 얻은 map frame pose

[P01750 | 9425:9474 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
goals: 각 waypoint를 map frame PoseStamped로 변환한 배열

[P01751 | 9474:9475 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01752 | 9475:9697 | NORMAL_TEXT]
waypoint orientation을 입력에서 별도로 읽지 않음. 각 goal pose의 yaw는 이전 pose에서 해당 waypoint로 향하는 선분 방향 atan2(dy, dx)로 계산함. 선분 길이가 거의 0이면 이전 yaw를 유지함. 이 때문에 planner가 받은 최종 orientation은 waypoint JSON의 orientation이 아니라 bridge가 계산한 진행 방향임.

[P01753 | 9697:9698 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01754 | 9698:9830 | NORMAL_TEXT]
planner가 2개 미만의 pose를 반환하면 path를 사용할 수 없는 것으로 보고 FAILED를 발행함. 성공한 path는 /path_follower/nav2_path에 발행되고 다음 controller action의 입력이 됨.

[P01755 | 9830:9855 | HEADING_3]
6.4 FollowPath action 호출

[P01756 | 9855:9903 | NORMAL_TEXT]
계획 path가 준비되면 bridge가 /follow_path action을 호출함.

[P01757 | 9903:9904 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01758 | 9904:9925 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
action: /follow_path

[P01759 | 9925:9951 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
controller id: FollowPath

[P01760 | 9951:9989 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
goal checker id: general_goal_checker

[P01761 | 9989:10001 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
성공: REACHED

[P01762 | 10001:10028 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
abort 또는 알 수 없는 결과: FAILED

[P01763 | 10028:10088 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
cancel: 새 generation이면 새 path를 다시 준비하고, 일반 취소이면 ABORTED 처리함

[P01764 | 10088:10110 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
goal 수락 시점: MOVING 발행

[P01765 | 10110:10111 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01766 | 10111:10261 | NORMAL_TEXT]
path bridge의 상태는 외부에서 관찰 가능한 MOVING, REACHED, FAILED, ABORTED 문자열 JSON과 내부 action handle 및 pending flag의 조합으로 구성됨. 별도 lifecycle state machine을 두지 않음.

[P01767 | 10261:10284 | HEADING_3]
6.5 Path bridge의 상태 흐름

[P01768 | 10284:10301 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@10284:10285 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧stateDiagram-v2

[P01769 | 10301:10324 | NORMAL_TEXT]
    [*] --> WAITING_TF

[P01770 | 10324:10379 | NORMAL_TEXT]
    WAITING_TF --> PLANNING: map -> body TF + waypoint

[P01771 | 10379:10441 | NORMAL_TEXT]
    PLANNING --> FOLLOWING: planner success + path >= 2 poses

[P01772 | 10441:10502 | NORMAL_TEXT]
    PLANNING --> FAILED: planner reject, abort, trivial path

[P01773 | 10502:10550 | NORMAL_TEXT]
    FOLLOWING --> REACHED: FollowPath succeeded

[P01774 | 10550:10595 | NORMAL_TEXT]
    FOLLOWING --> FAILED: FollowPath aborted

[P01775 | 10595:10647 | NORMAL_TEXT]
    FOLLOWING --> CANCELING: cancel 또는 새 generation

[P01776 | 10647:10714 | NORMAL_TEXT]
    CANCELING --> PLANNING: old action canceled + resend requested

[P01777 | 10714:10754 | NORMAL_TEXT]
    CANCELING --> IDLE: explicit cancel

[P01778 | 10754:10775 | NORMAL_TEXT]
    REACHED --> IDLE

[P01779 | 10775:10795 | NORMAL_TEXT]
    FAILED --> IDLE

[P01780 | 10795:10797 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@10795:10796 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P01781 | 10797:10860 | NORMAL_TEXT]
CoBiz task lifecycle의 TASK_STARTED와 path bridge의 상태 집합은 서로 다름.

[P01782 | 10860:10892 | HEADING_2]
7. Nav2 내부에서 경로가 속도 명령으로 바뀌는 과정

[P01783 | 10892:10904 | HEADING_3]
7.1 Planner

[P01784 | 10904:11075 | NORMAL_TEXT]
planner_server는 GridBased라는 planner id로 등록되어 있지만 실제 plugin은 ThetaStarPlanner임. global costmap의 frame은 map이고, global costmap은 rolling window이며 plugin은 InflationLayer만 포함함.

[P01785 | 11075:11076 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01786 | 11076:11248 | NORMAL_TEXT]
planner는 path bridge가 전달한 로봇 pose와 여러 goal pose를 받아 costmap 위에서 nav_msgs/msg/Path를 반환함. planner 결과를 직접 로봇에 보내지 않고, bridge가 같은 path를 FollowPath action으로 다시 controller에 전달함.

[P01787 | 11248:11268 | HEADING_3]
7.2 MPPI controller

[P01788 | 11268:11438 | NORMAL_TEXT]
controller_server의 FollowPath plugin은 nav2_mppi_controller::MPPIController임. path와 local costmap, robot pose, odometry를 사용해 후보 제어열을 평가하고 최종 geometry_msgs/msg/Twist를 발행함.

[P01789 | 11438:11439 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01790 | 11439:11464 | NORMAL_TEXT]
공용 설정에서 아래 critic을 활성화함.

[P01791 | 11464:11465 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01792 | 11465:11482 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
ConstraintCritic

[P01793 | 11482:11498 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
ObstaclesCritic

[P01794 | 11498:11509 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
CostCritic

[P01795 | 11509:11520 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
GoalCritic

[P01796 | 11520:11536 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
GoalAngleCritic

[P01797 | 11536:11552 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
PathAlignCritic

[P01798 | 11552:11569 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
PathFollowCritic

[P01799 | 11569:11585 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
PathAngleCritic

[P01800 | 11585:11605 | NORMAL_TEXT | LIST id=kix.i9bwlkcvsufk level=0]
PreferForwardCritic

[P01801 | 11605:11606 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01802 | 11606:11769 | NORMAL_TEXT]
ObstaclesCritic과 CostCritic은 footprint를 고려하도록 설정되어 있고, PathAngleCritic은 경로 방향 정렬 비용을 사용함. 로봇별 overlay는 vx_min, vy_max, wz_max, motion_model, footprint를 추가하거나 덮어씀.

[P01803 | 11769:11805 | HEADING_3]
7.3 local costmap: EMC와 STVL의 병렬 입력

[P01804 | 11805:11880 | NORMAL_TEXT]
local costmap은 odom frame의 12 m × 12 m rolling window임. layer는 아래 순서로 적용함.

[P01805 | 11880:11881 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01806 | 11881:11934 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@11881:11882 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧terrain_cost_layer -> stvl_layer -> inflation_layer

[P01807 | 11934:11936 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@11934:11935 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P01808 | 11936:12003 | NORMAL_TEXT]
EMC branch와 STVL branch는 같은 SLAM point cloud에서 시작하지만 서로 다른 의미를 갖음.

[P01809 | 12003:12005 | NORMAL_TEXT]
[INLINE_OBJECT kix.mft4yjb1tzah]

[P01810 | 12005:12019 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@12005:12006 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧flowchart LR

[P01811 | 12019:12060 | NORMAL_TEXT]
    C[/grit_slam/cloud_registered_body/]

[P01812 | 12060:12077 | NORMAL_TEXT]
    C --> E[EMC]

[P01813 | 12077:12132 | NORMAL_TEXT]
    E --> G[/elevation_map_raw\nGridMap terrain_cost/]

[P01814 | 12132:12168 | NORMAL_TEXT]
    G --> B[terrain_cost_bridge.py]

[P01815 | 12168:12228 | NORMAL_TEXT]
    B --> O[/terrain_cost/occupancy\nOccupancyGrid 0..100/]

[P01816 | 12228:12273 | NORMAL_TEXT]
    O --> S[StaticLayer\nterrain_cost_layer]

[P01817 | 12273:12310 | NORMAL_TEXT]
    C --> V[STVL\n3D voxel obstacle]

[P01818 | 12310:12344 | NORMAL_TEXT]
    S --> M[local costmap master]

[P01819 | 12344:12356 | NORMAL_TEXT]
    V --> M

[P01820 | 12356:12384 | NORMAL_TEXT]
    M --> I[InflationLayer]

[P01821 | 12384:12421 | NORMAL_TEXT]
    I --> L[/local_costmap/costmap/]

[P01822 | 12421:12441 | HEADING_4]
⟦PROTECTED: opaqueTemplateControl/copy-only@12421:12422 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧EMC terrain branch

[P01823 | 12441:12526 | NORMAL_TEXT | LIST id=kix.8xzqgjec4tr7 level=0]
EMC가 /grit_slam/cloud_registered_body를 받아 rolling elevation map과 traversability를 계산함

[P01824 | 12526:12608 | NORMAL_TEXT | LIST id=kix.8xzqgjec4tr7 level=0]
EMC가 terrain_cost layer를 /elevation_mapping_node/elevation_map_raw의 GridMap으로 발행함

[P01825 | 12608:12661 | NORMAL_TEXT | LIST id=kix.8xzqgjec4tr7 level=0]
terrain_cost_bridge.py가 layer를 찾아 OccupancyGrid로 변환함

[P01826 | 12661:12714 | NORMAL_TEXT | LIST id=kix.8xzqgjec4tr7 level=0]
유한하지 않은 값은 nan_fill=75로 채우고, 값은 반올림 및 0..100 범위로 제한함

[P01827 | 12714:12778 | NORMAL_TEXT | LIST id=kix.8xzqgjec4tr7 level=0]
bridge는 cost를 새로 계산하지 않고, transpose, flip_x, flip_y로 배열 방향만 보정함

[P01828 | 12778:12836 | NORMAL_TEXT | LIST id=kix.8xzqgjec4tr7 level=0]
/terrain_cost/occupancy가 local costmap의 StaticLayer 입력이 됨

[P01829 | 12836:12837 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01830 | 12837:12959 | NORMAL_TEXT]
설정의 trinary_costmap=false는 terrain cost의 연속적인 값을 보존하려는 구성임. lethal_cost_threshold=99가 적용되며, bridge는 unknown을 -1로 내보내지 않음.

[P01831 | 12959:12971 | HEADING_4]
STVL branch

[P01832 | 12971:13220 | NORMAL_TEXT]
STVL은 같은 /grit_slam/cloud_registered_body를 직접 받아 3D voxel obstacle을 생성함. 설정은 min_obstacle_height=0.12, max_obstacle_height=1.0, obstacle_range=5.0, clear_after_reading=true를 사용함. EMC의 terrain cost 결과를 다시 읽는 것이 아니라, cloud에서 독립적으로 obstacle layer를 만듦.

[P01833 | 13220:13235 | HEADING_4]
Global costmap

[P01834 | 13235:13496 | NORMAL_TEXT]
global costmap은 map frame의 100 m × 100 m rolling window임. plugins에는 InflationLayer만 있으므로 local costmap처럼 EMC terrain layer와 STVL을 직접 포함하지 않음. 전역 planner가 사용하는 장애물 표현은 이 설정과 upstream Nav2 기본 동작에 의해 결정되므로, global costmap이 local terrain cost를 자동으로 복사한다고 해석하면 안 됨.

[P01835 | 13496:13518 | HEADING_2]
8. 속도 명령과 safety gate

[P01836 | 13518:13555 | HEADING_3]
8.1 Nav2 cmd_vel에서 Unitree Request까지

[P01837 | 13555:13699 | NORMAL_TEXT]
bringup.launch.py에서 controller의 /cmd_vel을 /path_follower/cmd_vel_nav2로 remap함. unitree_cmd_vel_driver가 이 topic을 받아 unitree_api/msg/Request로 바꿈.

[P01838 | 13699:13700 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01839 | 13700:13737 | NORMAL_TEXT]
driver는 Twist의 세 값을 아래 JSON 필드로 복사함.

[P01840 | 13737:13738 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01841 | 13738:13741 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@13738:13739 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧{

[P01842 | 13741:13766 | NORMAL_TEXT]
  "x": "Twist.linear.x",

[P01843 | 13766:13791 | NORMAL_TEXT]
  "y": "Twist.linear.y",

[P01844 | 13791:13816 | NORMAL_TEXT]
  "z": "Twist.angular.z"

[P01845 | 13816:13818 | NORMAL_TEXT]
}

[P01846 | 13818:13820 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@13818:13819 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P01847 | 13820:13846 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
Unitree move API id: 1008

[P01848 | 13846:13883 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
output topic: /path_follower/request

[P01849 | 13883:13904 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
기본 rate limit: 10 Hz

[P01850 | 13904:13931 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
작은 명령 차이는 1e-3 이하를 정지로 판단함

[P01851 | 13931:13953 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
연속된 정지 명령은 반복 발행하지 않음

[P01852 | 13953:13996 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
이동 중 명령은 move_command_rate_hz보다 자주 발행하지 않음

[P01853 | 13996:14032 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
정지로 바뀌는 순간은 rate limit을 기다리지 않고 발행함

[P01854 | 14032:14033 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01855 | 14033:14114 | NORMAL_TEXT]
driver는 로봇 API를 직접 호출하지 않음. /path_follower/request는 별도 emergency-stop 컨테이너의 입력임.

[P01856 | 14114:14142 | HEADING_3]
8.2 emergency-stop의 gate 동작

[P01857 | 14142:14212 | NORMAL_TEXT]
emergency_stop_node는 navigation 컨테이너와 분리된 별도 container의 process로 실행됨.

[P01858 | 14212:14214 | NORMAL_TEXT]
[INLINE_OBJECT kix.qu6by4ept3y3]

[P01859 | 14214:14228 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@14214:14215 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧flowchart LR

[P01860 | 14228:14259 | NORMAL_TEXT]
    N[/path_follower/request/]

[P01861 | 14259:14279 | NORMAL_TEXT]
    G{Safety state}

[P01862 | 14279:14306 | NORMAL_TEXT]
    F[/api/sport/request/]

[P01863 | 14306:14332 | NORMAL_TEXT]
    S[Stop Move API 1003]

[P01864 | 14332:14344 | NORMAL_TEXT]
    N --> G

[P01865 | 14344:14364 | NORMAL_TEXT]
    G -->|NORMAL| F

[P01866 | 14364:14387 | NORMAL_TEXT]
    G -->|EMERGENCY| S

[P01867 | 14387:14389 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@14387:14388 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P01868 | 14389:14410 | NORMAL_TEXT]
node는 아래 다섯 항목을 감시함.

[P01869 | 14410:14411 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01870 | 14411:14461 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
raw odometry와 optimized odometry의 timestamp 최신 여부

[P01871 | 14461:14495 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
raw와 optimized position 사이의 위치 차이

[P01872 | 14495:14526 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
optimized odometry에서 계산한 물리 속도

[P01873 | 14526:14557 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
lf/sportmodestate의 roll, pitch

[P01874 | 14557:14603 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
Nav2 costmap의 TF cache error가 /rosout에 남는지 여부

[P01875 | 14603:14604 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01876 | 14604:14825 | NORMAL_TEXT]
100 ms timer마다 안전 상태를 갱신하고 /safety/status를 발행함. EMERGENCY이면 stop API 1003을 반복 발행함. NORMAL이면 navigation request를 /api/sport/request로 전달함. emergency 전환 전에 관찰한 gait mode API와 parameter를 저장하고, recovery 때 마지막 gait mode를 재적용함.

[P01877 | 14825:14826 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01878 | 14826:14929 | NORMAL_TEXT]
navigation process의 명령 발행과 로봇의 실제 수행은 서로 다른 단계임. 최종 로봇 구동은 safety gate 상태와 Unitree API 전달 경로까지 통과해야 함.

[P01879 | 14929:14942 | HEADING_2]
9. 실행 모드별 차이

[P01880 | 14942:15025 | NORMAL_TEXT]
deploy/entrypoint.nav.sh는 동일한 navigation image에서 NAV_MODE에 따라 launch package를 선택함.

[P01881 | 15025:15026 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01882 | 15029:15038 | NORMAL_TEXT | TABLE row=0 col=0]
NAV_MODE

[P01883 | 15039:15045 | NORMAL_TEXT | TABLE row=0 col=1]
실행 경로

[P01884 | 15046:15054 | NORMAL_TEXT | TABLE row=0 col=2]
외부 SLAM

[P01885 | 15055:15058 | NORMAL_TEXT | TABLE row=0 col=3]
상태

[P01886 | 15060:15072 | NORMAL_TEXT | TABLE row=1 col=0]
nav2_native

[P01887 | 15073:15156 | NORMAL_TEXT | TABLE row=1 col=1]
waypoint router, 선택적 CoBiz, ThetaStar planner, MPPI controller, EMC bridge, driver

[P01888 | 15157:15160 | NORMAL_TEXT | TABLE row=1 col=2]
필요

[P01889 | 15161:15164 | NORMAL_TEXT | TABLE row=1 col=3]
기본

[P01890 | 15166:15182 | NORMAL_TEXT | TABLE row=2 col=0]
teamgrit_native

[P01891 | 15183:15244 | NORMAL_TEXT | TABLE row=2 col=1]
waypoint router, 선택적 CoBiz, TeamGRIT pure-pursuit controller

[P01892 | 15245:15248 | NORMAL_TEXT | TABLE row=2 col=2]
필요

[P01893 | 15249:15263 | NORMAL_TEXT | TABLE row=2 col=3]
별도 controller

[P01894 | 15265:15276 | NORMAL_TEXT | TABLE row=3 col=0]
standalone

[P01895 | 15277:15313 | NORMAL_TEXT | TABLE row=3 col=1]
launch argument만 선언하고 node는 시작하지 않음

[P01896 | 15314:15322 | NORMAL_TEXT | TABLE row=3 col=2]
사용하지 않음

[P01897 | 15323:15331 | NORMAL_TEXT | TABLE row=3 col=3]
진입점만 존재

[P01898 | 15332:15333 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01899 | 15333:15521 | NORMAL_TEXT]
따라서 NAV_MODE=teamgrit_native일 때는 Nav2 planner, local costmap, path bridge action 경로가 실행되지 않음. 공통으로 사용하는 외부 경계는 waypoint router, CoBiz plugin, safety container이며, controller와 명령 생성 방식이 다름.

[P01900 | 15521:15531 | HEADING_2]
10. 코드 경로

[P01901 | 15531:15579 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
./teamgrit-navigation/deploy/docker-compose.yml

[P01902 | 15579:15626 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
./teamgrit-navigation/deploy/entrypoint.nav.sh

[P01903 | 15626:15679 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
./teamgrit-navigation/deploy/verify_nav2_dataflow.sh

[P01904 | 15679:15751 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
./teamgrit-navigation/nav/src/mode/nav2_native/launch/bringup.launch.py

[P01905 | 15751:15828 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
./teamgrit-navigation/nav/src/mode/nav2_native/src/nav2_path_bridge_node.cpp

[P01906 | 15828:15906 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
./teamgrit-navigation/nav/src/mode/nav2_native/scripts/terrain_cost_bridge.py

[P01907 | 15906:15977 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
./teamgrit-navigation/nav/src/mode/nav2_native/config/nav2_native.yaml

[P01908 | 15977:16052 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
./teamgrit-navigation/nav/src/mode/nav2_native/config/nav2_native.go2.yaml

[P01909 | 16052:16126 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
./teamgrit-navigation/nav/src/mode/nav2_native/config/nav2_native.a2.yaml

[P01910 | 16126:16193 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
./teamgrit-navigation/nav/src/core/src/waypoint_publisher_node.cpp

[P01911 | 16193:16257 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
./teamgrit-navigation/nav/src/core/src/path_simplifier_node.cpp

[P01912 | 16257:16319 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
./teamgrit-navigation/nav/src/cobiz/src/cobiz_plugin_node.cpp

[P01913 | 16319:16387 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
./teamgrit-navigation/nav/src/driver/src/unitree_cmd_vel_driver.cpp

[P01914 | 16387:16476 | NORMAL_TEXT | LIST id=kix.hi4hi1xgenp level=0]
./teamgrit-navigation/emergency-stop/src/grit_emergency_stop/src/emergency_stop_node.cpp

[P01915 | 16476:16477 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## 2026.08.25 Line Tracking (t.qv4xbd1nb881)

[P01916 | 1:26 | TITLE]
2026.08.25 Line Tracking

[P01917 | 26:46 | NORMAL_TEXT | LIST id=kix.p47ffyurvp0a level=0]
Created: 2026.08.25

[P01918 | 46:66 | NORMAL_TEXT | LIST id=kix.p47ffyurvp0a level=0]
Updated: 2026.08.26

[P01919 | 66:76 | NORMAL_TEXT | LIST id=kix.p47ffyurvp0a level=0]
Author: @MinWoo Kang <mwkang@teamgrit.kr>

[P01920 | 76:94 | NORMAL_TEXT | LIST id=kix.63fvyrcxv1fb level=0]
LLM: GPT-Sol 5.6 

[P01921 | 94:96 | NORMAL_TEXT]
[HORIZONTAL_RULE]

[P01922 | 96:119 | HEADING_2]
1. Line-tracking 구현 요약

[P01923 | 119:123 | HEADING_3]
목표 

[P01924 | 123:219 | NORMAL_TEXT | LIST id=kix.1pg5t8pwkjq7 level=0]
카메라로 아스팔트 위 노란색 선을 검출하고, 선 중심과 선의 진행 방향을 이용해 4족보행 로봇의 body-frame 속도 명령(vx, vy, yaw_rate)을 생성한다.

[P01925 | 219:246 | NORMAL_TEXT | LIST id=kix.1pg5t8pwkjq7 level=0]
AprilTag 인식시 Task 종료 해야한다.

[P01926 | 246:252 | HEADING_3]
구현 흐름

[P01927 | 252:300 | NORMAL_TEXT | LIST id=kix.rvi9cwk8f4rs level=0]
영상 전처리: 영상 하단의 사다리꼴 ROI(Region of Interest)만 사용

[P01928 | 300:306 | NORMAL_TEXT | LIST id=kix.rvi9cwk8f4rs level=0]
선 검출:

[P01929 | 306:313 | NORMAL_TEXT | LIST id=kix.rvi9cwk8f4rs level=1]
카메라 영상

[P01930 | 313:333 | NORMAL_TEXT | LIST id=kix.rvi9cwk8f4rs level=1]
→ road segmentation

[P01931 | 333:347 | NORMAL_TEXT | LIST id=kix.rvi9cwk8f4rs level=1]
→ road 영역만 추출

[P01932 | 347:374 | NORMAL_TEXT | LIST id=kix.rvi9cwk8f4rs level=1]
→ 그 영역에서 line segmentation

[P01933 | 374:487 | NORMAL_TEXT | LIST id=kix.rvi9cwk8f4rs level=0]
중심 계산: 가까운 ROI와 먼 ROI를 나누어 각 선 중심(cx_near, cx_far)을 구한다. 가까운 중심은 현재 좌우 위치 보정에 사용하고, 두 중심의 차이는 선의 방향 변화 판단에 사용한다.

[P01934 | 487:595 | NORMAL_TEXT | LIST id=kix.rvi9cwk8f4rs level=0]
명령 생성: vx는 일정한 전진 속도로 두고, 선 중심에서 벗어난 정도에 비례해 vy를 생성한다. 선이 꺾이는 방향은 가까운 중심과 먼 중심의 차이로 계산해 yaw_rate를 천천히 생성한다.

[P01935 | 595:652 | NORMAL_TEXT | LIST id=kix.rvi9cwk8f4rs level=1]
lateral_error = -(cx_near - image_center) / image_center

[P01936 | 652:709 | NORMAL_TEXT | LIST id=kix.rvi9cwk8f4rs level=1]
heading_error = -atan2(cx_far - cx_near, y_near - y_far)

[P01937 | 709:756 | NORMAL_TEXT | LIST id=kix.rvi9cwk8f4rs level=1]
vy = clip(Ky * lateral_error, -vy_max, vy_max)

[P01938 | 756:813 | NORMAL_TEXT | LIST id=kix.rvi9cwk8f4rs level=1]
yaw_rate = clip(Kyaw * heading_error, -yaw_max, yaw_max)

[P01939 | 813:934 | NORMAL_TEXT | LIST id=kix.rvi9cwk8f4rs level=0]
안정화: vy와 yaw_rate에 deadband, low-pass filter, command rate limit을 적용해 보행 중 명령이 갑자기 변하지 않도록 한다. 오차나 선의 굽힘이 클 때는 vx를 감속한다.

[P01940 | 934:943 | HEADING_2]
2. 구현 현황

[P01941 | 943:1035 | NORMAL_TEXT]
Test Source : [https://github.com/teamgrit-lab/cobiz-plugin-line-tracking/tree/master/tools](https://github.com/teamgrit-lab/cobiz-plugin-line-tracking/tree/master/tools) 

[P01942 | 1035:1051 | NORMAL_TEXT]
Test Result : [line-tracking 결과 비디오](https://drive.google.com/drive/folders/1CqMwVsEZIPgfVC93j6ft2vKGAL2tTts_)

[P01943 | 1051:1076 | NORMAL_TEXT]
프로젝트 폴더에서 아래처럼 실행하면 됩니다.

[P01944 | 1076:1142 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@1076:1077 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧cd /Users/kangminwoo/Documents/GitHub/cobiz-plugin-line-tracking

[P01945 | 1142:1143 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01946 | 1143:1182 | NORMAL_TEXT]
uv run python tools/segment_video.py \

[P01947 | 1182:1203 | NORMAL_TEXT]
  --backend opencv \

[P01948 | 1203:1254 | NORMAL_TEXT]
  --input "/Users/kangminwoo/Downloads/다른영상.mp4" \

[P01949 | 1254:1312 | NORMAL_TEXT]
  --output "/Users/kangminwoo/Downloads/다른영상_overlay.mp4"

[P01950 | 1312:1375 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@1312:1313 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧다른영상.mp4 부분만 확인할 영상 경로로 바꾸면 됩니다. 결과는 다른영상_overlay.mp4로 생성됩니다.

[P01951 | 1375:1383 | NORMAL_TEXT]
[line-seg-color_overay3.mp4](https://drive.google.com/file/d/1vjFIkcYsK9cig0hW13eNvUynagh69FxG/view?usp=drive_link), [line-seg-color_overay2.mp4](https://drive.google.com/file/d/1vEaZQ2ZkI9zzpuDPL5KGHqKTqQ_4pSsp/view?usp=drive_link), [line-seg-color_overay1.mp4](https://drive.google.com/file/d/1GCno5aRY0ynwhhBz7K8_pVF6r5u6IsaB/view?usp=drive_link)

[P01952 | 1383:1392 | NORMAL_TEXT]
잔디 등 오검출

[P01953 | 1392:1399 | NORMAL_TEXT]
[line-seg-feature_overay1.mp4](https://drive.google.com/file/d/1SpItYXj3iKDsM_BLkJ9MB3c7t437RDk6/view?usp=drive_link), [line-seg-feature_overay2.mp4](https://drive.google.com/file/d/1kIBoaW9n2FS2u6UIFKNB8slU5tswUvLs/view?usp=drive_link),[line-seg-feature_overay3.mp4](https://drive.google.com/file/d/11lZV05NJA8eMYqjKv-tmI_PyuI3fDAjU/view?usp=drive_link)

[P01954 | 1399:1501 | NORMAL_TEXT]
line의 feature 정보를 추가해서 segmentation 한 결과입니다. 중앙선이 ROI 로 들어가면 그 중앙선을 잘 찾는데 그 이외의 상황에서의 오검출이 발생하고 있습니다.

[P01955 | 1501:1502 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P01956 | 1502:1503 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## 2026.08.25 Tag Detection 시 Task_Complete 구현 관련 (t.4ilt33oibm5c)

[P01957 | 1:46 | TITLE]
2026.08.25 Tag Detection 기반 Task_complete 설계

[P01958 | 46:66 | NORMAL_TEXT | LIST id=kix.3m34d13rt4oc level=0]
Created: 2026.08.25

[P01959 | 66:86 | NORMAL_TEXT | LIST id=kix.3m34d13rt4oc level=0]
Updated: 2026.08.26

[P01960 | 86:96 | NORMAL_TEXT | LIST id=kix.3m34d13rt4oc level=0]
Author: @MinWoo Kang <mwkang@teamgrit.kr>

[P01961 | 96:114 | NORMAL_TEXT | LIST id=kix.3m34d13rt4oc level=0]
LLM: GPT-Sol 5.6 

[P01962 | 114:116 | NORMAL_TEXT]
[HORIZONTAL_RULE]

[P01963 | 116:127 | HEADING_2]
1. 목적 및 결론

[P01964 | 127:261 | NORMAL_TEXT]
이 문서는 주행 중 Path_following Task가 활성화된 상태에서 “relocalization에 사용할 수 있는 AprilTag가 허용 거리 안에 들어왔다”는 조건을 판정하고 Task_complete를 발생시키는 기준을 정의한다.

[P01965 | 261:298 | NORMAL_TEXT | LIST id=kix.m73u62a0xmfn level=0]
주 입력 — /fiducial_localizer/tag_point

[P01966 | 298:368 | NORMAL_TEXT | LIST id=kix.m73u62a0xmfn level=0]
완료 기준 — 동일한 Tag의 유효 관측이 최근 1.0초 동안 5회 이상이고 거리 중앙값과 최신 거리가 모두 4.0 m 이하

[P01967 | 368:458 | NORMAL_TEXT | LIST id=kix.m73u62a0xmfn level=0]
완료 전달 — /task_event로 Task_complete 발행 후 Cobiz-Core가 POST /api/tasks/{task_id}/complete 호출

[P01968 | 458:526 | NORMAL_TEXT | LIST id=kix.m73u62a0xmfn level=0]
판정 의미 — 실제 anchor solve 완료가 아니라 localization 입력으로 채택된 Tag에 접근했다는 의미

[P01969 | 526:631 | NORMAL_TEXT]
따라서 Task_complete의 직접 트리거는 /detections나 /tag_map_anchor/status가 아니라 /fiducial_localizer/tag_point로 통일한다.

[P01970 | 631:664 | HEADING_2]
2. /fiducial_localizer/tag_point

[P01971 | 664:698 | NORMAL_TEXT | LIST id=kix.1xbzmys54atr level=0]
Publisher — fiducial_localizer 노드

[P01972 | 698:744 | NORMAL_TEXT | LIST id=kix.1xbzmys54atr level=0]
Message type — geometry_msgs/msg/PointStamped

[P01973 | 744:789 | NORMAL_TEXT | LIST id=kix.1xbzmys54atr level=0]
QoS — RELIABLE, VOLATILE, KEEP_LAST depth 10

[P01974 | 789:873 | NORMAL_TEXT | LIST id=kix.1xbzmys54atr level=0]
Topic 이름 — 노드의 private topic ~/tag_point가 /fiducial_localizer/tag_point로 resolve된다.

[P01975 | 873:1006 | NORMAL_TEXT]
이 토픽은 카메라 영상에서 보인 모든 Tag를 전달하지 않는다. 지도에 등록되어 있고 검출 품질, 거리 및 TF freshness 조건을 통과해 fiducial_localizer가 localization 입력으로 채택한 관측만 전달한다.

[P01976 | 1006:1020 | HEADING_3]
2.1 메시지 필드 의미

[P01977 | 1020:1101 | NORMAL_TEXT | LIST id=kix.op2dhem5aaam level=0]
header.stamp — 원본 AprilTag detection의 시각이다. Tag TF와 로봇 pose를 같은 시각으로 조회할 때 사용한다.

[P01978 | 1101:1170 | NORMAL_TEXT | LIST id=kix.op2dhem5aaam level=0]
header.frame_id — tag_15와 같은 Tag frame 이름이며, 구현에서는 여기서 Tag ID를 파싱한다.

[P01979 | 1170:1268 | NORMAL_TEXT | LIST id=kix.op2dhem5aaam level=0]
point.x, point.y, point.z — body_frame 기준으로 본 Tag의 상대 위치 벡터 T_body_tag의 translation 성분이며 단위는 m이다.

[P01980 | 1268:1349 | NORMAL_TEXT | LIST id=kix.op2dhem5aaam level=0]
distance_m — sqrt(point.x² + point.y² + point.z²)로 계산하는 로봇 body와 Tag 사이의 직선거리이다.

[P01981 | 1349:1513 | NORMAL_TEXT]
주의: header.frame_id는 Tag ID 전달 용도로도 사용되지만 point 값 자체는 tag frame 기준 좌표가 아니라 body_frame 기준 좌표이다. 일반적인 PointStamped 해석대로 frame_id를 좌표계로 간주해 다시 TF 변환하면 잘못된 결과가 될 수 있다.

[P01982 | 1513:1535 | HEADING_2]
3. 이 메시지가 발행되는 조건과 시점

[P01983 | 1535:1651 | NORMAL_TEXT]
fiducial_localizer는 /detections 메시지를 받을 때마다 각 검출을 다음 순서로 검사하며, 모든 조건을 통과한 검출마다 /fiducial_localizer/tag_point를 발행한다.

[P01984 | 1651:1682 | NORMAL_TEXT | LIST id=kix.cmfk3hpfm4pb level=0]
Localization 파이프라인이 실행중이여야 한다.

[P01985 | 1682:1737 | NORMAL_TEXT | LIST id=kix.cmfk3hpfm4pb level=0]
Tag ID가 현재 map_origin.yaml의 사용 가능한 anchor 목록에 등록되어 있다.

[P01986 | 1737:1780 | NORMAL_TEXT | LIST id=kix.cmfk3hpfm4pb level=0]
decision_margin이 min_decision_margin 이상이다.

[P01987 | 1780:1871 | NORMAL_TEXT | LIST id=kix.cmfk3hpfm4pb level=0]
검출 시각의 body_frame ← tag_frame TF를 얻을 수 있고, 최신 TF fallback을 사용한 경우에도 max_tag_tf_age_s 이내이다.

[P01988 | 1871:1906 | NORMAL_TEXT | LIST id=kix.cmfk3hpfm4pb level=0]
Tag 거리 norm이 max_tag_range_m 이하이다.

[P01989 | 1906:1960 | NORMAL_TEXT | LIST id=kix.cmfk3hpfm4pb level=0]
위 조건을 통과하면 Tag ID와 body 기준 상대 위치를 PointStamped로 발행한다.

[P01990 | 1960:2125 | NORMAL_TEXT]
A2에서 확인한 실행 파라미터는 max_tag_range_m 4.0 m, min_decision_margin 10.0, max_tag_tf_age_s 0.2초, publish_once false이다. 따라서 배포 환경에서는 유효한 관측이 이어지는 동안 tag_point도 반복 발행될 수 있다.

[P01991 | 2125:2240 | NORMAL_TEXT]
단, 발행 주기는 고정 timer가 아니라 카메라 검출 성공 여부에 종속된다. Tag가 가려지거나 흔들림·블러·반사·급격한 시야각 변화·TF 지연이 생기면 가까이 있어도 메시지가 일시적으로 끊길 수 있다.

[P01992 | 2240:2327 | NORMAL_TEXT]
저장소의 기본값이나 YAML 값과 실제 컨테이너 런타임 파라미터가 다를 수 있으므로 운영 판단에는 ros2 param get으로 확인한 실행값을 우선한다.

[P01993 | 2327:2343 | HEADING_2]
4. 거리와 상대 위치 해석

[P01994 | 2343:2401 | NORMAL_TEXT | LIST id=kix.zgdvsv19rzjh level=0]
상대 위치 — point 벡터로 로봇 body 기준 Tag의 x, y, z 방향과 거리를 알 수 있다.

[P01995 | 2401:2458 | NORMAL_TEXT | LIST id=kix.zgdvsv19rzjh level=0]
거리 — 벡터 norm을 사용하므로 로봇 또는 Tag의 회전에 무관하게 접근 여부를 판정할 수 있다.

[P01996 | 2458:2560 | NORMAL_TEXT | LIST id=kix.zgdvsv19rzjh level=0]
방향 축 — x/y/z의 실제 방향은 설정된 body_frame의 축 정의를 따른다. 일반적인 base_link는 x 전방, y 좌측, z 상방이지만 실제 TF를 기준으로 확인한다.

[P01997 | 2560:2652 | NORMAL_TEXT | LIST id=kix.zgdvsv19rzjh level=0]
Orientation — PointStamped에는 quaternion이 없으므로 이 메시지만으로 Tag나 카메라의 roll, pitch, yaw를 구할 수 없다.

[P01998 | 2652:2833 | NORMAL_TEXT | LIST id=kix.zgdvsv19rzjh level=0]
전역 pose — 저장된 T_map_tag와 전체 T_body_tag가 있으면 T_map_body = T_map_tag × inverse(T_body_tag)로 계산할 수 있지만, tag_point에는 translation만 있으므로 외부 consumer가 이 메시지만으로 완전한 6-DoF pose를 재구성할 수는 없다.

[P01999 | 2833:2911 | NORMAL_TEXT]
Task_complete 판정에는 orientation이 필요하지 않으므로 point의 norm만 사용하는 것이 가장 단순하고 안정적이다.

[P02000 | 2911:2943 | HEADING_2]
5. Tag 검출 및 relocalization과의 관계

[P02001 | 2943:3051 | NORMAL_TEXT | LIST id=kix.lcajdxftkc2m level=0]
/detections — 영상에서 Tag가 검출되고 apriltag_node의 ID 및 Hamming filter를 통과한 원시 검출 결과이다. 빈 detections 배열도 발행될 수 있다.

[P02002 | 3051:3167 | NORMAL_TEXT | LIST id=kix.lcajdxftkc2m level=0]
/fiducial_localizer/tag_point — 지도 등록, decision margin, 거리, Tag TF freshness gate를 모두 통과한 localization용 상대 위치 관측이다.

[P02003 | 3167:3265 | NORMAL_TEXT | LIST id=kix.lcajdxftkc2m level=0]
/tag_map_anchor/status — anchor solve가 성공할 때 data: true가 이벤트성으로 발행된다. 지속 상태가 아니며 false는 발행하지 않는다.

[P02004 | 3265:3316 | NORMAL_TEXT | LIST id=kix.lcajdxftkc2m level=0]
prior_map → map — 실제 anchor 보정이 적용된 결과를 확인하는 TF이다.

[P02005 | 3316:3380 | NORMAL_TEXT | LIST id=kix.lcajdxftkc2m level=0]
/tag_map_anchor/pose — anchoring 이후 prior_map 기준 로봇의 전역 pose이다.

[P02006 | 3380:3604 | NORMAL_TEXT]
tag_point 수신은 “relocalization에 사용할 수 있는 관측이 생겼다”는 뜻이지 “relocalization 또는 anchor solve가 완료됐다”는 뜻은 아니다. 최초 full anchor는 최근 창의 최소 관측 수와 서로 다른 저장 Tag 간 XY baseline 등의 추가 조건을 요구할 수 있어, Tag 하나만 가까이 보이는 상황에서는 status가 발행되지 않을 수 있다.

[P02007 | 3604:3777 | NORMAL_TEXT]
현재 요구사항은 가까운 유효 Tag 도달 시 Task를 끝내는 것이므로 /tag_map_anchor/status를 필수 조건으로 두지 않는다. 요구사항이 “실제 anchor solve 성공 후 완료”로 바뀌면 status의 data: true 또는 prior_map → map 갱신을 추가 조건으로 사용한다.

[P02008 | 3777:3802 | HEADING_2]
6. 권장 Task_complete 알고리즘

[P02009 | 3802:3812 | HEADING_3]
6.1 판정 조건

[P02010 | 3812:3854 | NORMAL_TEXT]
다음 조건을 모두 만족할 때만 Task_complete를 한 번 발행한다.

[P02011 | 3854:3884 | NORMAL_TEXT | LIST id=kix.3kc6ph4tbnw2 level=0]
현재 Task 상태가 Path_following이다.

[P02012 | 3884:3926 | NORMAL_TEXT | LIST id=kix.3kc6ph4tbnw2 level=0]
동일한 tag_id의 유효 관측이 최근 1.0초 안에 5개 이상 존재한다.

[P02013 | 3926:3977 | NORMAL_TEXT | LIST id=kix.3kc6ph4tbnw2 level=0]
최근 5개 거리의 median이 4.0 m 이하이고 가장 최신 거리도 4.0 m 이하이다.

[P02014 | 3977:4027 | NORMAL_TEXT | LIST id=kix.3kc6ph4tbnw2 level=0]
연속 관측 간격이 0.5초를 넘지 않는다. 초과하면 해당 Tag의 관측 창을 초기화한다.

[P02015 | 4027:4065 | NORMAL_TEXT | LIST id=kix.3kc6ph4tbnw2 level=0]
현재 Task에서 Task_complete를 아직 발행하지 않았다.

[P02016 | 4065:4205 | NORMAL_TEXT]
4.0 m는 현재 fiducial_localizer의 최대 수용 거리와 같은 값이므로 tag_point가 수신된 시점에 대부분 이미 만족한다. 그래도 consumer에 같은 상한을 명시하면 런타임 설정 변경이나 비정상 입력에 대한 방어 조건이 된다.

[P02017 | 4205:4215 | HEADING_3]
6.2 처리 흐름

[P02018 | 4215:4292 | NORMAL_TEXT | LIST id=kix.531z4uxte1zb level=0]
Task manager가 Path_following 시작 시 Tag별 sliding window와 complete_sent를 초기화한다.

[P02019 | 4292:4374 | NORMAL_TEXT | LIST id=kix.531z4uxte1zb level=0]
tag_point callback에서 header.frame_id로 tag_id를 구하고 point의 norm으로 distance_m을 계산한다.

[P02020 | 4374:4427 | NORMAL_TEXT | LIST id=kix.531z4uxte1zb level=0]
동일 Tag의 최근 1.0초 관측만 유지하고 0.5초보다 긴 공백이 생기면 해당 창을 비운다.

[P02021 | 4427:4506 | NORMAL_TEXT | LIST id=kix.531z4uxte1zb level=0]
완료 조건을 만족하면 /task_event로 Task_complete를 한 번 발행하고 complete_sent를 true로 latch한다.

[P02022 | 4506:4573 | NORMAL_TEXT | LIST id=kix.531z4uxte1zb level=0]
Cobiz-Core는 이벤트를 수신해 POST /api/tasks/{task_id}/complete를 서버로 전송한다.

[P02023 | 4573:4623 | NORMAL_TEXT | LIST id=kix.531z4uxte1zb level=0]
새 Task 시작, 취소 또는 재시작 시 latch와 모든 Tag 관측 창을 초기화한다.

[P02024 | 4623:4635 | HEADING_2]
7. 운영 확인 방법

[P02025 | 4635:4695 | NORMAL_TEXT | LIST id=kix.cj5vxp3w5vc9 level=0]
토픽 타입/QoS: ros2 topic info /fiducial_localizer/tag_point -v

[P02026 | 4695:4749 | NORMAL_TEXT | LIST id=kix.cj5vxp3w5vc9 level=0]
관측값 확인: ros2 topic echo /fiducial_localizer/tag_point

[P02027 | 4749:4803 | NORMAL_TEXT | LIST id=kix.cj5vxp3w5vc9 level=0]
발행 빈도 확인: ros2 topic hz /fiducial_localizer/tag_point

[P02028 | 4803:4866 | NORMAL_TEXT | LIST id=kix.cj5vxp3w5vc9 level=0]
실행 파라미터 확인: ros2 param get /fiducial_localizer max_tag_range_m

[P02029 | 4866:4933 | NORMAL_TEXT | LIST id=kix.cj5vxp3w5vc9 level=0]
실행 파라미터 확인: ros2 param get /fiducial_localizer min_decision_margin

[P02030 | 4933:4997 | NORMAL_TEXT | LIST id=kix.cj5vxp3w5vc9 level=0]
실행 파라미터 확인: ros2 param get /fiducial_localizer max_tag_tf_age_s

[P02031 | 4997:5038 | NORMAL_TEXT | LIST id=kix.cj5vxp3w5vc9 level=0]
Task 이벤트 확인: ros2 topic echo /task_event

[P02032 | 5038:5094 | NORMAL_TEXT | LIST id=kix.cj5vxp3w5vc9 level=0]
Anchor solve 확인: ros2 topic echo /tag_map_anchor/status

[P02033 | 5094:5147 | NORMAL_TEXT | LIST id=kix.cj5vxp3w5vc9 level=0]
실제 보정 TF 확인: ros2 run tf2_ros tf2_echo prior_map map

[P02034 | 5147:5331 | NORMAL_TEXT]
A2 현장 관측에서는 Tag 15가 약 1.21 m에 있을 때 tag_point가 약 15~16 Hz로 들어오는 구간이 있었지만, 20초 동안 최대 약 2.36초의 공백도 확인됐다. 따라서 “현재 메시지가 계속 나오고 있는가”를 boolean 상태처럼 사용하지 말고 sliding window와 timeout으로 판정해야 한다.

[P02035 | 5331:5475 | NORMAL_TEXT]
같은 관측 구간에 tag_point는 발행됐지만 단일 Tag만 보이는 동안 /tag_map_anchor/status가 나오지 않은 사례가 있었다. 이는 tag_point 기반 완료 조건과 실제 full anchor 성공 조건이 서로 다르다는 것을 보여준다.

## 2026.08.25 Tag Detection 시 Task_Complete-chatgpt (t.ozq8o470652g)

[P02036 | 1:161 | NORMAL_TEXT]
Q : teamgrit-slam 에서 localization 중에 Tag를 detecting 하면 relocalization 한다든가 그런걸 확인할수 있는 토픽이나 그런게 있을까?A : 네, 확인할 수 있습니다. 가장 직접적인 토픽은 /tag_map_anchor/status입니다.

[P02037 | 161:200 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@161:162 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧# 태그가 실제 localization solve에 사용됐는지 확인

[P02038 | 200:239 | NORMAL_TEXT]
ros2 topic echo /tag_map_anchor/status

[P02039 | 239:347 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@239:240 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧data: true가 새로 발행되면, 유효한 태그 관측으로 tag-geometry solve가 성공했다는 뜻입니다. 다만 지속 상태가 아니라 이벤트성 신호이며 false는 발행하지 않습니다.

[P02040 | 347:365 | NORMAL_TEXT]
단계별로 보면 다음과 같습니다.

[P02041 | 365:398 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@365:366 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧# 1. AprilTag detector가 태그를 봤는지

[P02042 | 398:426 | NORMAL_TEXT]
ros2 topic echo /detections

[P02043 | 426:427 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02044 | 427:465 | NORMAL_TEXT]
# 2. 지도에 등록된 태그이고, 품질/거리/TF 검사를 통과했는지

[P02045 | 465:511 | NORMAL_TEXT]
ros2 topic echo /fiducial_localizer/tag_point

[P02046 | 511:512 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02047 | 512:543 | NORMAL_TEXT]
# 3. 태그 기반 anchor solve가 성공했는지

[P02048 | 543:582 | NORMAL_TEXT]
ros2 topic echo /tag_map_anchor/status

[P02049 | 582:583 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02050 | 583:615 | NORMAL_TEXT]
# 4. 실제 prior_map -> map 보정값 확인

[P02051 | 615:655 | NORMAL_TEXT]
ros2 run tf2_ros tf2_echo prior_map map

[P02052 | 655:656 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02053 | 656:679 | NORMAL_TEXT]
# 5. 보정된 전역 로봇 pose 확인

[P02054 | 679:716 | NORMAL_TEXT]
ros2 topic echo /tag_map_anchor/pose

[P02055 | 716:741 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@716:717 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧판정 기준은 다음처럼 보는 게 정확합니다.

[P02056 | 744:747 | NORMAL_TEXT | TABLE row=0 col=0]
관측

[P02057 | 748:751 | NORMAL_TEXT | TABLE row=0 col=1]
의미

[P02058 | 753:769 | NORMAL_TEXT | TABLE row=1 col=0]
/detections만 나옴

[P02059 | 770:814 | NORMAL_TEXT | TABLE row=1 col=1]
카메라가 태그를 인식했지만 localization에는 사용되지 않았을 수 있음

[P02060 | 816:849 | NORMAL_TEXT | TABLE row=2 col=0]
/fiducial_localizer/tag_point 나옴

[P02061 | 850:885 | NORMAL_TEXT | TABLE row=2 col=1]
등록된 태그이며 margin, 3 m 거리, TF 검사를 통과

[P02062 | 887:916 | NORMAL_TEXT | TABLE row=3 col=0]
/tag_map_anchor/status: true

[P02063 | 917:947 | NORMAL_TEXT | TABLE row=3 col=1]
태그 기반 relocalization solve 성공

[P02064 | 949:971 | NORMAL_TEXT | TABLE row=4 col=0]
prior_map -> map 값 변경

[P02065 | 972:987 | NORMAL_TEXT | TABLE row=4 col=1]
보정이 실제 TF에 적용됨

[P02066 | 989:1020 | NORMAL_TEXT | TABLE row=5 col=0]
/tag_map_anchor/pose가 2 Hz로 나옴

[P02067 | 1021:1059 | NORMAL_TEXT | TABLE row=5 col=1]
적어도 한 번 anchoring된 후 전역 pose가 정상 계산 중

[P02068 | 1060:1085 | NORMAL_TEXT]
로그에서도 성공 시 아래 메시지가 나옵니다.

[P02069 | 1085:1104 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@1085:1086 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧ANCHORED[pos] ...

[P02070 | 1104:1124 | NORMAL_TEXT]
ANCHORED[pos-t] ...

[P02071 | 1124:1181 | NORMAL_TEXT]
tag anchor (re)taking prior_map->map [hand-off jump ...]

[P02072 | 1181:1280 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@1181:1182 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧주의할 점은 현재 기본 설정에서 태그 하나를 단순히 검출했다고 바로 최초 relocalization이 되는 구조는 아니라는 것입니다. 최초 full solve에는 기본적으로:

[P02073 | 1280:1290 | NORMAL_TEXT | LIST id=kix.davkyxmecfec level=0]
최소 4개 관측점

[P02074 | 1290:1324 | NORMAL_TEXT | LIST id=kix.davkyxmecfec level=0]
서로 다른 저장 태그 사이의 baseline 1.0 m 이상

[P02075 | 1324:1331 | NORMAL_TEXT | LIST id=kix.davkyxmecfec level=0]
등록된 태그

[P02076 | 1331:1353 | NORMAL_TEXT | LIST id=kix.davkyxmecfec level=0]
decision margin 30 이상

[P02077 | 1353:1365 | NORMAL_TEXT | LIST id=kix.davkyxmecfec level=0]
거리 3.0 m 이하

[P02078 | 1365:1524 | NORMAL_TEXT]
가 필요합니다. 이미 yaw가 한 번 확보된 뒤에는 태그 하나를 다시 봐도 ANCHORED[pos-t] 방식으로 translation drift를 재보정할 수 있습니다. 또한 publish_once: false라서 운행 중 등록 태그를 다시 볼 때마다 계속 보정하도록 되어 있습니다.

[P02079 | 1524:1826 | NORMAL_TEXT]
관련 구현은 [tag_map_anchor_node.py (line 785)](/Users/kangminwoo/Documents/GitHub/teamgrit-slam/slam/src/apriltag_localization/scripts/tag_map_anchor_node.py:785)와 [localizer.yaml (line 117)](/Users/kangminwoo/Documents/GitHub/teamgrit-slam/slam/src/apriltag_localization/config/localizer.yaml:117)에 있습니다.

[P02080 | 1826:1827 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02081 | 1827:1829 | NORMAL_TEXT]
[HORIZONTAL_RULE]

[P02082 | 1829:1830 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02083 | 1830:1834 | NORMAL_TEXT]
---

[P02084 | 1834:1842 | NORMAL_TEXT]
header:

[P02085 | 1842:1851 | NORMAL_TEXT]
  stamp:

[P02086 | 1851:1871 | NORMAL_TEXT]
    sec: 1787631296

[P02087 | 1871:1894 | NORMAL_TEXT]
    nanosec: 398280382

[P02088 | 1894:1927 | NORMAL_TEXT]
  frame_id: camera_optical_frame

[P02089 | 1927:1939 | NORMAL_TEXT]
detections:

[P02090 | 1939:1958 | NORMAL_TEXT]
- family: tag36h11

[P02091 | 1958:1967 | NORMAL_TEXT]
  id: 14

[P02092 | 1967:1980 | NORMAL_TEXT]
  hamming: 0

[P02093 | 1980:1996 | NORMAL_TEXT]
  goodness: 0.0

[P02094 | 1996:2034 | NORMAL_TEXT]
  decision_margin: 141.22950744628906

[P02095 | 2034:2044 | NORMAL_TEXT]
  centre:

[P02096 | 2044:2069 | NORMAL_TEXT]
    x: 539.9546576497592

[P02097 | 2069:2094 | NORMAL_TEXT]
    y: 313.9380018597521

[P02098 | 2094:2105 | NORMAL_TEXT]
  corners:

[P02099 | 2105:2130 | NORMAL_TEXT]
  - x: 500.4846496582032

[P02100 | 2130:2155 | NORMAL_TEXT]
    y: 352.4928588867188

[P02101 | 2155:2177 | NORMAL_TEXT]
  - x: 578.1494140625

[P02102 | 2177:2203 | NORMAL_TEXT]
    y: 354.90008544921875

[P02103 | 2203:2228 | NORMAL_TEXT]
  - x: 579.8098754882812

[P02104 | 2228:2253 | NORMAL_TEXT]
    y: 275.0068664550782

[P02105 | 2253:2279 | NORMAL_TEXT]
  - x: 502.19650268554693

[P02106 | 2279:2304 | NORMAL_TEXT]
    y: 273.4441528320313

[P02107 | 2304:2318 | NORMAL_TEXT]
  homography:

[P02108 | 2318:2340 | NORMAL_TEXT]
  - 35.95549161585643

[P02109 | 2340:2364 | NORMAL_TEXT]
  - -1.0841215890944729

[P02110 | 2364:2386 | NORMAL_TEXT]
  - 539.9546576497592

[P02111 | 2386:2410 | NORMAL_TEXT]
  - -0.6722931171607416

[P02112 | 2410:2433 | NORMAL_TEXT]
  - 39.594298339900604

[P02113 | 2433:2455 | NORMAL_TEXT]
  - 313.9380018597521

[P02114 | 2455:2480 | NORMAL_TEXT]
  - -0.00530220003050106

[P02115 | 2480:2507 | NORMAL_TEXT]
  - -0.0004461174548118029

[P02116 | 2507:2515 | NORMAL_TEXT]
  - 1.0

[P02117 | 2515:2534 | NORMAL_TEXT]
- family: tag36h11

[P02118 | 2534:2543 | NORMAL_TEXT]
  id: 16

[P02119 | 2543:2556 | NORMAL_TEXT]
  hamming: 0

[P02120 | 2556:2572 | NORMAL_TEXT]
  goodness: 0.0

[P02121 | 2572:2609 | NORMAL_TEXT]
  decision_margin: 135.6107635498047

[P02122 | 2609:2619 | NORMAL_TEXT]
  centre:

[P02123 | 2619:2644 | NORMAL_TEXT]
    x: 943.9636376346003

[P02124 | 2644:2669 | NORMAL_TEXT]
    y: 306.3725153707255

[P02125 | 2669:2680 | NORMAL_TEXT]
  corners:

[P02126 | 2680:2705 | NORMAL_TEXT]
  - x: 896.1743164062502

[P02127 | 2705:2730 | NORMAL_TEXT]
    y: 350.1457214355469

[P02128 | 2730:2755 | NORMAL_TEXT]
  - x: 989.8516235351562

[P02129 | 2755:2780 | NORMAL_TEXT]
    y: 351.5604248046875

[P02130 | 2780:2805 | NORMAL_TEXT]
  - x: 993.1154174804688

[P02131 | 2805:2830 | NORMAL_TEXT]
    y: 261.3513488769531

[P02132 | 2830:2855 | NORMAL_TEXT]
  - x: 898.5301513671876

[P02133 | 2855:2880 | NORMAL_TEXT]
    y: 261.6321716308594

[P02134 | 2880:2894 | NORMAL_TEXT]
  homography:

[P02135 | 2894:2916 | NORMAL_TEXT]
  - 38.07780698536328

[P02136 | 2916:2939 | NORMAL_TEXT]
  - 2.8837668777763454

[P02137 | 2939:2961 | NORMAL_TEXT]
  - 943.9636376346003

[P02138 | 2961:2985 | NORMAL_TEXT]
  - -2.6280536788963302

[P02139 | 2985:3008 | NORMAL_TEXT]
  - 46.066275111689436

[P02140 | 3008:3030 | NORMAL_TEXT]
  - 306.3725153707255

[P02141 | 3030:3055 | NORMAL_TEXT]
  - -0.00951570939458645

[P02142 | 3055:3080 | NORMAL_TEXT]
  - 0.004538789702499544

[P02143 | 3080:3088 | NORMAL_TEXT]
  - 1.0

[P02144 | 3088:3092 | NORMAL_TEXT]
---

[P02145 | 3092:3093 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02146 | 3093:3140 | NORMAL_TEXT]
Type: apriltag_msgs/msg/AprilTagDetectionArray

[P02147 | 3140:3159 | NORMAL_TEXT]
Publisher count: 1

[P02148 | 3159:3181 | NORMAL_TEXT]
Subscription count: 1

[P02149 | 3181:3205 | NORMAL_TEXT]
std_msgs/Header header

[P02150 | 3205:3236 | NORMAL_TEXT]
	builtin_interfaces/Time stamp

[P02151 | 3236:3248 | NORMAL_TEXT]
		int32 sec

[P02152 | 3248:3265 | NORMAL_TEXT]
		uint32 nanosec

[P02153 | 3265:3282 | NORMAL_TEXT]
	string frame_id

[P02154 | 3282:3313 | NORMAL_TEXT]
AprilTagDetection[] detections

[P02155 | 3313:3328 | NORMAL_TEXT]
	string family

[P02156 | 3328:3338 | NORMAL_TEXT]
	int32 id

[P02157 | 3338:3353 | NORMAL_TEXT]
	int32 hamming

[P02158 | 3353:3371 | NORMAL_TEXT]
	float32 goodness

[P02159 | 3371:3396 | NORMAL_TEXT]
	float32 decision_margin

[P02160 | 3396:3431 | NORMAL_TEXT]
	Point centre                    #

[P02161 | 3431:3443 | NORMAL_TEXT]
		float64 x

[P02162 | 3443:3455 | NORMAL_TEXT]
		float64 y

[P02163 | 3455:3490 | NORMAL_TEXT]
	Point[4] corners                #

[P02164 | 3490:3502 | NORMAL_TEXT]
		float64 x

[P02165 | 3502:3514 | NORMAL_TEXT]
		float64 y

[P02166 | 3514:3549 | NORMAL_TEXT]
	float64[9] homography           #

[P02167 | 3549:3550 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02168 | 3550:3554 | NORMAL_TEXT]
---

[P02169 | 3554:3611 | NORMAL_TEXT]
detections 배열의 각 항목은 이미지 한 프레임에서 발견된 AprilTag 하나를 나타냅니다.

[P02170 | 3611:3618 | HEADING_3]
family

[P02171 | 3618:3636 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@3618:3619 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧family: tag36h11

[P02172 | 3636:3652 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@3636:3637 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧태그의 코드 패밀리입니다.

[P02173 | 3652:3674 | NORMAL_TEXT | LIST id=kix.a63m2d1o5zu level=0]
36: ID를 표현하는 데이터 비트 수

[P02174 | 3674:3717 | NORMAL_TEXT | LIST id=kix.a63m2d1o5zu level=0]
h11: 유효한 코드끼리 최소 Hamming distance가 11인 패밀리

[P02175 | 3717:3755 | NORMAL_TEXT | LIST id=kix.a63m2d1o5zu level=0]
서로 다른 패밀리는 태그 무늬와 사용 가능한 ID 집합이 다릅니다.

[P02176 | 3755:3789 | NORMAL_TEXT]
현재 프로젝트 설정도 family: 36h11을 사용합니다.

[P02177 | 3789:3792 | HEADING_3]
id

[P02178 | 3792:3800 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@3792:3793 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧id: 14

[P02179 | 3800:3824 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@3800:3801 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧검출된 태그에 인코딩된 고유 번호입니다.

[P02180 | 3824:3927 | NORMAL_TEXT]
이 프레임에서는 14, 16 두 태그가 검출됐습니다. 다만 localization에 사용되려면 해당 ID가 현재 지도 map_origin.yaml의 tags: 항목에도 있어야 합니다.

[P02181 | 3927:3935 | HEADING_3]
hamming

[P02182 | 3935:3947 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@3935:3936 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧hamming: 0

[P02183 | 3947:3978 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@3947:3948 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧태그 ID를 복원하면서 보정한 비트 오류 개수입니다.

[P02184 | 3978:3999 | NORMAL_TEXT | LIST id=kix.ll8rma2gvl3u level=0]
0: 비트 오류 없이 정확하게 디코딩

[P02185 | 3999:4011 | NORMAL_TEXT | LIST id=kix.ll8rma2gvl3u level=0]
1: 한 비트를 보정

[P02186 | 4011:4023 | NORMAL_TEXT | LIST id=kix.ll8rma2gvl3u level=0]
2: 두 비트를 보정

[P02187 | 4023:4042 | NORMAL_TEXT | LIST id=kix.ll8rma2gvl3u level=0]
값이 클수록 오검출 가능성이 증가

[P02188 | 4042:4111 | NORMAL_TEXT]
현재 설정은 max_hamming: 0이므로 hamming: 0인 검출만 통과합니다. 두 태그 모두 이 조건을 만족합니다.

[P02189 | 4111:4120 | HEADING_3]
goodness

[P02190 | 4120:4135 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@4120:4121 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧goodness: 0.0

[P02191 | 4135:4237 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@4135:4136 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧현재 사용하는 apriltag_ros 구현에서는 실질적으로 사용하지 않는 필드입니다. 메시지 정의에는 있지만 publisher가 값을 넣지 않기 때문에 기본값 0.0으로 나옵니다.

[P02192 | 4237:4325 | NORMAL_TEXT]
따라서 goodness: 0.0을 검출 품질이 나쁘다는 의미로 해석하면 안 됩니다. 이 프로젝트에서는 decision_margin을 품질 척도로 사용합니다.

[P02193 | 4325:4341 | HEADING_3]
decision_margin

[P02194 | 4341:4366 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@4341:4342 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧decision_margin: 141.22

[P02195 | 4366:4430 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@4366:4367 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧태그의 검은 비트와 흰 비트가 판정 임계값에서 얼마나 명확하게 떨어져 있는지를 나타내는 디코딩 품질 점수입니다.

[P02196 | 4430:4456 | NORMAL_TEXT | LIST id=kix.y04v8sn0bofr level=0]
클수록 명암 구분이 명확하고 신뢰도가 높은 편

[P02197 | 4456:4505 | NORMAL_TEXT | LIST id=kix.y04v8sn0bofr level=0]
작으면 거리, 흔들림, 반사광, 초점 또는 비스듬한 각도 때문에 불확실할 가능성이 있음

[P02198 | 4505:4525 | NORMAL_TEXT | LIST id=kix.y04v8sn0bofr level=0]
정해진 최대값이나 확률 단위는 아님

[P02199 | 4525:4580 | NORMAL_TEXT | LIST id=kix.y04v8sn0bofr level=0]
서로 다른 카메라나 노출 설정의 값을 절대적으로 비교하기보다는 동일 환경에서 비교하는 것이 적절함

[P02200 | 4580:4607 | NORMAL_TEXT]
이 프로젝트의 통과 기준은 30.0 이상입니다.

[P02201 | 4607:4622 | NORMAL_TEXT | LIST id=kix.iva3ddbc9ek8 level=0]
Tag 14: 141.23

[P02202 | 4622:4637 | NORMAL_TEXT | LIST id=kix.iva3ddbc9ek8 level=0]
Tag 16: 135.61

[P02203 | 4637:4668 | NORMAL_TEXT]
둘 다 기준보다 훨씬 높아 검출 품질은 매우 좋습니다.

[P02204 | 4668:4675 | HEADING_3]
centre

[P02205 | 4675:4684 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@4675:4676 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧centre:

[P02206 | 4684:4696 | NORMAL_TEXT]
  x: 539.95

[P02207 | 4696:4708 | NORMAL_TEXT]
  y: 313.94

[P02208 | 4708:4732 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@4708:4709 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧이미지에서 태그 중심의 픽셀 좌표입니다.

[P02209 | 4732:4752 | NORMAL_TEXT | LIST id=kix.5i80ahu0atpy level=0]
원점 (0, 0): 이미지 왼쪽 위

[P02210 | 4752:4764 | NORMAL_TEXT | LIST id=kix.5i80ahu0atpy level=0]
x: 오른쪽으로 증가

[P02211 | 4764:4776 | NORMAL_TEXT | LIST id=kix.5i80ahu0atpy level=0]
y: 아래쪽으로 증가

[P02212 | 4776:4815 | NORMAL_TEXT | LIST id=kix.5i80ahu0atpy level=0]
소수점이 나오는 것은 sub-pixel 수준으로 위치를 추정하기 때문

[P02213 | 4815:4873 | NORMAL_TEXT]
예를 들어 Tag 14의 중심은 이미지 왼쪽에서 약 540 px, 위쪽에서 약 314 px 위치입니다.

[P02214 | 4873:4881 | HEADING_3]
corners

[P02215 | 4881:4891 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@4881:4882 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧corners:

[P02216 | 4891:4903 | NORMAL_TEXT]
- x: 500.48

[P02217 | 4903:4915 | NORMAL_TEXT]
  y: 352.49

[P02218 | 4915:4927 | NORMAL_TEXT]
- x: 578.15

[P02219 | 4927:4939 | NORMAL_TEXT]
  y: 354.90

[P02220 | 4939:4951 | NORMAL_TEXT]
- x: 579.81

[P02221 | 4951:4963 | NORMAL_TEXT]
  y: 275.01

[P02222 | 4963:4975 | NORMAL_TEXT]
- x: 502.20

[P02223 | 4975:4987 | NORMAL_TEXT]
  y: 273.44

[P02224 | 4987:5032 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@4987:4988 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧태그 네 모서리의 픽셀 좌표입니다. 태그의 정방향을 기준으로 다음 순서입니다.

[P02225 | 5032:5038 | NORMAL_TEXT | LIST id=kix.9rzg8cz69ihq level=0]
왼쪽 아래

[P02226 | 5038:5045 | NORMAL_TEXT | LIST id=kix.9rzg8cz69ihq level=0]
오른쪽 아래

[P02227 | 5045:5051 | NORMAL_TEXT | LIST id=kix.9rzg8cz69ihq level=0]
오른쪽 위

[P02228 | 5051:5056 | NORMAL_TEXT | LIST id=kix.9rzg8cz69ihq level=0]
왼쪽 위

[P02229 | 5056:5088 | NORMAL_TEXT]
카메라 이미지 좌표계에서는 아래로 갈수록 y가 커집니다.

[P02230 | 5088:5110 | NORMAL_TEXT]
이 값으로 다음을 판단할 수 있습니다.

[P02231 | 5110:5128 | NORMAL_TEXT | LIST id=kix.7m7ikqp0r5k7 level=0]
태그가 이미지에서 차지하는 크기

[P02232 | 5128:5139 | NORMAL_TEXT | LIST id=kix.7m7ikqp0r5k7 level=0]
원근 왜곡과 기울기

[P02233 | 5139:5145 | NORMAL_TEXT | LIST id=kix.7m7ikqp0r5k7 level=0]
태그 방향

[P02234 | 5145:5168 | NORMAL_TEXT | LIST id=kix.7m7ikqp0r5k7 level=0]
카메라와 태그 사이의 3D pose 계산

[P02235 | 5168:5251 | NORMAL_TEXT]
Tag 14는 대략 78 × 80 px, Tag 16은 약 94 × 90 px 크기로 보입니다. 두 태그 모두 영상에서 충분히 크게 잡혀 있습니다.

[P02236 | 5251:5262 | HEADING_3]
homography

[P02237 | 5262:5275 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5262:5263 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧homography:

[P02238 | 5275:5281 | NORMAL_TEXT]
- h00

[P02239 | 5281:5287 | NORMAL_TEXT]
- h01

[P02240 | 5287:5293 | NORMAL_TEXT]
- h02

[P02241 | 5293:5299 | NORMAL_TEXT]
- h10

[P02242 | 5299:5305 | NORMAL_TEXT]
- h11

[P02243 | 5305:5311 | NORMAL_TEXT]
- h12

[P02244 | 5311:5317 | NORMAL_TEXT]
- h20

[P02245 | 5317:5323 | NORMAL_TEXT]
- h21

[P02246 | 5323:5329 | NORMAL_TEXT]
- h22

[P02247 | 5329:5432 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5329:5330 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧이상적인 평면 태그 좌표를 이미지 픽셀 좌표로 변환하는 3 × 3 projective transformation 행렬입니다. 배열은 row-major 순서이므로 다음처럼 해석합니다.

[P02248 | 5432:5453 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5432:5433 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧H = [ h00  h01  h02

[P02249 | 5453:5473 | NORMAL_TEXT]
      h10  h11  h12

[P02250 | 5473:5495 | NORMAL_TEXT]
      h20  h21  h22 ]

[P02251 | 5495:5539 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5495:5496 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧이상적인 태그 위 좌표 (x, y)를 이미지 픽셀 (u, v)로 변환할 때:

[P02252 | 5539:5564 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5539:5540 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧w = h20*x + h21*y + h22

[P02253 | 5564:5565 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02254 | 5565:5595 | NORMAL_TEXT]
u = (h00*x + h01*y + h02) / w

[P02255 | 5595:5625 | NORMAL_TEXT]
v = (h10*x + h11*y + h12) / w

[P02256 | 5625:5666 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5625:5626 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧이상적인 태그 중심은 (0, 0)이고 모서리는 다음 좌표를 사용합니다.

[P02257 | 5666:5704 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5666:5667 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧(-1,  1), (1,  1), (1, -1), (-1, -1)

[P02258 | 5704:5765 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5704:5705 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧따라서 (0, 0)을 homography에 넣으면 거의 centre 좌표가 나옵니다. Tag 14의 경우:

[P02259 | 5765:5781 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5765:5766 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧u = 539.954...

[P02260 | 5781:5796 | NORMAL_TEXT]
v = 313.938...

[P02261 | 5796:5943 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5796:5797 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧homography 자체는 3D 위치가 아닙니다. 카메라 내부 파라미터와 실제 태그 크기를 함께 사용해야 카메라→태그의 3D position/orientation을 구할 수 있습니다. 이 프로젝트에서는 그 결과가 /tf의 다음 transform으로 발행됩니다.

[P02262 | 5943:5975 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5943:5944 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧camera_optical_frame -> tag_14

[P02263 | 5975:6006 | NORMAL_TEXT]
camera_optical_frame -> tag_16

[P02264 | 6006:6034 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@6006:6007 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧현재 메시지를 종합하면 태그 14와 16 모두:

[P02265 | 6034:6044 | NORMAL_TEXT | LIST id=kix.dnxabt7yh7sq level=0]
정상 ID 디코딩

[P02266 | 6044:6053 | NORMAL_TEXT | LIST id=kix.dnxabt7yh7sq level=0]
비트 오류 없음

[P02267 | 6053:6075 | NORMAL_TEXT | LIST id=kix.dnxabt7yh7sq level=0]
매우 높은 decision margin

[P02268 | 6075:6092 | NORMAL_TEXT | LIST id=kix.dnxabt7yh7sq level=0]
영상 안에서 충분한 픽셀 크기

[P02269 | 6092:6179 | NORMAL_TEXT]
이므로 detector 단계에서는 매우 양호합니다. 다음 확인 대상은 두 태그가 지도에 등록되어 있고 localization gate까지 통과했는지입니다.

[P02270 | 6179:6226 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@6179:6180 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧ros2 topic echo /fiducial_localizer/tag_point

[P02271 | 6226:6265 | NORMAL_TEXT]
ros2 topic echo /tag_map_anchor/status

[P02272 | 6265:6267 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@6265:6266 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧

[P02273 | 6267:6268 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02274 | 6268:6269 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02275 | 6269:6271 | NORMAL_TEXT]
[HORIZONTAL_RULE]

[P02276 | 6271:6272 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02277 | 6272:6489 | NORMAL_TEXT]
Q : ssh dangjin-a2 , pw:1234 그리고 그 안에서 ssh A2 , pw:Unitree#24226 을 하면 teamgrit-slam 이 있거든? 거기서 teamgrit-slam 도커 컨테이너에서 토픽들을 확인한 `주행중에 relocalization 이 될 정도로 tag가 가까이 있으면 Task_complete 를 보낸다` 라는 알고리즘에 좋은 토픽이 무엇인지 찾아줘.

[P02278 | 6489:6567 | NORMAL_TEXT]
그리고 `/fiducial_localizer/tag_point` 가 (태그가 가까이 있는 상황에서) 계속해서 업데이트 되고있는지도 알려주고

[P02279 | 6567:6568 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02280 | 6568:6701 | NORMAL_TEXT]
A: 실제 A2의 teamgrit-slam 컨테이너에서 확인한 결과, 문장 그대로 “relocalization이 가능할 정도로 가까운 태그에 도착했다”를 판정하려면 /fiducial_localizer/tag_point가 가장 적합합니다.

[P02281 | 6701:6710 | HEADING_2]
현장 확인 결과

[P02282 | 6710:6737 | NORMAL_TEXT]
실제로 Tag 15가 다음과 같이 발행됐습니다.

[P02283 | 6737:6746 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@6737:6738 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧header:

[P02284 | 6746:6765 | NORMAL_TEXT]
  frame_id: tag_15

[P02285 | 6765:6772 | NORMAL_TEXT]
point:

[P02286 | 6772:6785 | NORMAL_TEXT]
  x: 1.20256

[P02287 | 6785:6799 | NORMAL_TEXT]
  y: -0.11688

[P02288 | 6799:6812 | NORMAL_TEXT]
  z: 0.11130

[P02289 | 6812:6835 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@6812:6813 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧거리로 환산하면 약 1.21 m입니다.

[P02290 | 6835:6864 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@6835:6836 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧sqrt(x² + y² + z²) ≈ 1.21 m

[P02291 | 6864:6893 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@6864:6865 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧같은 태그의 detector 결과도 확인했습니다.

[P02292 | 6893:6901 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@6893:6894 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧id: 15

[P02293 | 6901:6912 | NORMAL_TEXT]
hamming: 0

[P02294 | 6912:6936 | NORMAL_TEXT]
decision_margin: 131.56

[P02295 | 6936:6978 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@6936:6937 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧현재 A2에서 실제 적용 중인 파라미터는 로컬 저장소와 약간 달랐습니다.

[P02296 | 6978:7000 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@6978:6979 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧max_tag_range_m: 4.0

[P02297 | 7000:7026 | NORMAL_TEXT]
min_decision_margin: 10.0

[P02298 | 7026:7048 | NORMAL_TEXT]
max_tag_tf_age_s: 0.2

[P02299 | 7048:7068 | NORMAL_TEXT]
publish_once: false

[P02300 | 7068:7117 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@7068:7069 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧따라서 /fiducial_localizer/tag_point가 발행됐다는 것은 이미:

[P02301 | 7117:7131 | NORMAL_TEXT | LIST id=kix.jtqoaavi5x1m level=0]
현재 지도에 등록된 태그

[P02302 | 7131:7152 | NORMAL_TEXT | LIST id=kix.jtqoaavi5x1m level=0]
decision_margin ≥ 10

[P02303 | 7152:7166 | NORMAL_TEXT | LIST id=kix.jtqoaavi5x1m level=0]
태그 거리 ≤ 4.0 m

[P02304 | 7166:7184 | NORMAL_TEXT | LIST id=kix.jtqoaavi5x1m level=0]
Tag TF age ≤ 0.2초

[P02305 | 7184:7202 | NORMAL_TEXT | LIST id=kix.jtqoaavi5x1m level=0]
body→tag TF 계산 성공

[P02306 | 7202:7218 | NORMAL_TEXT]
조건을 통과했다는 뜻입니다.

[P02307 | 7218:7242 | HEADING_2]
/tag_point는 계속 업데이트되는가?

[P02308 | 7242:7306 | NORMAL_TEXT]
태그가 정상적으로 연속 검출되는 구간에는 계속 업데이트됩니다. 실제 측정에서는 약 15~16 Hz로 발행됐습니다.

[P02309 | 7306:7331 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@7306:7307 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧average rate: 15.315 Hz

[P02310 | 7331:7355 | NORMAL_TEXT]
average rate: 16.668 Hz

[P02311 | 7355:7382 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@7355:7356 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧하지만 항상 끊김 없이 발행되지는 않았습니다.

[P02312 | 7382:7392 | NORMAL_TEXT]
20초 관측에서:

[P02313 | 7392:7405 | NORMAL_TEXT | LIST id=kix.k9ivlqvx4ypg level=0]
총 52개 메시지 관측

[P02314 | 7405:7422 | NORMAL_TEXT | LIST id=kix.k9ivlqvx4ypg level=0]
순간적으로 약 15~16 Hz

[P02315 | 7422:7441 | NORMAL_TEXT | LIST id=kix.k9ivlqvx4ypg level=0]
관측 중 최대 간격 약 2.36초

[P02316 | 7441:7460 | NORMAL_TEXT | LIST id=kix.k9ivlqvx4ypg level=0]
이후 태그 검출이 끊긴 구간 존재

[P02317 | 7460:7562 | NORMAL_TEXT]
즉 /tag_point는 timer 기반 상태 토픽이 아니라, 유효한 태그 detection마다 발행되는 이벤트 스트림입니다. 태그가 가까워도 다음 상황에는 잠시 끊길 수 있습니다.

[P02318 | 7562:7579 | NORMAL_TEXT | LIST id=kix.gcb9t49m8c0h level=0]
카메라 시야에서 태그가 벗어남

[P02319 | 7579:7598 | NORMAL_TEXT | LIST id=kix.gcb9t49m8c0h level=0]
흔들림 또는 motion blur

[P02320 | 7598:7611 | NORMAL_TEXT | LIST id=kix.gcb9t49m8c0h level=0]
카메라 입력 일시 중단

[P02321 | 7611:7630 | NORMAL_TEXT | LIST id=kix.gcb9t49m8c0h level=0]
Tag TF가 0.2초보다 오래됨

[P02322 | 7630:7649 | NORMAL_TEXT | LIST id=kix.gcb9t49m8c0h level=0]
태그 ID 또는 품질 조건 불충족

[P02323 | 7649:7712 | NORMAL_TEXT]
따라서 “토픽이 현재 존재하는가”가 아니라 최근 일정 시간 안에 메시지가 여러 번 들어왔는지를 검사해야 합니다.

[P02324 | 7712:7720 | HEADING_2]
권장 알고리즘

[P02325 | 7720:7797 | NORMAL_TEXT]
Task 도착 판정에는 /fiducial_localizer/tag_point를 사용하고, 한 프레임만으로 완료하지 않는 것을 권장합니다.

[P02326 | 7797:7809 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@7797:7798 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧DRIVING 상태

[P02327 | 7809:7813 | NORMAL_TEXT]
  ↓

[P02328 | 7813:7846 | NORMAL_TEXT]
/fiducial_localizer/tag_point 수신

[P02329 | 7846:7850 | NORMAL_TEXT]
  ↓

[P02330 | 7850:7880 | NORMAL_TEXT]
distance = sqrt(x² + y² + z²)

[P02331 | 7880:7884 | NORMAL_TEXT]
  ↓

[P02332 | 7884:7907 | NORMAL_TEXT]
같은 tag_id가 1초 동안 5회 이상

[P02333 | 7907:7911 | NORMAL_TEXT]
  ↓

[P02334 | 7911:7932 | NORMAL_TEXT]
distance ≤ 1.5~2.0 m

[P02335 | 7932:7936 | NORMAL_TEXT]
  ↓

[P02336 | 7936:7957 | NORMAL_TEXT]
Task_complete 한 번 발행

[P02337 | 7957:7965 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@7957:7958 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧예시 조건:

[P02338 | 7965:7987 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@7965:7966 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧if state == DRIVING:

[P02339 | 7987:8021 | NORMAL_TEXT]
    if same_tag_count_in_1s >= 5:

[P02340 | 8021:8056 | NORMAL_TEXT]
        if median_distance <= 1.5:

[P02341 | 8056:8097 | NORMAL_TEXT]
            publish_task_complete_once()

[P02342 | 8097:8138 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@8097:8098 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧단일 측정값보다 최근 거리의 median을 사용하는 편이 안정적입니다.

[P02343 | 8138:8166 | HEADING_2]
/tag_map_anchor/status와의 차이

[P02344 | 8166:8308 | NORMAL_TEXT]
현장 관측 중 /fiducial_localizer/tag_point는 발행됐지만 /tag_map_anchor/status는 20초 동안 발행되지 않았습니다. /tag_map_anchor/pose와 landmark refiner 결과도 나오지 않았습니다.

[P02345 | 8308:8321 | NORMAL_TEXT]
현재 실제 파라미터는:

[P02346 | 8321:8341 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@8321:8322 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧min_full_points: 4

[P02347 | 8341:8361 | NORMAL_TEXT]
min_baseline_m: 1.0

[P02348 | 8361:8393 | NORMAL_TEXT]
use_orientation_fallback: false

[P02349 | 8393:8512 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@8393:8394 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧현재는 Tag 15 하나만 주로 보였기 때문에 “가까운 유효 태그 관측”은 성공했지만 최초 full anchor solve에 필요한 서로 다른 태그 사이의 1 m baseline을 만들지 못한 것으로 보입니다.

[P02350 | 8512:8538 | NORMAL_TEXT]
따라서 최종 선택은 요구사항에 따라 다릅니다.

[P02351 | 8538:8588 | NORMAL_TEXT | LIST id=kix.1q7s2h9qd8l4 level=0]
“가까운 태그 지점에 도착했다” → /fiducial_localizer/tag_point

[P02352 | 8588:8641 | NORMAL_TEXT | LIST id=kix.1q7s2h9qd8l4 level=0]
“태그 기반 anchor solve까지 성공했다” → /tag_map_anchor/status

[P02353 | 8641:8707 | NORMAL_TEXT | LIST id=kix.1q7s2h9qd8l4 level=0]
둘 다 보장 → /tag_point로 도착 판정 후 /tag_map_anchor/status를 기다리고 Task 완료

[P02354 | 8707:8901 | NORMAL_TEXT]
현재 문장인 “relocalization이 될 정도로 Tag가 가까이 있으면”에는 /fiducial_localizer/tag_point + 거리 + 연속 수신 조건이 가장 현실적인 선택입니다. 실제 relocalization 성공까지 Task 완료 조건에 포함해야 한다면 /tag_map_anchor/status를 추가 조건으로 사용해야 합니다.

[P02355 | 8901:8902 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## 2026.08.25 Line Tracking-chatgpt (t.p1jyt1l80732)

[P02356 | 1:72 | NORMAL_TEXT]
Q : 로봇 Navigation 분야에서 Vision Line Tracking 에 대해서 어떻게 기본적으로 구현되는지 알려줘.

[P02357 | 72:73 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02358 | 73:174 | NORMAL_TEXT]
A : 로봇의 Vision Line Tracking은 카메라 영상에서 주행선을 검출하고, 로봇 중심과 선 중심 사이의 오차를 계산해 좌우 바퀴 속도나 조향각을 조절하는 방식입니다.

[P02359 | 174:191 | NORMAL_TEXT]
핵심 흐름은 다음과 같습니다.

[P02360 | 191:199 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@191:192 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧카메라 영상

[P02361 | 199:204 | NORMAL_TEXT]
   ↓

[P02362 | 204:217 | NORMAL_TEXT]
ROI 설정 및 전처리

[P02363 | 217:222 | NORMAL_TEXT]
   ↓

[P02364 | 222:232 | NORMAL_TEXT]
주행선 영역 분리

[P02365 | 232:237 | NORMAL_TEXT]
   ↓

[P02366 | 237:249 | NORMAL_TEXT]
선의 중심·방향 계산

[P02367 | 249:254 | NORMAL_TEXT]
   ↓

[P02368 | 254:267 | NORMAL_TEXT]
횡방향/각도 오차 계산

[P02369 | 267:272 | NORMAL_TEXT]
   ↓

[P02370 | 272:289 | NORMAL_TEXT]
PID 등으로 조향 명령 생성

[P02371 | 289:294 | NORMAL_TEXT]
   ↓

[P02372 | 294:300 | NORMAL_TEXT]
모터 제어

[P02373 | 300:318 | HEADING_3]
⟦PROTECTED: opaqueTemplateControl/copy-only@300:301 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧1. 영상 입력과 ROI 설정

[P02374 | 318:413 | NORMAL_TEXT]
카메라는 일반적으로 로봇 전방 또는 바닥을 향합니다. 전체 영상 중 주행선이 존재할 가능성이 높은 아래쪽 영역만 ROI(Region of Interest)로 사용합니다.

[P02375 | 413:436 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@413:414 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧frame = camera.read()

[P02376 | 436:477 | NORMAL_TEXT]
roi = frame[int(height * 0.5):height, :]

[P02377 | 477:520 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@477:478 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧ROI를 좁히면 계산량이 줄고 배경이나 장애물에 의한 오검출도 감소합니다.

[P02378 | 520:528 | HEADING_3]
2. 선 검출

[P02379 | 528:553 | NORMAL_TEXT]
선의 색상과 환경에 따라 방법을 선택합니다.

[P02380 | 553:599 | NORMAL_TEXT]
가장 단순한 방법은 흑백 이진화입니다. 예를 들어 밝은 바닥 위의 검은 선이라면:

[P02381 | 599:645 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@599:600 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

[P02382 | 645:686 | NORMAL_TEXT]
blur = cv2.GaussianBlur(gray, (5, 5), 0)

[P02383 | 686:713 | NORMAL_TEXT]
_, binary = cv2.threshold(

[P02384 | 713:754 | NORMAL_TEXT]
    blur, 80, 255, cv2.THRESH_BINARY_INV

[P02385 | 754:756 | NORMAL_TEXT]
)

[P02386 | 756:807 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@756:757 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧조명 변화가 크면 고정 임계값 대신 적응형 이진화나 Otsu 방법을 사용할 수 있습니다.

[P02387 | 807:835 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@807:808 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧_, binary = cv2.threshold(

[P02388 | 835:853 | NORMAL_TEXT]
    blur, 0, 255,

[P02389 | 853:897 | NORMAL_TEXT]
    cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU

[P02390 | 897:899 | NORMAL_TEXT]
)

[P02391 | 899:948 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@899:900 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧빨간색·노란색처럼 특정 색상의 선이라면 RGB보다 HSV 색공간이 보통 안정적입니다.

[P02392 | 948:992 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@948:949 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

[P02393 | 992:1042 | NORMAL_TEXT]
mask = cv2.inRange(hsv, lower_color, upper_color)

[P02394 | 1042:1091 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@1042:1043 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧검출 결과에는 작은 노이즈가 생길 수 있으므로 morphology 연산을 적용합니다.

[P02395 | 1091:1127 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@1091:1092 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧kernel = np.ones((5, 5), np.uint8)

[P02396 | 1127:1181 | NORMAL_TEXT]
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

[P02397 | 1181:1236 | NORMAL_TEXT]
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

[P02398 | 1236:1249 | HEADING_3]
⟦PROTECTED: opaqueTemplateControl/copy-only@1236:1237 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧3. 선의 중심 계산

[P02399 | 1249:1284 | NORMAL_TEXT]
가장 간단한 구현은 이진 영상의 무게중심을 계산하는 것입니다.

[P02400 | 1284:1307 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@1284:1285 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧M = cv2.moments(mask)

[P02401 | 1307:1308 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02402 | 1308:1325 | NORMAL_TEXT]
if M["m00"] > 0:

[P02403 | 1325:1359 | NORMAL_TEXT]
    cx = int(M["m10"] / M["m00"])

[P02404 | 1359:1393 | NORMAL_TEXT]
    cy = int(M["m01"] / M["m00"])

[P02405 | 1393:1437 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@1393:1394 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧영상 중심을 image_center라고 하면 횡방향 오차는 다음과 같습니다.

[P02406 | 1437:1469 | NORMAL_TEXT]
\[ e_x = c_x - image\_center \]

[P02407 | 1469:1492 | NORMAL_TEXT | LIST id=kix.ledbw7ysquo9 level=0]
e_x > 0: 선이 영상 오른쪽에 있음

[P02408 | 1492:1514 | NORMAL_TEXT | LIST id=kix.ledbw7ysquo9 level=0]
e_x < 0: 선이 영상 왼쪽에 있음

[P02409 | 1514:1536 | NORMAL_TEXT | LIST id=kix.ledbw7ysquo9 level=0]
e_x ≈ 0: 로봇이 선 중앙에 있음

[P02410 | 1536:1566 | NORMAL_TEXT]
해상도에 따른 영향을 줄이기 위해 보통 정규화합니다.

[P02411 | 1566:1623 | NORMAL_TEXT]
\[ e_{norm} = \frac{c_x-image\_center}{image\_center} \]

[P02412 | 1623:1657 | NORMAL_TEXT]
이렇게 하면 대략 -1~1 범위의 오차를 얻을 수 있습니다.

[P02413 | 1657:1672 | HEADING_3]
4. 선의 진행 방향 계산

[P02414 | 1672:1765 | NORMAL_TEXT]
무게중심 하나만 사용하면 직선에서는 잘 동작하지만 곡선을 늦게 인식할 수 있습니다. 이를 개선하려면 ROI를 여러 수평 구간으로 나누고 각 구간의 중심을 계산합니다.

[P02415 | 1765:1780 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@1765:1766 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧먼 거리 ROI    ●

[P02416 | 1780:1796 | NORMAL_TEXT]
              /

[P02417 | 1796:1809 | NORMAL_TEXT]
중간 ROI     ●

[P02418 | 1809:1823 | NORMAL_TEXT]
            /

[P02419 | 1823:1833 | NORMAL_TEXT]
가까운 ROI ●

[P02420 | 1833:1876 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@1833:1834 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧가까운 점은 현재 위치 오차를, 먼 점은 앞으로 진행할 방향을 나타냅니다.

[P02421 | 1876:1960 | NORMAL_TEXT]
예를 들어 가까운 중심을 (x_near, y_near), 먼 중심을 (x_far, y_far)라고 하면 선의 방향각을 다음과 같이 구할 수 있습니다.

[P02422 | 1960:2033 | NORMAL_TEXT]
\[ \theta = \operatorname{atan2}(x_{far}-x_{near},\ y_{near}-y_{far}) \]

[P02423 | 2033:2073 | NORMAL_TEXT]
최종 제어 오차를 위치 오차와 방향 오차의 조합으로 만들 수 있습니다.

[P02424 | 2073:2109 | NORMAL_TEXT]
\[ e = w_x e_x + w_\theta \theta \]

[P02425 | 2109:2147 | NORMAL_TEXT]
이 방식은 단순 무게중심 추종보다 곡선에 미리 반응할 수 있습니다.

[P02426 | 2147:2156 | HEADING_3]
5. 조향 제어

[P02427 | 2156:2161 | HEADING_4]
P 제어

[P02428 | 2161:2176 | NORMAL_TEXT]
가장 기본적인 방법입니다.

[P02429 | 2176:2192 | NORMAL_TEXT]
\[ u = K_p e \]

[P02430 | 2192:2204 | NORMAL_TEXT]
차동구동 로봇이라면:

[P02431 | 2204:2253 | NORMAL_TEXT]
\[ v_L = v_{base} + u \]\[ v_R = v_{base} - u \]

[P02432 | 2253:2276 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@2253:2254 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧steering = kp * error

[P02433 | 2276:2277 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02434 | 2277:2313 | NORMAL_TEXT]
left_speed  = base_speed + steering

[P02435 | 2313:2349 | NORMAL_TEXT]
right_speed = base_speed - steering

[P02436 | 2349:2383 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@2349:2350 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧모터 방향과 좌표계에 따라 부호는 반대가 될 수 있습니다.

[P02437 | 2383:2390 | HEADING_4]
PID 제어

[P02438 | 2390:2446 | NORMAL_TEXT]
P 제어만 사용하면 로봇이 선을 중심으로 좌우 진동할 수 있습니다. 이때 PID 제어를 사용합니다.

[P02439 | 2446:2501 | NORMAL_TEXT]
\[ u(t)=K_p e(t)+K_i\int e(t)dt+K_d\frac{de(t)}{dt} \]

[P02440 | 2501:2525 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@2501:2502 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧integral += error * dt

[P02441 | 2525:2568 | NORMAL_TEXT]
derivative = (error - previous_error) / dt

[P02442 | 2568:2569 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02443 | 2569:2582 | NORMAL_TEXT]
steering = (

[P02444 | 2582:2597 | NORMAL_TEXT]
    kp * error

[P02445 | 2597:2617 | NORMAL_TEXT]
    + ki * integral

[P02446 | 2617:2639 | NORMAL_TEXT]
    + kd * derivative

[P02447 | 2639:2641 | NORMAL_TEXT]
)

[P02448 | 2641:2642 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02449 | 2642:2665 | NORMAL_TEXT]
previous_error = error

[P02450 | 2665:2701 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@2665:2666 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧실제 Line Tracking에서는 흔히 다음처럼 조정합니다.

[P02451 | 2701:2723 | NORMAL_TEXT | LIST id=kix.okkrguk7j9oj level=0]
Kp: 오차에 반응하는 기본 조향 강도

[P02452 | 2723:2737 | NORMAL_TEXT | LIST id=kix.okkrguk7j9oj level=0]
Kd: 좌우 진동을 억제

[P02453 | 2737:2752 | NORMAL_TEXT | LIST id=kix.okkrguk7j9oj level=0]
Ki: 지속적인 편향 보정

[P02454 | 2752:2781 | NORMAL_TEXT | LIST id=kix.okkrguk7j9oj level=0]
처음에는 Ki = 0으로 두고 Kp, Kd부터 조정

[P02455 | 2781:2790 | HEADING_3]
6. 속도 제어

[P02456 | 2790:2860 | NORMAL_TEXT]
항상 동일한 속도로 주행하면 급커브에서 선을 벗어나기 쉽습니다. 조향 오차가 클수록 전진 속도를 낮추는 방식이 효과적입니다.

[P02457 | 2860:2888 | NORMAL_TEXT]
\[ v = v_{max} - K_v |e| \]

[P02458 | 2888:2907 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@2888:2889 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧base_speed = max(

[P02459 | 2907:2922 | NORMAL_TEXT]
    min_speed,

[P02460 | 2922:2962 | NORMAL_TEXT]
    max_speed - speed_gain * abs(error)

[P02461 | 2962:2964 | NORMAL_TEXT]
)

[P02462 | 2964:2996 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@2964:2965 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧직선에서는 빠르게, 곡선에서는 느리게 주행하게 됩니다.

[P02463 | 2996:3008 | HEADING_3]
기본 제어 루프 예시

[P02464 | 3008:3030 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@3008:3009 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧previous_error = 0.0

[P02465 | 3030:3045 | NORMAL_TEXT]
integral = 0.0

[P02466 | 3045:3046 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02467 | 3046:3058 | NORMAL_TEXT]
while True:

[P02468 | 3058:3089 | NORMAL_TEXT]
    frame = get_camera_frame()

[P02469 | 3089:3090 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02470 | 3090:3118 | NORMAL_TEXT]
    roi = select_roi(frame)

[P02471 | 3118:3146 | NORMAL_TEXT]
    mask = detect_line(roi)

[P02472 | 3146:3187 | NORMAL_TEXT]
    center = calculate_line_center(mask)

[P02473 | 3187:3188 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02474 | 3188:3215 | NORMAL_TEXT]
    if center is not None:

[P02475 | 3215:3256 | NORMAL_TEXT]
        image_center = mask.shape[1] / 2

[P02476 | 3256:3311 | NORMAL_TEXT]
        error = (center - image_center) / image_center

[P02477 | 3311:3312 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02478 | 3312:3342 | NORMAL_TEXT]
        dt = get_delta_time()

[P02479 | 3342:3373 | NORMAL_TEXT]
        integral += error * dt

[P02480 | 3373:3424 | NORMAL_TEXT]
        derivative = (error - previous_error) / dt

[P02481 | 3424:3425 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02482 | 3425:3446 | NORMAL_TEXT]
        steering = (

[P02483 | 3446:3469 | NORMAL_TEXT]
            kp * error

[P02484 | 3469:3497 | NORMAL_TEXT]
            + ki * integral

[P02485 | 3497:3527 | NORMAL_TEXT]
            + kd * derivative

[P02486 | 3527:3537 | NORMAL_TEXT]
        )

[P02487 | 3537:3538 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02488 | 3538:3564 | NORMAL_TEXT]
        base_speed = max(

[P02489 | 3564:3587 | NORMAL_TEXT]
            min_speed,

[P02490 | 3587:3635 | NORMAL_TEXT]
            max_speed - speed_gain * abs(error)

[P02491 | 3635:3645 | NORMAL_TEXT]
        )

[P02492 | 3645:3646 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02493 | 3646:3689 | NORMAL_TEXT]
        left = clip(base_speed + steering)

[P02494 | 3689:3733 | NORMAL_TEXT]
        right = clip(base_speed - steering)

[P02495 | 3733:3734 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02496 | 3734:3771 | NORMAL_TEXT]
        set_motor_speed(left, right)

[P02497 | 3771:3802 | NORMAL_TEXT]
        previous_error = error

[P02498 | 3802:3803 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02499 | 3803:3813 | NORMAL_TEXT]
    else:

[P02500 | 3813:3840 | NORMAL_TEXT]
        handle_line_lost()

[P02501 | 3840:3850 | HEADING_3]
⟦PROTECTED: opaqueTemplateControl/copy-only@3840:3841 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧선을 놓쳤을 때

[P02502 | 3850:3897 | NORMAL_TEXT]
실제 구현에서는 선 검출 실패 처리가 중요합니다. 일반적인 전략은 다음과 같습니다.

[P02503 | 3897:3917 | NORMAL_TEXT | LIST id=kix.8dyu795n1wbr level=0]
직전 선 위치 방향으로 천천히 회전

[P02504 | 3917:3939 | NORMAL_TEXT | LIST id=kix.8dyu795n1wbr level=0]
일정 시간 동안 마지막 조향 명령 유지

[P02505 | 3939:3955 | NORMAL_TEXT | LIST id=kix.8dyu795n1wbr level=0]
선을 찾을 때까지 저속 탐색

[P02506 | 3955:3967 | NORMAL_TEXT | LIST id=kix.8dyu795n1wbr level=0]
제한 시간 이후 정지

[P02507 | 3967:3995 | NORMAL_TEXT | LIST id=kix.8dyu795n1wbr level=0]
검출된 선 영역이 너무 작으면 잘못된 검출로 판단

[P02508 | 3995:4018 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@3995:3996 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧if line_not_detected:

[P02509 | 4018:4045 | NORMAL_TEXT]
    if previous_error > 0:

[P02510 | 4045:4075 | NORMAL_TEXT]
        rotate_right_slowly()

[P02511 | 4075:4085 | NORMAL_TEXT]
    else:

[P02512 | 4085:4114 | NORMAL_TEXT]
        rotate_left_slowly()

[P02513 | 4114:4133 | HEADING_3]
⟦PROTECTED: opaqueTemplateControl/copy-only@4114:4115 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧환경에 따른 대표적인 구현 방식

[P02514 | 4136:4139 | NORMAL_TEXT | TABLE row=0 col=0]
환경

[P02515 | 4140:4151 | NORMAL_TEXT | TABLE row=0 col=1]
주로 사용하는 방법

[P02516 | 4153:4165 | NORMAL_TEXT | TABLE row=1 col=0]
밝은 바닥의 검은 선

[P02517 | 4166:4188 | NORMAL_TEXT | TABLE row=1 col=1]
Grayscale + Threshold

[P02518 | 4190:4199 | NORMAL_TEXT | TABLE row=2 col=0]
특정 색상의 선

[P02519 | 4200:4220 | NORMAL_TEXT | TABLE row=2 col=1]
HSV Color Threshold

[P02520 | 4222:4233 | NORMAL_TEXT | TABLE row=3 col=0]
선의 경계가 뚜렷함

[P02521 | 4234:4258 | NORMAL_TEXT | TABLE row=3 col=1]
Canny + Hough Transform

[P02522 | 4260:4269 | NORMAL_TEXT | TABLE row=4 col=0]
굴곡이 많은 선

[P02523 | 4270:4295 | NORMAL_TEXT | TABLE row=4 col=1]
다중 ROI 또는 Sliding Window

[P02524 | 4297:4306 | NORMAL_TEXT | TABLE row=5 col=0]
원근 왜곡이 큼

[P02525 | 4307:4340 | NORMAL_TEXT | TABLE row=5 col=1]
Perspective/Bird’s-eye Transform

[P02526 | 4342:4355 | NORMAL_TEXT | TABLE row=6 col=0]
복잡한 바닥·조명 변화

[P02527 | 4356:4373 | NORMAL_TEXT | TABLE row=6 col=1]
Segmentation 신경망

[P02528 | 4374:4401 | NORMAL_TEXT]
처음 구현한다면 다음 구성이 가장 현실적입니다.

[P02529 | 4401:4413 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@4401:4402 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧OpenCV 이진화

[P02530 | 4413:4425 | NORMAL_TEXT]
→ 화면 하단 ROI

[P02531 | 4425:4453 | NORMAL_TEXT]
→ contour 또는 moments로 중심 계산

[P02532 | 4453:4466 | NORMAL_TEXT]
→ 정규화된 중심 오차

[P02533 | 4466:4474 | NORMAL_TEXT]
→ PD 제어

[P02534 | 4474:4489 | NORMAL_TEXT]
→ 오차 크기에 따른 감속

[P02535 | 4489:4648 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@4489:4490 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧이 기본 구조만으로도 일정한 조명과 명확한 선이 있는 환경에서는 충분히 안정적인 Line Tracking을 구현할 수 있습니다. 이후 성능이 부족하면 다중 ROI, 선 방향 추정, 원근 변환, Kalman Filter, 딥러닝 segmentation 순서로 확장하는 것이 좋습니다.

[P02536 | 4648:4650 | NORMAL_TEXT]
[HORIZONTAL_RULE]

[P02537 | 4650:4651 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02538 | 4651:4700 | NORMAL_TEXT]
Q : 아스팔트 도로 위에 노란색 선이라면? 그리고 양 옆은 탁한 붉은색과 풀밭이라면?

[P02539 | 4700:4701 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02540 | 4701:4794 | NORMAL_TEXT]
A : 아스팔트 위 노란색 선이고 주변이 탁한 붉은색·풀밭이라면, 단순 명암 이진화보다는 HSV 색상 분리 + LAB 보강 + 도로 ROI 제한 조합이 안정적입니다.

[P02541 | 4794:4860 | NORMAL_TEXT]
핵심은 색상만으로 판단하지 않고, “노란색인가?”와 “도로 위에서 길게 이어지는 선인가?”를 함께 확인하는 것입니다.

[P02542 | 4860:4878 | HEADING_3]
1. HSV로 노란색 후보 검출

[P02543 | 4878:4944 | NORMAL_TEXT]
OpenCV의 HSV 범위는 H: 0~179, S/V: 0~255입니다. 초기값은 다음 정도에서 시작할 수 있습니다.

[P02544 | 4944:4988 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@4944:4945 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

[P02545 | 4988:4989 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02546 | 4989:5027 | NORMAL_TEXT]
lower_yellow = np.array([15, 70, 70])

[P02547 | 5027:5067 | NORMAL_TEXT]
upper_yellow = np.array([40, 255, 255])

[P02548 | 5067:5068 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02549 | 5068:5124 | NORMAL_TEXT]
hsv_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

[P02550 | 5124:5142 | NORMAL_TEXT | LIST id=kix.e5ztvnqehy28 level=0]
⟦PROTECTED: opaqueTemplateControl/copy-only@5124:5125 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧노란색: 대체로 H=15~40

[P02551 | 5142:5172 | NORMAL_TEXT | LIST id=kix.e5ztvnqehy28 level=0]
탁한 붉은색: 대체로 H=0~15 또는 170~179

[P02552 | 5172:5188 | NORMAL_TEXT | LIST id=kix.e5ztvnqehy28 level=0]
풀밭: 대체로 H=35~90

[P02553 | 5188:5205 | NORMAL_TEXT | LIST id=kix.e5ztvnqehy28 level=0]
아스팔트: 채도와 밝기가 낮음

[P02554 | 5205:5261 | NORMAL_TEXT]
풀의 황록색 부분이 노란색 범위에 들어올 수 있으므로 HSV만 사용하면 오검출이 생길 수 있습니다.

[P02555 | 5261:5283 | HEADING_3]
2. LAB 색공간으로 노란 성분 보강

[P02556 | 5283:5333 | NORMAL_TEXT]
LAB의 b 채널은 파란색↔노란색 성분을 나타냅니다. 값이 클수록 노란 성분이 강합니다.

[P02557 | 5333:5377 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5333:5334 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)

[P02558 | 5377:5402 | NORMAL_TEXT]
L, a, b = cv2.split(lab)

[P02559 | 5402:5403 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02560 | 5403:5463 | NORMAL_TEXT]
_, lab_mask = cv2.threshold(b, 145, 255, cv2.THRESH_BINARY)

[P02561 | 5463:5516 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5463:5464 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧HSV와 LAB 조건을 동시에 만족시키면 붉은 흙이나 녹색 풀의 오검출을 줄일 수 있습니다.

[P02562 | 5516:5567 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5516:5517 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧yellow_mask = cv2.bitwise_and(hsv_mask, lab_mask)

[P02563 | 5567:5619 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5567:5568 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧145는 시작점일 뿐이며 카메라와 날씨에 따라 약 135~170 범위에서 조정해야 합니다.

[P02564 | 5619:5637 | HEADING_3]
3. 도로 영역만 ROI로 사용

[P02565 | 5637:5709 | NORMAL_TEXT]
양옆에 붉은 흙과 풀밭이 있다면 영상 전체를 처리하지 말고, 로봇 앞쪽 도로 영역만 사다리꼴 ROI로 제한하는 것이 중요합니다.

[P02566 | 5709:5733 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@5709:5710 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧h, w = frame.shape[:2]

[P02567 | 5733:5734 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02568 | 5734:5756 | NORMAL_TEXT]
polygon = np.array([[

[P02569 | 5756:5780 | NORMAL_TEXT]
    (int(w * 0.10), h),

[P02570 | 5780:5804 | NORMAL_TEXT]
    (int(w * 0.90), h),

[P02571 | 5804:5840 | NORMAL_TEXT]
    (int(w * 0.65), int(h * 0.45)),

[P02572 | 5840:5875 | NORMAL_TEXT]
    (int(w * 0.35), int(h * 0.45))

[P02573 | 5875:5895 | NORMAL_TEXT]
]], dtype=np.int32)

[P02574 | 5895:5896 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02575 | 5896:5940 | NORMAL_TEXT]
roi_mask = np.zeros((h, w), dtype=np.uint8)

[P02576 | 5940:5977 | NORMAL_TEXT]
cv2.fillPoly(roi_mask, polygon, 255)

[P02577 | 5977:5978 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02578 | 5978:6030 | NORMAL_TEXT]
road = cv2.bitwise_and(frame, frame, mask=roi_mask)

[P02579 | 6030:6078 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@6030:6031 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧카메라가 거의 바닥을 보고 있다면 단순히 영상 하단 50~70%만 사용해도 됩니다.

[P02580 | 6078:6094 | HEADING_3]
4. 노이즈 제거와 선 연결

[P02581 | 6094:6148 | NORMAL_TEXT]
노란색 선이 마모되거나 그림자 때문에 끊어질 수 있으므로 morphology 연산을 적용합니다.

[P02582 | 6148:6190 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@6148:6149 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧kernel_open = cv2.getStructuringElement(

[P02583 | 6190:6220 | NORMAL_TEXT]
    cv2.MORPH_ELLIPSE, (3, 3)

[P02584 | 6220:6222 | NORMAL_TEXT]
)

[P02585 | 6222:6264 | NORMAL_TEXT]
kernel_close = cv2.getStructuringElement(

[P02586 | 6264:6291 | NORMAL_TEXT]
    cv2.MORPH_RECT, (7, 7)

[P02587 | 6291:6293 | NORMAL_TEXT]
)

[P02588 | 6293:6294 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02589 | 6294:6326 | NORMAL_TEXT]
yellow_mask = cv2.morphologyEx(

[P02590 | 6326:6371 | NORMAL_TEXT]
    yellow_mask, cv2.MORPH_OPEN, kernel_open

[P02591 | 6371:6373 | NORMAL_TEXT]
)

[P02592 | 6373:6405 | NORMAL_TEXT]
yellow_mask = cv2.morphologyEx(

[P02593 | 6405:6452 | NORMAL_TEXT]
    yellow_mask, cv2.MORPH_CLOSE, kernel_close

[P02594 | 6452:6454 | NORMAL_TEXT]
)

[P02595 | 6454:6476 | NORMAL_TEXT | LIST id=kix.d413ukyhvjmt level=0]
⟦PROTECTED: opaqueTemplateControl/copy-only@6454:6455 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧OPEN: 풀잎이나 작은 반사점 제거

[P02596 | 6476:6495 | NORMAL_TEXT | LIST id=kix.d413ukyhvjmt level=0]
CLOSE: 끊어진 노란 선 연결

[P02597 | 6495:6535 | NORMAL_TEXT]
진행 방향으로 긴 커널을 사용하면 선 연결에 더 유리할 수도 있습니다.

[P02598 | 6535:6555 | HEADING_3]
5. 색상뿐 아니라 선 형태도 확인

[P02599 | 6555:6592 | NORMAL_TEXT]
후보 영역의 contour를 구한 다음 다음 조건으로 걸러냅니다.

[P02600 | 6592:6606 | NORMAL_TEXT | LIST id=kix.z6h8m79sabwy level=0]
면적이 너무 작지 않은가

[P02601 | 6606:6621 | NORMAL_TEXT | LIST id=kix.z6h8m79sabwy level=0]
ROI 하단에서 시작하는가

[P02602 | 6621:6638 | NORMAL_TEXT | LIST id=kix.z6h8m79sabwy level=0]
진행 방향으로 길게 이어지는가

[P02603 | 6638:6657 | NORMAL_TEXT | LIST id=kix.z6h8m79sabwy level=0]
이전 프레임의 선 위치와 가까운가

[P02604 | 6657:6675 | NORMAL_TEXT | LIST id=kix.z6h8m79sabwy level=0]
폭이 갑자기 크게 변하지 않는가

[P02605 | 6675:6708 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@6675:6676 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧contours, _ = cv2.findContours(

[P02606 | 6708:6725 | NORMAL_TEXT]
    yellow_mask,

[P02607 | 6725:6748 | NORMAL_TEXT]
    cv2.RETR_EXTERNAL,

[P02608 | 6748:6776 | NORMAL_TEXT]
    cv2.CHAIN_APPROX_SIMPLE

[P02609 | 6776:6778 | NORMAL_TEXT]
)

[P02610 | 6778:6779 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02611 | 6779:6795 | NORMAL_TEXT]
candidates = []

[P02612 | 6795:6796 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02613 | 6796:6821 | NORMAL_TEXT]
for contour in contours:

[P02614 | 6821:6857 | NORMAL_TEXT]
    area = cv2.contourArea(contour)

[P02615 | 6857:6876 | NORMAL_TEXT]
    if area < 100:

[P02616 | 6876:6893 | NORMAL_TEXT]
        continue

[P02617 | 6893:6894 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02618 | 6894:6946 | NORMAL_TEXT]
    x, y, width, height = cv2.boundingRect(contour)

[P02619 | 6946:6947 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02620 | 6947:6967 | NORMAL_TEXT]
    if height < 20:

[P02621 | 6967:6984 | NORMAL_TEXT]
        continue

[P02622 | 6984:6985 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02623 | 6985:7016 | NORMAL_TEXT]
    candidates.append(contour)

[P02624 | 7016:7090 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@7016:7017 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧가장 큰 contour만 선택하는 것보다는, 이전 프레임에서 찾은 선과 가까운 contour에 높은 점수를 주는 것이 안전합니다.

[P02625 | 7090:7175 | NORMAL_TEXT]
\[ score = w_a \cdot area + w_h \cdot height - w_d \cdot distance\_from\_previous \]

[P02626 | 7175:7192 | HEADING_3]
6. 여러 높이에서 중심 추적

[P02627 | 7192:7287 | NORMAL_TEXT]
노란 선 전체의 무게중심 하나만 사용하면 커브 대응이 느릴 수 있습니다. ROI를 가로 방향으로 3~5개 구간으로 나눠 각 구간의 선 중심을 계산하는 방식을 추천합니다.

[P02628 | 7287:7313 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@7287:7288 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧먼 구간       ●  → 진행 방향 예측

[P02629 | 7313:7328 | NORMAL_TEXT]
             /

[P02630 | 7328:7338 | NORMAL_TEXT]
중간 구간   ●

[P02631 | 7338:7351 | NORMAL_TEXT]
           /

[P02632 | 7351:7373 | NORMAL_TEXT]
가까운 구간 ●  → 현재 횡방향 오차

[P02633 | 7373:7428 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@7373:7374 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧가까운 중심으로 횡방향 오차를 계산하고, 가까운 중심과 먼 중심의 차이로 선 방향을 계산합니다.

[P02634 | 7428:7587 | NORMAL_TEXT]
\[ e_x = \frac{x_{near}-x_{image}}{x_{image}} \]\[ e_\theta = \operatorname{atan2}(x_{far}-x_{near}, y_{near}-y_{far}) \]\[ e = w_x e_x + w_\theta e_\theta \]

[P02635 | 7587:7624 | NORMAL_TEXT]
이를 PD 제어기에 입력하면 커브를 더 일찍 인식할 수 있습니다.

[P02636 | 7624:7636 | HEADING_3]
권장 전체 코드 구조

[P02637 | 7636:7668 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@7636:7637 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧def detect_yellow_line(frame):

[P02638 | 7668:7695 | NORMAL_TEXT]
    h, w = frame.shape[:2]

[P02639 | 7695:7696 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02640 | 7696:7723 | NORMAL_TEXT]
    # 실제 환경에서는 사다리꼴 ROI 권장

[P02641 | 7723:7759 | NORMAL_TEXT]
    roi = frame[int(h * 0.45):h, :]

[P02642 | 7759:7760 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02643 | 7760:7807 | NORMAL_TEXT]
    blurred = cv2.GaussianBlur(roi, (5, 5), 0)

[P02644 | 7807:7808 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02645 | 7808:7825 | NORMAL_TEXT]
    # HSV 노란색 검출

[P02646 | 7825:7876 | NORMAL_TEXT]
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

[P02647 | 7876:7904 | NORMAL_TEXT]
    hsv_mask = cv2.inRange(

[P02648 | 7904:7917 | NORMAL_TEXT]
        hsv,

[P02649 | 7917:7949 | NORMAL_TEXT]
        np.array([15, 70, 70]),

[P02650 | 7949:7982 | NORMAL_TEXT]
        np.array([40, 255, 255])

[P02651 | 7982:7988 | NORMAL_TEXT]
    )

[P02652 | 7988:7989 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02653 | 7989:8008 | NORMAL_TEXT]
    # LAB 노란 성분 검출

[P02654 | 8008:8059 | NORMAL_TEXT]
    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)

[P02655 | 8059:8088 | NORMAL_TEXT]
    b_channel = lab[:, :, 2]

[P02656 | 8088:8121 | NORMAL_TEXT]
    _, lab_mask = cv2.threshold(

[P02657 | 8121:8168 | NORMAL_TEXT]
        b_channel, 145, 255, cv2.THRESH_BINARY

[P02658 | 8168:8174 | NORMAL_TEXT]
    )

[P02659 | 8174:8175 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02660 | 8175:8192 | NORMAL_TEXT]
    # 두 색상 조건 결합

[P02661 | 8192:8239 | NORMAL_TEXT]
    mask = cv2.bitwise_and(hsv_mask, lab_mask)

[P02662 | 8239:8240 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02663 | 8240:8263 | NORMAL_TEXT]
    # 노이즈 제거와 끊어진 선 연결

[P02664 | 8263:8308 | NORMAL_TEXT]
    open_kernel = cv2.getStructuringElement(

[P02665 | 8308:8342 | NORMAL_TEXT]
        cv2.MORPH_ELLIPSE, (3, 3)

[P02666 | 8342:8348 | NORMAL_TEXT]
    )

[P02667 | 8348:8394 | NORMAL_TEXT]
    close_kernel = cv2.getStructuringElement(

[P02668 | 8394:8425 | NORMAL_TEXT]
        cv2.MORPH_RECT, (7, 7)

[P02669 | 8425:8431 | NORMAL_TEXT]
    )

[P02670 | 8431:8432 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02671 | 8432:8461 | NORMAL_TEXT]
    mask = cv2.morphologyEx(

[P02672 | 8461:8503 | NORMAL_TEXT]
        mask, cv2.MORPH_OPEN, open_kernel

[P02673 | 8503:8509 | NORMAL_TEXT]
    )

[P02674 | 8509:8538 | NORMAL_TEXT]
    mask = cv2.morphologyEx(

[P02675 | 8538:8582 | NORMAL_TEXT]
        mask, cv2.MORPH_CLOSE, close_kernel

[P02676 | 8582:8588 | NORMAL_TEXT]
    )

[P02677 | 8588:8589 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02678 | 8589:8610 | NORMAL_TEXT]
    return roi, mask

[P02679 | 8610:8629 | HEADING_3]
⟦PROTECTED: opaqueTemplateControl/copy-only@8610:8611 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧야외 환경에서 특히 중요한 부분

[P02680 | 8629:8691 | NORMAL_TEXT | LIST id=kix.1n30subyfmre level=0]
자동 노출과 자동 화이트밸런스가 계속 변하면 색 임계값도 흔들립니다. 가능하면 노출·화이트밸런스를 고정합니다.

[P02681 | 8691:8734 | NORMAL_TEXT | LIST id=kix.1n30subyfmre level=0]
맑은 날, 흐린 날, 그늘, 해질녘 영상을 각각 수집해 임계값을 검증합니다.

[P02682 | 8734:8785 | NORMAL_TEXT | LIST id=kix.1n30subyfmre level=0]
젖은 아스팔트의 반사광은 밝기가 높아질 수 있으므로 밝기만으로 노란색을 판단하지 않습니다.

[P02683 | 8785:8843 | NORMAL_TEXT | LIST id=kix.1n30subyfmre level=0]
선 검출이 불확실하면 감속하고, 일정 프레임 이상 실패하면 정지하거나 마지막 검출 방향으로 탐색합니다.

[P02684 | 8843:8902 | NORMAL_TEXT | LIST id=kix.1n30subyfmre level=0]
풀밭이 ROI 내부에 많이 보인다면 색상 임계값보다 도로 영역 분리와 선의 연속성 검사가 더 중요합니다.

[P02685 | 8902:9004 | NORMAL_TEXT]
실용적인 첫 구현은 사다리꼴 ROI → HSV+LAB 노란색 마스크 → morphology → 다중 높이 중심점 → PD 조향 → 오차 비례 감속 구성으로 시작하는 것이 적합합니다.

[P02686 | 9004:9005 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02687 | 9005:9007 | NORMAL_TEXT]
[HORIZONTAL_RULE]

[P02688 | 9007:9008 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02689 | 9008:9071 | NORMAL_TEXT]
Q:Line detect 하고 선 중심을 따라서 4족보행 로봇의 명령를 생성한다고 하면 어떻게 구현해야 하지? 

[P02690 | 9071:9140 | NORMAL_TEXT]
예를 들어 선이 곡선으로 천천히 굽혀질때 x,y 명령을 주면서 yaw 도 천천히 따라서 움직이게 하려면 어떻게 해야 하지?

[P02691 | 9140:9171 | NORMAL_TEXT]
혹은 흐린날 맑은날의 노란색의 차이는 어떻게 케어하지?

[P02692 | 9171:9172 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02693 | 9172:9173 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02694 | 9173:9287 | NORMAL_TEXT]
A:4족보행 로봇이라면 검출된 선의 중심 한 점만 따라가기보다, 선을 지면 좌표계의 곡선으로 추정한 다음 전진 속도 vx, 측면 속도 vy, 회전 속도 yaw_rate를 동시에 생성하는 구조가 좋습니다.

[P02695 | 9287:9320 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@9287:9288 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧영상 → 노란 선 마스크 → Bird’s-eye View

[P02696 | 9320:9350 | NORMAL_TEXT]
    → 중심선 곡선 추정 → 위치·방향·곡률 오차

[P02697 | 9350:9376 | NORMAL_TEXT]
    → vx, vy, yaw_rate 생성

[P02698 | 9376:9400 | NORMAL_TEXT]
    → 보행 컨트롤러 / cmd_vel

[P02699 | 9400:9421 | HEADING_2]
⟦PROTECTED: opaqueTemplateControl/copy-only@9400:9401 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧1. 영상 좌표를 로봇 좌표로 변환

[P02700 | 9421:9534 | NORMAL_TEXT]
영상 픽셀 오차를 그대로 제어에 사용하면 가까운 픽셀과 먼 픽셀의 실제 거리가 다릅니다. 카메라 캘리브레이션과 평면 Homography를 이용해 노란 선을 지면의 로봇 좌표계로 변환하는 것이 좋습니다.

[P02701 | 9534:9556 | NORMAL_TEXT]
로봇 좌표계를 다음처럼 정의하겠습니다.

[P02702 | 9556:9571 | NORMAL_TEXT | LIST id=kix.vmjsuni4eklv level=0]
x: 로봇 전방, 단위 m

[P02703 | 9571:9586 | NORMAL_TEXT | LIST id=kix.vmjsuni4eklv level=0]
y: 로봇 왼쪽, 단위 m

[P02704 | 9586:9601 | NORMAL_TEXT | LIST id=kix.vmjsuni4eklv level=0]
yaw: 반시계 방향 양수

[P02705 | 9601:9650 | NORMAL_TEXT]
Bird’s-eye View로 변환한 뒤 각 전방 거리에서 노란 선의 중심을 구합니다.

[P02706 | 9650:9670 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@9650:9651 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧x = 2.0 m        ●

[P02707 | 9670:9688 | NORMAL_TEXT]
                /

[P02708 | 9688:9705 | NORMAL_TEXT]
x = 1.5 m      ●

[P02709 | 9705:9721 | NORMAL_TEXT]
              /

[P02710 | 9721:9736 | NORMAL_TEXT]
x = 1.0 m    ●

[P02711 | 9736:9750 | NORMAL_TEXT]
            /

[P02712 | 9750:9763 | NORMAL_TEXT]
x = 0.5 m  ●

[P02713 | 9763:9776 | NORMAL_TEXT]
          로봇

[P02714 | 9776:9814 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@9776:9777 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧검출된 중심점들을 2차 또는 3차 다항식으로 근사할 수 있습니다.

[P02715 | 9814:9848 | NORMAL_TEXT]
\[ y(x)=a_0+a_1x+a_2x^2+a_3x^3 \]

[P02716 | 9848:9871 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@9848:9849 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧# forward_x: 로봇 전방 거리

[P02717 | 9871:9908 | NORMAL_TEXT]
# lateral_y: 해당 거리에서 검출한 선 중심의 좌우 위치

[P02718 | 9908:9956 | NORMAL_TEXT]
coeff = np.polyfit(forward_x, lateral_y, deg=2)

[P02719 | 9956:9980 | NORMAL_TEXT]
path = np.poly1d(coeff)

[P02720 | 9980:10011 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@9980:9981 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧급격한 곡선이 아니라면 2차 다항식으로도 충분합니다.

[P02721 | 10011:10030 | HEADING_2]
2. 위치 오차와 방향 오차 계산

[P02722 | 10030:10037 | HEADING_3]
횡방향 오차

[P02723 | 10037:10061 | NORMAL_TEXT]
로봇 가까이에 있는 선의 좌우 위치입니다.

[P02724 | 10061:10085 | NORMAL_TEXT]
\[ e_y = y(x_{near}) \]

[P02725 | 10085:10113 | NORMAL_TEXT]
예를 들어 x_near = 0.3m로 설정합니다.

[P02726 | 10113:10130 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@10113:10114 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧e_y = path(0.3)

[P02727 | 10130:10149 | NORMAL_TEXT | LIST id=kix.pdmgdr6uxdes level=0]
⟦PROTECTED: opaqueTemplateControl/copy-only@10130:10131 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧e_y > 0: 선이 로봇 왼쪽

[P02728 | 10149:10168 | NORMAL_TEXT | LIST id=kix.pdmgdr6uxdes level=0]
e_y < 0: 선이 로봇 오른쪽

[P02729 | 10168:10174 | HEADING_3]
방향 오차

[P02730 | 10174:10209 | NORMAL_TEXT]
다항식의 미분을 이용하면 선의 진행 방향을 구할 수 있습니다.

[P02731 | 10209:10280 | NORMAL_TEXT]
\[ y'(x)=a_1+2a_2x+3a_3x^2 \]\[ e_\psi=\tan^{-1}(y'(x_{lookahead})) \]

[P02732 | 10280:10316 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@10280:10281 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧path_derivative = np.polyder(path)

[P02733 | 10316:10351 | NORMAL_TEXT]
slope = path_derivative(lookahead)

[P02734 | 10351:10376 | NORMAL_TEXT]
e_yaw = np.arctan(slope)

[P02735 | 10376:10427 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@10376:10377 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧lookahead는 보통 0.5~1.5m에서 시작하고, 속도가 높을수록 길게 설정합니다.

[P02736 | 10427:10453 | NORMAL_TEXT]
\[ L = L_{min}+K_L v_x \]

[P02737 | 10453:10493 | NORMAL_TEXT]
가까운 점은 측면 위치 보정에, 먼 점은 곡선 방향 예측에 사용합니다.

[P02738 | 10493:10514 | HEADING_2]
3. vx, vy, yaw 명령 생성

[P02739 | 10514:10567 | NORMAL_TEXT]
4족보행 로봇이 전방·측면 이동을 모두 지원한다면 제어를 비교적 단순하게 분리할 수 있습니다.

[P02740 | 10567:10640 | NORMAL_TEXT]
\[ v_y=K_y e_y + K_{dy}\dot e_y \]\[ \omega_z=K_\psi e_\psi+v_x\kappa \]

[P02741 | 10640:10701 | NORMAL_TEXT]
여기서 κ는 선의 곡률입니다. vx × κ 항은 곡선을 미리 따라가는 feed-forward 역할을 합니다.

[P02742 | 10701:10728 | NORMAL_TEXT]
2차원 곡선 y(x)의 곡률은 다음과 같습니다.

[P02743 | 10728:10788 | NORMAL_TEXT]
\[ \kappa = \frac{y''(x)} {\left(1+y'(x)^2\right)^{3/2}} \]

[P02744 | 10788:10814 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@10788:10789 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧d1 = np.polyder(path, 1)

[P02745 | 10814:10839 | NORMAL_TEXT]
d2 = np.polyder(path, 2)

[P02746 | 10839:10840 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02747 | 10840:10862 | NORMAL_TEXT]
slope = d1(lookahead)

[P02748 | 10862:10912 | NORMAL_TEXT]
curvature = d2(lookahead) / (1.0 + slope**2)**1.5

[P02749 | 10912:10913 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02750 | 10913:10938 | NORMAL_TEXT]
e_yaw = np.arctan(slope)

[P02751 | 10938:10939 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02752 | 10939:10974 | NORMAL_TEXT]
vy_cmd = ky * e_y + kdy * e_y_rate

[P02753 | 10974:11024 | NORMAL_TEXT]
yaw_rate_cmd = k_yaw * e_yaw + vx_cmd * curvature

[P02754 | 11024:11041 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@11024:11025 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧각 역할은 다음과 같습니다.

[P02755 | 11041:11054 | NORMAL_TEXT | LIST id=kix.h9536pug0hx7 level=0]
vx: 선을 따라 전진

[P02756 | 11054:11078 | NORMAL_TEXT | LIST id=kix.h9536pug0hx7 level=0]
vy: 로봇이 선 옆으로 벗어난 거리 보정

[P02757 | 11078:11108 | NORMAL_TEXT | LIST id=kix.h9536pug0hx7 level=0]
yaw_rate: 몸의 방향을 선의 접선 방향과 일치

[P02758 | 11108:11142 | NORMAL_TEXT | LIST id=kix.h9536pug0hx7 level=0]
vx × curvature: 커브에 들어가기 전에 회전 시작

[P02759 | 11142:11274 | NORMAL_TEXT]
횡이동 vy만으로 선 중앙을 맞추면 로봇 몸체가 선과 평행하지 않을 수 있고, yaw만 사용하면 완만한 측면 편차를 수정하는 데 시간이 걸립니다. 따라서 vy로 위치를 천천히 보정하고 yaw_rate로 방향을 맞추는 조합이 적합합니다.

[P02760 | 11274:11292 | HEADING_2]
4. 곡선에서 전진 속도 낮추기

[P02761 | 11292:11315 | NORMAL_TEXT]
곡률과 오차가 커질수록 속도를 낮춥니다.

[P02762 | 11315:11391 | NORMAL_TEXT]
\[ v_x = \frac{v_{nominal}} {1+K_\kappa|\kappa|+K_e|e_y|+K_\psi|e_\psi|} \]

[P02763 | 11391:11416 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@11391:11392 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧vx_cmd = nominal_vx / (

[P02764 | 11416:11424 | NORMAL_TEXT]
    1.0

[P02765 | 11424:11462 | NORMAL_TEXT]
    + curvature_gain * abs(curvature)

[P02766 | 11462:11492 | NORMAL_TEXT]
    + lateral_gain * abs(e_y)

[P02767 | 11492:11524 | NORMAL_TEXT]
    + heading_gain * abs(e_yaw)

[P02768 | 11524:11526 | NORMAL_TEXT]
)

[P02769 | 11526:11527 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02770 | 11527:11568 | NORMAL_TEXT]
vx_cmd = np.clip(vx_cmd, min_vx, max_vx)

[P02771 | 11568:11613 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@11568:11569 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧예시 초기 제한값은 로봇 성능에 맞춰 다음처럼 보수적으로 시작할 수 있습니다.

[P02772 | 11613:11670 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@11613:11614 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧vx_cmd       = np.clip(vx_cmd,       0.05, 0.30)  # m/s

[P02773 | 11670:11726 | NORMAL_TEXT]
vy_cmd       = np.clip(vy_cmd,      -0.10, 0.10)  # m/s

[P02774 | 11726:11784 | NORMAL_TEXT]
yaw_rate_cmd = np.clip(yaw_rate_cmd, -0.40, 0.40) # rad/s

[P02775 | 11784:11801 | HEADING_2]
⟦PROTECTED: opaqueTemplateControl/copy-only@11784:11785 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧5. 명령을 부드럽게 만들기

[P02776 | 11801:11865 | NORMAL_TEXT]
영상 검출이 조금씩 흔들리면 vy, yaw_rate도 흔들립니다. 저역통과 필터와 변화율 제한을 적용해야 합니다.

[P02777 | 11865:11873 | HEADING_3]
저역통과 필터

[P02778 | 11873:11917 | NORMAL_TEXT]
\[ u_f(t)=\alpha u(t)+(1-\alpha)u_f(t-1) \]

[P02779 | 11917:11976 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@11917:11918 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧filtered_yaw = alpha * yaw_raw + (1-alpha) * previous_yaw

[P02780 | 11976:12029 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@11976:11977 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧alpha는 약 0.1~0.3에서 시작할 수 있습니다. 작을수록 부드럽지만 반응이 늦습니다.

[P02781 | 12029:12036 | HEADING_3]
변화율 제한

[P02782 | 12036:12085 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@12036:12037 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧def rate_limit(target, previous, max_rate, dt):

[P02783 | 12085:12106 | NORMAL_TEXT]
    delta = np.clip(

[P02784 | 12106:12133 | NORMAL_TEXT]
        target - previous,

[P02785 | 12133:12157 | NORMAL_TEXT]
        -max_rate * dt,

[P02786 | 12157:12180 | NORMAL_TEXT]
         max_rate * dt

[P02787 | 12180:12186 | NORMAL_TEXT]
    )

[P02788 | 12186:12214 | NORMAL_TEXT]
    return previous + delta

[P02789 | 12214:12222 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@12214:12215 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧예를 들어:

[P02790 | 12222:12270 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@12222:12223 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧vx_cmd = rate_limit(vx_raw, prev_vx, 0.30, dt)

[P02791 | 12270:12317 | NORMAL_TEXT]
vy_cmd = rate_limit(vy_raw, prev_vy, 0.25, dt)

[P02792 | 12317:12367 | NORMAL_TEXT]
yaw_cmd = rate_limit(yaw_raw, prev_yaw, 0.80, dt)

[P02793 | 12367:12501 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@12367:12368 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧4족보행 로봇에서는 카메라가 보행 충격으로 흔들리기 때문에 영상 중심점 자체에도 EMA 또는 Kalman Filter를 적용하면 좋습니다. IMU로 카메라의 roll/pitch 변화를 보정하면 Bird’s-eye 변환도 더 안정적입니다.

[P02794 | 12501:12513 | HEADING_2]
6. 전체 제어 예시

[P02795 | 12513:12565 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@12513:12514 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧def generate_velocity_command(path, previous, dt):

[P02796 | 12565:12583 | NORMAL_TEXT]
    near_x = 0.30

[P02797 | 12583:12604 | NORMAL_TEXT]
    lookahead = 0.80

[P02798 | 12604:12605 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02799 | 12605:12634 | NORMAL_TEXT]
    d1 = np.polyder(path, 1)

[P02800 | 12634:12663 | NORMAL_TEXT]
    d2 = np.polyder(path, 2)

[P02801 | 12663:12664 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02802 | 12664:12677 | NORMAL_TEXT]
    # 횡방향 오차

[P02803 | 12677:12707 | NORMAL_TEXT]
    e_y = float(path(near_x))

[P02804 | 12707:12708 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02805 | 12708:12722 | NORMAL_TEXT]
    # 방향 및 곡률

[P02806 | 12722:12755 | NORMAL_TEXT]
    slope = float(d1(lookahead))

[P02807 | 12755:12784 | NORMAL_TEXT]
    e_yaw = np.arctan(slope)

[P02808 | 12784:12785 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02809 | 12785:12808 | NORMAL_TEXT]
    curvature = float(

[P02810 | 12808:12832 | NORMAL_TEXT]
        d2(lookahead) /

[P02811 | 12832:12862 | NORMAL_TEXT]
        (1.0 + slope**2)**1.5

[P02812 | 12862:12868 | NORMAL_TEXT]
    )

[P02813 | 12868:12869 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02814 | 12869:12921 | NORMAL_TEXT]
    e_y_rate = (e_y - previous.e_y) / max(dt, 1e-3)

[P02815 | 12921:12922 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02816 | 12922:12943 | NORMAL_TEXT]
    # 곡선과 오차가 클수록 감속

[P02817 | 12943:12965 | NORMAL_TEXT]
    vx_raw = 0.30 / (

[P02818 | 12965:12977 | NORMAL_TEXT]
        1.0

[P02819 | 12977:13008 | NORMAL_TEXT]
        + 2.0 * abs(curvature)

[P02820 | 13008:13033 | NORMAL_TEXT]
        + 1.5 * abs(e_y)

[P02821 | 13033:13060 | NORMAL_TEXT]
        + 1.0 * abs(e_yaw)

[P02822 | 13060:13066 | NORMAL_TEXT]
    )

[P02823 | 13066:13067 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02824 | 13067:13093 | NORMAL_TEXT]
    # 측면 위치와 몸체 방향을 각각 제어

[P02825 | 13093:13134 | NORMAL_TEXT]
    vy_raw = 0.8 * e_y + 0.05 * e_y_rate

[P02826 | 13134:13181 | NORMAL_TEXT]
    yaw_raw = 1.2 * e_yaw + vx_raw * curvature

[P02827 | 13181:13182 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02828 | 13182:13194 | NORMAL_TEXT]
    # 명령 제한

[P02829 | 13194:13235 | NORMAL_TEXT]
    vx_raw = np.clip(vx_raw, 0.05, 0.30)

[P02830 | 13235:13277 | NORMAL_TEXT]
    vy_raw = np.clip(vy_raw, -0.10, 0.10)

[P02831 | 13277:13321 | NORMAL_TEXT]
    yaw_raw = np.clip(yaw_raw, -0.40, 0.40)

[P02832 | 13321:13322 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02833 | 13322:13338 | NORMAL_TEXT]
    # 명령 변화율 제한

[P02834 | 13338:13393 | NORMAL_TEXT]
    vx_cmd = rate_limit(vx_raw, previous.vx, 0.30, dt)

[P02835 | 13393:13448 | NORMAL_TEXT]
    vy_cmd = rate_limit(vy_raw, previous.vy, 0.25, dt)

[P02836 | 13448:13506 | NORMAL_TEXT]
    yaw_cmd = rate_limit(yaw_raw, previous.yaw, 0.80, dt)

[P02837 | 13506:13507 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02838 | 13507:13542 | NORMAL_TEXT]
    return vx_cmd, vy_cmd, yaw_cmd

[P02839 | 13542:13576 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@13542:13543 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧ROS 2를 사용한다면 일반적으로 다음과 같이 연결합니다.

[P02840 | 13576:13599 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@13576:13577 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧msg.linear.x = vx_cmd

[P02841 | 13599:13621 | NORMAL_TEXT]
msg.linear.y = vy_cmd

[P02842 | 13621:13645 | NORMAL_TEXT]
msg.angular.z = yaw_cmd

[P02843 | 13645:13676 | NORMAL_TEXT]
cmd_vel_publisher.publish(msg)

[P02844 | 13676:13719 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@13676:13677 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧단, 실제 로봇 SDK가 사용하는 좌표축과 부호는 반드시 확인해야 합니다.

[P02845 | 13719:13738 | HEADING_2]
7. 선 검출 신뢰도에 따른 동작

[P02846 | 13738:13813 | NORMAL_TEXT]
검출된 선을 항상 신뢰하면 오검출 순간 로봇이 갑자기 회전할 수 있습니다. 다음 요소로 confidence를 계산하는 것이 좋습니다.

[P02847 | 13813:13822 | NORMAL_TEXT | LIST id=kix.c68spczd8l68 level=0]
노란색 픽셀 수

[P02848 | 13822:13843 | NORMAL_TEXT | LIST id=kix.c68spczd8l68 level=0]
전방 여러 구간에서 검출된 중심점 수

[P02849 | 13843:13860 | NORMAL_TEXT | LIST id=kix.c68spczd8l68 level=0]
다항식과 실제 점 사이의 오차

[P02850 | 13860:13875 | NORMAL_TEXT | LIST id=kix.c68spczd8l68 level=0]
이전 프레임 경로와의 차이

[P02851 | 13875:13889 | NORMAL_TEXT | LIST id=kix.c68spczd8l68 level=0]
검출된 선의 폭과 연속성

[P02852 | 13889:13908 | NORMAL_TEXT | LIST id=kix.c68spczd8l68 level=0]
도로 ROI 내부에 존재하는 비율

[P02853 | 13908:13928 | NORMAL_TEXT]
동작 규칙 예시는 다음과 같습니다.

[P02854 | 13928:13950 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@13928:13929 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧if confidence > 0.7:

[P02855 | 13950:13962 | NORMAL_TEXT]
    # 정상 추종

[P02856 | 13962:13979 | NORMAL_TEXT]
    track_line()

[P02857 | 13979:13980 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02858 | 13980:14003 | NORMAL_TEXT]
elif confidence > 0.4:

[P02859 | 14003:14031 | NORMAL_TEXT]
    # 감속하고 이전 경로와 현재 경로를 혼합

[P02860 | 14031:14049 | NORMAL_TEXT]
    vx_cmd *= 0.5

[P02861 | 14049:14073 | NORMAL_TEXT]
    use_smoothed_path()

[P02862 | 14073:14074 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02863 | 14074:14080 | NORMAL_TEXT]
else:

[P02864 | 14080:14119 | NORMAL_TEXT]
    # 즉시 잘못된 방향으로 회전하지 않도록 정지 또는 저속 탐색

[P02865 | 14119:14136 | NORMAL_TEXT]
    vx_cmd = 0.0

[P02866 | 14136:14153 | NORMAL_TEXT]
    vy_cmd = 0.0

[P02867 | 14153:14187 | NORMAL_TEXT]
    search_using_last_direction()

[P02868 | 14187:14210 | HEADING_2]
⟦PROTECTED: opaqueTemplateControl/copy-only@14187:14188 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧흐린 날과 맑은 날의 노란색 차이 처리

[P02869 | 14210:14254 | NORMAL_TEXT]
날씨 변화에는 고정 HSV 범위 하나만 사용하는 것보다 다음 조합이 좋습니다.

[P02870 | 14254:14278 | HEADING_3]
1. HSV의 H와 LAB의 b 채널 결합

[P02871 | 14278:14326 | NORMAL_TEXT]
흐린 날에는 채도와 밝기가 낮아질 수 있지만 노란색의 색상축은 어느 정도 유지됩니다.

[P02872 | 14326:14351 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@14326:14327 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧hsv_mask = cv2.inRange(

[P02873 | 14351:14360 | NORMAL_TEXT]
    hsv,

[P02874 | 14360:14388 | NORMAL_TEXT]
    np.array([14, 45, 40]),

[P02875 | 14388:14417 | NORMAL_TEXT]
    np.array([42, 255, 255])

[P02876 | 14417:14419 | NORMAL_TEXT]
)

[P02877 | 14419:14420 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02878 | 14420:14444 | NORMAL_TEXT]
lab_mask = cv2.inRange(

[P02879 | 14444:14462 | NORMAL_TEXT]
    lab[:, :, 2],

[P02880 | 14462:14482 | NORMAL_TEXT]
    adaptive_b_min,

[P02881 | 14482:14490 | NORMAL_TEXT]
    255

[P02882 | 14490:14492 | NORMAL_TEXT]
)

[P02883 | 14492:14493 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02884 | 14493:14527 | NORMAL_TEXT]
yellow_mask = hsv_mask & lab_mask

[P02885 | 14527:14591 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@14527:14528 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧흐린 날을 고려해 S, V 하한을 낮추되, 넓어진 범위에서 발생하는 오검출은 선의 형태와 연속성으로 제거합니다.

[P02886 | 14591:14604 | HEADING_3]
2. 밝기 채널만 보정

[P02887 | 14604:14697 | NORMAL_TEXT]
색상 채널 전체를 강하게 histogram equalization하면 노란색 자체가 변형될 수 있습니다. LAB의 L 채널에만 CLAHE를 적용하는 편이 안전합니다.

[P02888 | 14697:14743 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@14697:14698 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

[P02889 | 14743:14768 | NORMAL_TEXT]
L, a, b = cv2.split(lab)

[P02890 | 14768:14769 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02891 | 14769:14794 | NORMAL_TEXT]
clahe = cv2.createCLAHE(

[P02892 | 14794:14813 | NORMAL_TEXT]
    clipLimit=2.0,

[P02893 | 14813:14837 | NORMAL_TEXT]
    tileGridSize=(8, 8)

[P02894 | 14837:14839 | NORMAL_TEXT]
)

[P02895 | 14839:14858 | NORMAL_TEXT]
L = clahe.apply(L)

[P02896 | 14858:14859 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02897 | 14859:14892 | NORMAL_TEXT]
corrected = cv2.merge([L, a, b])

[P02898 | 14892:14947 | NORMAL_TEXT]
corrected = cv2.cvtColor(corrected, cv2.COLOR_LAB2BGR)

[P02899 | 14947:14964 | HEADING_3]
⟦PROTECTED: opaqueTemplateControl/copy-only@14947:14948 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧3. 카메라 자동 설정 관리

[P02900 | 14964:15007 | NORMAL_TEXT | LIST id=kix.jaxk5213klx8 level=0]
자동 화이트밸런스가 계속 바뀌면 같은 노란색의 Hue도 움직일 수 있습니다.

[P02901 | 15007:15055 | NORMAL_TEXT | LIST id=kix.jaxk5213klx8 level=0]
시작 시 몇 초간 자동 노출과 화이트밸런스를 안정화한 후 고정하는 방법이 유용합니다.

[P02902 | 15055:15118 | NORMAL_TEXT | LIST id=kix.jaxk5213klx8 level=0]
맑은 곳과 그늘을 자주 오간다면 노출은 자동으로 두되 화이트밸런스 변화 속도를 제한하는 편이 나을 수 있습니다.

[P02903 | 15118:15167 | NORMAL_TEXT | LIST id=kix.jaxk5213klx8 level=0]
과노출된 노란색은 흰색처럼 보여 Hue 정보가 사라지므로 노출 상한을 제한해야 합니다.

[P02904 | 15167:15184 | HEADING_3]
4. 장면에 따라 임계값 적응

[P02905 | 15184:15245 | NORMAL_TEXT]
LAB의 b 채널 평균과 분산, 또는 상위 percentile을 이용해 노란색 임계값을 조정할 수 있습니다.

[P02906 | 15245:15270 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@15245:15246 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧b_values = lab[:, :, 2]

[P02907 | 15270:15292 | NORMAL_TEXT]
adaptive_b_min = max(

[P02908 | 15292:15301 | NORMAL_TEXT]
    135,

[P02909 | 15301:15333 | NORMAL_TEXT]
    np.percentile(b_values, 85)

[P02910 | 15333:15335 | NORMAL_TEXT]
)

[P02911 | 15335:15406 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@15335:15336 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧다만 화면에 노란 선이 작으면 percentile 방식이 불안정할 수 있으므로, 고정 하한과 적응값을 함께 사용해야 합니다.

[P02912 | 15406:15436 | HEADING_3]
5. 가장 강한 해결책: Segmentation 모델

[P02913 | 15436:15550 | NORMAL_TEXT]
맑음·흐림·그늘·젖은 도로 변화가 크다면 색상 threshold만으로는 한계가 있습니다. 실제 환경 영상을 수집해 “노란 주행선”을 segmentation하는 경량 모델을 사용하는 것이 가장 견고합니다.

[P02914 | 15550:15576 | NORMAL_TEXT]
학습 데이터에는 다음 변형을 포함해야 합니다.

[P02915 | 15576:15588 | NORMAL_TEXT | LIST id=kix.kvz4ge4v1nq3 level=0]
밝기·대비·감마 변화

[P02916 | 15588:15599 | NORMAL_TEXT | LIST id=kix.kvz4ge4v1nq3 level=0]
색온도와 채도 변화

[P02917 | 15599:15603 | NORMAL_TEXT | LIST id=kix.kvz4ge4v1nq3 level=0]
그림자

[P02918 | 15603:15616 | NORMAL_TEXT | LIST id=kix.kvz4ge4v1nq3 level=0]
비 또는 젖은 아스팔트

[P02919 | 15616:15626 | NORMAL_TEXT | LIST id=kix.kvz4ge4v1nq3 level=0]
선의 마모와 낙엽

[P02920 | 15626:15647 | NORMAL_TEXT | LIST id=kix.kvz4ge4v1nq3 level=0]
카메라 흔들림과 motion blur

[P02921 | 15647:15667 | NORMAL_TEXT]
권장 개발 순서는 다음과 같습니다.

[P02922 | 15667:15700 | NORMAL_TEXT | LIST id=kix.9mx1is6yz1yl level=0]
HSV + LAB + 기하학적 선 검사로 기본 시스템 구현

[P02923 | 15700:15732 | NORMAL_TEXT | LIST id=kix.9mx1is6yz1yl level=0]
Bird’s-eye View에서 중심선을 다항식으로 추정

[P02924 | 15732:15775 | NORMAL_TEXT | LIST id=kix.9mx1is6yz1yl level=0]
e_y, e_yaw, curvature로 vx, vy, yaw_rate 생성

[P02925 | 15775:15798 | NORMAL_TEXT | LIST id=kix.9mx1is6yz1yl level=0]
필터·변화율 제한·신뢰도 기반 감속 추가

[P02926 | 15798:15835 | NORMAL_TEXT | LIST id=kix.9mx1is6yz1yl level=0]
날씨 변화에서 부족하면 segmentation 모델로 검출부 교체

[P02927 | 15835:15992 | NORMAL_TEXT]
제어 관점에서 가장 중요한 원칙은 vy는 선 중심으로의 위치 복원에 사용하고, yaw_rate는 선의 접선 방향 추종에 사용하며, 곡률을 feed-forward로 넣는 것입니다. 이렇게 해야 완만한 커브에서 몸을 갑자기 틀지 않고 자연스럽게 옆으로 정렬하면서 회전할 수 있습니다.

[P02928 | 15992:15994 | NORMAL_TEXT]
[HORIZONTAL_RULE]

[P02929 | 15994:15995 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02930 | 15995:15996 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P02931 | 15996:16071 | NORMAL_TEXT]
결론부터 말하면, 현재 결과물은 “노란색 선 segmentation”이 아니라 “도로 위의 차선 형태 segmentation”입니다.

[P02932 | 16071:16088 | NORMAL_TEXT | LIST id=kix.66eqi2ed3ib5 level=0]
하단이 잘 인식되지 않는 이유

[P02933 | 16088:16157 | NORMAL_TEXT]
camera2와 yolop_overlay2는 길이가 동일해 정상적인 비교가 가능합니다. 결과를 보면 다음 문제가 있습니다.

[P02934 | 16157:16189 | NORMAL_TEXT | LIST id=kix.r7aean163gx level=0]
카메라가 어안 렌즈라 하단의 선이 크게 휘고 왜곡됩니다.

[P02935 | 16189:16241 | NORMAL_TEXT | LIST id=kix.r7aean163gx level=0]
하단·측면의 선이 도로와 인도/연석 경계에 걸쳐 있어 도로 mask에서 제외될 수 있습니다.

[P02936 | 16241:16277 | NORMAL_TEXT | LIST id=kix.r7aean163gx level=0]
최종 선 mask는 다음처럼 도로 mask와 교집합 처리됩니다.

[P02937 | 16277:16324 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@16277:16278 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧line_mask = raw_line_mask & dilated_road_mask

[P02938 | 16324:16378 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@16324:16325 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧즉, 모델이 선을 일부 인식했더라도 도로 영역으로 판단되지 않으면 최종 결과에서 사라집니다. 

[P02939 | 16378:16519 | NORMAL_TEXT]
또한 YOLOP는 BDD100K 일반 도로 영상으로 학습된 모델이라 A2 카메라의 저시점·어안 왜곡·보라색 색감·실내에서 실외로 이어지는 영상과 차이가 큽니다. 그래서 하단 도로를 인도로 판단하거나, 흰색 연석선을 차선으로 판단하는 현상이 발생합니다.

[P02940 | 16519:16539 | NORMAL_TEXT | LIST id=kix.pzjed665ud60 level=0]
노란색만 인식하도록 설정되어 있나?

[P02941 | 16539:16574 | NORMAL_TEXT]
현재 YOLOP 모델 경로에서는 노란색 전용 설정이 없습니다.

[P02942 | 16574:16600 | NORMAL_TEXT]
Overlay의 색상 의미는 다음과 같습니다.

[P02943 | 16600:16621 | NORMAL_TEXT | LIST id=kix.3h57uvjlwojg level=0]
초록색: 도로 segmentation

[P02944 | 16621:16652 | NORMAL_TEXT | LIST id=kix.3h57uvjlwojg level=0]
빨간색: YOLOP raw 차선 segmentation

[P02945 | 16652:16681 | NORMAL_TEXT | LIST id=kix.3h57uvjlwojg level=0]
노란색: 도로 mask와 교집합된 최종 선 mask

[P02946 | 16681:16769 | NORMAL_TEXT]
여기서 화면에 표시되는 노란색은 단순히 시각화 색상일 뿐, “노란색 픽셀만 검출했다”는 뜻이 아닙니다. 따라서 흰색 연석선도 노란색으로 표시될 수 있습니다.

[P02947 | 16769:17178 | NORMAL_TEXT]
현재 hsv_lower, hsv_upper, lab_b_min 설정은 존재하지만, YOLOP 모델을 사용할 때는 적용되지 않습니다. 이 값들은 모델이 없을 때 사용하는 기존 색상 기반 fallback에만 사용됩니다. [vision.py (line 354)](/Users/kangminwoo/Documents/GitHub/cobiz-plugin-line-tracking/ros_ws/src/line_tracking/line_tracking/vision.py:354), [line_tracking.yaml (line 39)](/Users/kangminwoo/Documents/GitHub/cobiz-plugin-line-tracking/ros_ws/src/line_tracking/config/line_tracking.yaml:39)

[P02948 | 17178:17207 | NORMAL_TEXT]
따라서 요구사항대로 하려면 다음 구조가 필요합니다.

[P02949 | 17207:17232 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@17207:17208 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧YOLOP road segmentation

[P02950 | 17232:17242 | NORMAL_TEXT]
        +

[P02951 | 17242:17267 | NORMAL_TEXT]
노란색 HSV/LAB color filter

[P02952 | 17267:17277 | NORMAL_TEXT]
        =

[P02953 | 17277:17290 | NORMAL_TEXT]
최종 노란 선 mask

[P02954 | 17290:17340 | NORMAL_TEXT]
⟦PROTECTED: opaqueTemplateControl/copy-only@17290:17291 U+E907 — preserve this control and its containing structure⟧
⟦PROTECTED_CONTROL U+E907⟧현재 구조에는 이 “노란색 color filter”가 YOLOP 경로에 빠져 있습니다.

[P02955 | 17340:17341 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## 2026.08.31 인도·차도 검출 자율 개선 중간 결과 보고서 (t.9v3gj1y7d0rf)

[P02956 | 1:43 | TITLE]
[2026.08.31] 인도·차도 세그멘테이션 자율 개선 중간 결과 보고서

[P02957 | 43:54 | NORMAL_TEXT | LIST id=kix.of33swz0wimf level=0]
최종 업데이트: 8월 31, 2026

[P02958 | 54:61 | NORMAL_TEXT | LIST id=kix.of33swz0wimf level=0]
작성자: @MinWoo Kang <mwkang@teamgrit.kr>

[P02959 | 61:84 | NORMAL_TEXT | LIST id=kix.of33swz0wimf level=0]
LLM : Chat GPT-Sol 5.6

[P02960 | 84:90 | NORMAL_TEXT | LIST id=kix.of33swz0wimf level=0]
대상: [full](https://drive.google.com/drive/folders/1uQnp7FLFojb7lSmM8zsg2KjyvBPQZcnt)

[P02961 | 90:99 | HEADING_1]
1. 실행 개요

[P02962 | 99:240 | NORMAL_TEXT]
원본 8개 동영상에서 인도(Sidewalk)와 차도(Road)를 서로 다른 의미 클래스로 분리하는 semantic segmentation 모델을 평가하고, 대표 연속 프레임에서 노이즈·깜빡임·면적 급변·경계 불안정을 줄이기 위한 반복 개선을 수행했다.

[P02963 | 240:485 | NORMAL_TEXT]
최종 모델은 Mapillary Vistas로 학습된 Mask2Former이며, Road와 Sidewalk의 관련 세부 클래스를 각각 하나의 표면 클래스로 통합했다. 모델 추론 점수에 시간축 지수 평활과 낮은 확신의 클래스 전환만 보류하는 히스테리시스를 적용했다. 수치 게이트, 후보 영상 전체 디코딩, contact sheet 시각 검토, 30연속 프레임 장기 검증, 서로 다른 세 구간 검증을 통과한 설정만 고정 최고 결과로 유지했다.

[P02964 | 485:494 | HEADING_1]
2. 입력 영상

[P02965 | 494:504 | NORMAL_TEXT | LIST id=kix.n4y7encmal0p level=0]
정자역 주행 영상

[P02966 | 504:518 | NORMAL_TEXT | LIST id=kix.n4y7encmal0p level=1]
[20260827_070327_teamgrit_camera_full.mp4](https://drive.google.com/file/d/1E60605qLRxGVBf18bCqce6goPJz0tKz4/view?usp=drive_link), [20260827_064730_teamgrit_camera_full.mp4](https://drive.google.com/file/d/1ENYPrxwWWnVT2f6N4jHhK9X2KyIRLe_9/view?usp=drive_link), [20260827_062352_teamgrit_camera_full.mp4](https://drive.google.com/file/d/1PbqvXVhMZun6txRp1jmP93vYLGIpp6xX/view?usp=drive_link), [20260827_063215_teamgrit_camera_full.mp4](https://drive.google.com/file/d/1cPs-rRz_L1BSGHhFG11v-DHEfKA-dAC2/view?usp=drive_link), [20260827_070619_teamgrit_camera_full.mp4](https://drive.google.com/file/d/1l8v9ny4StuK9aqLLa3oMGkFPg9v5GCRN/view?usp=drive_link)

[P02967 | 518:527 | NORMAL_TEXT | LIST id=kix.n4y7encmal0p level=0]
당진 현장 촬영

[P02968 | 527:535 | NORMAL_TEXT | LIST id=kix.n4y7encmal0p level=1]
[rssp_instant_9e5cf641_de4e_413f_b86d_824466023a27_base_a2_front_camera2.mp4](https://drive.google.com/file/d/17dxM4S8-Nqogqe3l71KcujmR6-c0LyNG/view?usp=drive_link), [rssp_instant_9e5cf641_de4e_413f_b86d_824466023a27_base_a2_front_camera.mp4](https://drive.google.com/file/d/1TtaoCW-aKpH335IsbljIwM_IwK7qmfH2/view?usp=drive_link), [rssp_instant_9e5cf641_de4e_413f_b86d_824466023a27_base_a2_front_camera3.mp4](https://drive.google.com/file/d/1VZEME43lNNjvjP50yNC-Za63uMicBhUQ/view?usp=drive_link)

[P02969 | 535:544 | NORMAL_TEXT | LIST id=kix.n4y7encmal0p level=0]
상세 감사 기록

[P02970 | 544:546 | NORMAL_TEXT | LIST id=kix.n4y7encmal0p level=1]
[SOURCE_DECODE_AUDIT.md](https://docs.google.com/document/d/1KJiPWLFk02A5_Dr4C4Hhmq1DT3VZZnbscNa9-QdbOTs/edit?usp=drive_link)

[P02971 | 546:571 | HEADING_1]
3. 최종 best-so-far 모델과 설정

[P02972 | 571:633 | NORMAL_TEXT | LIST id=kix.a7exphdtjw0s level=0]
모델: facebook/mask2former-swin-large-mapillary-vistas-semantic

[P02973 | 633:687 | NORMAL_TEXT | LIST id=kix.a7exphdtjw0s level=0]
모델 revision: 4772b6bf101d91f2534c106dc524d906aeb3c68a

[P02974 | 687:714 | NORMAL_TEXT | LIST id=kix.a7exphdtjw0s level=0]
평가 프로파일: surface-aggregate

[P02975 | 714:730 | NORMAL_TEXT | LIST id=kix.a7exphdtjw0s level=0]
평가 해상도: 360×640

[P02976 | 730:773 | NORMAL_TEXT | LIST id=kix.a7exphdtjw0s level=0]
시간축 점수 혼합: current × 0.62 + history × 0.38

[P02977 | 773:793 | NORMAL_TEXT | LIST id=kix.a7exphdtjw0s level=0]
시간축 히스테리시스 마진: 0.07

[P02978 | 793:871 | NORMAL_TEXT | LIST id=kix.a7exphdtjw0s level=0]
히스테리시스 동작: 선택 클래스가 바뀌면서 현재 상위 두 클래스 점수 차가 0.07 미만일 때만 직전 Road/Sidewalk 선택을 유지

[P02979 | 871:916 | NORMAL_TEXT | LIST id=kix.a7exphdtjw0s level=0]
작은 연결요소 제거 최소 면적: max(48, 평가 픽셀 수 × 0.00035)

[P02980 | 916:952 | NORMAL_TEXT | LIST id=kix.a7exphdtjw0s level=0]
confidence threshold: 사용하지 않음 (0.0)

[P02981 | 952:984 | NORMAL_TEXT | LIST id=kix.a7exphdtjw0s level=0]
morphology closing: 사용하지 않음 (0)

[P02982 | 984:1018 | NORMAL_TEXT | LIST id=kix.a7exphdtjw0s level=0]
scene-change reset: 사용하지 않음 (0.0)

[P02983 | 1018:1063 | NORMAL_TEXT | LIST id=kix.a7exphdtjw0s level=0]
confidence-selective blending: 사용하지 않음 (0.0)

[P02984 | 1063:1105 | NORMAL_TEXT | LIST id=kix.a7exphdtjw0s level=0]
optical-flow motion compensation: 사용하지 않음

[P02985 | 1105:1145 | NORMAL_TEXT | LIST id=kix.a7exphdtjw0s level=0]
Road와 Sidewalk는 상호 배타적인 1/2 클래스 마스크로 생성

[P02986 | 1145:1157 | HEADING_2]
Road 통합 클래스

[P02987 | 1157:1170 | NORMAL_TEXT | LIST id=kix.xpbgjdhzfq8a level=0]
Road (ID 13)

[P02988 | 1170:1187 | NORMAL_TEXT | LIST id=kix.xpbgjdhzfq8a level=0]
Bike Lane (ID 7)

[P02989 | 1187:1212 | NORMAL_TEXT | LIST id=kix.xpbgjdhzfq8a level=0]
Crosswalk - Plain (ID 8)

[P02990 | 1212:1228 | NORMAL_TEXT | LIST id=kix.xpbgjdhzfq8a level=0]
Parking (ID 10)

[P02991 | 1228:1249 | NORMAL_TEXT | LIST id=kix.xpbgjdhzfq8a level=0]
Service Lane (ID 14)

[P02992 | 1249:1282 | NORMAL_TEXT | LIST id=kix.xpbgjdhzfq8a level=0]
Lane Marking - Crosswalk (ID 23)

[P02993 | 1282:1313 | NORMAL_TEXT | LIST id=kix.xpbgjdhzfq8a level=0]
Lane Marking - General (ID 24)

[P02994 | 1313:1329 | HEADING_2]
Sidewalk 통합 클래스

[P02995 | 1329:1346 | NORMAL_TEXT | LIST id=kix.qbxjrqkmazwh level=0]
Sidewalk (ID 15)

[P02996 | 1346:1370 | NORMAL_TEXT | LIST id=kix.qbxjrqkmazwh level=0]
Pedestrian Area (ID 11)

[P02997 | 1370:1386 | NORMAL_TEXT | LIST id=kix.qbxjrqkmazwh level=0]
Curb Cut (ID 9)

[P02998 | 1386:1402 | HEADING_1]
4. 최종 대표 프레임 지표

[P02999 | 1402:1469 | NORMAL_TEXT]
최종 승격 실험은 영상별 2개 구간, 구간당 15연속 프레임으로 총 240프레임과 224개 인접 프레임 쌍을 평가했다.

[P03000 | 1469:1479 | NORMAL_TEXT]
지표 · 최종 값

[P03001 | 1479:1509 | NORMAL_TEXT | LIST id=kix.5hg10rbukepn level=0]
선택 클래스 평균 변화율 · 0.02386875698

[P03002 | 1509:1537 | NORMAL_TEXT | LIST id=kix.5hg10rbukepn level=0]
Road 평균 변화율 · 0.01267227415

[P03003 | 1537:1569 | NORMAL_TEXT | LIST id=kix.5hg10rbukepn level=0]
Sidewalk 평균 변화율 · 0.01932243226

[P03004 | 1569:1601 | NORMAL_TEXT | LIST id=kix.5hg10rbukepn level=0]
Road 인접 프레임 IoU · 0.86222660980

[P03005 | 1601:1637 | NORMAL_TEXT | LIST id=kix.5hg10rbukepn level=0]
Sidewalk 인접 프레임 IoU · 0.89644753481

[P03006 | 1637:1667 | NORMAL_TEXT | LIST id=kix.5hg10rbukepn level=0]
Road 최대 면적 점프 · 0.39135416667

[P03007 | 1667:1701 | NORMAL_TEXT | LIST id=kix.5hg10rbukepn level=0]
Sidewalk 최대 면적 점프 · 0.38317708333

[P03008 | 1701:1724 | NORMAL_TEXT | LIST id=kix.5hg10rbukepn level=0]
Road 작은 연결요소 노이즈 · 0.0

[P03009 | 1724:1751 | NORMAL_TEXT | LIST id=kix.5hg10rbukepn level=0]
Sidewalk 작은 연결요소 노이즈 · 0.0

[P03010 | 1751:1780 | NORMAL_TEXT | LIST id=kix.5hg10rbukepn level=0]
원시 예측 보존 IoU · 0.86309733349

[P03011 | 1780:1812 | NORMAL_TEXT | LIST id=kix.5hg10rbukepn level=0]
히스테리시스 유지 픽셀 비율 · 0.00809208623

[P03012 | 1812:1902 | NORMAL_TEXT]
히스테리시스는 표본 픽셀의 약 0.81%에만 작용했다. 이전 마진 0.06보다 추적한 안정성 지표가 모두 개선되었고, 보존 IoU는 0.85 게이트를 통과했다.

[P03013 | 1902:1915 | HEADING_1]
5. 추가 강건성 검증

[P03014 | 1915:1931 | HEADING_2]
영상별 30연속 프레임 검증

[P03015 | 1931:1951 | NORMAL_TEXT | LIST id=kix.nscv1w87fy0k level=0]
총 240프레임, 232개 인접 쌍

[P03016 | 1951:1980 | NORMAL_TEXT | LIST id=kix.nscv1w87fy0k level=0]
선택 클래스 평균 변화율: 0.02076561003

[P03017 | 1980:2031 | NORMAL_TEXT | LIST id=kix.nscv1w87fy0k level=0]
Road / Sidewalk IoU: 0.91987914683 / 0.90239088339

[P03018 | 2031:2059 | NORMAL_TEXT | LIST id=kix.nscv1w87fy0k level=0]
원시 예측 보존 IoU: 0.86390339376

[P03019 | 2059:2098 | NORMAL_TEXT | LIST id=kix.nscv1w87fy0k level=0]
Road / Sidewalk 작은 연결요소 노이즈: 0.0 / 0.0

[P03020 | 2098:2148 | NORMAL_TEXT | LIST id=kix.nscv1w87fy0k level=0]
시작·중간·끝 시각 검토에서 재귀적인 오래된 마스크 누적이나 줄무늬 경계를 발견하지 못함

[P03021 | 2148:2171 | HEADING_2]
영상별 세 구간 × 10연속 프레임 검증

[P03022 | 2171:2191 | NORMAL_TEXT | LIST id=kix.o06amlhsjvfp level=0]
총 240프레임, 216개 인접 쌍

[P03023 | 2191:2220 | NORMAL_TEXT | LIST id=kix.o06amlhsjvfp level=0]
선택 클래스 평균 변화율: 0.02281127427

[P03024 | 2220:2271 | NORMAL_TEXT | LIST id=kix.o06amlhsjvfp level=0]
Road / Sidewalk IoU: 0.84376326198 / 0.88893458363

[P03025 | 2271:2299 | NORMAL_TEXT | LIST id=kix.o06amlhsjvfp level=0]
원시 예측 보존 IoU: 0.86314087567

[P03026 | 2299:2338 | NORMAL_TEXT | LIST id=kix.o06amlhsjvfp level=0]
Road / Sidewalk 작은 연결요소 노이즈: 0.0 / 0.0

[P03027 | 2338:2379 | NORMAL_TEXT | LIST id=kix.o06amlhsjvfp level=0]
서로 떨어진 시간대에서도 Road/Sidewalk 영역이 일관되게 유지됨

[P03028 | 2379:2400 | HEADING_1]
6. 실험 타임라인과 채택·기각 근거

[P03029 | 2400:2496 | NORMAL_TEXT | LIST id=kix.aex5pxsasezq level=0]
Cityscapes 계열 SegFormer 기준선을 평가했다. 보행 포장과 다양한 도로 표면 의미가 부족해 Mapillary Vistas Mask2Former로 교체했다.

[P03030 | 2496:2675 | NORMAL_TEXT | LIST id=kix.aex5pxsasezq level=0]
Mapillary의 기본 Road/Sidewalk에서 Pedestrian Area, Curb Cut, Bike Lane, Crosswalk, Parking, Service Lane, Lane Marking을 포함하는 surface aggregation으로 확장했다. 보행 표면 포함 범위와 차선 틈이 개선되어 채택했다.

[P03031 | 2675:2806 | NORMAL_TEXT | LIST id=kix.aex5pxsasezq level=0]
temporal alpha 0.56, 0.59, 0.62, 0.65를 비교했다. 0.62가 안정성과 원시 예측 보존의 균형이 가장 좋아 기준으로 채택했고, 0.65는 보존은 소폭 개선되지만 변화율·IoU·최대 점프가 회귀해 기각했다.

[P03032 | 2806:2889 | NORMAL_TEXT | LIST id=kix.aex5pxsasezq level=0]
confidence threshold 0.35와 morphology closing 3은 의미 영역 개선 없이 안정성 또는 경계가 회귀하여 기각했다.

[P03033 | 2889:3051 | NORMAL_TEXT | LIST id=kix.aex5pxsasezq level=0]
scene-change reset 0.25/0.75, confidence-selective 0.70, dense optical-flow motion compensation을 시험했다. 보존 또는 일부 국소 지표는 개선됐지만 전체 변화율과 Road/Sidewalk IoU가 나빠져 기각했다.

[P03034 | 3051:3137 | NORMAL_TEXT | LIST id=kix.aex5pxsasezq level=0]
히스테리시스 마진 0.03, 0.04는 안정성 지표가 충분히 좋아지지 않아 기각했다. 0.05, 0.06은 순차적으로 개선되어 중간 최고값으로 승격했다.

[P03035 | 3137:3229 | NORMAL_TEXT | LIST id=kix.aex5pxsasezq level=0]
마진 0.07은 0.06보다 변화율, Road/Sidewalk IoU, 최대 면적 점프를 모두 개선하고 시각적 오래된 경계를 만들지 않아 최종 최고값으로 승격했다.

[P03036 | 3229:3343 | NORMAL_TEXT | LIST id=kix.aex5pxsasezq level=0]
마진 0.08은 수치 안정성이 더 높았지만 전환 프레임에서 오래된 Road/Sidewalk 줄무늬가 보였고 보존율이 게이트에 가까워져 기각했다. 이는 수치만으로 승격하지 않고 시각 검토를 우선한 사례다.

[P03037 | 3343:3481 | NORMAL_TEXT | LIST id=kix.aex5pxsasezq level=0]
alpha 0.61 + 마진 0.07은 30연속 프레임에서 변화율과 IoU가 소폭 개선됐지만 보존 IoU가 0.86390에서 0.86346으로 낮아졌고, 마감 전 세 구간 검증을 받지 못했다. 따라서 승격하지 않고 alpha 0.62를 유지했다.

[P03038 | 3481:3493 | HEADING_1]
7. 시각 검토 결과

[P03039 | 3493:3539 | NORMAL_TEXT | LIST id=kix.unlq3kwvk2yn level=0]
보행자용 포장, 보행자 영역, curb cut은 대체로 Sidewalk로 묶였다.

[P03040 | 3539:3599 | NORMAL_TEXT | LIST id=kix.unlq3kwvk2yn level=0]
주행로, 자전거도로, 횡단보도, 주차·서비스 차로와 차선 표시는 Road로 묶여 도로 표면의 틈이 줄었다.

[P03041 | 3599:3640 | NORMAL_TEXT | LIST id=kix.unlq3kwvk2yn level=0]
최종 마진 0.07에서는 작은 노이즈가 제거되면서 경계 형상이 유지됐다.

[P03042 | 3640:3692 | NORMAL_TEXT | LIST id=kix.unlq3kwvk2yn level=0]
마진 0.08에서 발견된 줄무늬 형태의 오래된 클래스 잔상은 최종 설정에서 관찰되지 않았다.

[P03043 | 3692:3738 | NORMAL_TEXT | LIST id=kix.unlq3kwvk2yn level=0]
서로 다른 촬영 환경과 실내외 전환을 포함한 contact sheet를 검토했다.

[P03044 | 3738:3788 | NORMAL_TEXT | LIST id=kix.unlq3kwvk2yn level=0]
시각 평가는 대표 표본에 대한 검토이며 모든 원본 프레임의 정답 검증을 의미하지 않는다.

[P03045 | 3788:3803 | HEADING_1]
8. 최종 산출물과 무결성

[P03046 | 3803:3812 | HEADING_2]
고정 최고 영상

[P03047 | 3812:3814 | NORMAL_TEXT]
[best-temporal-preview.mp4](https://drive.google.com/file/d/16YrQ-1k3ByzULz0gpZho-SfkiK5xC_Qc/view?usp=drive_link)

[P03048 | 3814:3834 | NORMAL_TEXT | LIST id=kix.iybrh7i55l62 level=0]
크기: 5,836,651 bytes

[P03049 | 3834:3889 | NORMAL_TEXT | LIST id=kix.iybrh7i55l62 level=0]
비디오: MPEG-4, 960×540, 10 fps, 240 frames, 24.0 seconds

[P03050 | 3889:3909 | NORMAL_TEXT | LIST id=kix.iybrh7i55l62 level=0]
전체 FFmpeg 디코딩 오류: 0

[P03051 | 3909:3983 | NORMAL_TEXT | LIST id=kix.iybrh7i55l62 level=0]
SHA-256: f4c967bd69b64ad192bda750d25bdd80fbb8332f37d5b7f1fa3bc6a4d044ae73

[P03052 | 3983:4009 | NORMAL_TEXT | LIST id=kix.iybrh7i55l62 level=0]
평균 처리 속도: 3.5 fps = 3.5hz

[P03053 | 4009:4026 | HEADING_2]
고정 contact sheet

[P03054 | 4026:4028 | NORMAL_TEXT]
[best-contact-sheet.jpg](https://drive.google.com/file/d/1RjUEmyLPlSDmLnc2m8Cd-FtwLfMDMH7y/view?usp=drive_link)

[P03055 | 4028:4042 | NORMAL_TEXT | LIST id=kix.bqlyvbxeqzfo level=0]
크기: 1440×4848

[P03056 | 4042:4116 | NORMAL_TEXT | LIST id=kix.bqlyvbxeqzfo level=0]
SHA-256: ba5dd645fdaaad51ca38e9f685eed3daf51753bcf5a6f1e243b4a9d57f668d28

[P03057 | 4116:4125 | HEADING_2]
최종 승격 근거

[P03058 | 4125:4218 | NORMAL_TEXT | LIST id=kix.8lb62dj5zn6x level=0]
Metrics: [https://drive.google.com/file/d/1qXEnSdvwvxKhUJrxwimlcfGW-srvDSw_/view?usp=sharing](https://drive.google.com/file/d/1qXEnSdvwvxKhUJrxwimlcfGW-srvDSw_/view?usp=sharing) 

[P03059 | 4218:4229 | NORMAL_TEXT | LIST id=kix.8lb62dj5zn6x level=0]
Report: [REPORT.md](https://docs.google.com/document/d/1ssTZ03kzpwTfMdJDmx7FCLax51FsaT4lcyFxSI8AkU0/edit?usp=drive_link) 

[P03060 | 4229:4246 | NORMAL_TEXT | LIST id=kix.8lb62dj5zn6x level=0]
Contact sheet: [temporal-comparison-contact-sheet.jpg](https://drive.google.com/file/d/1GmvyO3naqQuo26-03p6e-1TApZAOG3Jx/view?usp=drive_link)

[P03061 | 4246:4255 | NORMAL_TEXT | LIST id=kix.8lb62dj5zn6x level=0]
장기 검증: [mapillary-aggregation-temporal-alpha062-hyst007-longburst30-repaired-input](https://drive.google.com/drive/folders/1-jL9ciM6iyPscqazgsx9fw1enVnkaR2_)

[P03062 | 4255:4266 | NORMAL_TEXT | LIST id=kix.8lb62dj5zn6x level=0]
세 구간 검증: [mapillary-aggregation-temporal-alpha062-hyst007-coverage3x10-repaired-input](https://drive.google.com/drive/folders/1mW3rzH49ik0IKkE60cvthJWS-ZGD_4pO)

[P03063 | 4266:4357 | NORMAL_TEXT | LIST id=kix.8lb62dj5zn6x level=0]
상태 파일: [https://drive.google.com/file/d/119eRxJS34cHrkmbb6xX-Xtt1_sgp3M_L/view?usp=sharing](https://drive.google.com/file/d/119eRxJS34cHrkmbb6xX-Xtt1_sgp3M_L/view?usp=sharing) 

[P03064 | 4357:4366 | HEADING_1]
9. 재현 방법

[P03065 | 4366:4375 | HEADING_2]
임시 입력 복구

[P03066 | 4375:4474 | NORMAL_TEXT]
다섯 H.264 입력은 .repair-input에 심볼릭 링크로 배치한다. 첫 번째 HEVC 입력은 20 fps CFR로, 나머지 두 입력은 VFR H.264로 임시 변환한다.

[P03067 | 4474:4525 | NORMAL_TEXT]
ffmpeg -y -err_detect ignore_err -i INPUT_HEVC_1 \

[P03068 | 4525:4594 | NORMAL_TEXT]
  -map 0:v:0 -an -vf 'fps=20,setpts=N/(20*TB)' -r 20 -fps_mode cfr \

[P03069 | 4594:4653 | NORMAL_TEXT]
  -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p \

[P03070 | 4653:4696 | NORMAL_TEXT]
  -movflags +faststart REPAIR_OUTPUT_1.mp4

[P03071 | 4696:4697 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P03072 | 4697:4748 | NORMAL_TEXT]
ffmpeg -y -err_detect ignore_err -i INPUT_HEVC_2 \

[P03073 | 4748:4819 | NORMAL_TEXT]
  -map 0:v:0 -an -fps_mode vfr -c:v libx264 -preset veryfast -crf 18 \

[P03074 | 4819:4879 | NORMAL_TEXT]
  -pix_fmt yuv420p -movflags +faststart REPAIR_OUTPUT_2.mp4

[P03075 | 4879:4880 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P03076 | 4880:4931 | NORMAL_TEXT]
ffmpeg -y -err_detect ignore_err -i INPUT_HEVC_3 \

[P03077 | 4931:5002 | NORMAL_TEXT]
  -map 0:v:0 -an -fps_mode vfr -c:v libx264 -preset veryfast -crf 18 \

[P03078 | 5002:5062 | NORMAL_TEXT]
  -pix_fmt yuv420p -movflags +faststart REPAIR_OUTPUT_3.mp4

[P03079 | 5062:5063 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

[P03080 | 5063:5091 | NORMAL_TEXT]
복구본은 다음 명령으로 오류가 0건인지 확인한다.

[P03081 | 5091:5149 | NORMAL_TEXT]
ffmpeg -v error -i REPAIR_OUTPUT.mp4 -map 0:v:0 -f null -

[P03082 | 5149:5158 | HEADING_2]
최종 설정 평가

[P03083 | 5158:5213 | NORMAL_TEXT]
uv run --script tools/evaluate_mapillary_temporal.py \

[P03084 | 5213:5280 | NORMAL_TEXT]
  --input-dir rosbag-results/sidewalk-road-results/.repair-input \

[P03085 | 5280:5334 | NORMAL_TEXT]
  --output-dir rosbag-results/sidewalk-road-results \

[P03086 | 5334:5358 | NORMAL_TEXT]
  --surface-aggregate \

[P03087 | 5358:5384 | NORMAL_TEXT]
  --temporal-alpha 0.62 \

[P03088 | 5384:5422 | NORMAL_TEXT]
  --temporal-hysteresis-margin 0.07 \

[P03089 | 5422:5447 | NORMAL_TEXT]
  --bursts-per-video 2 \

[P03090 | 5447:5469 | NORMAL_TEXT]
  --burst-length 15 \

[P03091 | 5469:5550 | NORMAL_TEXT]
  --experiment-id mapillary-aggregation-temporal-alpha062-hyst007-repaired-input

[P03092 | 5550:5751 | NORMAL_TEXT]
스크립트에 선언된 주요 버전은 Python 3.12 이상, NumPy 2.5.2, OpenCV headless 5.0.0.93, Pillow 12.3.0, SciPy 1.18.1, PyTorch 2.13.0, TorchVision 0.28.0, Transformers 5.16.1이다. 실행 환경에서는 uv 0.11.29와 FFmpeg 8.0.1을 사용했다.

[P03093 | 5751:5770 | HEADING_1]
10. 동영상 보존 및 정리 정책

[P03094 | 5770:5811 | NORMAL_TEXT | LIST id=kix.eczlzulxu7ye level=0]
rosbag-results/full의 원본은 삭제하거나 덮어쓰지 않았다.

[P03095 | 5811:5866 | NORMAL_TEXT | LIST id=kix.eczlzulxu7ye level=0]
최종 생성 영상은 current/best-temporal-preview.mp4 한 개만 보존했다.

[P03096 | 5866:5903 | NORMAL_TEXT | LIST id=kix.eczlzulxu7ye level=0]
후보 비교가 끝날 때마다 .staging의 후보 영상을 삭제했다.

[P03097 | 5903:5940 | NORMAL_TEXT | LIST id=kix.eczlzulxu7ye level=0]
alpha 0.61 미승격 후보 영상도 최종화 과정에서 삭제했다.

[P03098 | 5940:5989 | NORMAL_TEXT | LIST id=kix.eczlzulxu7ye level=0]
HEVC 임시 복구본 세 개와 모든 오래된 review-frame 디렉터리를 삭제했다.

[P03099 | 5989:6046 | NORMAL_TEXT | LIST id=kix.eczlzulxu7ye level=0]
재검증 근거인 metrics JSON, 작은 REPORT.md, contact sheet는 유지했다.

[P03100 | 6046:6089 | NORMAL_TEXT | LIST id=kix.eczlzulxu7ye level=0]
디스크 여유 공간은 관측만 했으며 품질 또는 중단 조건으로 사용하지 않았다.

[P03101 | 6089:6109 | HEADING_1]
11. 완벽 판정을 하지 않은 이유

[P03102 | 6109:6148 | NORMAL_TEXT | LIST id=kix.4vtdshvb5x86 level=0]
원본 영상에 픽셀 단위 Road/Sidewalk 정답 마스크가 없다.

[P03103 | 6148:6201 | NORMAL_TEXT | LIST id=kix.4vtdshvb5x86 level=0]
IoU는 정답 대비 IoU가 아니라 인접 프레임 안정성 또는 원시 예측 보존을 측정한 값이다.

[P03104 | 6201:6268 | NORMAL_TEXT | LIST id=kix.4vtdshvb5x86 level=0]
세 HEVC 원본에는 총 2,017줄의 디코딩 오류가 있어 모든 원본 프레임의 신뢰할 수 있는 복원을 보장할 수 없다.

[P03105 | 6268:6307 | NORMAL_TEXT | LIST id=kix.4vtdshvb5x86 level=0]
대표 구간 시각 검토는 수행했지만 모든 프레임의 수동 판독은 아니다.

[P03106 | 6307:6379 | NORMAL_TEXT | LIST id=kix.4vtdshvb5x86 level=0]
최종 영상은 전체 8개 원본의 완전한 길이별 납품본이 아니라, 비교에 사용한 대표 240프레임을 연결한 검증용 preview다.

[P03107 | 6379:6470 | NORMAL_TEXT]
따라서 결과는 현재 데이터와 평가 기준에서 가장 잘 검증된 best-so-far이며, ground-truth 기반 정확도나 완전한 현장 안전성을 의미하지 않는다.

[P03108 | 6470:6483 | HEADING_1]
12. 권장 후속 작업

[P03109 | 6483:6555 | NORMAL_TEXT | LIST id=kix.7xyuy3835z30 level=0]
각 촬영 조건에서 Road/Sidewalk 경계가 포함된 프레임을 층화 표본 추출하고 사람이 픽셀 단위 정답 마스크를 제작한다.

[P03110 | 6555:6653 | NORMAL_TEXT | LIST id=kix.7xyuy3835z30 level=0]
정답 기반 per-class IoU, Dice/F1, boundary F-score, confusion matrix를 계산해 temporal 안정성 지표와 분리하여 관리한다.

[P03111 | 6653:6695 | NORMAL_TEXT | LIST id=kix.7xyuy3835z30 level=0]
손상된 HEVC 세 파일은 원본 카메라 또는 ROS bag에서 재추출한다.

[P03112 | 6695:6776 | NORMAL_TEXT | LIST id=kix.7xyuy3835z30 level=0]
실제 배포가 필요하면 전체 길이 영상별 overlay와 class-index mask를 별도로 렌더링하고, 속도·메모리·지연 시간까지 검증한다.

[P03113 | 6776:6852 | NORMAL_TEXT | LIST id=kix.7xyuy3835z30 level=0]
야간, 역광, 우천, 가려짐, 실내외 전환, curb cut, 횡단보도 구간을 포함하는 실패 사례 세트를 고정 회귀 테스트로 운영한다.

[P03114 | 6852:6853 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

## 2026.08.31 검출 모델 변경 후 성능 및 품질 비교 보고서 (t.ez9z0n1dlo3x)

[P03115 | 1:65 | TITLE]
[2026.08.31] 인도·차도 Semantic Segmentation 모델 변경 후 성능 및 품질 비교 보고서

[P03116 | 65:76 | NORMAL_TEXT | LIST id=kix.nkdbnw43ggab level=0]
최종 업데이트: 8월 31, 2026

[P03117 | 76:83 | NORMAL_TEXT | LIST id=kix.nkdbnw43ggab level=0]
작성자: @MinWoo Kang <mwkang@teamgrit.kr>

[P03118 | 83:106 | NORMAL_TEXT | LIST id=kix.nkdbnw43ggab level=0]
LLM : Chat GPT-Sol 5.6

[P03119 | 106:108 | NORMAL_TEXT]
[HORIZONTAL_RULE]

[P03120 | 108:114 | HEADING_1]
1. 요약

[P03121 | 114:203 | NORMAL_TEXT | LIST id=kix.l58l4i1dqsin level=0]
MCAP 카메라 토픽의 전체 6,226프레임을 대상으로 Swin-L과 MaskFormer R50 프로파일의 Semantic Segmentation을 완료했다.

[P03122 | 203:266 | NORMAL_TEXT | LIST id=kix.l58l4i1dqsin level=0]
R50 전체 영상 처리 속도는 10.227 FPS로, Swin-L의 3.291 FPS 대비 약 3.1배 빨랐다.

[P03123 | 266:331 | NORMAL_TEXT | LIST id=kix.l58l4i1dqsin level=0]
실시간 latest-frame 벤치마크에서도 R50은 10.175 Hz를 기록해 최소 10 FPS 목표를 달성했다.

[P03124 | 331:387 | NORMAL_TEXT | LIST id=kix.l58l4i1dqsin level=0]
품질 측면에서는 R50의 인도 시간축 안정성이 개선됐지만, 차도 일관성은 Swin-L보다 낮아졌다.

[P03125 | 387:463 | NORMAL_TEXT | LIST id=kix.l58l4i1dqsin level=0]
운영 기본값은 속도 우선 R50으로 전환하고, 품질 민감·오프라인 분석에는 Swin-L을 롤백 기준선으로 유지하는 것이 현재 최선이다.

[P03126 | 463:481 | HEADING_1]
2. 대상 데이터 및 실행 환경

[P03127 | 481:577 | NORMAL_TEXT | LIST id=kix.dhbmhlepof2h level=0]
입력 MCAP: [https://drive.google.com/file/d/1bGdHiPk4IALda-X5JmTAHgPsgZJ1WCSp/view?usp=drive_link](https://drive.google.com/file/d/1bGdHiPk4IALda-X5JmTAHgPsgZJ1WCSp/view?usp=drive_link) 

[P03128 | 577:644 | NORMAL_TEXT | LIST id=kix.dhbmhlepof2h level=0]
MCAP 크기: 23,194,492,357 bytes(약 22 GB), 기록 길이: 311.3169초(약 5분 11초)

[P03129 | 644:688 | NORMAL_TEXT | LIST id=kix.dhbmhlepof2h level=0]
카메라 토픽: /a2/front_camera/res_360p/image_raw

[P03130 | 688:742 | NORMAL_TEXT | LIST id=kix.dhbmhlepof2h level=0]
메시지 형식: sensor_msgs/msg/Image, RGB8, 640×360, 약 20 Hz

[P03131 | 742:761 | NORMAL_TEXT | LIST id=kix.dhbmhlepof2h level=0]
평가 프레임 수: 6,226프레임

[P03132 | 761:843 | NORMAL_TEXT | LIST id=kix.dhbmhlepof2h level=0]
환경: Apple M4 Mac mini, 16 GB, MPS, Python 3.12, PyTorch 2.13, Transformers 5.16.1

[P03133 | 843:854 | HEADING_1]
3. 모델 프로파일

[P03134 | 854:885 | HEADING_2]
3.1 Swin-L best-so-far(품질 기준선)

[P03135 | 885:947 | NORMAL_TEXT | LIST id=kix.z20rti7fzth6 level=0]
모델: facebook/mask2former-swin-large-mapillary-vistas-semantic

[P03136 | 947:1001 | NORMAL_TEXT | LIST id=kix.z20rti7fzth6 level=0]
고정 revision: 4772b6bf101d91f2534c106dc524d906aeb3c68a

[P03137 | 1001:1036 | NORMAL_TEXT | LIST id=kix.z20rti7fzth6 level=0]
입력 384×384, 평가·스코어 맵 640×360, FP32

[P03138 | 1036:1060 | NORMAL_TEXT | LIST id=kix.z20rti7fzth6 level=0]
Temporal alpha : 	0.62 

[P03139 | 1060:1086 | NORMAL_TEXT | LIST id=kix.z20rti7fzth6 level=0]
hysteresis margin : 	0.07

[P03140 | 1086:1204 | NORMAL_TEXT | LIST id=kix.z20rti7fzth6 level=0]
Road 집계 : Road, Bike Lane, Crosswalk - Plain, Parking, Service Lane, Lane Marking - Crosswalk, Lane Marking - General

[P03141 | 1204:1253 | NORMAL_TEXT | LIST id=kix.z20rti7fzth6 level=0]
Sidewalk 집계: Sidewalk, Pedestrian Area, Curb Cut

[P03142 | 1253:1293 | HEADING_2]
3.2 MaskFormer R50 FP16 640×360(현재 기본값)

[P03143 | 1293:1333 | NORMAL_TEXT | LIST id=kix.n3af5t39np4f level=0]
모델: facebook/maskformer-resnet50-vistas

[P03144 | 1333:1387 | NORMAL_TEXT | LIST id=kix.n3af5t39np4f level=0]
고정 revision: ae4b8c2590c0a090fc32d5c217d78738a2dd4b19

[P03145 | 1387:1405 | NORMAL_TEXT | LIST id=kix.n3af5t39np4f level=0]
입력·평가 해상도 640×360

[P03146 | 1405:1444 | NORMAL_TEXT | LIST id=kix.n3af5t39np4f level=0]
MPS/CUDA에서는 FP16, CPU에서는 FP32 fallback

[P03147 | 1444:1497 | NORMAL_TEXT | LIST id=kix.n3af5t39np4f level=0]
Swin-L과 동일한 클래스 집계 및 시간축 파라미터를 사용해 모델 차이를 중심으로 비교했다.

[P03148 | 1497:1506 | HEADING_1]
4. 검증 방법

[P03149 | 1506:1573 | NORMAL_TEXT | LIST id=kix.bxfhl789lwq3 level=0]
MCAP에서 카메라 토픽을 추출하고 전체 6,226프레임에 대해 segmentation overlay 영상을 생성했다.

[P03150 | 1573:1666 | NORMAL_TEXT | LIST id=kix.bxfhl789lwq3 level=0]
전체 영상 처리 시간, 추론·후처리 시간, 클래스 면적 비율, 인접 프레임 IoU, selected-label change, hysteresis hold를 기록했다.

[P03151 | 1666:1734 | NORMAL_TEXT | LIST id=kix.bxfhl789lwq3 level=0]
실시간 latest-frame 모드로 200프레임을 평가해 처리 프레임 수와 overwrite/drop 비율을 측정했다.

[P03152 | 1734:1779 | NORMAL_TEXT | LIST id=kix.bxfhl789lwq3 level=0]
순수 처리량은 R50 100프레임 throughput 벤치마크로 별도 측정했다.

[P03153 | 1779:1848 | NORMAL_TEXT | LIST id=kix.bxfhl789lwq3 level=0]
시작부(0~99), 중간부(3000~3099), 종료부(6000~6099)를 동일 조건으로 비교하고 육안 QA를 수행했다.

[P03154 | 1848:1907 | NORMAL_TEXT | LIST id=kix.bxfhl789lwq3 level=0]
생성한 전체 영상은 프레임 수, 해상도, 재생 시간, 전체 디코딩과 SHA-256으로 무결성을 확인했다.

[P03155 | 1907:1916 | HEADING_1]
5. 정량 결과

[P03156 | 1916:1932 | HEADING_2]
5.1 전체 영상 처리 성능

[P03157 | 1932:1989 | NORMAL_TEXT | LIST id=kix.51yuu84qfcul level=0]
Swin-L: 6,226/6,226프레임, 1,892.082초(약 31분 32초), 3.291 FPS

[P03158 | 1989:2041 | NORMAL_TEXT | LIST id=kix.51yuu84qfcul level=0]
R50: 6,226/6,226프레임, 608.789초(약 10분 9초), 10.227 FPS

[P03159 | 2041:2069 | NORMAL_TEXT | LIST id=kix.51yuu84qfcul level=0]
R50 속도 향상: Swin-L 대비 약 3.1배

[P03160 | 2069:2094 | NORMAL_TEXT | LIST id=kix.51yuu84qfcul level=0]
R50 평균 추론 시간: 47.417 ms 

[P03161 | 2094:2119 | NORMAL_TEXT | LIST id=kix.51yuu84qfcul level=0]
R50 평균 후처리 시간: 40.808 ms

[P03162 | 2119:2153 | HEADING_2]
5.2 실시간 latest-frame 벤치마크(200프레임)

[P03163 | 2153:2172 | NORMAL_TEXT | LIST id=kix.s5o3or64vwhh level=0]
입력 주기: 약 19.916 Hz

[P03164 | 2172:2248 | NORMAL_TEXT | LIST id=kix.s5o3or64vwhh level=0]
Swin-L: segmentation 3.910 Hz, 처리 39프레임, drop 161프레임(80.5%), p50 252.247 ms

[P03165 | 2248:2319 | NORMAL_TEXT | LIST id=kix.s5o3or64vwhh level=0]
R50: segmentation 10.175 Hz, 처리 99프레임, drop 101프레임(50.5%), p50 98.7 ms

[P03166 | 2319:2375 | NORMAL_TEXT | LIST id=kix.s5o3or64vwhh level=0]
R50은 최소 10 FPS 목표를 달성했지만, 20 Hz 입력 전체를 실시간으로 따라잡지는 못했다.

[P03167 | 2375:2393 | HEADING_2]
5.3 대표 구간 모델 간 비교

[P03168 | 2393:2436 | NORMAL_TEXT | LIST id=kix.1mqg6otg05 level=0]
3개 구간 평균: Swin-L 3.503 FPS, R50 12.676 FPS

[P03169 | 2436:2472 | NORMAL_TEXT | LIST id=kix.1mqg6otg05 level=0]
평균 selected-label agreement: 91.42%

[P03170 | 2472:2505 | NORMAL_TEXT | LIST id=kix.1mqg6otg05 level=0]
평균 Road inter-profile IoU: 0.539

[P03171 | 2505:2542 | NORMAL_TEXT | LIST id=kix.1mqg6otg05 level=0]
평균 Sidewalk inter-profile IoU: 0.769

[P03172 | 2542:2604 | NORMAL_TEXT | LIST id=kix.1mqg6otg05 level=0]
0~99프레임: agreement 91.23%, Road IoU 0.970, Sidewalk IoU 0.753

[P03173 | 2604:2671 | NORMAL_TEXT | LIST id=kix.1mqg6otg05 level=0]
3000~3099프레임: agreement 91.85%, Road IoU 0.443, Sidewalk IoU 0.755

[P03174 | 2671:2738 | NORMAL_TEXT | LIST id=kix.1mqg6otg05 level=0]
6000~6099프레임: agreement 91.18%, Road IoU 0.205, Sidewalk IoU 0.798

[P03175 | 2738:2752 | HEADING_2]
5.4 시간축 품질 지표

[P03176 | 2752:2842 | NORMAL_TEXT | LIST id=kix.2mrp5if0apbt level=0]
Swin-L: selected change 0.02249, Road adjacent IoU 0.87199, Sidewalk adjacent IoU 0.90765

[P03177 | 2842:2929 | NORMAL_TEXT | LIST id=kix.2mrp5if0apbt level=0]
R50: selected change 0.01559, Road adjacent IoU 0.73013, Sidewalk adjacent IoU 0.93929

[P03178 | 2929:2990 | NORMAL_TEXT | LIST id=kix.2mrp5if0apbt level=0]
해석: R50은 전체 라벨 변화량과 Sidewalk 안정성이 개선됐지만, Road 시간축 일관성은 저하됐다.

[P03179 | 2990:3076 | NORMAL_TEXT | LIST id=kix.2mrp5if0apbt level=0]
평균 면적 비율: Swin-L Road 0.02879 / Sidewalk 0.29538, R50 Road 0.02593 / Sidewalk 0.28511

[P03180 | 3076:3089 | HEADING_1]
6. 시각적 품질 검증

[P03181 | 3089:3144 | NORMAL_TEXT | LIST id=kix.knm5ozvof44s level=0]
시작 구간에서 R50은 파란 펜스와 영상 가장자리의 과도한 segmentation을 일부 줄였다.

[P03182 | 3144:3195 | NORMAL_TEXT | LIST id=kix.knm5ozvof44s level=0]
중간 구간에서는 어두운 보행면 일부를 R50이 놓치고 Road로 분류하는 오류가 관찰됐다.

[P03183 | 3195:3256 | NORMAL_TEXT | LIST id=kix.knm5ozvof44s level=0]
종료 구간의 놀이터·녹색 인조잔디에서는 두 모델의 예측 차이가 크게 나타났으며, 어느 쪽도 완벽하지 않았다.

[P03184 | 3256:3324 | NORMAL_TEXT | LIST id=kix.knm5ozvof44s level=0]
따라서 R50은 속도 목표를 충족하지만 segmentation 품질이 Swin-L을 일관되게 능가한다고 판단할 수 없다.

[P03185 | 3324:3338 | HEADING_1]
7. 결론 및 운영 권고

[P03186 | 3338:3373 | NORMAL_TEXT | LIST id=kix.rxv3u634aks1 level=0]
속도 우선 운영 기본 프로파일: r50-fp16-640x360

[P03187 | 3373:3415 | NORMAL_TEXT | LIST id=kix.rxv3u634aks1 level=0]
품질 민감 또는 오프라인 재처리 기준선: swin-l-best-so-far

[P03188 | 3415:3454 | NORMAL_TEXT | LIST id=kix.rxv3u634aks1 level=0]
R50은 10 FPS 요구를 만족하므로 실시간 적용 후보로 적합하다.

[P03189 | 3454:3543 | NORMAL_TEXT | LIST id=kix.rxv3u634aks1 level=0]
Road 품질 저하가 확인된 중간·종료 구간을 중심으로 클래스 집계, temporal smoothing, hysteresis 및 후처리를 추가 조정해야 한다.

[P03190 | 3543:3645 | NORMAL_TEXT | LIST id=kix.rxv3u634aks1 level=0]
현재 평가는 모델 간 일치도와 시간축 안정성에 기반한다. 최종 정확도 판정을 위해서는 픽셀 단위 정답 라벨로 mIoU와 클래스별 Precision/Recall을 추가 측정해야 한다.

[P03191 | 3645:3718 | NORMAL_TEXT | LIST id=kix.rxv3u634aks1 level=0]
‘완벽한 segmentation’으로 종결할 단계는 아니며, 현재 판정은 속도 목표를 충족한 best-so-far 운영 설정이다.

[P03192 | 3718:3733 | HEADING_1]
8. 주요 산출물 절대경로

[P03193 | 3733:3746 | NORMAL_TEXT | LIST id=kix.poaz9oz0iilh level=0]
원본 추출 영상: [20260827_062352_teamgrit_rosbag_0-camera-full.mp4](https://drive.google.com/file/d/16KNWBFoWxWsn1Bkrp8znQMXiD9Vsa1Hh/view?usp=sharing) 

[P03194 | 3746:3767 | NORMAL_TEXT | LIST id=kix.poaz9oz0iilh level=0]
Swin-L 전체 overlay: [20260827_062352_teamgrit_rosbag_0-camera-full-segmented-full.mp4](https://drive.google.com/file/d/1Hgq-OhyYHf3THVbe_wHX4NffRh6Ix11a/view?usp=drive_link)

[P03195 | 3767:3870 | NORMAL_TEXT | LIST id=kix.poaz9oz0iilh level=0]
Swin-L 결과 JSON: [https://drive.google.com/file/d/1eg_9ILpfE2EYwiui1J1WMPgT7WSqlVzl/view?usp=drive_link](https://drive.google.com/file/d/1eg_9ILpfE2EYwiui1J1WMPgT7WSqlVzl/view?usp=drive_link) 

[P03196 | 3870:3888 | NORMAL_TEXT | LIST id=kix.poaz9oz0iilh level=0]
R50 전체 overlay: [20260827_062352_teamgrit_rosbag_0-camera-full-r50-fp16-640x360-segmented-full.mp4](https://drive.google.com/file/d/1ukTapW_2ahRBIPMtoxOr2hGGPoOuLYLk/view?usp=drive_link)

[P03197 | 3888:3988 | NORMAL_TEXT | LIST id=kix.poaz9oz0iilh level=0]
R50 결과 JSON: [https://drive.google.com/file/d/1_UbM_PlPAXq-EntoomWcmr_dojiappLE/view?usp=drive_link](https://drive.google.com/file/d/1_UbM_PlPAXq-EntoomWcmr_dojiappLE/view?usp=drive_link) 

[P03198 | 3988:3999 | NORMAL_TEXT | LIST id=kix.poaz9oz0iilh level=0]
전체 비교 영상: 

[P03199 | 3999:4003 | NORMAL_TEXT | LIST id=kix.poaz9oz0iilh level=1]
[20260827_062352_teamgrit_rosbag_0-camera-full-frames-0-99-comparison.mp4](https://drive.google.com/file/d/12I6q0P41NHlxQ_iFi-q9QMeQ-aqG6NqA/view?usp=drive_link), 

[P03200 | 4003:4005 | NORMAL_TEXT | LIST id=kix.poaz9oz0iilh level=1]
[20260827_062352_teamgrit_rosbag_0-camera-full-frames-3000-3099-comparison.mp4](https://drive.google.com/file/d/1FKuuRHZ6szHnqMT6Z70I37d3exQpgbZ5/view?usp=drive_link)

[P03201 | 4005:4007 | NORMAL_TEXT | LIST id=kix.poaz9oz0iilh level=1]
[20260827_062352_teamgrit_rosbag_0-camera-full-frames-6000-6099-comparison.mp4](https://drive.google.com/file/d/1p7_IjSzrTK0sBayK8POJSEQOxw5-B7qH/view?usp=drive_link)

[P03202 | 4007:4009 | NORMAL_TEXT | LIST id=kix.poaz9oz0iilh level=1]
[20260827_062352_teamgrit_rosbag_0-full-swin-l-vs-r50.mp4](https://drive.google.com/file/d/1uJoPyOpvK3rt4EtLlFLMmEUAlxE5Kx4S/view?usp=drive_link)

[P03203 | 4009:4026 | NORMAL_TEXT | LIST id=kix.poaz9oz0iilh level=0]
대표 구간 비교 JSON: [profile-comparison](https://drive.google.com/drive/u/0/folders/1tgFsY___nbouVLLTWmh07EbBCxVtwF0n)

[P03204 | 4026:4127 | NORMAL_TEXT | LIST id=kix.poaz9oz0iilh level=0]
R50 실시간 벤치마크: [https://drive.google.com/file/d/1dThSv8RnfPz7q8iu26i0JTQsMLADYD8P/view?usp=drive_link](https://drive.google.com/file/d/1dThSv8RnfPz7q8iu26i0JTQsMLADYD8P/view?usp=drive_link) 

[P03205 | 4127:4228 | NORMAL_TEXT | LIST id=kix.poaz9oz0iilh level=0]
R50 처리량 벤치마크: [https://drive.google.com/file/d/1_fY0TZ-ubADtbd5EC_ZDQ2c4_JWnvFj5/view?usp=drive_link](https://drive.google.com/file/d/1_fY0TZ-ubADtbd5EC_ZDQ2c4_JWnvFj5/view?usp=drive_link) 

[P03206 | 4228:4229 | NORMAL_TEXT]
⟦EMPTY PARAGRAPH⟧

