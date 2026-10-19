# Jetson의 대용량 영상 수신

2026-10-19 당진 A2 검사에서 `/a2/front_camera/res_360p/image_raw`의 입력에 약
20초 공백이 있었고, `/proc/net/sockstat`의 FRAG memory가 4,196,264 bytes로
`ipfrag_high_thresh=4,194,304` 한도에 찬 상태였다. UDP socket 수신 오류는 0이었다.
이것은 [ROS 2 Humble의 DDS 튜닝 문서](https://github.com/ros2/ros2_documentation/blob/humble/source/How-To-Guides/DDS-tuning.rst)
가 설명하는 조각 재조립 버퍼 포화 증상과 일치한다.

이 장비에는 아래 커널 설정을 적용했고 `/etc/sysctl.d/90-cobiz-dds-fragments.conf`에
저장했다. 호스트 전체의 수신 IP 조각에 적용되며 컨테이너의 모델/ROS 설정과는 별도다.
128MiB는 조각 저장의 상한이고 즉시 그 양을 할당하는 설정은 아니다.

```ini
net.ipv4.ipfrag_high_thresh=134217728
net.ipv4.ipfrag_time=3
```

적용 후 60초 검사에서 영상 1,092개를 받았고 최대 수신 공백은 1.50초였다.
이전의 약 20초 공백은 줄었지만 모든 초에 새 영상이 온다는 보장은 아니다.
영상/센서 source stamp를 바꾸거나 기존 만료 한계를 늘리지 않는다. 입력 공백 때에는
추론/제어를 계속 가능하다고 표시하지 않고 기존 freshness 검사로 정지한다.

원래 값과 파일 존재 여부는 장비의
`/home/teamgrit/diagnostics/line-tracking-drive-optimization-20261019/ipfrag-before.json`,
`ipfrag-config-before.json`에 저장했다. 이 장비의 원래 값으로 되돌릴 때에는
관리자 권한으로 새 설정 파일을 제거하고 아래 두 값을 복원한다.

```bash
sudo rm /etc/sysctl.d/90-cobiz-dds-fragments.conf
sudo sysctl -w net.ipv4.ipfrag_time=30 net.ipv4.ipfrag_high_thresh=4194304
```

네트워크 설정 변경만으로 현장 주행 성공이나 1Hz의 무조건적 달성을 주장하지 않는다.
정상 영상이 들어오는 구간의 새 추론 완료 간격과 영상 입력 공백을 별도로 측정한다.
