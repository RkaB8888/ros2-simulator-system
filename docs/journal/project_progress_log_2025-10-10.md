# 프로젝트 작업 일지 — 2025-10-10

## 1) 개요
- **목표**: 시뮬레이터 → ROS2 방향의 모든 수신 파이프라인을 정리하고(환경/자세/IoT/커스텀오브젝트/IMU/LiDAR/Camera), 파서들을 한 번에 띄우는 런치를 구성. 카메라 JPEG 파서 안정화.
- **범위**: `udp_raw_bridge`, `udp_parsers_cpp`, `bridge_msgs` 정리/추가, 네트워킹/QoS/프레이밍 트러블슈팅.

---

## 2) 오늘 변경/추가된 산출물

### A. udp_raw_bridge
- **`src/udp_raw_bridge/src/udp_raw_node.cpp`**
  - 커널 수신 버퍼 상향: `receive_buffer_size = 4MB`.
  - 사용자 버퍼 상향: 단일 datagram 수신 버퍼 `128KB`.
  - QoS: **SensorDataQoS(BestEffort, depth 10)** 로 RAW 토픽 퍼블리시.
- **런치**: `launch/raw_6ports.launch.py`
  - 포트 → 토픽 매핑:
    - 7802 → `/env_info_raw`
    - 8002 → `/iot_status_raw`
    - 8202 → `/ego_status_raw`
    - 8302 → `/object_info_raw`
    - 9092 → `/imu_raw`
    - 9094 → `/lidar_raw`
    - 1232 → `/camera_jpeg_raw`
  - **respawn = false** (UDP 바인딩 충돌 방지 목적).

### B. udp_parsers_cpp
- **파서 노드 추가/정리**
  - `env_status_parser` → `/env_info_raw` → `/env_status`
  - `ego_status_parser` → `/ego_status_raw` → `/ego_status`
  - `iot_status_parser` → `/iot_status_raw` → `/iot_status` (IotStatus.msg 사용)
  - `custom_object_parser` → `/object_info_raw` → `/custom_object_info`
  - `imu_parser` → `/imu_raw` → `/imu`
    - 포맷: 더블 10개(LE) `qw,qx,qy,qz, wx,wy,wz, ax,ay,az`
    - 단위: 각속도(rad/s), 가속도(m/s²) 가정 (기존 브릿지와 동일 매핑)
  - `lidar_parser` → `/lidar_raw` → `/scan` (LaserScan)
    - 고정 파라미터로 angle_min/max/increment, range_min/max, time_increment 설정
  - `camera_jpeg_parser` → `/camera_jpeg_raw` → `/image_jpeg/compressed`
    - **프레이밍 1순위**: `MOR\0 + 4(reserved) + 4(lenLE) + JPEG`
    - **백업 스캐너**: JPEG **SOI/EOI(FF D8 .. FF D9)** 마커 스캔
    - 퍼블리시: `sensor_msgs/CompressedImage(format="jpeg")`
- **런치**: `launch/parsers_all.launch.py`
  - 7개 파서를 한 번에 실행
  - **respawn = true** (예외 시 자동 재기동)

### C. bridge_msgs
- **메시지 빌드 파이프라인 정리**
  - `TurtlebotStatus.msg`
  - `EnviromentStatus.msg`
  - `IotStatus.msg` (이전 AppStatus → IotStatus로 개명)
  - `CustomObjectInfo.msg` (Point 배열)

---

## 3) 네트워크/토픽/이름 체계 정리

- **토픽 의미 고정**
  - `/iot_status_raw`: 시뮬레이터 내 IoT(App) 상태 프레임
  - `/object_info_raw`: 사용자 생성 Unity 오브젝트 위치 프레임
- **포트/라우팅**
  - 시뮬레이터 Host/Destination IP가 **WSL IP/포워딩 체계**와 일치하도록 설정
  - Windows 방화벽 인바운드 규칙(포트별 UDP) 추가
- **QoS 정책**
  - RAW: **BestEffort**(SensorDataQoS) — 높은 주기/낙하 허용
  - 파싱 결과: **Reliable(depth=10)** — 시각화/하위 모듈 소비 안정화

---

## 4) 주요 이슈 & 해결 타임라인

1) **UDP 수신 불명확**
- 현상: RAW 수신이 간헐/무반응
- 조치:
  - WSL: `tcpdump`로 포트 수신 확인
  - Win: `netstat -ano -p udp` 확인 + 방화벽 인바운드 규칙 추가
  - 시뮬레이터 센서 **Disconnect/Connect** 시 경고 발생 → **시뮬 전체 재시작** 시 정상화 재현
- 결과: 7802/8002/8202/8302/9092/9094/1232 수신 확인

2) **rqt 경고(RELIBILITY_QOS_POLICY)**
- 현상: RAW 구독 시 QoS 불일치 경고
- 원인: RAW(=BestEffort) vs rqt 기본(Reliable) 상이
- 조치: 파서 토픽은 Reliable로 퍼블리시, 시각화 문제 해소

3) **IMU 파싱 단위/오더 검증**
- 포맷: 헤더/길이 뒤 10개 `double`(LE)
- 매핑: `orientation(qw..qz)`, `angular_velocity(w)`, `linear_acceleration(a)`
- 고정 규격으로 코드 내 반영(파라미터 의존 제거)
- 결과: `/imu` 정상 갱신(오도메 연동 기대치 동일)

4) **LiDAR 파싱**
- 프레임을 `LaserScan`으로 매핑
- angle/range 파라미터 고정(현 시뮬 설정 기준)
- 결과: `/scan` 정상 갱신, rqt Hz OK

5) **카메라 JPEG 파싱(핵심)**
- 현상: 프레임 미출력/정지처럼 보임
- 원인/파악:
  - 시뮬 재시작 후에만 송출되는 Unity 특성 재현
  - rqt로 바이트 자체를 보면 일부가 고정처럼 보여 혼동 → 이미지 플러그인으로 확인 필요
- 조치:
  - 파서에 **MOR 헤더+길이** 우선, **SOI/EOI** 백업 스캐너 구현
  - `rqt_image_view` 및 `image_transport_plugins` 설치로 `/image_jpeg/compressed` 시각화
- 결과: 실시간 이미지 표출 정상

6) **런치 전략**
- RAW 런치: **respawn=false** (UDP 포트 즉시 재바인딩시 충돌 방지)
- 파서 런치: **respawn=true** (예외 시 자동 재시작)

---

## 5) 검증 커맨드 스냅샷

```bash
# 빌드/셋업
cd ~/ros2_ws
colcon build --cmake-clean-cache
source install/setup.bash

# RAW 수신 런치
ros2 launch udp_raw_bridge raw_6ports.launch.py

# 파서 통합 런치
ros2 launch udp_parsers_cpp parsers_all.launch.py
```

```bash
# 네트워크 수신 확인 (WSL)
sudo tcpdump -i eth0 udp port 1232 -n -vv -c 5

# RAW hz/샘플
ros2 topic hz /camera_jpeg_raw
ros2 topic echo --qos-reliability best_effort /camera_jpeg_raw --once
```

```bash
# 파싱 결과 확인
ros2 topic echo /imu
ros2 topic echo /scan
ros2 run rqt_image_view rqt_image_view  # /image_jpeg/compressed 선택
```

---

## 6) 커밋 묶음 (권장 플랜)

1. **RAW 확장/런치**
   - `udp_raw_bridge`: `udp_raw_node.cpp` 버퍼 확장
   - `udp_raw_bridge/launch/raw_6ports.launch.py` 추가/정리
   - **메시지(한국어)**: `udp_raw_bridge: UDP 수신 버퍼 확장 및 6포트 RAW 런치 추가`

2. **IMU/LiDAR 파서**
   - `udp_parsers_cpp`: `imu_parser.cpp`, `lidar_parser.cpp` 추가 + CMakeLists 등록
   - **메시지**: `parsers: IMU/LiDAR 파서 추가 (/imu, /scan)`

3. **카메라 파서**
   - `udp_parsers_cpp`: `camera_jpeg_parser.cpp` 추가(프레이밍+백업 스캐너)
   - **메시지**: `parsers: Camera JPEG 파서 추가 (MOR+LEN / SOI-EOI 스캐너)`

4. **파서 통합 런치**
   - `udp_parsers_cpp/launch/parsers_all.launch.py` (respawn=true)
   - **메시지**: `launch: 파서 일괄 실행 런치(parsers_all) 추가`

---

## 7) 다음 단계(ROS2 → 시뮬 송신 브릿지)
- 신규 패키지 제안: **`udp_tx_bridge`**
  - 대상 명령: **Ego Ctrl Cmd(760x)**, **Object Status Control(790x)**, **Stuff Object Control(810x)**
  - 토픽→UDP 직렬화: 기존 `sub_to_udp.py` 프로토콜(헤더/엔디언/길이) 재현
  - QoS:
    - `/cmd_vel` 계열: **BestEffort + keep_last(1)**
    - 상태변경/질의형: Reliable 고려
  - 런치: `cmd_all.launch.py` 로 송신 3종 일괄 실행

---

## 8) 메모/결정 사항
- **이름 체계 합일**: App → **IoT** 로 명확화 (`IotStatus.msg`, `/iot_status`)
- **프레이밍 파악 중요**: 카메라처럼 독자 프레이밍 존재 가능 → 길이+시그니처, 최소 SOI/EOI 백업 스캐너 확보
- **시각화 툴 필수**: `rqt_image_view`, `image_transport_plugins` 설치

---
