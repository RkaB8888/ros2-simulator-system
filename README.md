# ROS 2 Simulator System (Humble, C++ / WSL)

교육용 시뮬레이터(MORAI SIM) 기반 환경을 **ROS 2 Humble + C++ + WSL(리눅스)** 로 재구현한 프로젝트입니다.  
Windows/Python(Eloquent) 시절의 레거시를 참고하되, **UDP I/O ↔ 파서 ↔ 브릿지 ↔ 세이프티 체인**을 현대화하고
프레임·타임스탬프·QoS를 정돈하여 **재현 가능한 개발 베이스**를 제공합니다.

- 착수: 2025‑09‑27
- 최신 마일스톤: **M1.5b 센서 기반 안전 제동(Stop/Slowdown) 검증 완료** — 2025‑11‑01

> 🔗 레거시 참고: WellDone (Windows/Eloquent)  
> https://github.com/RkaB8888/SSAFY-Specialized-PJT-WellDone.git

---

## 1) 목표
- Python(Eloquent, Windows) → **C++(Humble, Linux/WSL)** 전환
- **중앙집중 YAML**로 네트워크/IP/포트 및 센서 설정 일원화
- **Safety Chain 표준화**(twist_mux → velocity_smoother → collision_monitor)
- 프레임/타임스탬프 정합으로 RViz/Nav2 경고 제거
- 이후 SLAM/경로추종/FSM을 위한 기반 확보

---

## 2) 패키지 구조
- **`bridge_bringup`**: 통합 런치 & 공용 설정(YAML). 모든 IP/포트 및 세이프티 토글 관리.
- **`udp_raw_bridge`**: 시뮬레이터 → ROS2 수신(RX). 각 UDP 포트를 바인딩해 raw 바이트를 퍼블리시.
- **`udp_parsers_cpp`**: RAW 토픽을 ROS 메시지로 파싱(ego/env/iot/object/imu/lidar/camera).
- **`udp_tx_bridge`**: ROS 토픽(`/cmd_vel`, `/hand_control`, `/iot_control`)을 **레거시와 동일한 UDP 프레이밍**으로 송신(TX).
- **`safety_bringup`**: `twist_mux` → `nav2_velocity_smoother` → `nav2_collision_monitor` 체인 구성
- **`sensor_bringup`** *(신규)*:
  - 센서 공통 설정 `config/sensors.yaml`
  - `scan_normalizer` 노드로 LiDAR 배열/프레임 보정
  - 프레임 일관화: **LiDAR=`laser_link`, IMU=`imu_link`, Camera=`camera_link`**
- **`state_estimator`**: `/ego_status` → `/odom` + `TF(odom→base_link)` 변환
- **`bridge_msgs`**: 공용 메시지 정의

---

## 3) 현재 상태 (2025‑11‑01)
- ✅ **M1.5b 완료: LiDAR → Collision Monitor 연동**
  - `/scan` 기반 전방/후방 폴리곤 감시
  - `action_type: slowdown/stop` 동작 검증, `slowdown_ratio` 반영 확인
  - **타임스탬프 정합**으로 “미래 외삽” 경고 해소
- ✅ **프레임 정리**
  - LiDAR=`laser_link`, IMU=`imu_link`, Camera=`camera_link`
  - (필요 시) `static_transform_publisher`로 센서→`base_link` 정합
- ✅ **LiDAR 정규화**
  - 시뮬 특성 보상: **180° 회전 옵션** 지원(`rotate_180_degrees: true`), **좌우 반전 비활성화**(`invert_cw_to_ccw: false`)
  - `scan_time/time_increment = 0` 정책(시뮬 단일 스탬프 프레임)
- 🔜 **다음 단계 (M2.0)**: `/ego_status` + `/imu` EKF 융합(odometry 고도화)

---

## 4) 빠른 시작
> 상세 설치/네트워크/검증 절차는 **`docs/setup_manual_2025-10-31.md`** 참조.

```bash
# 0) 빌드 & 환경설정
colcon build
source install/setup.bash

# 1) 통합 브릿지 실행
ros2 launch bridge_bringup bridge.launch.py log_level_raw:=info

# 2) 센서 브링업 (LiDAR 정규화 포함)
ros2 launch sensor_bringup sensor_bringup.launch.py log_level:=info

# 3) odom 발행
ros2 launch state_estimator odom.launch.py

# 4) 세이프티 체인
ros2 launch safety_bringup safety_chain.launch.py log_level:=info
# (필요 시) 라이프사이클 노드 활성화
ros2 lifecycle set /velocity_smoother configure
ros2 lifecycle set /velocity_smoother activate
ros2 lifecycle set /collision_monitor configure
ros2 lifecycle set /collision_monitor activate

# 5) 주행 명령 예시(teleop/nav)
ros2 topic pub /cmd_vel_nav geometry_msgs/Twist '{linear: {x: 0.2}}' -r 10
```

**네임스페이스(NS)**: `ns:=robot1` 인자를 런치에 넘기면 모든 토픽/파라미터가 NS 하위로 정렬됩니다.  
YAML에서는 **절대 경로 대신 상대 토픽**을 사용해 NS 호환을 유지합니다(예: `scan` ⭕, `/scan` ❌).

---

## 5) 센서 설정 요약 (`sensor_bringup/config/sensors.yaml`)
- **LiDAR**
  - `frame_id: laser_link`
  - `topics: {in: scan_raw, out: scan}`
  - `normalize:`
    - `invert_cw_to_ccw: false`  (시뮬 데이터가 이미 CCW)
    - `rotate_180_degrees: true` (시뮬 배열 기준 보상)
- **IMU**: `frame_id: imu_link`, `topics: {pub_out: imu}`
- **Camera**: `frame_id: camera_link`, `topics: {pub_out: image_raw}`  
  - (필요 시) `camera_optical_frame` 명시, URDF/TF 정합 권장

---

## 6) Safety Chain 설정 개요 (`safety_bringup/config`)
- **twist_mux.yaml**
  - 입력: `cmd_vel_nav`(prio 1), `cmd_vel_teleop`(prio 2), `cmd_vel_emergency`(prio 3)
  - 출력: `cmd_vel_mux`
- **velocity_smoother.yaml**
  - 입력: `cmd_vel_mux` → 출력: `cmd_vel_smooth`
  - `OPEN_LOOP`, 가감속/속도 제한, timeout 등
- **collision_monitor.yaml**
  - 프레임: `base_frame_id=base_link`, `odom_frame_id=odom`
  - 관측원: `observation_sources: [lidar_scan]`
    - `lidar_scan: {type: scan, topic: scan, enabled: true}`
  - 폴리곤: `polygons: [slow_zone_front, stop_zone_front, stop_zone_back]`
    - `action_type: slowdown | stop`
  - 전역 감속비: `slowdown_ratio: 0.5`  
  - **주의**: Nav2 Humble에는 `use_slowdown`, `use_stop` 파라미터가 **존재하지 않습니다.**  
    감속/정지는 **폴리곤의 `action_type`** 으로 결정됩니다.

---

## 7) 프레임 & 타임스탬프 정책
- **헤더 타임스탬프**
  - 파서 단계에서 수신 시각으로 `header.stamp` 설정
  - 정규화 노드(예: `scan_normalizer`)는 **입력 스탬프를 그대로 전달**
- **스캔 타이밍 필드**
  - 시뮬 특성상 포인트별 지연이 없어 `scan_time`, `time_increment`는 **0**
- **TF**
  - 필수: `odom → base_link`
  - 센서 프레임(`laser_link`/`imu_link`/`camera_link`) ↔ `base_link`는 고정 변환 사용 권장

---

## 8) 검증 방법
- **rqt_plot**
  - `/cmd_vel_nav/linear/x` → `/cmd_vel_smooth/linear/x` → `/cmd_vel/linear/x` 램프/계단 확인
- **RViz**
  - Fixed Frame=`odom`
  - LiDAR: `/scan` 표시, `polygon_*` 토픽 추가(Transient Local)로 감시영역 확인
- **토픽/스탬프 점검**
  ```bash
  ros2 topic echo -n 1 /scan header
  ros2 topic echo -n 1 /imu   header
  ros2 run tf2_tools view_frames  # (그래프 확인)
  ```

---

## 9) 로드맵
- [x] **M0. 브릿지 안정화**
- [x] **M1. 오도메트리(ego→odom)** + TF(odom→base_link)
- [x] **M1.5a. Safety 체인 구축**
- [x] **M1.5b. LiDAR 입력 기반 slowdown/stop 검증**
- [ ] **M2.0. EKF** (ego_status + IMU 융합, `robot_localization`)
- [ ] **M2.5. Lifecycle 자동화** (configure/activate 자동)
- [ ] **M3. TF/URDF 정합 강화**
- [ ] **M4. 맵핑(SLAM) — slam_toolbox**
- [ ] **M5. Nav2 자율주행**
- [ ] **M6. 커스텀 Path Tracker 플러그인**
- [ ] **M7. FSM/BT 하이브리드 제어**
- [ ] **M8. 물체 제어(Pick & Place)**
- [ ] **M9. 시나리오 회귀 테스트**

---

## License
MIT

## 출처
- WellDone Simulator Repository (레거시)
- Nav2 / ROS 2 Humble 문서
