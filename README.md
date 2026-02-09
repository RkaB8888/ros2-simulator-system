# ROS 2 Simulator System (Humble, C++ / WSL)

교육용 시뮬레이터(MORAI SIM) 기반 환경을 **ROS 2 Humble + C++ + WSL(리눅스)** 로 재구현한 프로젝트입니다.  
Windows/Python(Eloquent) 시절의 레거시를 참고하되, **UDP I/O ↔ 파서 ↔ 브릿지 ↔ 세이프티 체인 ↔ EKF 센서 퓨전**을 현대화하고
안정적인 실시간 통신을 위한 **방어 로직(Anti-Lag)**을 탑재하여 **재현 가능한 개발 베이스**를 제공합니다.

- 착수: 2025‑09‑27
- 최신 마일스톤: **M3. TF 정합 및 시스템 안정화 완료** — 2026‑02‑09

> 🔗 레거시 참고: WellDone (Windows/Eloquent)  
> https://github.com/RkaB8888/SSAFY-Specialized-PJT-WellDone.git

---

## 1) 목표
- Python(Eloquent, Windows) → **C++(Humble, Linux/WSL)** 전환
- **중앙집중 YAML**로 네트워크/IP/포트 및 센서 설정 일원화
- **Safety Chain 표준화** (twist_mux → velocity_smoother → collision_monitor)
- **정밀 위치 추정** (Wheel Odom + IMU → EKF Fusion)
- **통신 안정화** (Windows Host Load로 인한 패킷 지연/버스트 방어)

---

## 2) 패키지 구조
- **`bridge_bringup`**: 통합 런치 & 공용 설정(YAML). EKF 및 Lifecycle Manager 포함 전체 실행.
- **`udp_raw_bridge`**: 시뮬레이터 → ROS2 수신(RX). 각 UDP 포트를 바인딩해 raw 바이트를 퍼블리시.
- **`udp_parsers_cpp`**: RAW 토픽을 ROS 메시지로 파싱.
  - `imu_parser`: IMU 버스트 방어 로직 적용.
- **`udp_tx_bridge`**: ROS 토픽(`/cmd_vel` 등)을 UDP로 송신(TX).
- **`safety_bringup`**: `twist_mux` → `nav2_velocity_smoother` → `nav2_collision_monitor` 체인 구성.
- **`sensor_bringup`**: 센서 공통 설정 및 `scan_normalizer`(LiDAR 보정) 실행.
- **`state_estimator`**: 
  - `odom_publisher`: `/ego_status` → `/wheel/odom` 변환 (지연 방어 로직 포함).
  - **`robot_localization` (EKF)**: `/wheel/odom` + `/imu` → `/odom` 융합 및 TF 발행.
- **`bridge_msgs`**: 공용 메시지 정의.

---

## 3) 현재 상태 (2026‑02‑09)
- ✅ **M2.0 완료: EKF 센서 퓨전**
  - **EKF 적용**: `robot_localization` 패키지를 통해 Wheel Odom과 IMU 데이터 융합.
  - **공분산 튜닝**: 센서 특성에 맞춘 Covariance(`0.02`~`0.05`) 설정.
  - **Calibration**: Wheel Odom 회전 오차 보정 (오차율 6% → **0.15%**).
- ✅ **M2.5 완료: Lifecycle 자동화**
  - `nav2_lifecycle_manager`를 도입하여 `velocity_smoother`, `collision_monitor` 등 Nav2 노드를 **런치 파일 실행 시 자동으로 활성화(Configure -> Activate)** 하도록 개선.
  - 수동 명령(`ros2 lifecycle set ...`) 불필요.
- ✅ **M3 완료: TF 트리 정합**
  - EKF가 `odom` → `base_link` TF를 전담.
  - 센서 프레임(`laser`, `imu`)과 `base_link` 간의 관계 정립.
- ✅ **Host Load Latency 해결 (Troubleshooting)**
  - **원인**: Windows 호스트 부하 시 Hyper-V/WSL2 가상 네트워크 경로의 패킷 전달 지연(Stall).
  - **대응**: 
    - **Anti-Lag**: Odom 적분 시 `dt > 0.1s` 데이터 스킵 (텔레포트 방지).
    - **Anti-Burst**: IMU `dt < 2ms` 데이터 드랍 (EKF 발산 방지).

---

## 4) 빠른 시작
> 상세 설치/네트워크/검증 절차는 **`docs/setup_manual_2026-02-09.md`** 참조.

```bash
# 0) 빌드 & 환경설정
colcon build --symlink-install
source install/setup.bash

# 1) 통합 브릿지 실행 (EKF, Safety, Lifecycle 자동화 포함)
ros2 launch bridge_bringup bridge.launch.py log_level_raw:=info

# [팁] 충돌 방지 없이 실행하려면:
# ros2 launch bridge_bringup bridge.launch.py enable_safety:=false
```

**실행 확인:**
- **TF 트리**: `map` -> `odom` -> `base_link` -> 센서 프레임 연결 확인.
- **EKF 동작**: `ros2 topic echo /odometry/filtered` (또는 `/odom`).
- **Safety**: 장애물 접근 시 로봇 감속/정지 확인.

---

## 5) 시스템 아키텍처
```text
[Simulator] 
    │ (UDP)
    ▼
[udp_raw_bridge] 
    │ (Topic: *_raw)
    ▼
[udp_parsers_cpp] 
    │ (Topic: scan_raw, ego_status, imu_raw)
    │ (* Anti-Burst Logic applied to IMU)
    ▼
[sensor_bringup] / [state_estimator]
    │ (Wheel Odom: Anti-Lag Logic applied)
    │ (Fusion: Wheel Odom + IMU -> EKF)
    │ (Topic: scan, odom)
    ▼
[safety_bringup] (TwistMux -> Smoother -> CollisionMonitor)
    │ (Lifecycle Managed: Auto-Active)
    │ (Topic: cmd_vel)
    ▼
[udp_tx_bridge] 
    │ (UDP)
    ▼
[Simulator]
```

---

## 6) 주요 이슈 해결 (Troubleshooting Log)
**Q. 시뮬레이터가 렉 걸리면 로봇이 순간이동해요.** **A.** Windows 호스트 부하로 인해 WSL2로 들어오는 패킷이 밀렸다가 한꺼번에 들어오는 현상입니다.  
`odom_publisher`에 **`dt > 0.1s` 스킵 로직**을 적용하여, 지연된 데이터로 인한 위치 튀는 현상을 방어했습니다.

**Q. 회전이 실제보다 덜/더 돌아요.** **A.** 시뮬레이터의 각속도 단위가 표준(rad/s)과 달라서 발생한 문제입니다.  
10회전 실험을 통해 보정 계수(`angular_scale: -2.098`)를 산출하여 적용했습니다.

---

## 7) 로드맵
- [x] **M0. 브릿지 안정화 (C++ Porting)**
- [x] **M1. 오도메트리 기본 구현**
- [x] **M1.5. Safety 체인 구축 (Nav2 Stack)**
- [x] **M2.0. EKF 센서 퓨전 & 시스템 안정화 (Anti-Lag)**
- [x] **M2.5. Lifecycle 자동화 (Launch 통합)**
- [x] **M3. TF/URDF 정합 (EKF 기반)**
- [ ] **M4. 맵핑(SLAM) — Cartographer / SLAM Toolbox**
- [ ] **M5. Nav2 자율주행 (Path Planning/Following)**
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