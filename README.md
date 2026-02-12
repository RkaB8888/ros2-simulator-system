# ROS 2 Simulator System (Humble, C++ / WSL)

교육용 시뮬레이터(MORAI SIM) 기반 환경을 **ROS 2 Humble + C++ + WSL(리눅스)** 로 재구현한 프로젝트입니다.  
Windows/Python(Eloquent) 시절의 레거시를 참고하되, **UDP I/O ↔ 파서 ↔ 브릿지 ↔ 세이프티 체인 ↔ EKF 센서 퓨전 ↔ SLAM**을 현대화하고
안정적인 실시간 통신을 위한 **방어 로직(Anti-Lag)**을 탑재하여 **재현 가능한 개발 베이스**를 제공합니다.

- 착수: 2025‑09‑27
- 최신 마일스톤: **M4. 맵핑(SLAM) 및 아키텍처 고도화 완료** — 2026‑02‑13

> 🔗 레거시 참고: WellDone (Windows/Eloquent)  
> https://github.com/RkaB8888/SSAFY-Specialized-PJT-WellDone.git

---

## 1) 목표
- Python(Eloquent, Windows) → **C++(Humble, Linux/WSL)** 전환
- **중앙집중 YAML**로 네트워크/IP/포트 및 센서 설정 일원화
- **Safety Chain 표준화** (twist_mux → velocity_smoother → collision_monitor)
- **정밀 위치 추정** (Wheel Odom + IMU → EKF Fusion)
- **통신 안정화** (Windows Host Load로 인한 패킷 지연/버스트 방어)
- **고정밀 지도 작성** (SLAM Toolbox 기반 Loop Closure 적용)

---

## 2) 패키지 구조
- **`bridge_bringup`**: 하드웨어 통합 런치(Base Layer). EKF 및 Lifecycle Manager 포함.
- **`udp_raw_bridge`**: 시뮬레이터 → ROS2 수신(RX). UDP 포트 바인딩 및 퍼블리시.
- **`udp_parsers_cpp`**: RAW 토픽 파싱 및 데이터 정규화.
  - `imu_parser`: IMU 버스트 방어 로직 적용.
- **`udp_tx_bridge`**: ROS 토픽(`/cmd_vel` 등)을 UDP로 송신(TX).
- **`safety_bringup`**: `twist_mux` → `velocity_smoother` → `collision_monitor` 체인.
- **`sensor_bringup`**: 센서 공통 설정 및 `scan_normalizer` 실행.
- **`state_estimator`**: 
  - `odom_publisher`: 지연 방어 로직이 적용된 Odom 변환.
  - **`robot_localization` (EKF)**: Odom + IMU 융합 및 TF 발행.
- **`mapping_localization`**: SLAM Toolbox 런치 및 파라미터 구성.
- **`bridge_msgs`**: 공용 메시지 정의.

---

## 3) 현재 상태 (2026‑02‑13)
- ✅ **M4 완료: 맵핑(SLAM) 및 계층형 아키텍처**
  - **SLAM Toolbox**: Loop Closure가 적용된 정밀 지도(`map_test.pgm`) 작성 완료.
  - **Layered Launch**: 하드웨어(Tier 1)와 응용 프로그램(Tier 2)의 런치 파일 분리.

---

## 4) 빠른 시작
> 상세 설치/네트워크/검증 절차는 **`docs/setup_manual_2026-02-12.md`** 참조.

**Tier 1: 하드웨어 실행 (Base Layer)**
센서, 모터, EKF 등 기본 시스템을 구동합니다. (항상 실행)
```bash
ros2 launch bridge_bringup bridge.launch.py log_level_raw:=info
```

**Tier 2: 응용 프로그램 실행 (App Layer)**
목적에 맞는 기능을 선택하여 실행합니다.
```bash
# 옵션 A: 지도 작성 (Mapping)
ros2 launch mapping_localization mapping.launch.py

# 옵션 B: 지도 저장 (Save Map)
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/mapping_localization/maps/my_map
```

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
[sensor_bringup] + [state_estimator]
    │ (Processing: Scan Normalizer + EKF Fusion)
    │ (* Anti-Lag Logic applied to Odom)
    │ (Topic: /scan, /odom, /tf)
    ▼
    +----------------------------------------------+
    │                                              │
[mapping_localization] (New!)             [safety_bringup]
(SLAM Toolbox)                        (TwistMux -> Smoother -> Col.Monitor)
    │ (Sub: /scan, /tf)                            │ (Lifecycle Managed: Auto-Active)
    │ (Pub: /map)                                  │ (Sub: /cmd_vel_nav, /scan)
    ▼                                              │ (Pub: /cmd_vel)
[Map Server / Saver]                               ▼
(Save to .pgm/.yaml)                       [udp_tx_bridge] 
                                                   │ (UDP)
                                                   ▼
                                              [Simulator]
```

---

## 6) 주요 이슈 해결 (Troubleshooting Log)
**Q. 시뮬레이터가 렉 걸리면 로봇이 순간이동해요.** **A.** Windows 호스트 부하로 인해 패킷이 지연되어 들어오는 현상입니다.  
`odom_publisher`에 **`dt > 0.1s` 스킵 로직**을 적용하여 위치 튐 현상을 방어했습니다.

**Q. 지도가 겹쳐서 그려져요.** **A.** 오도메트리 정밀도 부족 문제입니다.  
IMU와 Wheel Odom을 **EKF로 융합**하여 회전 오차를 최소화하고, SLAM의 **Loop Closure** 기능을 활성화하여 해결했습니다.

---

## 7) 로드맵
- [x] **M0. 브릿지 안정화 (C++ Porting)**
- [x] **M1. 오도메트리 기본 구현**
- [x] **M1.5. Safety 체인 구축 (Nav2 Stack)**
- [x] **M2.0. EKF 센서 퓨전 & 시스템 안정화 (Anti-Lag)**
- [x] **M2.5. Lifecycle 자동화 (Launch 통합)**
- [x] **M3. TF/URDF 정합 (EKF 기반)**
- [x] **M4. 맵핑(SLAM) — SLAM Toolbox**
- [ ] **M5. 정밀 측위(AMCL) 및 기본 주행(Nav2)**
- [ ] **M6. 시스템 슈퍼바이저 (FSM + BT)**
- [ ] **M7. 물체 제어 (Pick & Place)**
- [ ] **M8. 제어 성능 고도화 (Optimization)**
- [ ] **M9. 통합 회귀 테스트**

---

## License
MIT

## 출처
- WellDone Simulator Repository (레거시)
- Nav2 / ROS 2 Humble 문서