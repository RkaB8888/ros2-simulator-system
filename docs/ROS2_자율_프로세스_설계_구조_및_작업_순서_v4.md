# ROS 2 자율 프로세스 설계 구조 & 작업 순서 (큰 줄기) — v4

작성일: 2026-02-12 (Updated)
목적: **전체 프로젝트의 상태와 향후 로드맵을 추적**합니다. 세부 작업은 별도 대화방에서 진행하며, 이 문서는 프로젝트의 **Source of Truth**로 활용합니다.

## v4 변경 요약 (2026-02-13)
- **M4(맵핑) 완료**: SLAM Toolbox 적용 및 정밀 지도(`map_test.pgm`) 확보 완료.
- **실행 아키텍처 재정립**: **Base(Bridge)**와 **App(Mapping/Nav)**의 **Layer 분리** 구조 확정.
  - 기존: 런치 파일 하나로 통일 (All-in-One)
  - 변경: T1(하드웨어) + T2(응용 프로그램) 분리 실행 (Odom 유지 및 재사용성 증대)

---

## 1. 전체 아키텍처 개요 (As-Is)
```text
[Simulator] 
    │ (UDP)
    ▼
[I/O Layer] 
    udp_raw_node → parsers (Anti-Burst)
    │
    ▼
[State Estimation]
    odom_publisher (Anti-Lag) → /wheel/odom 
    IMU Parser → /imu
    + robot_localization (EKF) 
    = 최종 /odom 및 /tf (odom->base_link)
    │
    ▼
[Perception & Mapping]
    slam_toolbox (Mapping) OR amcl (Localization)
    │
    ▼
[Navigation & Safety]
    Nav2 Stack (Planner/Controller)
    ↓
    [Safety Chain] (Lifecycle Managed)
    Velocity Smoother → Twist Mux → Collision Monitor
    │
    ▼
[Output]
    /cmd_vel → udp_tx_bridge → Simulator
```

**프레임 구조 (Standard):**
- `map` → (AMCL) → `odom` → (EKF) → `base_link` → `sensors...`

---

## 2. 패키지 구성 (Actual)
### ✅ 구현됨 (Existing)
- **`bridge_bringup`**: 시스템 통합 런치 (Entry Point).
- **`bridge_msgs`**: 커스텀 메시지 정의.
- **`udp_raw_bridge`** / **`udp_tx_bridge`**: UDP 송수신(RX/TX) 노드.
- **`udp_parsers_cpp`**: 데이터 파싱 (IMU Burst 방어 포함).
- **`sensor_bringup`**: 센서(`scan`, `imu`) 관련 설정 및 전처리.
- **`state_estimator`**: 
  - `odom_publisher` (Lag 방어), `robot_localization` (EKF)
- **`mapping_localization`** (New): 
  - SLAM Toolbox 런치(Hardware 분리형) 및 파라미터(WSL2 최적화).
- **`safety_bringup`**: 
  - `velocity_smoother` → `twist_mux` → `collision_monitor` 체인 구성.

### 🚧 예정됨 (Planned for M5~)
- **`mapping_localization`** (예정): AMCL 파라미터/런치.
- **`task_manager`** (예정): FSM 및 BT 관련 로직.

---

## 3. 데이터 플로우 (Implemented)
- **Odom**: `ego_status` → `odom_publisher`(dt>0.1s 스킵) → `/wheel/odom`
- **IMU**: `imu_raw` → `imu_parser`(dt<2ms 드랍) → `/imu`
- **Fusion**: `/wheel/odom` + `/imu` → **`EKF`** → `/odom` + `/tf`
- **Safety**: Nav2/Teleop → `TwistMux` → `Smoother` → **`CollisionMonitor`** → `/cmd_vel`

---

## 4. Lifecycle 운영 (Implemented)
- **`nav2_lifecycle_manager`**가 시스템 기동 시 주요 노드를 자동 전이시킴.
- **대상**: `velocity_smoother`, `collision_monitor`, (향후) `map_server`, `amcl`
- **상태**: `Unconfigured` → `Inactive` → `Active` (자동)

---

## 5. 작업 순서(로드맵) & 수락 기준

### ✅ Phase 0: 기반 시스템 안정화 (완료)
- [x] **M0. 브릿지 안정화**: C++ Porting, QoS 최적화 (Best Effort).
- [x] **M1. 오도메트리 & Safety**: TwistMux/CollisionMonitor 적용.
- [x] **M2. EKF 센서 퓨전**: Wheel+IMU 융합, 회전 오차 0.15% 달성.
- [x] **M3. 시스템 최적화**: Anti-Lag(Windows 부하 방어), Lifecycle 자동화.

### 🚧 Phase 1: 환경 인지 및 기본 주행 (진행 중)
- [x] **M4. 맵핑 (Mapping) — SLAM Toolbox** (완료)
    - **도구**: `slam_toolbox` (async)
    - **결과**: Loop Closure가 적용된 고정밀 지도(`map_test`) 확보 및 Gitignore 처리.
- [ ] **M5. 정밀 측위 및 기본 주행 (Localization & Basic Nav)**
    - **도구**: `amcl`, `nav2_controller` (RPP/MPPI)
    - **수락 기준**: 
        1. 초기 위치 파악 후 제자리 회전 시 입자(Particle) 수렴.
        2. RViz Goal 클릭 시 충돌 없이 목적지 도착 (성공률 90%↑).

### 🧠 Phase 2: 자율화 로직 (System Logic)
- [ ] **M6. 시스템 슈퍼바이저 (FSM + BT)**
    - **내용**: `INIT` → `LOCALIZE` → `Maps` → `ACTION` 상태 머신.
    - **수락 기준**: FSM 상태에 따라 하위 노드들이 자동으로 제어되고 시나리오가 BT로 수행됨.
- [ ] **M7. 물체 제어 (Task & Manipulation)**
    - **내용**: 정밀 정렬(Align), 픽앤플레이스.
    - **수락 기준**: 작업 수행 후 복귀하는 전체 사이클 성공.

### 🔧 Phase 3: 최적화 (Optimization)
- [ ] **M8. 제어 성능 고도화**
    - **내용**: 커스텀 Path Tracker 또는 고난이도 튜닝.
    - **수락 기준**: S자 코스, 좁은 골목 주행 성능 개선.
- [ ] **M9. 통합 회귀 테스트**
    - **내용**: 장시간 연속 가동 및 스트레스 테스트.

---

## 6. 런치/파라미터 구성 가이드 (Updated)
**계층형 실행 전략 (Layered Execution Strategy)**
- **Tier 1: Base Layer (Always On)**
  - **`bridge_bringup/bridge.launch.py`**: 하드웨어/센서/Safety/EKF/Tf 실행. (절대 끄지 않음)
- **Tier 2: Application Layer (Select One)**
  - **`mapping.launch.py`**: SLAM Toolbox 실행. (Bridge 포함 X)
  - **`navigation.launch.py`** (예정): Map Server + AMCL + Nav2 실행. (Bridge 포함 X)

---

## 7. Next Action Item
- **[M5] 정밀 측위 및 주행 (Navigation)**
  - `nav2_bringup` 패키지 분석 및 `navigation.launch.py` (Tier 2) 생성.
  - 저장된 지도(`map_test`)를 불러오는 Map Server 구성.
  - AMCL 파라미터 튜닝 및 초기 위치 추정(Localization) 테스트.