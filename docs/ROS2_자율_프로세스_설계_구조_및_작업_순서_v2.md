# ROS2 자율 프로세스 설계 구조 & 작업 순서 (큰 줄기) — v2

작성일: 2025-10-17  
목적: **이 대화방에서는 전체 큰 흐름만 추적**합니다. 각 세부 작업은 _별도의 대화방_을 만들어 진행하고, 이 문서를 북마크처럼 사용합니다.

## v2 변경 요약
- **Safety Layer 추가**: `velocity_smoother → twist_mux → collision_monitor`로 구성된 **최종 게이트 체인**을 도입하여 Nav2 외부에서 충돌 임박 시 강제 정지(E-stop 레벨) 수행.
- **Lifecycle Nodes 도입**: 상위 FSM과 연동해 `Unconfigured → Inactive → Active` 절차로 **기동/정지/복구 절차화**, Nav2 Lifecycle Manager 활용.

---

## 0. 전제
- 시뮬레이터 ↔ ROS2 UDP 브릿지는 안정적으로 동작한다.
- 센서 토픽(`/ego_status`, `/imu`, `/scan` 등)은 규격에 맞게 발행된다.
- ROS 2 Humble + WSL2(또는 Ubuntu 22.04) 환경 기준.

---

## 1. 전체 아키텍처 개요
```
I/O 레이어: udp_raw_node → *parsers (ego/imu/scan) → ROS 토픽

기초 상태 추정: odom_publisher(ego_status → /odom) + TF2(odom→base_link)
  (선택) robot_localization EKF: /wheel_odom + /imu → /odom 품질 향상

지각/지도: slam_toolbox(맵핑) ↔ nav2_map_server

측위: amcl (맵 기반 위치 추정)

경로 계획/추종: Nav2(Planner/Controller) 
  또는 커스텀 path_tracker(Pure Pursuit/Stanley/MPC)

상태 관리: 상위 FSM(모드 전환) + 모드 내부는 BT(행동 트리; Nav2 액션/리커버리 조합)

작업/물체 제어: object_control(픽앤플레이스/정렬), FSM과 액션 인터페이스

안전 체인(Safety Layer; 최종 게이트):
  Controller(Server) → [Velocity Smoother] → [Twist Mux] → [Collision Monitor] → /cmd_vel → 드라이버
```
프레임 구조(권장):
- `odom → base_link → (lidar_link, imu_link, wheel_link…)`
- 맵 주행 시 `map → odom`(amcl 제공)

---

## 2. 패키지 구성(제안)
- `bridge_msgs/` · 공통 msg/idl
- `udp_raw_bridge/` · UDP 수신/송신
- `udp_parsers_cpp/` · ego/imu/scan 파서
- `state_estimator/`
  - `odom_publisher` (필수; C++ 권장)
  - `tf_broadcaster` (필수)
  - `robot_localization` 파라미터(EKF/UKF, 선택)
- `mapping_localization/`
  - `slam_toolbox` 런치/파라미터
  - `map_server` + `amcl` 런치/파라미터
- `navigation/`
  - `nav2_params.yaml`
  - (옵션) `path_tracker` 컨트롤러 플러그인
- `safety/`
  - `velocity_smoother`(Lifecycle 컴포넌트)
  - `twist_mux`(우선순위·락 기반 다중 cmd_vel 중재)
  - `collision_monitor`(충돌 임박 시 감속/정지, 최종 발행자)
- `task_manager/`
  - 상위 FSM 노드(모드: INIT/TELEOP_MAP/LOCALIZE/NAVIGATE/DOCK/IDLE/ERROR)
  - BT 정의(.xml) + Nav2 액션/리커버리 노드

---

## 3. 데이터 플로우 요약
- `/ego_status` → **`odom_publisher`** → `/odom(nav_msgs/Odometry)` + `/tf [odom→base_link]`
- `/scan` → `slam_toolbox`(맵 생성) → `map.yaml`
- `map.yaml` → `map_server` + `amcl` → `/tf [map→odom]`
- Nav2(Planner/Controller) ← `/odom`, `/tf`, `/costmap`(← `/scan`)
- **Safety 체인**: Controller 출력 → Velocity Smoother → Twist Mux(텔레옵/자율/비상 우선순위) → **Collision Monitor**(감속/정지 정책) → `/cmd_vel` 최종 발행
- FSM(상위 모드) → BT(행동 조합) → Nav2/오브젝트 액션

---

## 4. Lifecycle 기반 운영(요약)
- **목표**: “노드는 떴지만 준비 전” 상태와 “활성 상태”를 **명확히 구분**하고, **장애 시 정의된 절차로 복구**.
- **전이 단계**: `unconfigured → inactive → active` (필요 시 `deactivate → cleanup → shutdown`)
- **감독자**: 
  - Nav2 계열은 **`lifecycle_manager`**를 사용해 플래너/컨트롤러/BT/맵서버/AMCL 등을 일괄 전이.
  - 자체 노드(파서/오돔/세이프티)는 FSM이 서비스 호출(`change_state`)로 전이 관리.
- **권장 전이 시퀀스**:
  1) 브릿지/파서: configure → activate  
  2) 오돔/TF: configure → activate  
  3) (맵핑 모드) slam_toolbox: configure → activate  
     (맵 주행 모드) map_server → amcl: configure → activate  
  4) Nav2: planner/controller/bt_navigator: configure → activate  
  5) Safety: velocity_smoother → twist_mux → collision_monitor: configure → activate  
- **장애 대응**: 종속 역순으로 `deactivate → cleanup` 후 필요한 집합만 재기동.

---

## 5. Safety Layer 설계 메모
- **위치**: 최종 발행 체인의 **맨 끝**(Collision Monitor가 `/cmd_vel` 최종 발행자)
- **초기 파라미터(실내 시작값)**:
  - 감속 존: 약 **0.5 m**
  - 정지 존: 약 **0.25 m**
- **정지거리 보정**:  
  \( d_\text{total} = \frac{v^2}{2 a_\text{max}} + v \cdot t_\text{delay} \)  
  - 최대 감속 \(a_\text{max}\), 제어/센서 지연 \(t_\text{delay}\)를 반영해 존 크기 보수적으로 설정
- **우선순위**: 텔레옵/자율/비상 입력은 **Twist Mux**에서 우선순위·락으로 중재 → **Collision Monitor**가 최종 안전 보루.

---

## 6. 작업 순서(마일스톤) & 수락 기준
### M0. 브릿지 안정화(완료 가정)
- 수락: `/ego_status`, `/imu`, `/scan` 갱신률·지연 로그 OK

### M1. 오도메트리 파이프라인 (최우선)
- `state_estimator/odom_publisher`: `/ego_status` → `/odom` + `/tf(odom→base_link)`
- 수락: RViz에서 1m 전진/90° 회전 시 TF/ODOM 일치(±10%) 및 드리프트 로그

### **M1.5. Safety 체인 구축 (신규)**
- 구성: `velocity_smoother → twist_mux → collision_monitor`
- 전체 입력(텔레옵/자율)은 반드시 이 체인을 **통과**하도록 배선
- 수락: 
  - 정면 장애물 0.25 m에서 즉시 정지(지연 ≤ 목표), 0.5 m에서 감속
  - Nav2 다운/오작동 시에도 충돌 미발생(모의 테스트)

### M2. 상태추정 고도화(선택 권장)
- `robot_localization`(EKF): `/wheel_odom` + `/imu` 융합 → 스무딩된 `/odom`
- 수락: 노이즈 조건에서 yaw·속도 RMSE 개선

### **M2.5. Lifecycle 도입 (신규)**
- Nav2 Lifecycle Manager 및 자체 노드 Lifecycle 전환
- FSM이 모드 전환 시 각 그룹을 순차적으로 `configure→activate`/역순 `deactivate→cleanup`
- 수락: 
  - 기동/정지/재기동 과정에서 미정의 상태 없음(로그 무에러)
  - 장애 주입 테스트에서 안전한 철수·재기동 성공

### M3. TF 트리·URDF 정합
- `base_link`, `imu_link`, `lidar_link` 고정 TF, 휠 베이스/센서 위치 반영
- 수락: RViz TF에 루프/불연속 없음

### M4. 맵핑(SLAM)
- `slam_toolbox` online → 수동 주행 → `map_saver`로 `map.yaml` 산출
- 수락: 재주행 시 스캔-맵 정합 OK

### M5. 맵 기준 측위 + Nav2 기동
- `map_server` + `amcl` + `nav2_bringup`
- 수락: RViz “Nav2 Goal” 도착 성공률 ≥ 95%

### M6. 컨트롤러 튜닝 또는 커스텀 Path Tracking
- Nav2 Controller 파라미터(속도/가속/회전/footprint/inflation)
  또는 `path_tracker`(Pure Pursuit/Stanley) 플러그인화
- 수락: S자/문턱형 코스 오버슛·진동 ≤ 목표

### M7. 상위 제어(FSM + BT 하이브리드)
- FSM: 큰 모드 전환과 에러/리커버리 정책
- BT: Nav2 액션·정밀정렬·리커버리 시퀀스/폴백
- 수락: 장애/실패 시 자동 리커버리 규칙 정상 동작

### M8. 물체 제어(픽앤플레이스/정렬)
- Nav2 도착 후 미세 정렬(AR 태그/라인업) → 그리퍼 액션
- 수락: 위치/각도 허용오차 만족, 성공률 기준 충족

### M9. 회귀/운영
- 시나리오별 `ros2 bag` + 성능 리포트(주행 시간, 실패율, CPU/RAM)
- 수락: 주간 회귀 시나리오 전부 통과

---

## 7. 런치/파라미터 구성 팁
- `bringup.launch.py`: **맵핑용**과 **맵주행용**을 분리 제공
- Safety 체인 파라미터 묶음: `params/velocity_smoother.yaml`, `params/twist_mux.yaml`, `params/collision_monitor.yaml`
- Lifecycle 전환은 `lifecycle_manager` 또는 FSM의 서비스 호출로 일원화
- RViz 프로파일: `rviz/warehouse.rviz`(TF/odom/scan/path/goal/safety 표시)

---

## 8. 다음에 새 대화방에서 진행할 첫 작업
- **[M1] 오도메트리 발행 노드(ego_status → /odom) 설계 요구사항 문서**를 첨부하고 개발 착수.
- **[M1.5] Safety 체인** 배선 후 모의 충돌 테스트.
- **[M2.5] Lifecycle** 기동/정지 절차 자동화.
