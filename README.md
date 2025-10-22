# ROS2 Simulator System (Humble, C++ / WSL)

이 저장소는 교육용 시뮬레이터(MORAI SIM) 환경에서 사용된 브릿지 및 자율주행 관련 시스템을  
**ROS 2 Humble + C++ + WSL(리눅스)** 기반으로 재구현하고, 네트워크 구조를 현대화한 프로젝트입니다.

기존 Windows/Python(Eloquent) 기반 환경을 참고하여 **UDP I/O, 파서, 브릿지, 세이프티 체인 구조를 최적화**하였으며,  
2025-09-27에 시작해 **M1.5a Safety 체인 구축**(2025-10-23)까지 완료되었습니다.

이후 단계로 SLAM, 경로 추적, FSM 등 고수준 자율주행 알고리즘을 이 환경에서 재설계할 예정입니다.

> 🔗 참고 레거시: [원본 프로젝트 (WellDone)](https://github.com/RkaB8888/SSAFY-Specialized-PJT-WellDone.git)

---

## 목표와 배경
- Python(Eloquent, Windows) → **C++(Humble, Linux/WSL)** 환경으로 전환  
- 기존 시뮬레이터 UDP 프로토콜 호환 유지  
  (Bridge Layer: RX → Parser → TX)
- **중앙집중식 YAML 설정**으로 IP/Port 관리 단일화  
- **Safety Chain 구축**을 통해 ROS2 Nav2 기반 속도 제어 표준화  
- 향후 SLAM·FSM·Path Tracking을 위한 통합 기반 확보

---

## 구성(패키지)
- **`bridge_bringup`**: 통합 런치 & 공용 설정(YAML). 모든 IP/포트 및 세이프티 토글 관리.
- **`udp_raw_bridge`**: 시뮬레이터 → ROS2 수신(RX). 각 UDP 포트를 바인딩해 raw 바이트를 퍼블리시.
- **`udp_parsers_cpp`**: RAW 토픽을 ROS 메시지로 파싱(ego/env/iot/object/imu/lidar/camera).
- **`udp_tx_bridge`**: ROS 토픽(`/cmd_vel`, `/hand_control`, `/iot_control`)을 **레거시와 동일한 UDP 프레이밍**으로 송신(TX).
- **`safety_bringup`** *(신규)*:  
  - `twist_mux` → `velocity_smoother` → `collision_monitor` 체인 구성  
  - 속도 중재, 가감속 제한, 충돌 감지 등 안전 계층 관리  
  - Humble 기준 YAML 호환성(Flat double polygon, dummy observation source 등)
- **`state_estimator`**: `/ego_status` → `/odom` + `TF(odom→base_link)` 변환
- **`bridge_msgs`**: 공용 메시지 정의

---

## 현재 상태 (2025-10-23)
- ✅ **M1.5a Safety 체인 통합 완료**
  - twist_mux, nav2_velocity_smoother, nav2_collision_monitor 구성
  - `/cmd_vel_nav` → `/cmd_vel_mux` → `/cmd_vel_smooth` → `/cmd_vel` 체인 검증
  - rqt_plot에서 스무딩 램프 업/다운 정상 확인
  - RViz에서 polygon_* 시각화 정상 표시 (Trans. Local QoS)
- ✅ **라이프사이클 노드 활성화 지원** (`ros2 lifecycle set ... configure → activate`)
- ✅ **Humble 기준 호환성 문제 해결**
  - collision_monitor의 points 형식(double array) 수정
  - observation_sources dummy_scan 추가
- ✅ **WSL↔Windows UDP 통신 및 시뮬레이터 제어 검증 완료**
- 🔧 **다음 단계 (M1.5b)**: LiDAR/Scan 센서 입력과 slowdown/stop 활성화 예정

---

## 빠른 시작 (요약)
> 상세 설치 및 검증 절차는 `docs/setup_manual_2025-10-23.md` 참고

```bash
# 1) ROS2 Humble + colcon 환경 (WSL/Ubuntu 22.04)
cd ~/ros2_ws/src
git clone <this_repo> .
cd ..
colcon build
source install/setup.bash

# 2) 통합 브릿지 실행
ros2 launch bridge_bringup bridge.launch.py log_level_raw:=info

# 3) 라이프사이클 활성화
ros2 lifecycle set /velocity_smoother configure
ros2 lifecycle set /velocity_smoother activate
ros2 lifecycle set /collision_monitor configure
ros2 lifecycle set /collision_monitor activate
```

---

## 실행 및 검증 팁
- **RQt**로 파싱된 토픽 실시간 확인
- **제어 송신 테스트:**
  ```bash
  # WSL → 시뮬레이터 제어
  ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 0.0}}' -1
  ```
- **PowerShell UDP 모니터링 예시 (7601):**
  ```powershell
  $u = New-Object System.Net.Sockets.UdpClient(7601)
  $ep = New-Object System.Net.IPEndPoint([IPAddress]::Any,0)
  while ($true) { $b = $u.Receive([ref]$ep); ($b | % { '{0:X2}' -f $_ }) -join ' ' }
  ```
- 방화벽: 1232, 7802, 8002, 8202, 8302, 9092, 9094, 7601, 7901, 8101 포트 UDP 인바운드 허용
- **WSL ↔ Windows IP 설정**
  - WSL에서 Windows는 `172.23.0.1`
  - Windows에서 WSL은 `hostname -I` 로 확인되는 `172.23.x.x`
  - WSL 재시작 시 IP가 바뀔 수 있음 → 시뮬레이터 송신 대상 재설정 필요

---

## 레이아웃
```
ros2_ws/
└── src/
    ├── bridge_bringup/     # 통합 런치 & 중앙 YAML
    ├── bridge_msgs/        # 공용 메시지
    ├── safety_bringup/     # cmd_vel 출력
    ├── state_estimator/    # Odometry 발행
    ├── udp_raw_bridge/     # UDP 수신 → raw 토픽
    ├── udp_parsers_cpp/    # raw → 구조화 메시지
    └── udp_tx_bridge/      # ROS 토픽 → UDP 송신
  docs/
    ├── setup_manual.md     # 전체 세팅/검증 매뉴얼
    └── project_progress_log_YYYY-MM-DD.md  # 작업 일지
  README.md                 # (이 파일)
```

---

## 로드맵 (v2)
- [x] **M0. 브릿지 안정화** — 센서 수신/파싱/제어 송신 통합 테스트 완료
- [x] **M1. 오도메트리 파이프라인** — `/ego_status → /odom` + `tf(odom→base_link)`
- [x] **M1.5a. Safety 체인 구축** — twist_mux → velocity_smoother → collision_monitor
- [ ] **M1.5b. 센서 기반 감속/정지 구현**
- [ ] **M2. 상태추정 고도화(EKF)** — `/wheel_odom`+`/imu` 융합
- [ ] **M2.5. Lifecycle 자동화** — configure/activate 자동 실행
- [ ] **M3. TF/URDF 정합** — 센서 프레임 트리 검증
- [ ] **M4. 맵핑(SLAM)** — slam_toolbox 적용
- [ ] **M5. Nav2 자율주행 기동**
- [ ] **M6. 커스텀 Path Tracker 플러그인**
- [ ] **M7. FSM/BT 하이브리드 제어**
- [ ] **M8. 물체 제어(Pick & Place)**
- [ ] **M9. 시나리오 회귀 테스트**

---

## 라이선스
MIT License

---

## 출처
- 원본 환경: [WellDone Simulator Repository](https://github.com/RkaB8888/SSAFY-Specialized-PJT-WellDone.git)
