# ROS2 Simulator System (Humble, C++ / WSL)

이 저장소는 교육용 시뮬레이터(MORAI SIM) 환경에서 사용된 브릿지 및 자율주행 관련 시스템을  
**ROS 2 Humble + C++ + WSL(리눅스)** 기반으로 재구현하고, 네트워크 구조를 현대화한 프로젝트입니다.  
기존 Windows/Python(Eloquent) 기반 환경을 참고하여 **UDP I/O, 파서, 브릿지 구조를 최적화**하였으며,  
2025-09-27에 시작해 **2025-10-13 기준 브릿지 계층(RX/Parser/TX)의 완전한 재구현 및 통합 테스트를 완료**했습니다. 현재(2025-10-18) **M1 오도메트리 파이프라인 구현**까지 완료되었습니다.  
이후 단계로 SLAM, 경로 추적, FSM 등 고수준 자율주행 알고리즘을 이 환경에서 재설계할 예정입니다.

> 🔗 참고 레거시: [원본 프로젝트 (WellDone)](https://github.com/RkaB8888/SSAFY-Specialized-PJT-WellDone.git)

---

## 목표와 배경
- Python(Eloquent, Windows) → **C++(Humble, Linux/WSL)** 환경으로 전환
- 기존 시뮬레이터 UDP 프로토콜 호환 유지  
  (Bridge Layer: RX → Parser → TX)
- **중앙집중식 YAML 설정**으로 IP/Port 관리 단일화
- 향후 SLAM·FSM·Path Tracking을 위한 통합 기반 확보

---

## 구성(패키지)
- **`bridge_bringup`**: 통합 런치 & 공용 설정(YAML). 모든 IP/포트는 여기에서만 관리.
- **`udp_raw_bridge`**: 시뮬레이터 → ROS2 수신(RX). 각 UDP 포트를 바인딩해 raw 바이트를 퍼블리시.
- **`udp_parsers_cpp`**: RAW 토픽을 ROS 메시지로 파싱(ego/env/iot/object/imu/lidar/camera).
- **`udp_tx_bridge`**: ROS 토픽(`/cmd_vel`, `/hand_control`, `/iot_control`)을 **레거시와 동일한 UDP 프레이밍**으로 송신(TX).

- **`bridge_msgs`**: 공용 메시지 정의

## 현재 상태 (2025-10-18)
- **센서 수신 → 파싱 → 시각화 + 제어 명령 송신** 정상 동작  
- **M1: 오도메트리 파이프라인** — `/ego_status` → `/odom` + `TF(odom→base_link)` 구현 및 RViz 검증 완료
- **네임스페이스 런치 인자 도입** — `bridge_bringup`에서 `ns`를 받아 raw/parsers/tx 런치에 전달 (멀티 인스턴스 지원)
- **레거시 UDP 프로토콜 완전 준수**
  - `cmd_vel`: `#Turtlebot_cmd$` + `len=8` + `0,0,0` + `float32 linear, angular` + `\r\n`
  - `hand_control`: `#hand_control$` + `len=9` + `0,0,0` + `uint8 mode, float32 distance, height` + `\r\n`
  - `iot_control`: `#Appliances$` + `len=17` + `0,0,0` + `uint8[17]` + `\r\n`
- **WSL↔Windows 양방향 통신 확인**, 실제 시뮬레이터 제어 검증 완료

---

## 빠른 시작 (요약)
> 상세 설치 및 검증 절차는 `docs/setup_manual.md` 참고

```bash
# 1) ROS2 Humble + colcon 환경 (WSL/Ubuntu 22.04)
cd ~/ros2_ws/src
git clone <this_repo> .
cd ..
colcon build
source install/setup.bash

# 2) 통합 브릿지 실행
ros2 launch bridge_bringup bridge.launch.py
# 또는
ros2 launch bridge_bringup bridge.launch.py config_file:=/absolute/path/to/system.<env>.yaml
```

---

## 설정 (YAML)
- 경로: `bridge_bringup/config/system.<env>.yaml`
- **모든 IP/포트는 여기에서만 관리**
- 예시 (WSL 환경):
  - RX: `udp_rx_*` → `listen_port`, `topic_name`
  - TX: `udp_tx_*` → `remote_ip`(보통 `172.23.0.1`), `remote_port`
- 환경별 설정 파일만 바꿔 다른 PC에서도 실행 가능
- `state_estimator`는 별도 IP/Port 등 환경 의존 설정 없음(기본 파라미터로 동작).


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
    ├── udp_raw_bridge/     # UDP 수신 → raw 토픽
    ├── udp_parsers_cpp/    # raw → 구조화 메시지
    └── udp_tx_bridge/      # ROS 토픽 → UDP 송신
docs/
  ├── setup_manual.md       # 전체 세팅/검증 매뉴얼
  └── project_progress_log_YYYY-MM-DD.md  # 작업 일지
README.md                   # (이 파일)
```

---

## 로드맵 (v2)

- [x] **M0. 브릿지 안정화** — 센서 수신/파싱/제어 송신 통합 테스트 완료 (2025-10-13)
- [x] **M1. 오도메트리 파이프라인** — `/ego_status → /odom` + `tf(odom→base_link)` 구현
- [ ] **M1.5. Safety 체인 구축** — `velocity_smoother → twist_mux → collision_monitor` (최종 게이트)
- [ ] **M2. 상태추정 고도화(EKF)** — `/wheel_odom`+`/imu` 융합으로 `/odom` 품질 향상
- [ ] **M2.5. Lifecycle 도입** — FSM 연동 `configure/activate` 절차화(기동/정지/복구 자동화)
- [ ] **M3. TF/URDF 정합** — 센서/휠 링크 정합 및 프레임 트리 검증
- [ ] **M4. 맵핑(SLAM)** — `slam_toolbox`로 맵 생성 및 저장
- [ ] **M5. 측위+Nav2 기동** — `map_server + amcl + nav2_bringup`로 목표점 자율주행
- [ ] **M6. 경로 추종 튜닝/개발** — Nav2 컨트롤러 튜닝 또는 커스텀 `path_tracker` 플러그인
- [ ] **M7. 상위 제어(FSM + BT)** — 모드 전환(FSM)과 세부 행동(BT) 하이브리드
- [ ] **M8. 물체 제어** — 도착 후 정렬/그리퍼 액션(픽앤플레이스)
- [ ] **M9. 회귀/운영 자동화** — 시나리오별 rosbag 회귀/리포팅

> 참고 문서: `docs/setup_manual.md`, (권장) `docs/ROS2_자율_프로세스_설계_구조_및_작업_순서_v2.md`


## 라이선스
MIT License

---

## 출처
- 원본 환경 참고: [WellDone Simulator Repository](https://github.com/RkaB8888/SSAFY-Specialized-PJT-WellDone.git)
