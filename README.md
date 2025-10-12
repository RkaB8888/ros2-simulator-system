# ROS2 Simulator Bridge (Humble, C++ / WSL)

이 저장소는 SSAFY 프로젝트([원본 레거시](https://github.com/RkaB8888/SSAFY-Specialized-PJT-WellDone.git)) 중 **시뮬레이터 브릿지 영역**을 재설계·최적화·학습 목적으로 **ROS 2 Humble + C++ + WSL(리눅스)** 환경에 맞춰 다시 구현한 프로젝트입니다.  
레거시의 `ssafy_bridge`(Windows, Python, ROS2 Eloquent 기반)를 참고하여 **네트워킹/브릿지/파서 구조를 현대화**했고, 2025-09-27에 시작하여 **2025-10-13 기준 브릿지 레이어(수신/파싱/송신) 재구현 및 통합 테스트를 완료**했습니다. 이후 단계로 경로추적, SLAM, FSM 등 고수준 알고리즘을 이 환경에 맞춰 구현/이식할 계획입니다.

## 목표와 배경
- 레거시(Windows, Python, ROS2 Eloquent) → **현업 친화 스택**(Linux/WSL, C++, ROS2 Humble)로 업그레이드
- 시뮬레이터 I/O 규격을 유지하면서 코드 구조를 **RX(UDP Raw) → Parser → TX(ROS→UDP)** 3단 레이어로 정리
- **중앙 집중형 설정(YAML)** 로 모든 IP/포트를 한 곳에서 관리 (환경 전환 용이)

## 구성(패키지)
- **`bridge_bringup`**: 통합 런치 & 공용 설정(YAML). 모든 IP/포트는 여기에서만 관리.
- **`udp_raw_bridge`**: 시뮬레이터 → ROS2 수신(RX). 각 UDP 포트를 바인딩해 raw 바이트를 퍼블리시.
- **`udp_parsers_cpp`**: RAW 토픽을 ROS 메시지로 파싱(ego/env/iot/object/imu/lidar/camera).
- **`udp_tx_bridge`**: ROS 토픽(`/cmd_vel`, `/hand_control`, `/iot_control`)을 **레거시와 동일한 UDP 프레이밍**으로 송신(TX).

메시지 패키지: **`bridge_msgs`** (공용 메시지 정의)

## 현재 상태 (2025-10-13)
- 통합 브릿지 실행으로 **센서 수신/파싱/표출 + 제어 명령 송신**이 동작
- **레거시 UDP 프로토콜 준수**:
  - `cmd_vel`: `#Turtlebot_cmd$` + `len=8` + `0,0,0` + `float32 linear, float32 angular` + `\r\n`
  - `hand_control`: `#hand_control$` + `len=9` + `0,0,0` + `uint8 mode, float32 distance, float32 height` + `\r\n`
  - `iot_control`: `#Appliances$` + `len=17` + `0,0,0` + `uint8[17]` + `\r\n`
- **WSL↔Windows 네트워킹 확인** 및 시뮬레이터 상호작용 검증 완료

## 빠른 시작 (요약)
> 상세 설치/검증은 `docs/setup_manual.md` 참고.

```bash
# 1) ROS2 Humble + colcon 환경 (WSL/Ubuntu 22.04 권장)
# 2) 워크스페이스 구성
cd ~/ros2_ws/src
git clone <this_repo> .
cd ..
colcon build
source install/setup.bash

# 3) 통합 브릿지 실행 (환경별 YAML 선택 가능)
ros2 launch bridge_bringup bridge.launch.py
# 또는
ros2 launch bridge_bringup bridge.launch.py config_file:=/absolute/path/to/system.<env>.yaml
```

### 설정(YAML)
- **중앙 설정**: `bridge_bringup/config/system.<env>.yaml`
- 모든 **IP/포트는 여기서만** 정의(노드 기본값 없음, 미지정 시 즉시 실패).
- 예: (WSL 예시)
  - RX 포트: `udp_rx_*` 아래 `listen_port`, `topic_name`
  - TX 목적지: `udp_tx_*` 아래 `remote_ip`(일반적으로 **Windows(vEthernet/WSL) 주소 172.23.0.1**), `remote_port`
- 환경별로 파일만 바꿔 끼우면 다른 PC에서도 그대로 실행 가능.

### 실행/검증 팁
- RQt로 파서 산출 토픽 확인 (이미지, 라이다 등)
- 송신 테스트:
  ```bash
  # WSL → 시뮬레이터 제어
  ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 0.0}}' -1
  ```
- Windows PowerShell에서 수신 확인(예: 7601):
  ```powershell
  $u = New-Object System.Net.Sockets.UdpClient(7601)
  $ep = New-Object System.Net.IPEndPoint([IPAddress]::Any,0)
  while ($true) { $b = $u.Receive([ref]$ep); ($b | % { '{0:X2}' -f $_ }) -join ' ' }
  ```
- 방화벽: 포트(1232, 7802, …, 9094, 7601, 7901, 8101 등) **UDP 인바운드 허용**
- **WSL/Windows IP 주의**  
  - WSL에서 보이는 **Windows 주소**: 보통 `172.23.0.1` (TX `remote_ip`에 사용)  
  - Windows에서 보이는 **WSL 주소**: `hostname -I` 로 확인되는 `172.23.x.x` (시뮬레이터가 WSL로 보낼 때 필요)  
  - WSL IP는 재시작/네트워크 변동 시 바뀔 수 있음 → 센서 송신 측에서 목적지 IP 재설정 필요

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

## 로드맵
- [ ] 경로 추적 알고리즘 이식/개선
- [ ] SLAM 통합(ROS2 Humble 호환)
- [ ] FSM 구조 재설계
- [ ] 시뮬레이터 제어 파이프라인 안정화/테스트 자동화

## 라이선스
MIT

## 감사
- SSAFY Specialized PJT – WellDone (원본 브릿지/시뮬레이터 환경)
