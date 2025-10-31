# 시뮬레이터 ↔ ROS2 브리지 셋업 매뉴얼 (업데이트: 2025-10-31)

> 이 문서는 **새로운 PC**에서 본 프로젝트를 실행하기 위한 **기본 세팅 과정**을 정리합니다.  
> ROS 2 **Humble** 설치부터 프로젝트 실행, **최초 통신 테스트**까지의 과정을 다룹니다.

---

## 0. 대상 및 전제

- **OS**: Ubuntu 22.04 (WSL2 권장) + Windows 10/11 (시뮬레이터 구동용)
- **ROS 2 배포판**: **Humble Hawksbill**
- **워크스페이스**: `~/ros2_ws` 기준

---

## 1. 필수 설치 (ROS2 Humble + 의존성)

### 1-1. 기본 유틸/빌드 도구

```bash
sudo apt update
sudo apt install -y build-essential cmake git curl python3-pip python3-colcon-common-extensions
```

### 1-2. ROS 2 Humble 데스크톱

```bash
sudo apt install -y ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1-3. 프로젝트 의존성 설치 (필수)

`safety_bringup` 패키지가 의존하는 핵심 Nav2 패키지들입니다. **반드시 설치**해야 합니다.

```bash
sudo apt install -y   ros-humble-twist-mux   ros-humble-nav2-velocity-smoother   ros-humble-nav2-collision-monitor
```

### 1-4. 권장 유틸 (선택)

디버깅 및 시각화에 유용한 도구들입니다.

```bash
sudo apt install -y   ros-humble-rviz2   ros-humble-rqt   ros-humble-rqt-common-plugins   ros-humble-rqt-image-view   ros-humble-rqt-plot
```

> **설치 확인**: `ros2 --version` (Humble) 및 `rviz2` 실행으로 설치를 점검합니다.

---

## 2. 프로젝트 클론 & 빌드

```bash
# 1) 워크스페이스 생성 및 클론
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <이 레포 URL>

# 2) 빌드
cd ~/ros2_ws
colcon build

# 3) 환경 설정
source install/setup.bash
```

> **팁**: 셸 시작 시 자동 로드를 원하면 `echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc`

---

## 3. 네트워크 및 통합 설정 (가장 중요)

ROS 2(WSL2)와 시뮬레이터(Windows) 간의 UDP 통신을 위한 설정입니다.

### 3-1. IP 주소 파악

**A. WSL2 IP (Ubuntu 터미널)**: 시뮬레이터가 데이터를 **보낼** 주소

```bash
ip -4 addr show eth0 | grep inet
# 예: 172.23.15.206/20 → WSL2 IP는 172.23.15.206
```

**B. Windows "vEthernet (WSL)" IP (PowerShell)**: 시뮬레이터가 데이터를 **받을** 주소

```powershell
Get-NetIPAddress -InterfaceAlias "vEthernet (WSL (Hyper-V firewall))" -AddressFamily IPv4
# 예: 172.23.0.1
```

### 3-2. 통합 YAML 설정 (`bridge_bringup`)

`udp_tx_bridge`가 시뮬레이터로 명령을 보내려면 **Windows IP (B)** 를 알아야 합니다.

1. `~/ros2_ws/src/bridge_bringup/config/system.wsl.yaml` 파일을 엽니다.  
2. `udp_tx_...` 노드들의 `remote_ip` 값을 **(B)에서 찾은 Windows IP**로 수정합니다.

```yaml
# 예: src/bridge_bringup/config/system.wsl.yaml
# ... (udp_rx_... 설정은 그대로 둠) ...

# ---- TX (udp_tx_bridge) ----
# remote_ip를 (B)에서 찾은 vEthernet (WSL) IP로 설정해야 함
udp_tx_cmd_vel:
  ros__parameters: { remote_ip: 172.23.0.1, remote_port: 7601 }
udp_tx_hand_control:
  ros__parameters: { remote_ip: 172.23.0.1, remote_port: 7901 }
udp_tx_iot_control:
  ros__parameters: { remote_ip: 172.23.0.1, remote_port: 8101 }
```

### 3-3. 시뮬레이터 설정 (Windows)

시뮬레이터의 UDP 설정 화면에서 다음을 구성합니다.

- **센서/상태 (시뮬 → ROS2)**
  - **Destination IP**: (A)에서 찾은 **WSL2 IP** (예: `172.23.15.206`)
  - 포트: 7802, 8002, 8202, ... (YAML과 일치시킴)

- **명령 (ROS2 → 시뮬)**
  - **Host(Local) IP**: (B)에서 찾은 **Windows vEthernet (WSL) IP** (예: `172.23.0.1`)
  - 포트: 7601, 7901, 8101 (YAML과 일치시킴)

> **중요**: ROS 2 → 시뮬 명령 채널의 Host IP를 `172.23.0.1` (vEthernet)로 설정해야 WSL2 → Windows 간의 명령 UDP 패킷이 안정적으로 수신됩니다.

### 3-4. Windows 방화벽 설정

Windows 방화벽이 UDP 포트를 차단하지 않도록 설정합니다.

**PowerShell(관리자)** 을 열고 아래 스크립트를 실행합니다.

```powershell
# 수신(RX) 포트 허용
$rx = 7802,8002,8202,8302,9092,9094,1232
$rx | ForEach-Object { New-NetFirewallRule -DisplayName "UDP In (ROS2 Bridge) $_" -Direction Inbound -Protocol UDP -LocalPort $_ -Action Allow }

# 명령(TX) 포트 허용
$tx = 7601,7901,8101
$tx | ForEach-Object { New-NetFirewallRule -DisplayName "UDP In (ROS2 Bridge) $_" -Direction Inbound -Protocol UDP -LocalPort $_ -Action Allow }
```

---

## 4. 아키텍처 및 패키지 역할

### 4-1. 패키지 역할 요약

- **bridge_bringup**: 통합 런치/인자 허브. 모든 레이어(RAW→Parsers→Safety→TX)를 한 번에 실행합니다.
- **safety_bringup**: 세이프티 체인 런치/파라미터 패키지.
- **udp_raw_bridge**: Windows 시뮬 → UDP 수신 → `/..._raw` 퍼블리시.
- **udp_parsers_cpp**: RAW → 구조화 토픽(ego, scan, imu 등) 변환.
- **sensor_bringup** (신규): 센서 설정/정규화 패키지. `sensors.yaml`에 frame/topic 규약을 정의하고, `scan_normalizer`로 LiDAR 스캔을 CCW/180° 보정하여 표준 `/scan`으로 발행.
- **udp_tx_bridge**: ROS2 토픽 → UDP 직렬화 송신.
- **state_estimator**: `/ego_status` → `/odom` 및 TF(odom→base_link) 발행.
- **bridge_msgs**: 프로젝트 커스텀 메시지.

> **Humble 버전 주의 (필독)**  
> 1) `collision_monitor.yaml`의 `polygons.*.points`는 문자열 `"[[...]]"`이 아니라 **flat double 배열** `[x1, y1, x2, y2, ...]` 형식이어야 합니다.  
> 2) `observation_sources`는 빈 배열(`[]`)이나 주석 처리를 허용하지 않습니다. `enabled: false`인 **더미(dummy) 소스 1개**가 반드시 선언되어야 합니다.  
> (※ 본 레포에는 이미 이 사항이 반영되어 있습니다.)

### 4-2. 안전 체인(최종 속도 라인)

```
/cmd_vel_nav, /cmd_vel_teleop, /cmd_vel_emergency
          └──► [twist_mux] ─► /cmd_vel_mux ─► [velocity_smoother] ─► /cmd_vel_smooth ─► [collision_monitor] ─► /cmd_vel
```

---

## 5. 실행

```bash
# 1) 환경 설정 (매번 새 터미널마다 실행)
source ~/ros2_ws/install/setup.bash

# 2) 통합 실행 (모든 노드 시작)
# log_level_raw:=info : 노드 내부 로그를 INFO 레벨로 설정
# --ros-args --log-level warn : 런치 시스템 로그(process started)는 숨김
ros2 launch bridge_bringup bridge.launch.py log_level_raw:=info --ros-args --log-level warn

# 3) 라이프사이클 활성화 (필수)
# 새 터미널을 열고 safety 노드들을 활성화해야 동작합니다.
ros2 lifecycle set /velocity_smoother configure
ros2 lifecycle set /velocity_smoother activate
ros2 lifecycle set /collision_monitor configure
ros2 lifecycle set /collision_monitor activate
```

---

## 6. 동작 확인 (테스트)

시뮬레이터가 실행 중이고 런치 파일이 동작 중인 상태에서 진행합니다.

### 6-1. 수신 경로 (시뮬 → ROS2)

시뮬레이터에서 오는 센서 데이터가 ROS 2에서 잘 수신되는지 확인합니다.

```bash
# 새 터미널
source ~/ros2_ws/install/setup.bash

# A. RAW 토픽 수신율 확인 (카메라/라이다 등)
ros2 topic hz /camera_jpeg_raw
# (예: average rate: 29.998)

# B. 파싱된 토픽 내용 1회 확인
ros2 topic echo --once /ego_status
# (ego_status 메시지 내용이 출력되는지 확인)
```

### 6-2. 송신 경로 (ROS2 → 시뮬)

ROS 2에서 보낸 명령이 시뮬레이터에 잘 전달되는지 확인합니다.

```bash
# 새 터미널
source ~/ros2_ws/install/setup.bash

# A. /cmd_vel_nav 토픽으로 속도 명령 발행 (-r 10: 초당 10회)
ros2 topic pub /cmd_vel_nav geometry_msgs/Twist '{linear: {x: 0.1}}' -r 10
```

- **최종 확인**: **시뮬레이터 상의 로봇이 0.1 m/s의 속도로 전진**하는지 눈으로 확인합니다.  
- **정지**: `Ctrl+C`로 `ros2 topic pub` 명령을 중지하면 로봇이 멈추는지 확인합니다.

---

## 부록) 레포 구조(요약)

```
ros2_ws/src
├─ bridge_bringup/
│  ├─ config/system.wsl.yaml
│  └─ launch/bridge.launch.py
├─ safety_bringup/ 
├─ sensor_bringup/            # (신규) 센서 정규화
│  ├─ config/ (sensors.yaml)
│  └─ sensor_bringup.launch.py
├─ udp_raw_bridge/
├─ udp_parsers_cpp/
├─ udp_tx_bridge/
├─ state_estimator/
└─ bridge_msgs/
```

**끝.**