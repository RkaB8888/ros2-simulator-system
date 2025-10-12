# 시뮬레이터 ↔ ROS2 브리지 셋업 매뉴얼 (Evergreen)

> 이 문서는 **처음 세팅하는 사람도** 이 매뉴얼만 보고 환경 구성 → 실행 → 테스트까지 완료할 수 있도록 상시 최신 상태로 유지됩니다.

---

## 1) 개요

Windows(유니티 시뮬레이터)와 WSL2/Ubuntu(ROS2) 사이를 **UDP**로 연결합니다.

- **수신 경로(시뮬 → ROS2)**  
  Windows 시뮬이 UDP로 내보낸 바이트 스트림을 WSL2에서 받아 `/..._raw` 토픽으로 퍼블리시 → 파서가 구조화 메시지로 변환

- **송신 경로(ROS2 → 시뮬)**  
  ROS2 토픽(`/cmd_vel`, `/hand_control`, `/iot_control`)을 정해진 바이너리 규격으로 직렬화하여 Windows 시뮬로 UDP 전송

### 패키지 역할

- **`bridge_bringup`** : **통합 설정 YAML**과 **일괄 런치** 제공 (본 매뉴얼의 기본 진입점)
- **`bridge_msgs`** : 프로젝트 전용 메시지들
- **`udp_raw_bridge`** : UDP → `std_msgs/ByteMultiArray` RAW 퍼블리셔
- **`udp_parsers_cpp`** : RAW → 의미 있는 토픽(ego/env/iot/object/imu/lidar/camera)으로 파싱
- **`udp_tx_bridge`** : `/cmd_vel`, `/hand_control`, `/iot_control` → UDP 직렬화/송신

아키텍처(개요):

```
[Unity Simulator on Windows]
   │  UDP (sensors/status)
   ▼
[WSL2 Ubuntu: udp_raw_bridge] --ROS2--> [udp_parsers_cpp] --> /ego_status, /scan, /imu, /image_jpeg/compressed, ...
   ▲
   │  UDP (commands)
[udp_tx_bridge  ←  /cmd_vel, /hand_control, /iot_control]
```

---

## 2) 요구사항

- **OS** : Ubuntu 22.04 (WSL2 권장) + Windows 10/11
- **ROS** : ROS 2 **Humble**
- **빌드 도구**
  ```bash
  sudo apt update
  sudo apt install -y build-essential cmake git python3-colcon-common-extensions
  ```
- **ROS 2 설치(요약)**
  ```bash
  sudo apt install -y ros-humble-desktop
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```
- **유틸**
  ```bash
  sudo apt install -y ros-humble-rqt ros-humble-rqt-common-plugins                       ros-humble-image-view ros-humble-rqt-image-view
  ```

---

## 3) 레포/워크스페이스 준비

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <이 레포 URL>
cd ~/ros2_ws
colcon build
source install/setup.bash
```

> 빌드가 꼬였으면 `colcon build --cmake-clean-cache` 후 재빌드.

---

## 4) 통합 설정(YAML)

통합 설정은 **`bridge_bringup/config/system.<env>.yaml`** 에 있습니다. (예: `system.wsl.yaml`)

예시:
```yaml
# ---- RX (udp_raw_bridge) ----
udp_rx_env:
  ros__parameters: { listen_port: 7802, topic_name: env_info_raw }
udp_rx_iot:
  ros__parameters: { listen_port: 8002, topic_name: iot_status_raw }
udp_rx_ego:
  ros__parameters: { listen_port: 8202, topic_name: ego_status_raw }
udp_rx_object:
  ros__parameters: { listen_port: 8302, topic_name: object_info_raw }
udp_rx_imu:
  ros__parameters: { listen_port: 9092, topic_name: imu_raw }
udp_rx_camera:
  ros__parameters: { listen_port: 1232, topic_name: camera_jpeg_raw }
udp_rx_lidar:
  ros__parameters: { listen_port: 9094, topic_name: lidar_raw }

# ---- TX (udp_tx_bridge) ----
udp_tx_cmd_vel:
  ros__parameters: { remote_ip: 172.23.0.1, remote_port: 7601 }
udp_tx_hand_control:
  ros__parameters: { remote_ip: 172.23.0.1, remote_port: 7901 }
udp_tx_iot_control:
  ros__parameters: { remote_ip: 172.23.0.1, remote_port: 8101 }
```

> **중요:** YAML의 키는 런치에서 넘겨주는 **노드 이름**과 일치해야 합니다. (예: `udp_rx_env`, `udp_tx_cmd_vel` 등)

---

## 5) Windows 네트워크 설정

### 5.1 IP 파악

- **WSL2 Ubuntu IP (eth0)**  
  ```bash
  ip -4 addr show eth0 | grep inet
  # 예: 172.23.15.206/20 → WSL2 IP는 172.23.15.206
  ```

- **Windows: vEthernet (WSL) 인터페이스 IP**  
  PowerShell:
  ```powershell
  Get-NetIPAddress -InterfaceAlias "vEthernet (WSL (Hyper-V firewall))" -AddressFamily IPv4
  # 예: 172.23.0.1
  ```

- **Windows: 유선/무선 LAN IP**  
  ```powershell
  ipconfig
  # 예: 192.168.200.107
  ```

### 5.2 시뮬레이터 UDP 설정 가이드

- **센서/상태(시뮬 → ROS2) 채널**  
  - **Destination IP** : **WSL2 IP(eth0)** → 예) `172.23.x.x`  
  - **Host(Local) IP**  : Windows가 송신에 사용할 로컬 인터페이스 (환경에 맞게 선택). 일반적으로 LAN IP(`192.168.x.y`) 또는 vEthernet(`172.23.0.1`) 중 하나를 사용합니다.
  - **포트** : 7802(env), 8002(iot), 8202(ego), 8302(object), 9092(imu), 9094(lidar), 1232(camera)

- **명령(ROS2 → 시뮬) 채널** — **실제 동작 확인된 설정(권장)**  
  - **Host(Local) IP**  : **vEthernet (WSL) IP = `172.23.0.1`**  
  - **Destination IP** : **WSL2 IP(eth0) = `172.23.x.x`**  
  - **포트** : 7601(`/cmd_vel`), 7901(`/hand_control`), 8101(`/iot_control`)

> 위와 같이 두 채널을 **모두 172.23/16 대역**으로 통일하면, WSL2와의 라우팅이 확실해져 **ROS2 → 시뮬 쪽 명령이 먹지 않는 문제**를 방지할 수 있습니다.

### 5.3 방화벽(Windows)

PowerShell(관리자):
```powershell
# 수신(RX) 포트 허용
$rx = 7802,8002,8202,8302,9092,9094,1232
$rx | ForEach-Object { New-NetFirewallRule -DisplayName "UDP In $_" -Direction Inbound -Protocol UDP -LocalPort $_ -Action Allow }

# 명령(TX) 포트 허용
$tx = 7601,7901,8101
$tx | ForEach-Object { New-NetFirewallRule -DisplayName "UDP In $_" -Direction Inbound -Protocol UDP -LocalPort $_ -Action Allow }
```

---

## 6) 실행 방법

### 6.1 통합 실행(권장)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch bridge_bringup bridge.launch.py
```

선택 인자:
```bash
ros2 launch bridge_bringup bridge.launch.py   config_file:=/absolute/path/to/system.<env>.yaml   respawn_raw:=false respawn_parsers:=true respawn_tx:=true
```

- RAW 레이어는 포트 재바인드 문제를 피하려고 **respawn=false** 권장
- Parsers/TX 레이어는 **respawn=true** 기본

### 6.2 레이어별 개별 실행(선택)

```bash
# RAW
ros2 launch udp_raw_bridge raw_6ports.launch.py config_file:=<yaml>

# Parsers
ros2 launch udp_parsers_cpp parsers_all.launch.py

# TX
ros2 launch udp_tx_bridge tx_all.launch.py config_file:=<yaml>
```

---

## 7) 동작 확인/테스트

### 7.1 수신 경로(시뮬 → ROS2)

```bash
# RAW 토픽 존재/Hz 확인
ros2 topic list | grep raw
ros2 topic hz /camera_jpeg_raw

# 파싱 결과 1회 확인
ros2 topic echo --once /ego_status
```

- 카메라는 `rqt` → **rqt_image_view** 에서 `/image_jpeg/compressed` 선택

### 7.2 송신 경로(ROS2 → 시뮬)

1) ROS에서 명령 퍼블리시
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 0.0}}' -1
```

2) Windows PowerShell에서 수신 확인(옵션)
```powershell
$u = New-Object System.Net.Sockets.UdpClient(7601)
$ep = New-Object System.Net.IPEndPoint([IPAddress]::Any,0)
while ($true) {
  $b = $u.Receive([ref]$ep)
  ($b | ForEach-Object { '{0:X2}' -f $_ }) -join ' '
}
# "#Turtlebot_cmd$" ... 08 00 00 00 00 00 00 00 00 00 80 3F 0D 0A  (예시)
```

- 시뮬레이터에서 실제 **터틀봇 이동**과 **ego_status 속도 변화**로 최종 검증

---

## 8) 프로토콜(명령 TX) 규격

> **엔디언 = Little Endian** 가정(x86_64). 모든 프레임 끝에는 **CRLF(0x0D 0x0A)** 추가.

- **`/cmd_vel`** (포트 7601)  
  `"#Turtlebot_cmd$"`(15B) + `int32 len=8` + `int32 zero*3` + `float32 linear` + `float32 angular` + `CRLF`

- **`/hand_control`** (포트 7901)  
  `"#hand_control$"`(14B) + `int32 len=9` + `int32 zero*3` + `uint8 mode` + `float32 distance` + `float32 height` + `CRLF`

- **`/iot_control`** (포트 8101)  
  `"#Appliances$"`(12B) + `int32 len=17` + `int32 zero*3` + `uint8[17] payload` + `CRLF`

> 실제 구현은 `udp_tx_bridge` 의 각 노드에서 수행됩니다. (기본값 파라미터 **없음**: 필수 파라미터는 YAML로만 주입)

---

## 9) QoS/성능 정책

- RAW 퍼블리시/구독: `SensorDataQoS()` (BestEffort, keep_last(10))
- 결과 토픽 퍼블리시: 보통 Reliable + depth 10
- UDP 커널 수신 버퍼(WSL): 4MB, 사용자 버퍼: 128KB
- 카메라 파서: **MOR+길이** 우선, 실패 시 **SOI/EOI** 백업 스캔

---

## 10) 트러블슈팅

- **런치 직후 노드가 즉시 종료 + "parameter ... invalid type"**  
  → 런치에 `config_file:=.../system.<env>.yaml` 이 제대로 전달됐는지 확인.  
  → YAML의 키(노드명)와 런치의 `name=` 이 일치해야 합니다.

- **포트 점유/재바인드 실패**  
  → RAW 레이어는 `respawn=false` 권장. 포트 잡고 있던 터미널을 모두 종료 후 재실행.

- **시뮬에서 패킷은 보이는데 명령이 먹지 않음**  
  → **명령 채널의 Host(Local) IP 를 `172.23.0.1`(vEthernet WSL)로 설정**하십시오.  
  → WSL IP(eth0)가 바뀌었는지 확인하고 YAML/시뮬 Dest IP를 함께 갱신.

- **네트워크 점검**  
  - Windows: `netstat -ano -p udp | findstr :<port>`  
  - Ubuntu: `sudo tcpdump -i eth0 udp port <port> -n -vv -c 5`  
  - WSL IP: `hostname -I`

---

## 11) 일괄 런치/버전관리 팁

- 메인 명령:  
  ```bash
  ros2 launch bridge_bringup bridge.launch.py
  ```
- 환경별 YAML만 교체하면 **다른 PC에서도 동일하게 실행**됩니다.
- 커밋은 **작업 단위**로, 메시지에 목적/변경/검증(테스트 방법)을 적습니다.

---

## 부록: 레포 구조(요약)

```
ros2_ws/src
├─ bridge_bringup/            # 통합 설정(YAML), 일괄 런치
│  ├─ config/system.wsl.yaml
│  └─ launch/bridge.launch.py
├─ bridge_msgs/               # 커스텀 메시지
├─ udp_raw_bridge/            # UDP→ROS RAW 브리지
│  └─ launch/raw_6ports.launch.py
├─ udp_parsers_cpp/           # RAW 파서
│  └─ launch/parsers_all.launch.py
└─ udp_tx_bridge/             # ROS2→시뮬 명령 송신
   └─ launch/tx_all.launch.py
```

---

### 끝.