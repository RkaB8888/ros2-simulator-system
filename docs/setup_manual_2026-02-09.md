# 시뮬레이터 ↔ ROS2 브리지 셋업 매뉴얼

> 이 문서는 **새로운 환경**에서 프로젝트를 구동하기 위한 **설치 및 실행 가이드**입니다.  
> Ubuntu 22.04 (WSL2) 기반의 ROS 2 **Humble** 환경을 기준으로 작성되었습니다.

---

## 1. 개요 및 전제 조건

본 프로젝트는 Windows 기반 시뮬레이터와 WSL2 기반 ROS 2 간의 UDP 통신 브리지를 구축합니다.

- **Host OS**: Windows 10/11 (시뮬레이터 구동)
- **Guest OS**: Ubuntu 22.04 on WSL2 (ROS 2 구동)
- **ROS Distro**: Humble Hawksbill
- **Workspace**: `~/ros2_ws`

---

## 2. 설치 (Installation)

### 2-1. 기본 유틸 및 빌드 도구

런치 파일 실행(`import yaml`)과 빌드에 필요한 도구들을 설치합니다.

```bash
sudo apt update
sudo apt install -y build-essential cmake git curl \
                    python3-pip python3-yaml \
                    python3-colcon-common-extensions
```

### 2-2. ROS 2 Humble Desktop

```bash
sudo apt install -y ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2-3. 프로젝트 필수 의존성 (Critical)

Nav2 스택, Lifecycle 관리, **위치 추정(EKF)**을 위한 필수 패키지입니다. **설치되지 않을 경우 시스템이 실행되지 않습니다.**

```bash
sudo apt install -y ros-humble-twist-mux \
                    ros-humble-nav2-velocity-smoother \
                    ros-humble-nav2-collision-monitor \
                    ros-humble-nav2-lifecycle-manager \
                    ros-humble-robot-localization
```

### 2-4. 디버깅 및 시각화 도구 (Recommended)

데이터 시각화(Image, Plot) 및 노드 그래프 확인을 위한 도구입니다.

```bash
sudo apt install -y ros-humble-rviz2 \
                    ros-humble-rqt \
                    ros-humble-rqt-common-plugins \
                    ros-humble-rqt-image-view \
                    ros-humble-rqt-plot \
                    ros-humble-plotjuggler-ros \
                    ros-humble-rosbag2-storage-mcap
```

---

## 3. 빌드 (Build)

소스 코드를 다운로드하고 빌드합니다. 파이썬 스크립트 수정 시 재빌드 과정을 최소화하기 위해 심볼릭 링크 옵션을 사용합니다.

```bash
# 1) 워크스페이스 생성 및 클론
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <REPOSITORY_URL>

# 2) 빌드 (Clean Build 권장)
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install --cmake-clean-cache

# 3) 환경 변수 로드
source install/setup.bash
```

---

## 4. 네트워크 설정 (Network Configuration)

WSL2와 Windows 간의 통신을 위해 IP 설정이 필요합니다.

### 4-1. IP 주소 확인

1.  **WSL2 IP (Ubuntu)**: 데이터 수신용 (RX)
    ```bash
    ip -4 addr show eth0 | grep inet
    # 출력 예: 172.23.15.206
    ```

2.  **Windows IP (PowerShell)**: 데이터 송신용 (TX)
    ```powershell
    Get-NetIPAddress -InterfaceAlias "vEthernet (WSL (Hyper-V firewall))" -AddressFamily IPv4
    # 출력 예: 172.23.0.1
    ```

### 4-2. ROS 2 설정 파일 수정

`src/bridge_bringup/config/system.wsl.yaml` 파일을 열어 `remote_ip`를 **Windows IP**로 변경합니다.

```yaml
# system.wsl.yaml 예시
udp_tx_cmd_vel:
  ros__parameters:
    remote_ip: "172.23.0.1"  # <--- 위에서 확인한 Windows IP 입력
    remote_port: 7601
```

### 4-3. 시뮬레이터 및 방화벽 설정

1.  **시뮬레이터 설정**:
    * **Destination IP (Sensor)**: WSL2 IP 입력
    * **Host IP (Command)**: Windows IP 입력
2.  **Windows 방화벽 (PowerShell 관리자)**:
    ```powershell
    # UDP 포트 허용 (RX/TX 전체)
    $ports = 7802,8002,8202,8302,9092,9094,1232,7601,7901,8101
    $ports | ForEach-Object { New-NetFirewallRule -DisplayName "ROS2 UDP $_" -Direction Inbound -Protocol UDP -LocalPort $_ -Action Allow }
    ```

---

## 5. 실행 (Usage)

통합 런치 파일을 통해 통신, 센서 처리, EKF 위치 추정, 안전 제어 노드를 일괄 실행합니다.

```bash
# 1) 환경 변수 설정
source ~/ros2_ws/install/setup.bash

# 2) 메인 런치 실행
ros2 launch bridge_bringup bridge.launch.py log_level_raw:=info
```

> **참고**: `lifecycle_manager`가 포함되어 있어, 실행 후 자동으로 모든 노드가 활성화(Active) 상태로 전환됩니다.

---

## 6. 테스트 (Verification)

### 6-1. 수신 테스트 (Sensor)

```bash
# LiDAR 데이터 수신 확인 (20~30Hz 권장)
ros2 topic hz /scan
```

### 6-2. 송신 테스트 (Command)

```bash
# 로봇 전진 명령 (0.1 m/s)
ros2 topic pub /cmd_vel_nav geometry_msgs/Twist '{linear: {x: 0.1}}' -r 10
```

---

## 7. 시스템 구조 (Architecture)

### 7-1. 패키지 구성

| 패키지명 | 역할 | 비고 |
| :--- | :--- | :--- |
| **bridge_bringup** | 전체 시스템 통합 실행 (Launch) | Entry Point |
| **udp_raw_bridge** | UDP 패킷 송수신 | Best Effort |
| **udp_parsers_cpp** | 바이너리 데이터 파싱 | Optimized C++ |
| **sensor_bringup** | LiDAR 정규화 및 TF 관리 | Scan Normalizer |
| **state_estimator** | Odometry 적분 및 **EKF 센서 퓨전** | Robot Localization |
| **safety_bringup** | 속도 평활화 및 충돌 방지 | Nav2 Based |

### 7-2. QoS 정책

시스템 성능 최적화를 위해 데이터 특성에 따른 QoS 정책이 적용되어 있습니다.

* **Sensor Data (LiDAR/Camera)**: `Best Effort` (Low Latency)
* **State Data (Odom/Status)**: `Reliable` (Data Integrity)

### 7-3. 데이터 흐름도

```text
[Simulator] 
    │ (UDP)
    ▼
[udp_raw_bridge] 
    │ (Topic: *_raw)
    ▼
[udp_parsers_cpp] 
    │ (Topic: scan_raw, ego_status, imu_raw)
    ▼
[sensor_bringup] / [state_estimator]
    │ (Wheel Odom + IMU -> EKF Fusion)
    │ (Topic: scan, odom)
    ▼
[safety_bringup] (TwistMux -> Smoother -> CollisionMonitor)
    │ (Topic: cmd_vel)
    ▼
[udp_tx_bridge] 
    │ (UDP)
    ▼
[Simulator]
```