# 특화 프로젝트 진행 일지 (RAW 레이어까지)

> 본 문서는 Windows 11 + WSL2(Ubuntu 22.04) + ROS2 Humble 환경에서 **시뮬레이터 ↔ ROS2** 연동 및 **RAW 레이어 구축**까지의 작업을 일지/보고서 형태로 정리한 것입니다. 향후 Parser 레이어 작업의 기반 자료로 사용합니다.

---

## 0. 배경과 목표

### 문제의식
- 기존 ROS2 Eloquent + Python 기반 프로젝트가 **논리/복잡도/구조** 측면에서 유지보수 어려움.
- **절대 좌표 의존**, **TF/odometry/scan 불안정**, **FSM/노드 통신 난잡** 등의 문제가 존재.

### 결정사항
- **새 프로젝트**로 재구축 (현업 수준 품질 지향).
- **OS/플랫폼**: Windows 11 + **WSL2** (Ubuntu 22.04).
- **ROS2 배포판**: **Humble (LTS, 안정성 우선)**.
- **언어**: **C++ 중심**, 필요 시 Python 보조.
- **아키텍처(3-레이어)**:
  1) **RAW 수집**: UDP → ROS `ByteMultiArray` (재현/디버깅 가능하게 원본 유지)
  2) **Parser**: RAW → 표준/커스텀 메시지 (의미 있는 토픽으로 변환)
  3) **App**: SLAM/제어/시각화 (표준 토픽 기반)

---

## 1. 환경 구축 (WSL2 & ROS2)

### WSL2 설치/확인
- PowerShell:
  ```powershell
  wsl --install -d Ubuntu-22.04
  wsl -l -v
  ```
- Ubuntu 22.04.5 LTS 로그인 배너 확인.

### ROS2 Humble 설치
- 로케일/리포지토리/키 등록 후 설치, 환경 설정.
- 확인:
  ```bash
  ros2 --version
  ```

### VSCode
- WSL 원격 모드로 워크스페이스 열기.
- C/C++ 확장 설치(ms-vscode.cpptools).  

---

## 2. 네트워킹 구성 및 검증 (Windows ↔ WSL)

### IP/인터페이스
- Windows `ipconfig`:
  - 물리 NIC: `192.168.200.109/24`
  - vEthernet(WSL): `172.23.0.1/20`
- WSL `ip a`:
  - `eth0`: `172.23.15.206/20`

### Windows → WSL (수신 경로) 테스트
- WSL:
  ```bash
  sudo apt-get install -y netcat-openbsd
  nc -u -l -p 7802
  ```
- PowerShell:
  ```powershell
  $udp = New-Object System.Net.Sockets.UdpClient
  $bytes = [Text.Encoding]::ASCII.GetBytes("test")
  $udp.Send($bytes, $bytes.Length, "172.23.15.206", 7802)
  $udp.Close()
  ```
- 결과: **WSL에서 `test` 수신 확인**.

### 시뮬레이터 → WSL 포워딩
- 시뮬 설정 파일 점검:(문제 없음)
  - `.../SSAFYLauncher_SSAFY_Win_Data/networkInfo.txt`
  - `.../SaveFile/Network/.../NetworkInfo_TurtleBot.json` (각 포트 host/dest 매핑)
- **UDP 포워더(Python, Windows)** 실행으로 4포트 포워딩:
  - 수신 포트들: **7802, 8002, 8202, 8302**
  - 초기 포트 점유 오류(WinError 10048) 발생 → **프로세스 정리 후 재실행**으로 해결.
  - PowerShell 로그에 `from 192.168.200.109:7801 => #Enviroment$` 등 반복 확인.

### WSL → Windows (송신 경로) 테스트
- PowerShell에서 7601/7901/8101 UDP 수신기 실행.
- WSL에서 송신:
  ```bash
  echo "WSL->WIN 7601" | nc -u 172.23.0.1 7601
  ```
- 결과: **172.23.0.1**로 송신 시 정상 수신.  
  (LAN IP `192.168.200.109`로 송신 시 미수신 → **WSL→Windows는 172.23.0.1 사용 권장**)
- 방화벽: 해당 UDP 포트 인바운드 허용 필요.

---

## 3. RAW 레이어 구현 (ROS2 C++)

### 목표
- 시뮬에서 오는 UDP를 **원본 그대로** ROS 토픽으로 발행 (디코딩 없이 기록/재현 가능).

### 패키지: `udp_raw_bridge`
- 생성:
  ```bash
  cd ~/ros2_ws/src
  ros2 pkg create --build-type ament_cmake udp_raw_bridge --dependencies rclcpp std_msgs
  ```
- 노드: `src/udp_raw_node.cpp`
  - Boost.Asio `receive_from()`로 **블로킹 수신 루프**.
  - 파라미터화: `listen_port`(기본 7802), `topic_name`(기본 `env_info_raw`).
  - 메시지: `std_msgs::msg::ByteMultiArray` (원시 바이트 유지).
  - QoS: `rclcpp::SensorDataQoS().keep_last(10)`.
  - 로그 포맷 경고(`%d` vs int64_t)는 `RCLCPP_INFO_STREAM`으로 해결.

- CMakeLists.txt 발췌:
  ```cmake
  add_executable(udp_raw_node src/udp_raw_node.cpp)
  ament_target_dependencies(udp_raw_node rclcpp std_msgs)
  install(TARGETS udp_raw_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  ```

- 런치: `launch/raw_4ports.launch.py`
  - 4개 인스턴스 동시에 실행:
    - 7802 → `env_info_raw`
    - 8002 → `object_info_raw`
    - 8202 → `ego_status_raw`
    - 8302 → `stuff_info_raw`

- 빌드 & 실행:
  ```bash
  cd ~/ros2_ws
  colcon build
  source install/setup.bash

  # 한번에 4개
  ros2 launch udp_raw_bridge raw_4ports.launch.py

  # 또는 개별
  ros2 run udp_raw_bridge udp_raw_node --ros-args -p listen_port:=8202 -p topic_name:=ego_status_raw
  ```

- 검증:
  ```bash
  ros2 topic list
  ros2 topic echo /env_info_raw
  ros2 topic echo /object_info_raw
  ros2 topic echo /ego_status_raw
  ros2 topic echo /stuff_info_raw
  ```
  → **4포트 모두 RAW 데이터 수신/발행 정상** 확인.

---

## 4. Parser 레이어 계획 (다음 단계)

### 구조
- **레이어1(완료)**: `udp_raw_bridge` (RAW 수집)
- **레이어2(예정)**: `udp_parsers_cpp` (RAW→메시지)
  - 8202: `/ego_status_raw` → `/odom` + `/tf` (+ `ssafy_msgs/TurtlebotStatus` 유지)
  - 7802: `/env_info_raw` → `ssafy_msgs/EnviromentStatus`
  - 8002: `/app_status_raw` → `std_msgs/Int8MultiArray`
  - 8302: `/object_info_raw` → `visualization_msgs/MarkerArray` (→ 필요 시 `ssafy_msgs`로 교체)
  - (옵션) 9092 IMU, 9094 LiDAR

### 구현 원칙
- 헤더/길이/버전/엔디안 **검증 후 파싱**(불일치 시 드롭 + 스로틀 로그).
- 타임스탬프/프레임: ROS 표준(REP-103/105) 준수.
- 센서 QoS: `SensorDataQoS()` / 상태 QoS: Reliable KeepLast(10).
- **rosbag2**로 RAW 4토픽 녹화 후 리플레이로 파서 유닛 테스트.
- 라이다는 **표준 `sensor_msgs/LaserScan` 우선**, Pose는 별도 토픽으로 분리.

---

## 5. Git 정리

### 초기화 & 설정
```bash
cd ~/ros2_ws
git init
git config --global user.name "정한균"
git config --global user.email "j1113019@naver.com"
```

### `.gitignore` (VSCode에서 생성)
```gitignore
build/
install/
log/
__pycache__/
*.pyc
.vscode/
```

### 커밋 (한글 메시지, 쪼개서)
1) `.gitignore`
   - `chore: .gitignore 추가 (build, install, log 등 무시)`
2) 패키지 뼈대 (`CMakeLists.txt`, `package.xml`)
   - `chore: udp_raw_bridge 패키지 기본 구조 추가`
3) `udp_raw_node.cpp`
   - `feat: UDP RAW 수신 노드 구현`
4) `raw_4ports.launch.py`
   - `feat: 4개 포트 수신용 launch 파일 추가`

> 원격(origin) 추가/푸시는 별도 진행(레포 생성 후 `git remote add origin ...`, `git push -u origin master` 등).

---

## 6. 이슈 & 해결 메모

- **WSL→Windows UDP 경로**: `172.23.0.1`로 송신 시 정상, `192.168.200.109`로는 미수신 → WSL NAT 특성.
- **포워더 포트 충돌**: `WinError 10048` → 점유 프로세스 종료 후 재실행으로 해결.
- **시뮬 설정 A안(`Connect:true`)**: 로그인 실패 → **B안(포워더 경유)**로 정상 운영.
- **C++ 로그 포맷 경고**: `"%d"` vs `int64_t` → `RCLCPP_INFO_STREAM`로 수정.

---

## 7. 다음 액션 체크리스트

1) RAW 4토픽 **rosbag 녹화**(1~5분):
   ```bash
   ros2 bag record /env_info_raw /object_info_raw /ego_status_raw /stuff_info_raw -o raw_all
   ```
2) **8202 파서(C++)**: `/ego_status_raw` → `/odom` + `/tf` (RViz에서 즉시 확인)
3) **7802 / 8002 / 8302** 순서로 파서 확장
4) 라이다(9094): 표준 `LaserScan` 정밀화 → Pose는 별도 토픽
5) 필요 시 IMU(9092) 추가

---

## 부록: 디렉터리/파일 참조 (요약)
- `udp_raw_bridge/`
  - `src/udp_raw_node.cpp` – RAW 수신/발행 노드
  - `launch/raw_4ports.launch.py` – 4 인스턴스 실행
  - `CMakeLists.txt`, `package.xml`
- `ssafy_bridge/` – 기존 Python 브릿지(포맷 참조용)
- `ssafy_msgs/` – 커스텀 메시지 패키지

