# 프로젝트 작업 일지 — 2025-10-13

## 1) 개요
- **목표**
  - 송신 브릿지(`udp_tx_bridge`) 완성 및 레거시 UDP 프레이밍 100% 재현
  - 시스템 전역 설정을 **단일 YAML**로 통합(수신/송신 IP·포트 모두)
  - 레이어 일괄 기동용 **통합 런치(`bridge_bringup/launch/bridge.launch.py`)** 구축
  - 모든 노드에서 **기본값 제거 + YAML 미지정 시 즉시 실패(fail-fast)** 정책 확립
  - WSL2↔Windows 네트워킹 경로 확정 및 시뮬레이터 연동 검증

---

## 2) 오늘 변경/추가된 산출물

### A. 새 패키지: `bridge_bringup` (ament_python)
- **역할**: 통합 설정/런치 제공(패키지 간 공통 YAML 한 곳에서 관리)
- **구성**
  - `config/system.wsl.yaml`
    - RX(udp_raw_bridge): `udp_rx_env / udp_rx_iot / udp_rx_ego / udp_rx_object / udp_rx_imu / udp_rx_camera / udp_rx_lidar`  
      → 각 `listen_port`, `topic_name` 정의
    - TX(udp_tx_bridge): `udp_tx_cmd_vel / udp_tx_hand_control / udp_tx_iot_control`  
      → 각 `remote_ip`, `remote_port` 정의
  - `launch/bridge.launch.py`
    - 인자: `config_file`(기본값=위 YAML), `respawn_raw=false`, `respawn_parsers=true`, `respawn_tx=true`
    - 내부에서 각 레이어 런치 포함:
      - `udp_raw_bridge/launch/raw_6ports.launch.py`
      - `udp_parsers_cpp/launch/parsers_all.launch.py`
      - `udp_tx_bridge/launch/tx_all.launch.py`
  - `setup.py`: `resource/bridge_bringup`, `launch/*.launch.py`, `config/*.yaml` 설치 등록
  - `package.xml`: `ament_python` 빌드, `launch/launch_ros/ament_index_python` 실행 의존, **MIT** 라이선스

### B. `udp_tx_bridge` (송신)
- **정책**: **기본 파라미터 제거**, YAML에서만 값 주입. 런타임 파라미터 변경 **차단**.
- **노드**
  1) `udp_tx_cmd_vel_node.cpp`  
     - **레거시 프레이밍 정확 재현**
       - 헤더: `"#Turtlebot_cmd$"` **15B(패딩 없음)**
       - 길이: `int32 LE = 8`
       - AUX: `int32 LE 0` x 3
       - 페이로드: `float32 LE linear`, `float32 LE angular` (8B)
       - 꼬리: `"
"`(CRLF)
     - 구독: `/cmd_vel`, QoS `KeepLast(1).BestEffort`
     - 파라미터: `remote_ip`, `remote_port`(필수, 미지정 시 즉시 종료)
  2) `udp_tx_hand_control_node.cpp`  
     - 레거시 프레이밍
       - 헤더 `"#hand_control$"`(14B), 길이 `9`, AUX `0*3`,
       - 페이로드: `mode:uint8 + distance:float32 + height:float32`,
       - CRLF
     - 구독: `/hand_control` (메시지 `bridge_msgs/msg/HandControlCommand`)
  3) `udp_tx_iot_control_node.cpp`  
     - 레거시 프레이밍
       - 헤더 `"#Appliances$"`(12B), 길이 `17`, AUX `0*3`,
       - 페이로드: `uint8[17]` 그대로, CRLF
     - 구독: `/iot_control` (메시지 `bridge_msgs/msg/IotControl`)
- **런치**: `launch/tx_all.launch.py`  
  - 인자: `config_file`(기본: `bridge_bringup/config/system.wsl.yaml`), `respawn=true`  
  - 각 노드에 **YAML 경로만 전달**(개별 포트/IP는 코드에 없음)

### C. `udp_raw_bridge` (수신)
- **노드**: `src/udp_raw_node.cpp`
  - **기본 파라미터 제거** → `NodeOptions().automatically_declare_parameters_from_overrides(true)` 사용
  - 필수 파라미터 미지정/유효성 실패 시 **즉시 종료(fail-fast)**
  - 런타임 파라미터 변경 **차단**
  - UDP 바인딩: `reuse_address`, 바인드 실패 시 즉시 종료
  - RX 버퍼: 커널 4MB, 사용자 128KB
  - 퍼블리시 QoS: `SensorDataQoS().keep_last(10)` (BestEffort)
- **런치**: `launch/raw_6ports.launch.py`
  - 인자: `config_file`(기본: bringup YAML), `respawn=false`(포트 충돌 방지)
  - 노드 이름을 **정보 전달용**(`udp_raw_{port}`)으로 유지하되, 실제 포트/토픽은 **YAML만 신뢰**

### D. `udp_parsers_cpp`
- IP/Port 사용 없음(변경 없음). 런치에서 `respawn=true` 확인.  
  (카메라 JPEG 파서 포함: MOR+LEN 우선, SOI/EOI 백업 스캐너 동작 재확인)

### E. 메타/패키징 정리
- 각 `package.xml`(메시지 패키지 제외) 에 **`exec_depend`로 `launch`, `launch_ros`** 추가(런치 직접 실행 보장)
- 라이선스/버전/설명 갱신(MIT, 0.1.0 등)
- 테스트 스크립트(`test_copyright.py`, `test_flake8.py`, `test_pep257.py`)
  - **커밋/유지**: ament 기본 템플릿으로 정적 점검 시 사용

---

## 3) 네트워킹 트러블슈팅 & 해결

1) **증상**  
   - `/cmd_vel` 퍼블리시 시 Windows PowerShell에서 바이트 수신은 되나, 시뮬레이터 터틀봇이 미동작.

2) **원인 A: 프레이밍 불일치**  
   - 이전 C++ 송신은 “16B 패딩 헤더 + payload” 방식.  
   - 레거시는 “**15B 헤더 + 길이(4) + AUX(12) + payload + CRLF**”.
   - → `udp_tx_cmd_vel_node` 프레이밍을 레거시대로 수정 후 재빌드.

3) **원인 B: WSL2↔Windows 경로**  
   - WSL2 IP: `172.23.15.206` (WSL 가상 NIC)  
   - Windows vEthernet(WSL) IP: `172.23.0.1`  
   - **해결 세팅(시뮬레이터 ‘cmd_vel’ 네트워크 탭)**  
     - **Host(바인드/송신원)**: `172.23.0.1` (Windows 쪽 vEthernet)  
     - **Destination(목적지)**: `172.23.15.206` (WSL IP)  
   - 이 설정으로 **WSL→Windows, Windows→WSL** 경로가 NAT 내부에서 안정화.  
   - 과거엔 로컬호스트 포워딩/물리 NIC(192.168.200.107) 등을 섞어 써서 간헐 실패.  
   - 이번엔 **포워딩 불요**, vEthernet 경로만으로 양방향 통신 성립.

4) **포트 충돌/방화벽**  
   - 실행 중 포트 점유로 시뮬레이터 연결 실패 사례 발견 → 모든 터미널 종료 후 재연결로 해결.  
   - Windows 방화벽 인바운드 규칙(7601/7901/8101/… 등) 추가 확인.  
   - `netstat -ano -p udp`로 포트 점유 프로세스 확인 습관화.

---

## 4) 검증 결과

- `ros2 launch bridge_bringup bridge.launch.py` 기동 시,  
  - RAW 7개 포트 수신 노드 정상 바인딩/퍼블리시 로그 확인
  - 파서 7개 노드 정상 구동, `/scan`, `/imu`, `/image_jpeg/compressed` 등 실시간 표출
  - TX 3개 노드가 bringup YAML에서 IP/Port 수신, 시작 로그에 대상 IP/Port 출력
- Windows PowerShell로 **수신 바이트 구조 확인** (헤더/길이/AUX/페이로드/CRLF 매칭)
- 시뮬레이터 **터틀봇 실제 이동 확인**(Twist 퍼블리시 → 이동, `/ego_status` 값 변동 확인)

---

## 5) 자주 쓰는 명령 스냅샷

```bash
# 전체 빌드 & 환경
cd ~/ros2_ws
rm -rf build/ install/ log/   # 필요 시
colcon build
source install/setup.bash

# 통합 브릿지
ros2 launch bridge_bringup bridge.launch.py
# (환경 교체) ros2 launch bridge_bringup bridge.launch.py config_file:=/path/to/system.<env>.yaml

# 송신 테스트
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 1.0}}' -1
```

---

## 6) 커밋(오늘 분)

- **bridge_bringup**
  - feat(bringup): 통합 YAML/런치 도입 — 시스템 전역 IP/Port 단일 관리, 레이어 일괄 기동(bridge.launch.py)
  - chore(packaging): setup.py data_files 등록, package.xml 실행의존/라이선스/버전 정리
  - docs(config): `config/system.wsl.yaml` 초판

- **udp_tx_bridge**
  - feat(tx/cmd_vel): 레거시 UDP 프레이밍 재현(15B헤더+len+aux+payload+CRLF), 기본값 제거+fail-fast+param lock
  - feat(tx/hand,iot): 레거시 프레이밍 적용(길이/AUX/CRLF), 기본값 제거+fail-fast+param lock
  - feat(launch): `tx_all.launch.py` — bringup YAML만으로 기동

- **udp_raw_bridge**
  - refactor(rx): 기본값 제거 + 자동 파라미터 선언 + fail-fast + param lock
  - chore(launch): `raw_6ports.launch.py`가 bringup YAML만 주입받도록 수정, respawn=false
  - chore(pkg): package.xml 설명/라이선스/exec_depend 보강

---

## 7) 결정 사항
- **“설정은 YAML만”**: 코드 기본값 전면 제거, 파라미터 미지정 시 즉시 종료
- 레이어별 **respawn 정책**: RX=false, Parsers=true, TX=true
- **네트워크 기준**: Windows vEthernet(WSL)=`172.23.0.1`, WSL IP=`172.23.15.206`  
  → 시뮬레이터 Host=172.23.0.1, Dest=172.23.15.206 원칙 유지

---

## 8) 다음 할 일
- 환경별 YAML 템플릿 추가(예: `system.labpc.yaml`, `system.laptop.yaml`)
- `udp_tx_*` 노드에 **주기 제한/최신값 유지**(옵션) 및 간단한 송신 rate 로그 추가 검토
- RAW 레이어 포트 충돌 감지/에러 메시지 고도화
- bringup용 README 초안(네트워크 가이드 포함: vEthernet/WSL IP 배치도)
