# M1.5b 안전 체인 & 센서 브링업 작업 일지 (2025‑10‑31)

> 목적: **Unity 시뮬레이터(UDP) → ROS2 센서 파이프라인 안정화**와 **안전 체인(twist_mux → velocity_smoother → collision_monitor)** 1차 검증을 완료한다.  
> 범위: LiDAR 파서/정규화, TF/타임스탬프 정합, Safety Chain 적용 및 rqt_plot 기반 동작 확인, IMU/Camera 프레임 정렬, 설정 파일/런치 반영, 커밋 완료.  
> 다음 단계 예고: M2.0(오도메트리 고도화: IMU 융합 또는 EKF).

---

## 1) 작업 개요 (TL;DR)

- **LiDAR 파서**를 레거시(파이썬)와 동일 프로토콜로 맞추고, **frame_id = `laser_link`**로 통일.  
  - 페이로드 포맷: `3바이트/포인트(거리2 + 강도1)`, 단위 변환 **mm → m**.  
  - `scan_time`, `time_increment`는 시뮬에서 per‑beam 지연이 없어 **0**으로 둠(소비 노드 영향 없음).
- **scan_normalizer 노드**를 추가해 **180° 회전 보상**(Unity 보정) 및 **CW↔CCW 반전 옵션** 제공.  
  - 실제 데이터는 이미 CCW → `invert_cw_to_ccw: false`, **`rotate_180_degrees: true`** 로 확정.
- **TF/타임스탬프 문제**(RViz: *extrapolation into the future*) 원인 규명 및 수정.  
  - 모든 파서/정규화 노드가 **수신 시각으로 header.stamp**를 일관 적용(파이프라인 전역 동기화).  
  - `use_sim_time=false` 구성 확인, 절대/상대 토픽 경로 정리.
- **Safety Chain**(Nav2): `twist_mux → velocity_smoother → collision_monitor` 구성 검증.  
  - **감속·정지 폴리곤** 동작 OK.  
  - 문서에 없는 파라미터(`use_slowdown`, `use_stop`)는 제거하고 **`polygons[].action_type`**에만 의존.
- **IMU/Camera 파서**: `frame_id` 정리(`imu_link`, `camera_link`), 압축 이미지 발행.  
- **커밋 완료**: 패키지/런치/설정/코드/테스트 자산 반영.  
- **다음**: 오도메트리 고도화(M2.0) – `ego_status`(휠/속도 추정) + **IMU** 융합(EKF) 또는 LiDAR 매칭 기반 대안.

---

## 2) 환경 & 공통 규칙

- **좌표계**: ROS 표준 (x: 전방 +, y: 좌측 +, z: 위 +), **CCW = +회전**.  
- **프레임 명**: `odom` → `base_link` → `laser_link`(LiDAR), `imu_link`(IMU), `camera_link`(카메라).  
- **QoS**: RAW 입력은 `SensorDataQoS()`(BEST_EFFORT), 시각화/공유 토픽은 `depth=10` RELIABLE 권장.  
- **타임스탬프**: 모든 파서/정규화 노드는 **수신 시각**을 `header.stamp`로 사용하여 **동일 기준 시계** 보장.  
- **네임스페이스**: 설정 파일은 **상대 토픽명** 사용(런치에서 NS 적용).

---

## 3) LiDAR 파이프라인 정리

### 3.1 레거시 대비 변경점

- **레거시(파이썬)**:  
  - 스캔 각도: `angle_min=0`, `angle_inc=π/180`, `angle_max=360°` 전제.  
  - `ranges[-1]=0.0` 무효화, `scan_time`, `time_increment`는 보조 데이터에서 수신.
- **현행(C++)**:  
  - 페이로드: **3B/pt** (`d_lo`, `d_hi`, `intensity`), `num_points = data_len / 3`.  
  - 각 필드:  
    - `angle_min = 0.0`  
    - `angle_increment = 2π / N` (N=포인트 수에 자동 적응)  
    - `angle_max = angle_min + inc*(N-1)`  
    - `range_min=0.0`, `range_max=10.0`  
    - `scan_time=0.0`, `time_increment=0.0`  
  - **frame_id = `laser_link`**(통일)  
  - `ranges.back() = 0.0`(레거시 호환) 유지

```cpp
// 핵심 스니펫: lidar_parser.cpp
scan.header.stamp = this->now();         // 수신 시각
scan.header.frame_id = "laser_link";
scan.angle_min = 0.0;
scan.angle_increment = 2.0 * M_PI / num_points;
scan.angle_max = scan.angle_min + scan.angle_increment * (num_points - 1);
scan.range_min = 0.0;
scan.range_max = 10.0;
scan.scan_time = 0.0f;
scan.time_increment = 0.0f;
// payload: [d_lo, d_hi, intensity] * N, 단위 변환 mm→m
```
- **왜 `scan_time=0`?**: 시뮬 데이터는 빔 간 시간 지연이 없으므로 **의미 없는 값**을 넣는 대신 0으로 두어, RViz/소비 노드의 **시간 외삽 경고**를 예방.

### 3.2 scan_normalizer (정규화/보정)

- 파라미터:  
  - `frame_id: laser_link`  
  - `enable_normalize: false`  
  - `invert_cw_to_ccw: false` (**이미 CCW**)  
  - `rotate_180_degrees: true` (**Unity 보정**)  
- 처리: `ranges`/`intensities`를 **절반 기준으로 회전**, 각도 필드 `±π` 보정.  
- **시간 필드**: 입력 `msg.header.stamp` **그대로 전달**(중요).

```python
# 핵심 스니펫: scan_normalizer.py
out.header = msg.header              # 타임스탬프 보존
out.header.frame_id = self.frame_id

if self.rotate_180:
    mid = len(msg.ranges) // 2
    ranges_buf = msg.ranges[mid:] + msg.ranges[:mid]
    out.angle_min = wrap_angle(msg.angle_min + math.pi)
    out.angle_max = wrap_angle(msg.angle_max + math.pi)
# CCW 반전은 비활성 (invert=false)
```

---

## 4) TF/타임스탬프 장애 분석 & 조치

### 4.1 증상
- RViz/Collision Monitor에서 **Message Filter drop**, **extrapolation into the future** 에러 다수.  
- Collision Monitor 로그에 **스캔과 노드 시각 차이 수 초**로 표기되며 소스 무시.

### 4.2 원인
- 일부 노드에서 `header.stamp = now()`를 **각자 호출**하여 **노드 간 시계 차** 발생.  
- 절대 토픽(`/scan`) 사용 등으로 NS/시간 경로가 섞여 진단 난해.

### 4.3 조치
- 파서/정규화 노드는 **같은 시각 기준(수신 시각)**으로 `header.stamp` 일원화.  
- Collision Monitor의 `topic`을 **상대 경로**(`scan`)로 통일.  
- `use_sim_time` 전부 **false** 확인(실시간 시계 사용).

### 4.4 결과
- RViz 경고/에러 소거, Collision Monitor의 소스 무시 경고 소거.  
- rqt_plot 상 **계단식 속도 저하**는 정상(아래 5장 참조).

---

## 5) Safety Chain 동작 점검 (rqt_plot)

### 5.1 체인 구성
`cmd_vel_teleop` → **twist_mux** → **velocity_smoother** → **collision_monitor** → `cmd_vel`

### 5.2 관찰 포인트
- **즉시 반영 + 완만한 상승**: mux 출력은 teleop을 **즉시** 반영, smoother가 **점진** 반영.  
- **중간 하락 구간**: smoother의 가속/감속 프로파일 영향.  
- **장애물 접근 시 계단식 하락**: collision_monitor가 **폴리곤 진입 레벨**에 따라 출력을 단계적으로 **클램핑** → 정상.

### 5.3 설정 주의
- 문서에 없는 `use_slowdown`, `use_stop` 파라미터 **사용 금지**.  
- **감속/정지는 오직 `polygons[].action_type`**로 동작: `"slowdown"` / `"stop"`.

---

## 6) IMU & Camera

- **IMU 파서**: `frame_id = imu_link`, 쿼터니언+각속도+가속도 더블(LE) 파싱.  
  - 공분산은 미정(-1)로 세팅하여 소비 노드가 무시 가능.  
- **Camera 파서**: UDP 조립(MOR+LEN 프레이밍 → SOI/EOI Fallback),  
  - 출력: `sensor_msgs/CompressedImage` → `frame_id = camera_link`, `format="jpeg"`.

---

## 7) 오도메트리(현재) & 고도화 계획(M2.0)

### 7.1 현재: Twist‑only 적분
- 입력: `ego_status` (`geometry_msgs/Twist` 유사), **선속/각속도 스케일** 파라미터 제공.  
- 통합: 전진 오일러 적분, `publish_tf` 옵션.  
- 각속도 방향성: **ROS 기준(CCW=+)에 맞게 스케일 통일**.

### 7.2 고도화 후보(M2.0)
1) **robot_localization(EKF)** 기반 융합: `ego_status`(+휠) + **IMU** (필수), 필요시 LiDAR odom.  
2) **LiDAR Scan Matching + IMU**: `laser_scan_matcher`/SLAM 기반 odom 추정 + IMU bridging.  
→ 실제 프로젝트 지향: 먼저 **EKF**로 휠‑IMU 융합을 구축하고, 이후 필요시 LiDAR까지 확장.

---

## 8) 커밋(통합)

- `feat(package): sensor_bringup 패키지 스캐폴딩 및 의존성 추가`
- `feat(launch): sensor_bringup.launch, safety_chain.launch 추가`
- `feat(config): sensors.yaml, twist_mux.yaml, velocity_smoother.yaml, collision_monitor.yaml`
- `feat(node): scan_normalizer 구현(180° 회전·반전 옵션, 타임스탬프 보존)`
- `refactor(lidar): frame_id=laser_link, 3B/pt 파싱, scan_time=0`  
- `feat(imu/cam): imu_link/camera_link 프레임 통일, CompressedImage 발행`
- `fix(config): collision_monitor 문서 외 파라미터 제거, 상대 토픽 사용`
- `docs: 작업 로그 반영(본 문서)`

---

## 9) 재현/검증 체크리스트

```bash
# 1) 센서/안전 체인 런치
ros2 launch sensor_bringup sensor_bringup.launch.py
ros2 launch safety_bringup safety_chain.launch.py

# 2) 토픽/프레임 확인
ros2 topic list | egrep "scan$|scan_raw|cmd_vel"
ros2 topic echo /scan -n 1
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_link

# 3) 파라미터 점검
ros2 param get /lidar_parser use_sim_time        # False
ros2 param get /collision_monitor source_timeout  # 0.5 (예시)

# 4) rqt_plot
rqt_plot /cmd_vel_teleop/linear/x:/cmd_vel_smooth/linear/x:/cmd_vel/linear/x

# 5) RViz
# Fixed Frame=odom, LaserScan=scan, TF 시각 경고 없는지 확인
```

---

## 10) 남은 이슈 & 권고

- **scan_time / time_increment**: 0으로 둠. 하위 소비 노드가 이를 필수로 요구하면, **프레임 간 Δt**로 근사 산출 가능(후순위).  
- **정확한 각속도**: `ego_status`가 회전 추정치라면 오차 누적 가능 → **IMU 융합(EKF)** 권장.  
- **QoS 튜닝**: 실제 네트워크 상황에 따라 `KEEP_LAST` depth 조정, 백프레셔 모니터링.  
- **정적 TF**: `odom→base_link`, `base_link→laser_link/imu_link/camera_link` 정밀 보정(마운트 위치/각도).

---

## 11) 변경 파일

- `sensor_bringup/launch/sensor_bringup.launch.py`  
- `sensor_bringup/config/sensors.yaml` (frame_id·정규화 옵션)  
- `sensor_bringup/sensor_bringup/scan_normalizer.py`  
- `safety_bringup/launch/safety_chain.launch.py`  
- `safety_bringup/config/{twist_mux.yaml, velocity_smoother.yaml, collision_monitor.yaml}`  
- `udp_parsers_cpp/src/{lidar_parser.cpp, imu_parser.cpp, camera_jpeg_parser.cpp}`  
- `state_estimator/src/odom_publisher.cpp` (Twist‑only 적분, 스케일/시간 보호)

---

## 12) 오늘의 결론

- **M1.5 목표(센서 라이다 정합 + 안전체인 동작 검증)** **완료**.  
- RViz/Collision Monitor 관련 **시간·프레임 문제 해결**.  
- LiDAR 스캔 데이터는 **정상**, 안전 폴리곤에 따라 **감속·정지** 동작 확인.  
- 다음(M2.0): **오도메트리 고도화(EKF 융합)**로 회전 오차 축소 및 추정 안정화 추진.
