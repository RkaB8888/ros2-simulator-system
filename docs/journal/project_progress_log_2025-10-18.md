# 프로젝트 작업 일지 — 2025-10-18

## 1) 개요
- **목표**
  - M1 달성: **Twist-only 오도메트리 파이프라인 구현**(`state_estimator/odom_publisher`)
  - Unity 시뮬레이터 기준 **단위/스케일 정합** 및 `/odom`·TF 발행 검증
  - 브릿지 런치 전반에 **네임스페이스(ns) 템플릿** 도입(멀티로봇/멀티 인스턴스 대비)
  - 런치/파라미터 구성 원칙 정립: **코드는 상대이름, 런치/파라미터로 주입**

---

## 2) 오늘 변경/추가된 산출물

### A. 패키지 `state_estimator`
- **노드**: `src/state_estimator/src/odom_publisher.cpp` 신규 구현
  - 입력: `bridge_msgs/msg/TurtlebotStatus` (내부 `geometry_msgs/Twist` 사용)
  - 적분: **Twist-only**(선속도 `vx`, 각속도 `wz`) → 오일러 적분으로 `(x, y, yaw)` 추정
  - 파라미터:
    - `frame_id="odom"`, `child_frame_id="base_link"`, `publish_tf=true`
    - `max_dt_sec=2.0`(긴 간격 클램프), `publish_rate=0.0`(입력 콜백 동기 발행)
    - `linear_scale=1.0`, `angular_scale=2.222222` *(Unity 기준 보정값)*
    - `initial_yaw_deg=0.0`
    - `covariance.{x=0.02, y=0.02, yaw=0.04}` *(초기값)*
  - 구현 디테일:
    - **timestamp 보호 & 클램프**: `dt > max_dt_sec` 시 `dt=max_dt_sec` 적용
    - **yaw 정규화**: `atan2(sin(yaw), cos(yaw))`
    - **TF 발행**: `odom → base_link` (map ↔ odom 은 타 노드 담당)
    - **공분산**: 단순 초기값 세팅(추후 센서퓨전 시 재조정)
- **설정**: `params/odom_publisher.yaml`
  ```yaml
  odom_publisher:
    ros__parameters:
      frame_id: "odom"
      child_frame_id: "base_link"
      publish_tf: true
      max_dt_sec: 2.0
      publish_rate: 0.0
      initial_yaw_deg: 0.0
      linear_scale: 1.0
      angular_scale: 2.222222
      covariance: { x: 0.02, y: 0.02, yaw: 0.04 }
  ```
- **런치**: `launch/odom.launch.py`
  - 네임스페이스/리맵 패턴을 지원하도록 정리(현재 기본값은 로컬 이름 유지)

### B. 브릿지 런치 템플릿 통일
- **udp_raw_bridge/launch/raw_6ports.launch.py**
  - `ns` 인자 추가(기본 `''`), 기타는 기존 유지(설정은 YAML `system.wsl.yaml` 신뢰)
- **udp_parsers_cpp/launch/parsers_all.launch.py**
  - `ns` 인자 추가, 파서는 파라미터/리맵 기본 미사용(필요 시 선택적 훅 추가 예정)
- **udp_tx_bridge/launch/tx_all.launch.py**
  - `ns` 인자 추가, `config_file` 전달 유지
- **bridge_bringup/launch/bridge.launch.py**
  - 상위에서 **`ns`를 한 번만 받아** raw/parsers/tx 런치에 **전달**
  - `config_file`, `respawn_*` 인자 전달 구조 유지

### C. CMake/패키징 정리(요지)
- `state_estimator/CMakeLists.txt`
  - `ament_target_dependencies(odom_publisher rclcpp nav_msgs geometry_msgs tf2 tf2_ros bridge_msgs)`
  - 경고 옵션은 필요시 재도입 가능(`-Wall -Wextra -Wpedantic`)
- `package.xml`
  - `bridge_msgs` 의존 추가, 런치/파라미터 설치 항목 점검

---

## 3) 진단 & 트러블슈팅 기록

- **/odom 회전이 반으로 보이던 문제**: 원인 2가지
  1) `/ego_status` 간헐 큰 간격(최대 1.536s) → `max_dt_sec=0.2`로 **과도 클램프**되어 회전 누적 손실
  2) `/ego_status.twist.angular.z`가 ±1.0 **정규화 보고치** → 물리 yaw 속도와 **스케일 불일치**
  - **조치**: `max_dt_sec=2.0` 상향 + `angular_scale=2.222222` 보정 → **한 바퀴 정합**
- **BrokenPipe 에러**: `ros2 topic echo ... | head` 파이프 종료로 인한 정상 메시지 (무시 가능)
- **시각화 팁**
  - MatPlot/rqt_plot: 쿼터니언(RVIZ/Odometry) 관찰, 필요시 **yaw 토픽** 보조 생성
  - `ros2 topic hz /ego_status`로 주기/간격 진단
  - `/odom` Fixed Frame 확인, TF 충돌 여부 점검

---

## 4) 검증 결과
- `/ego_status` → `/odom` 변환 정상, RViz에서 궤적/방향각(Arrow) 움직임 정상
- 제자리 정속 회전 시 yaw 누적 ≈ **2π rad**(오차 미소)
- `odom → base_link` TF 지속 발행, 다른 퍼블리셔와 충돌 없음 확인
- 런치 오케스트레이션: `bridge_bringup/bridge.launch.py ns:=robot1` 등 **멀티 인스턴스 기동 정상**

---

## 5) 오늘 결정 사항
- 브릿지 계열 런치에는 **ns만 기본 제공**, remap 훅은 **필요 시 개별 노드에만** 열기로 함
- `state_estimator`는 **상대 이름 사용 + 런치/파라미터로 주입** 원칙 채택
- `map ↔ odom` 변환은 SLAM/Localization 노드에서 담당(본 노드는 **odom↔base_link만** 발행)
- 공분산은 테스트용 초기값 유지(센서퓨전 도입 시 재조정)

---

## 6) 다음 할 일 (M2 예고)
- **IMU 융합 오도메트리**: `/imu`(자이로 yaw rate) 보강, EKF(`robot_localization`) 적용
- 공분산 파라미터를 YAML로 더 노출/조정 가능하게 개선
- 상태 추정 품질 로깅(회전/이동 누적 오차 추적) 및 간단한 자가 진단

---

## 7) 커밋 메시지 예시 (오늘 분)

- `state_estimator`  
  ```
  feat(state_estimator): Twist-only 오도메트리 구현 및 보정 완료

  - odom_publisher 노드 추가: /ego_status(TurtlebotStatus) → /odom
  - TF(odom→base_link) 발행, yaw 정규화(atan2(sin,cos))
  - 단위/스케일 정합: linear_scale=1.0, angular_scale=2.222222
  - 긴 간격 대응: max_dt_sec=2.0, publish_rate=0.0
  - 초기 공분산 파라미터화(covariance.{x,y,yaw})
  ```

- `udp_raw_bridge` / `udp_parsers_cpp` / `udp_tx_bridge` / `bridge_bringup`
  ```
  chore(launch): ns 인자 통일 도입(멀티 로봇/인스턴스 대비)
  - bringup에서 ns 전달 → raw/parsers/tx 각 런치 반영
  - config_file 및 respawn_* 인자 전달 구조 유지
  ```

---

## 8) 자주 쓰는 명령 스냅샷
```bash
# 빌드 & 세션 준비
cd ~/ros2_ws && colcon build && source install/setup.bash

# 통합 브릿지 기동(루트 네임스페이스)
ros2 launch bridge_bringup bridge.launch.py

# 네임스페이스 분리 실행 예
ros2 launch bridge_bringup bridge.launch.py ns:=robot1
ros2 launch bridge_bringup bridge.launch.py ns:=robot2

# 오도메트리 런치 (state_estimator 단독)
ros2 launch state_estimator odom.launch.py

# 토픽/주기 확인
ros2 topic echo /ego_status --qos-reliability best_effort --once
ros2 topic hz /ego_status

# RViz 확인(필요시)
rviz2
```

---

## 9) 부록: 설계 핸드노트(핵심 이유 요약)
- **왜 Twist-only?** 시뮬/초기 단계에서 가장 단순·안정. 이후 IMU/엔코더 융합으로 확장.
- **왜 ns만?** 브릿지는 공용 계층 → 이름은 표준화, 격리는 네임스페이스로 해결.
- **왜 클램프/정규화?** dt 이상치와 수치 누적 폭주 방지, 실시간 안정성 확보.
- **왜 공분산 노출?** 상위 로컬라이저/SLAM이 신뢰도 가중치를 활용할 수 있도록.
