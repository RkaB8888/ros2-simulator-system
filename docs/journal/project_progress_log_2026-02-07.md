# 📝 개발 작업 일지 (2026-02-07)

## 1. 주요 작업 내용
**목표:** 브리지 시스템 통합 실행 환경 구축 및 안전 장치(Safety Chain) 안정화

### 🛠️ 시스템 통합 (Launch)
- **`bridge.launch.py` 통합:**
    - 기존에 개별 실행하던 `sensor`, `odom`, `safety`, `connector` 런치 파일들을 하나로 통합.
    - **`nav2_lifecycle_manager` 추가:** 실행 시 `velocity_smoother`와 `collision_monitor`가 자동으로 `Active` 상태가 되도록 구성하여 수동 조작 제거.
    - `log_level_raw` 인자 추가로 디버깅 편의성 증대.

### ⚡ QoS 최적화 (Latency 해결)
- **LiDAR / Camera 데이터:** `Reliable` → **`Best Effort`** 로 변경.
- **성과:** 데이터 재전송 대기로 인한 2~3초가량의 데이터 지연(Latency) 문제 해결.

### 🛡️ 안전 장치 파라미터 수정
- **`velocity_smoother.yaml`:**
    - `max_decel` (감속도) 값을 양수(`2.0`)에서 **음수(`-2.0`)**로 수정. (Nav2 Humble 스펙 준수)
    - 설정 단계(Configure)에서 노드가 죽는 치명적 오류 해결.

---

## 2. 트러블슈팅 (Troubleshooting)

### 🔴 Rviz2에서 라이다 포인트 미표시 현상
- **증상:** `/scan` 토픽은 정상적으로 수신되나(hz 확인 완료), Rviz2 화면에는 데이터가 표시되지 않음.
- **원인:** **QoS 정책 불일치 (Incompatible QoS)**
    - 송신측(Bridge)은 지연 방지를 위해 **`Best Effort`** 로 설정.
    - 수신측(Rviz2 기본값)은 **`Reliable`** 로 설정되어 있어, 데이터를 받지 못하고 드랍시킴.
- **해결:** Rviz2의 LaserScan QoS 설정을 **`Best Effort`** 로 변경하고 Fixed Frame을 `odom`으로 설정하여 해결.

---

## 3. 현재 시스템 상태 (Status)
- **실행:** `ros2 launch bridge_bringup bridge.launch.py` 단일 명령어로 전체 시스템 구동.
- **데이터 흐름:** LiDAR, Odom, TF 정상 발행 (지연 없음).
- **제어 파이프라인:**
    - [Teleop] → [TwistMux] → [Smoother] → [Collision Monitor] → [Robot] 흐름 검증 완료.
- **안전 기능:** 장애물 접근 시 감속 및 **즉시 정지(0.0 m/s)** 기능 정상 작동 확인.

---

## 4. 다음 계획 (Next Step)
- **Odometry 고도화 (EKF 적용):**
    - 현재의 단일 소스 Odom(Encoder only)의 회전 오차를 보정하기 위해 **`robot_localization` 패키지(EKF)** 도입.
    - **IMU 데이터**와 휠 오도메트리를 센서 퓨전하여 위치 추정 정밀도 향상.