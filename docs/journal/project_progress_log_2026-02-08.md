## 🔧 Wheel Odometry Calibration (2026-02-08)

**목표**: 시뮬레이터의 부정확한 회전 속도 단위를 ROS 2 표준 단위(rad/s)로 정밀 보정.

### 1. 문제 및 원인
- **현상**: Unity 시뮬레이터의 `angular.z` 출력이 물리 단위가 아닌 정규화된 값(-1.0 ~ 1.0)으로 나옴.
- **영향**: 기존 눈대중 값(-2.222) 사용 시 약 6%의 회전 오차가 발생하여 내비게이션 정밀도 저하.

### 2. 수행 내용
1. **분석 환경 구축 (도구 설치)**:
   - `ros-humble-plotjuggler-ros`: 데이터 시각화 및 적분(Integral) 분석을 위해 설치.
   - `ros-humble-rosbag2-storage-mcap`: 고성능 로그 기록 포맷(MCAP) 지원을 위해 설치.
2. **데이터 수집**: 로봇을 제자리에서 **10회전(360°×10)** 시키며 `ros2 bag`으로 `/ego_status` 녹화.
3. **데이터 분석**: `PlotJuggler`를 통해 각속도 데이터를 적분하여 시뮬레이터 기준 총 회전량 측정.
4. **계수 산출**:
   - 목표 회전각 (Target): $20\pi \approx 62.83 \text{ rad}$
   - 측정된 적분값 (Measured): $29.95$
   - **보정 계수 ($K$)**: $62.83 / 29.95 \approx \mathbf{-2.098}$

### 3. 결과
- `odom_publisher.yaml`에 산출된 계수 적용.
- 적용 후 재측정 결과: **62.73 rad** (이론값 62.83 rad 대비 **오차율 0.15%** 달성).

### 4. 코드 변경 사항
```yaml
# src/state_estimator/params/odom_publisher.yaml 수정
odom_publisher:
  ros__parameters:
    # ... 기존 설정 ...
    
    # [Calibration] Target(62.83) / Measured(29.95) = 2.098
    angular_scale: -2.098
```