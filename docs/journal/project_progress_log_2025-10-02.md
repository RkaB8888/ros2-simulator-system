# 2025-10-02 작업 일지

## 작업 개요

오늘은 ROS2 기반 시뮬레이터 데이터 파싱 파이프라인 정비와 새로운 패키지 생성 및 연동 작업을 진행하였다. 크게 보면 `bridge_msgs` 메시지 패키지 생성, `udp_parsers_cpp` 패키지 생성 및 `EgoStatusParser` 노드 구현, 기존 `udp_raw_bridge` 패키지의 런치 및 CMake 수정이 있었다.

---

## 세부 진행 내용

### 1. `udp_raw_bridge` 패키지 정리

* 기존 CMakeLists.txt를 정리하여 **install 순서와 tests 블록**을 명확히 구분.
* `raw_4ports.launch.py`를 작성하여 7802, 8002, 8202, 8302 포트를 각각 `/env_info_raw`, `/object_info_raw`, `/ego_status_raw`, `/stuff_info_raw` 토픽으로 포워딩.
* `ros2 launch udp_raw_bridge raw_4ports.launch.py` 실행으로 RAW UDP 수신 및 토픽 발행 확인.

### 2. `bridge_msgs` 메시지 패키지 생성

* 새로운 메시지 패키지 `bridge_msgs`를 생성.
* `TurtlebotStatus.msg` 정의:

  ```
  geometry_msgs/Twist twist
  uint8 power_supply_status
  float32 battery_percentage
  bool can_use_hand
  bool can_put
  bool can_lift
  ```
* `CMakeLists.txt`와 `package.xml`에 `rosidl_default_generators`, `std_msgs`, `geometry_msgs` 의존성 추가.
* 빌드 시 `--cmake-clean-cache` 옵션으로 잔재 충돌 제거 후 정상 빌드 성공.

### 3. `udp_parsers_cpp` 패키지 생성 및 `EgoStatusParser` 노드 구현

* 새로운 패키지 `udp_parsers_cpp`를 생성.
* `ego_status_parser.cpp` 작성:

  * `/ego_status_raw` 구독.
  * 시뮬레이터에서 전송되는 바이트 배열을 파싱.
  * 속도, 배터리, 좌표, heading, 조작 가능 여부 등을 추출.
  * `bridge_msgs::msg::TurtlebotStatus`로 `/robot/ego_status` 발행.
  * heading은 **deg → rad 변환 고정 처리**.
* 초기에는 Odometry 및 TF도 발행했으나, 역할 분리를 위해 제거. (`udp_parsers_cpp`는 파싱 전용 패키지로 유지)
* `CMakeLists.txt`, `package.xml`에 `bridge_msgs` 의존성 추가 후 빌드 정상 완료.

### 4. rqt 확인 및 문제 해결

* `rqt` 실행 시 `can not get message class for type "bridge_msgs/msg/TurtlebotStatus"` 오류 발생.
* 원인: 새로 생성된 msg 패키지의 빌드/설치 잔재 및 언더레이/오버레이 충돌.
* 조치:

  * `colcon build --cmake-clean-cache` 재실행.
  * `source install/setup.bash` 실행으로 새 메시지 타입 로딩.
* 이후 `/robot/ego_status` 토픽 정상 확인 완료.

---

## 오늘의 성과

* `bridge_msgs` 메시지 패키지 신규 생성 및 적용.
* `udp_parsers_cpp` 패키지 신규 생성 및 `EgoStatusParser` 노드 구현.
* `/ego_status_raw` → `/robot/ego_status` 파싱 파이프라인 완성.
* `udp_raw_bridge` 런치 및 설정 정비 완료.

---

## 다음 할 일

* `env_parser` 등 다른 파서 노드 추가 구현 예정.
* Odometry, SLAM 계산용 별도 패키지 설계 예정 (IMU, LiDAR, Camera 활용).
* rqt 토픽 플러그인 기반 모니터링 체계 보완.
* symlink-install 사용 금지 방침 유지 (빌드 안정성 확보).
