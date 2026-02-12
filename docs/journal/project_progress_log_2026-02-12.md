## 🗺️ SLAM Implementation & Launch Architecture Refinement (2026-02-12)

**목표**: 자율 주행(Navigation)을 위한 고정밀 지도(Occupancy Grid Map) 작성 및 유지보수성을 고려한 **하드웨어/소프트웨어 런치 파일 분리**.

### 1. 아키텍처 설계 및 변경 (Architecture Design)
기존의 단일 런치 파일(All-in-One) 방식에서 벗어나, 실제 로봇 운용 시나리오에 적합한 **계층형 실행 구조**로 변경함.

- **기존 방식**: `mapping.launch.py` 실행 시 로봇 드라이버(Bridge)와 SLAM이 동시에 켜짐.
  - **문제점**: 매핑을 중단하고 내비게이션으로 전환할 때, 로봇 하드웨어까지 재부팅되면서 **Odom(오도메트리) 좌표가 초기화됨**.
- **변경 방식**: **Hardware Layer**와 **Application Layer** 분리.
  1. **Base (T1)**: `bridge.launch.py` (Lidar, IMU, Odom) → **항상 켜둠 (Odom 유지)**.
  2. **App (T2)**: `mapping.launch.py` (SLAM) 또는 `navigation.launch.py` → **필요에 따라 교체 실행**.

### 2. SLAM 구현 및 최적화 (Implementation & Optimization)
`slam_toolbox`를 도입하여 시뮬레이션 환경(WSL2)에 맞는 파라미터 튜닝 수행.

- **WSL2 성능 병목 해결**:
  - `Message Filter dropping` 및 `Queue full` 경고 발생.
  - **조치**: `throttle_scans`를 1로 유지하되, `minimum_time_interval`을 0.1s로 조정하여 CPU 부하 조절.
  - **조치**: `ceres_num_threads` 경고(50개 요청)는 시스템 최대치(12개)로 자동 bounding 됨을 확인 후 무시(성능 영향 미미).
- **데이터 관리 전략**:
  - 생성된 지도 데이터(`*.pgm`, `*.yaml`)는 바이너리 파일로 용량이 크고 변경이 잦음.
  - **Git 전략**: 소스 코드는 관리하되, 지도 데이터는 `.gitignore`에 등록하여 저장소 비대화 방지.

### 3. 주요 코드 변경 사항

#### A. Mapping Launch (하드웨어 의존성 제거)
```python
# src/mapping_localization/launch/mapping.launch.py
# [Change] Bridge 실행 구문을 삭제하고, 오직 SLAM 노드만 실행하도록 변경
def generate_launch_description():
    # ... 경로 설정 ...
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'false' # Bridge 환경
        }.items()
    )
    return LaunchDescription([slam_launch]) # Bridge 제외
```

#### B. SLAM Parameters (시뮬레이터 최적화)
```yaml
# src/mapping_localization/config/mapper_params_online_async.yaml
slam_toolbox:
  ros__parameters:
    # 실시간성 확보를 위한 주기 단축
    map_update_interval: 1.0
    minimum_time_interval: 0.1  # 데이터 처리 간격 조정
    transform_publish_period: 0.05 # 20Hz
    
    # Loop Closure 성능 강화
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
```

#### C. Gitignore (데이터 제외 설정)
```gitignore
# ==========================
# SLAM Map Data (지도 데이터 제외)
# ==========================
*.pgm            # 이미지 파일 전체 무시
maps/* # maps 폴더 내 모든 파일 무시
!maps/.gitkeep   # (옵션) 폴더 구조는 유지
```

### 4. 결과 (Result)
- **지도 작성**: Teleop을 이용한 수동 주행으로 Loop Closure가 적용된 선명한 지도 작성 완료.
- **파일 저장**: `nav2_map_server`를 이용해 `~/ros2_ws/maps/` 경로에 `map_test.pgm`, `map_test.yaml` 저장 성공.
- **M5 준비**: 작성된 지도를 기반으로 Navigation(자율 주행) 단계 진입 준비 완료.