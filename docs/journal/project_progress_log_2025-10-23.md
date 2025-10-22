# 프로젝트 작업 일지 — 2025-10-23

## 1) 개요
- **오늘 목표(M1.5a)**  
  안전 체인 인프라 구축(**twist_mux → nav2_velocity_smoother → nav2_collision_monitor**)과 통합 브릿지 연동.  
  센서 없는 상태에선 **pass-through**로 동작하되, 이후 센서 합류 시 즉시 활성화 가능하도록 구조/파라미터 정비.
- **완료 판단 기준**  
  - 체인 노드가 정상 기동, 토픽 흐름이 일관되게 연결  
  - velocity_smoother가 설정된 가속/감속으로 **램프 업/다운** 출력  
  - collision_monitor는 **라이프사이클 활성화** 후 pass-through로 최종 `/cmd_vel` 통과  
  - RViz에서 영역 폴리곤이 올바른 프레임/좌표로 시각화

---

## 2) 변경/추가된 산출물

### A) `bridge_bringup`
- `launch/bridge.launch.py`
  - **세이프티 체인 포함**: `IncludeLaunchDescription(safety_bringup/launch/safety_chain.launch.py)`
  - `enable_safety` 토글 추가(기본 `true`)
  - 공통 로그 레벨 인자 전달: `log_level_raw` → 하위 런치/노드로 전달
  - 기존 RAW/PARSERS/TX 런치에 **ns, log_level** 등 인자 일관 전달

### B) `safety_bringup` (신규 bringup 패키지)
- **CMake/패키징**
  - `CMakeLists.txt`: `ament_cmake` 최소 의존 + `launch/config` 설치만
  - `package.xml`: `exec_depend` 중심으로 정리  
    (`launch`, `launch_ros`, `ament_index_python`, `twist_mux`, `nav2_velocity_smoother`, `nav2_collision_monitor`, `rclcpp`)
- **런치**
  - `launch/safety_chain.launch.py`
    - `twist_mux`, `velocity_smoother`, `collision_monitor` 세 노드 기동
    - 공통 인자: `ns`, `log_level`  
    - 리맵:
      - `twist_mux`: `cmd_vel_out → cmd_vel_mux`
      - `velocity_smoother`: `cmd_vel → cmd_vel_mux`, `cmd_vel_smoothed → cmd_vel_smooth`
      - `collision_monitor`: **리맵 제거**(YAML의 `cmd_vel_in/out_topic`로 고정)
- **설정**
  - `config/twist_mux.yaml`  
    - **입력 소스**: `nav(1)`, `teleop(2)`, `emergency(3)`  
    - **락**: `emergency_stop(priority=255)`  
    - Humble에서 정상 파싱되는 **맵 기반 구조** 사용(이전 리스트 스키마에서 교정)
  - `config/velocity_smoother.yaml`  
    - Humble 파라미터 스키마에 맞춰 **배열형(double array)** 파라미터로 교정  
    - `feedback: OPEN_LOOP`, `max_velocity/min_velocity`, `max_accel/max_decel`, `velocity_timeout=0.5`, `smoothing_frequency=30.0` 등
  - `config/collision_monitor.yaml`  
    - `base_frame_id=base_link`, `odom_frame_id=odom`  
    - **pass-through**: `use_slowdown=false`, `use_stop=false`  
    - **observation_sources 필수** → 비어있으면 오류 → `dummy_scan(enabled:false)` 추가  
    - **폴리곤 좌표**: Humble 기준 **flat double array**로 지정  
      (`[x1,y1,x2,y2,...]`), `visualize: true` + `polygon_pub_topic`로 RViz 노출  
    - `cmd_vel_in_topic="cmd_vel_smooth"`, `cmd_vel_out_topic="cmd_vel"`

---

## 3) 트러블슈팅 로그 (중요 이슈 & 해결)

1) **CMake 에러(`find_package(launch)` 미발견)**  
   - 원인: bringup 패키지는 **런치만** 제공하므로 컴파일 타겟 없음.  
   - 조치: `CMakeLists.txt`에서 `find_package(launch*)` 제거, `package.xml`을 **`exec_depend`**로 선언 전환.

2) **`twist_mux.yaml` 파싱 오류**  
   - 증상: `Sequence should be of same type`  
   - 원인: Humble에서 기대하는 파라미터 스키마와 맞지 않는 리스트/타입 구성이 섞임.  
   - 조치: **맵 기반 스키마**로 교정(토픽 이름을 키로 사용) → 정상 기동.

3) **`collision_monitor.yaml` 파싱/라이프사이클 실패**  
   - 증상: `points` 타입 오류, `observation_sources is not initialized`  
   - 원인: 문서 예시의 `"[[...]]"` 문자열 포맷은 Humble 구현과 **불일치**. 또한 `observation_sources`는 **비어 있으면 안 됨**.  
   - 조치: `points`를 **flat double array**로 교정, `observation_sources: ["dummy_scan"]` + `dummy_scan.enabled:false` 추가 → `configure/activate` 성공.

4) **로그 레벨 전체 옵션 동작 안 함**  
   - `ros2 launch ... --log-level ...` 전역 옵션은 환경/버전에 따라 적용 안 됨.  
   - 조치: 각 `Node(arguments=[--ros-args, --log-level ...])`로 **노드별** 로그 레벨 주입.

5) **RViz에서 폴리곤 미표시**  
   - 원인: 폴리곤이 **단발성 퍼블리시** + 퍼블리셔가 **Transient Local**, RViz 디폴트는 **Volatile**.  
   - 조치: RViz Polygon Display의 **Durability=Transient Local**로 변경 또는 노드 재-Activate.  
   - 참고: 폴리곤은 **정적 geometry**(로봇 기준)로 1회 발행이 정상. RViz Fixed Frame을 `base_link`로 두면 로봇에 “붙어” 보임.

6) **velocity_smoother 라이프사이클 `configure` 실패**  
   - 원인: `max_velocity` 등 배열형 파라미터를 단일 double로 제공했기 때문.  
   - 조치: 공식 스키마에 맞게 **배열형(double array)** 로 수정 → `configure/activate` 성공.

7) **state 토픽 오해**  
   - `state_topic`은 패키지 고유 상태 브로드캐스트(빌드에 따라 미제공).  
   - 라이프사이클 상태 모니터링은 `/collision_monitor/transition_event` 사용 가능.  
   - 현재 바이너리에선 `collision_monitor_state` 미발행 → 옵션으로 간주.

---

## 4) 검증 결과

- **토픽 체인**
  - 입력: `/cmd_vel_nav`, `/cmd_vel_teleop`, `/cmd_vel_emergency`  
  - 체인: `/cmd_vel_mux` → `/cmd_vel_smooth` → `/cmd_vel`(final)
- **rqt_plot 확인**
  - `/cmd_vel_nav.linear.x = 1.0` step 입력 시  
    - `/cmd_vel_smooth.linear.x`는 **0→1.0** 약 **2.0 s** 램프 업(가속 0.5 m/s²)  
    - 입력 종료 후 **timeout 0.5 s 유지 → 2.0 s 램프 다운**  
    - `/cmd_vel`은 pass-through로 동일 파형
- **라이프사이클**
  - `velocity_smoother`, `collision_monitor`: `configure → activate` 성공
- **RViz 시각화**
  - `polygon_slow_front`, `polygon_stop_front`, `polygon_stop_back` 표시  
  - Fixed Frame `odom`/`base_link` 전환 시 의도대로 위치/크기 확인

---

## 5) 오늘 결정 사항
- collision_monitor의 I/O는 **파라미터로 고정**하고 리맵은 노이즈 최소화 목적일 때만 사용
- Humble 호환을 우선:  
  - `points`는 **flat double array**
  - `observation_sources`는 **반드시 1개 이상(더미 허용)**
- 전역 로그 레벨 대신 **노드별 `--ros-args --log-level`** 전략 유지
- pass-through 구성으로 **M1.5a 완료**로 마일스톤 커밋

---

## 6) 다음 할 일(M1.5b 계획)
1. **센서 합류**: LiDAR `/scan` 또는 PointCloud2 소스 연결  
   - `observation_sources`에 실제 소스 추가, `dummy_scan` 제거  
   - `use_slowdown/use_stop = true` 활성화, `slowdown_ratio`/폴리곤 튜닝
2. **자동 라이프사이클 제어**: bringup에서 `configure→activate` 자동화 스크립트/노드 도입
3. **테스트 시나리오**: 벽 접근 감속/정지 자동화 테스트, rqt_plot & rosbag 기록
4. **문서화**: 운영/장애 대응 플로우, 파라미터 가이드, RViz 프로필 공유

---

## 7) 커밋/브랜치
- 브랜치: `feature/m1.5a-safety-chain`
- 커밋(요지):  
  - bridge_bringup에 세이프티 체인 통합 및 log_level 전달  
  - safety_bringup 런치/설정/패키징 일체  
  - Humble 호환 파라미터로 YAML 수정, 라이프사이클 활성 검증

---

## 8) 실행/진단 스냅샷

```bash
# 빌드 & 환경
colcon build --packages-select bridge_bringup safety_bringup && source install/setup.bash

# 통합 기동
ros2 launch bridge_bringup bridge.launch.py log_level_raw:=info

# 라이프사이클
ros2 lifecycle set /velocity_smoother configure
ros2 lifecycle set /velocity_smoother activate
ros2 lifecycle set /collision_monitor configure
ros2 lifecycle set /collision_monitor activate

# 토픽 흐름
ros2 topic list | grep cmd_vel

# 스무딩 검증
ros2 topic pub /cmd_vel_nav geometry_msgs/Twist '{linear: {x: 1.0}}' -r 10
rqt_plot /cmd_vel_nav/linear/x:/cmd_vel_mux/linear/x:/cmd_vel_smooth/linear/x:/cmd_vel/linear/x

# RViz 폴리곤 (QoS 주의: Durability=Transient Local)
rviz2
```

---

## 9) 메모(핵심 차이 요약)
- **문서 vs 구현(Humble)**: collision_monitor의 `points`는 문자열 `"[[...]]"`가 아니라 **double array**가 올바름  
- **폴리곤 시각화**는 **단발성 발행**이 정상이며, 내부 계산은 TF로 지속 수행됨  

---

**결론**: M1.5a(안전 체인 인프라, pass-through) **완료**.  
M1.5b에서 센서 합류 및 실제 감속/정지 로직 검증으로 진행한다.
