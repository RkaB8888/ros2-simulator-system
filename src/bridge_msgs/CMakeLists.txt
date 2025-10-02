cmake_minimum_required(VERSION 3.8)
project(bridge_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1) 필수 패키지
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 2) msg -> 언어별 헤더/모듈 자동 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TurtlebotStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 3) 런타임 의존성 export (다른 패키지가 헤더를 include 가능)
ament_export_dependencies(rosidl_default_runtime)

# 4) 테스트
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
