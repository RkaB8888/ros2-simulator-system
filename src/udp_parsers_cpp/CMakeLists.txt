cmake_minimum_required(VERSION 3.8)
project(udp_parsers_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bridge_msgs REQUIRED)

# executables
add_executable(ego_status_parser src/ego_status_parser.cpp)
ament_target_dependencies(ego_status_parser rclcpp std_msgs bridge_msgs) 
install(TARGETS ego_status_parser DESTINATION lib/${PROJECT_NAME})

# 아직 env_parser.cpp 없으면 주석 처리
# add_executable(env_parser src/env_parser.cpp)
# ament_target_dependencies(env_parser rclcpp std_msgs bridge_msgs)
# install(TARGETS env_parser DESTINATION lib/${PROJECT_NAME})

# tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

