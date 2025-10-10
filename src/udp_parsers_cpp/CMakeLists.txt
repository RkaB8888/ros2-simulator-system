# src/udp_parsers_cpp/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(udp_parsers_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bridge_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# executables
add_executable(ego_status_parser src/ego_status_parser.cpp)
ament_target_dependencies(ego_status_parser rclcpp std_msgs bridge_msgs)

add_executable(env_status_parser src/env_status_parser.cpp)
ament_target_dependencies(env_status_parser rclcpp std_msgs bridge_msgs)

add_executable(iot_status_parser src/iot_status_parser.cpp)
ament_target_dependencies(iot_status_parser rclcpp std_msgs bridge_msgs)

add_executable(custom_object_parser src/custom_object_parser.cpp)
ament_target_dependencies(custom_object_parser rclcpp std_msgs geometry_msgs bridge_msgs)

add_executable(imu_parser src/imu_parser.cpp)
ament_target_dependencies(imu_parser rclcpp std_msgs sensor_msgs)

add_executable(lidar_parser src/lidar_parser.cpp)
ament_target_dependencies(lidar_parser rclcpp std_msgs sensor_msgs)

add_executable(camera_jpeg_parser src/camera_jpeg_parser.cpp)
ament_target_dependencies(camera_jpeg_parser rclcpp std_msgs sensor_msgs)

install(TARGETS ego_status_parser env_status_parser iot_status_parser custom_object_parser imu_parser lidar_parser camera_jpeg_parser DESTINATION lib/${PROJECT_NAME})

# tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

