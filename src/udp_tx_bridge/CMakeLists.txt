cmake_minimum_required(VERSION 3.8)
project(udp_tx_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bridge_msgs REQUIRED)

# executable
add_executable(udp_tx_cmd_vel_node src/udp_tx_cmd_vel_node.cpp)
ament_target_dependencies(udp_tx_cmd_vel_node rclcpp geometry_msgs)

# install
install(TARGETS udp_tx_cmd_vel_node DESTINATION lib/${PROJECT_NAME})

# tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
