cmake_minimum_required(VERSION 3.8)
project(final_test_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(
  maze_explorer
  src/maze_explorer.cpp
)

ament_target_dependencies(
  maze_explorer
  rclcpp
  nav_msgs
  geometry_msgs
  nav2_msgs
  tf2
  tf2_ros
  rclcpp_action
)

add_executable(
  cmd_vel_safety_filter_v9
  src/cmd_vel_safety_filter_v9.cpp
)

ament_target_dependencies(
  cmd_vel_safety_filter_v9
  rclcpp
  geometry_msgs
  sensor_msgs
)

install(
  TARGETS
  maze_explorer
  cmd_vel_safety_filter_v9
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
