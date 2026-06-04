cmake_minimum_required(VERSION 3.8)
project(final_test_2_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(maze_explorer
  src/maze_explorer.cpp
)

ament_target_dependencies(
  maze_explorer
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  nav2_msgs
  tf2_ros
  rclcpp_action
)

add_executable(cmd_vel_safety_filter_v9
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
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
