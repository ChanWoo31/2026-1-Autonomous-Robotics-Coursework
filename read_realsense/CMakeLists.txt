cmake_minimum_required(VERSION 3.8)
project(read_realsense)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 퍼블리셔 빌드 노드 옵션
add_executable(cw_pub src/cw_pub.cpp)
ament_target_dependencies(cw_pub rclcpp std_msgs sensor_msgs)

# 섭
add_executable(ex01 src/ex01.cpp)
ament_target_dependencies(ex01 rclcpp std_msgs sensor_msgs)
add_executable(cw_sub src/cw_sub.cpp)
ament_target_dependencies(cw_sub rclcpp std_msgs sensor_msgs)

install(TARGETS
  ex01
  cw_pub
  cw_sub
  DESTINATION lib/${PROJECT_NAME})

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
