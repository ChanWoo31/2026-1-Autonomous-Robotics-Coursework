cmake_minimum_required(VERSION 3.8)
project(midterm_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(velocity_publisher src/velocity_publisher.cpp)
ament_target_dependencies(velocity_publisher rclcpp std_msgs)
add_executable(angle_publisher src/angle_publisher.cpp)
ament_target_dependencies(angle_publisher rclcpp std_msgs)
add_executable(motor_control src/motor_control.cpp)
ament_target_dependencies(motor_control rclcpp std_msgs geometry_msgs sensor_msgs tf2 tf2_geometry_msgs)

install(TARGETS
  velocity_publisher
  angle_publisher
  motor_control
  DESTINATION lib/${PROJECT_NAME}
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
