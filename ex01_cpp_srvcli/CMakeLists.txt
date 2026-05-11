cmake_minimum_required(VERSION 3.8)
project(ex01_cpp_srvcli)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(more_interfaces REQUIRED)
find_package(cw_interfaces REQUIRED)

# 퍼블리셔 빌드 노드 옵션
add_executable(cw_pub src/cw_pub.cpp)
ament_target_dependencies(cw_pub rclcpp std_msgs)

# 섭 클라
add_executable(cw_sub_cli src/cw_sub_cli.cpp)
ament_target_dependencies(cw_sub_cli rclcpp std_msgs cw_interfaces)

# 서버 노드 빌드 옵션
add_executable(ex01_server src/add_two_ints_server.cpp)
ament_target_dependencies(ex01_server rclcpp more_interfaces)

add_executable(cw_server src/cw_server.cpp)
ament_target_dependencies(cw_server std_msgs rclcpp cw_interfaces)

# 클라이언트 노드 빌드 옵션
add_executable(ex01_client src/add_two_ints_client.cpp)
ament_target_dependencies(ex01_client rclcpp more_interfaces)

# 해당 executable을 빌드하여 destination에 넣음.
install(TARGETS
  ex01_server
  ex01_client
  cw_pub
  cw_sub_cli
  cw_server
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
