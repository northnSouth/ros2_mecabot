cmake_minimum_required(VERSION 3.8)
project(mecabot_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# executables
add_executable(encoder_to_odometry src/encoder_to_odometry.cpp)
ament_target_dependencies(
  encoder_to_odometry
  rclcpp
  rosgraph_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2
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

install(TARGETS
  encoder_to_odometry
  DESTINATION lib/${PROJECT_NAME}
)

install (DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
