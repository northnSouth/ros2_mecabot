cmake_minimum_required(VERSION 3.8)
project(mecabot_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# Encoder to Odometry
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

# Sim Time Forward
add_executable(sim_time_forward src/sim_time_forward.cpp)
ament_target_dependencies(
  sim_time_forward
  rclcpp
  rosgraph_msgs
)

# Mecabot Move Node
add_executable(mecabot_move_node src/mecabot_move_node.cpp)
ament_target_dependencies(
  mecabot_move_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  tf2
)

include_directories(include)

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

# Build Executables
install(TARGETS
  encoder_to_odometry
  sim_time_forward
  mecabot_move_node
  DESTINATION lib/${PROJECT_NAME}
)

# Build launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Build robot models
install(DIRECTORY
  ../../robot_desc
  DESTINATION share/${PROJECT_NAME}/
)

# Build world models
install(DIRECTORY
  ../../world_desc
  DESTINATION share/${PROJECT_NAME}/
)

# Build config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
