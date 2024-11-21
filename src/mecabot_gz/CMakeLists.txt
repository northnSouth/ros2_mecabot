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
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_toolbox REQUIRED)

# Encoder to Odometry
add_executable(encoder_to_odometry src/encoder_to_odometry.cpp)
ament_target_dependencies( encoder_to_odometry
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
)

# Sim Time Forward
add_executable(sim_time_forward src/sim_time_forward.cpp)
ament_target_dependencies( sim_time_forward
  rclcpp
  rosgraph_msgs
)

# Mecabot Move Node
add_executable(kinematics_control src/kinematics_control.cpp)
ament_target_dependencies( kinematics_control
  rclcpp
  std_msgs
  geometry_msgs
)

# Directed Map Broadcaster
add_executable(directed_map_broadcaster src/directed_map_broadcaster.cpp)
target_link_libraries(directed_map_broadcaster yaml-cpp)
ament_target_dependencies( directed_map_broadcaster
  rclcpp
  ament_index_cpp
  geometry_msgs
  tf2_ros
  tf2 
  yaml-cpp
)

# Motion Control
add_executable(motion_control src/motion_control.cpp)
ament_target_dependencies( motion_control
  rclcpp
  control_toolbox
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2
)

# Path Planner
add_executable(path_planner src/path_planner.cpp)
target_link_libraries(path_planner yaml-cpp)
ament_target_dependencies( path_planner
  rclcpp
  ament_index_cpp
  std_msgs
  yaml-cpp
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

# Build Executables
install(TARGETS
  sim_time_forward
  kinematics_control
  directed_map_broadcaster
  motion_control
  path_planner
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
