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
find_package(control_toolbox REQUIRED)

add_executable(odometry_worker src/odometry_worker.cpp)
ament_target_dependencies( odometry_worker
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2
)

add_executable(sim_time_forward src/sim_time_forward.cpp)
ament_target_dependencies( sim_time_forward
  rclcpp
  rosgraph_msgs
)

add_executable(kinematics_control src/kinematics_control.cpp)
ament_target_dependencies( kinematics_control
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(directed_map_broadcaster src/directed_map_broadcaster.cpp)
target_link_libraries(directed_map_broadcaster yaml-cpp)
ament_target_dependencies( directed_map_broadcaster
  rclcpp
  yaml-cpp
  ament_index_cpp
  geometry_msgs
  tf2_ros
  tf2 
)

add_executable(trajectory_master src/trajectory_master.cpp)
ament_target_dependencies( trajectory_master
  rclcpp
  control_toolbox
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  tf2
)

add_executable(directed_pathfinder src/directed_pathfinder.cpp)
target_link_libraries(directed_pathfinder yaml-cpp)
ament_target_dependencies( directed_pathfinder
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
  trajectory_master
  directed_pathfinder
  odometry_worker
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

# Build rviz config
install(DIRECTORY
  ../../rviz2
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
