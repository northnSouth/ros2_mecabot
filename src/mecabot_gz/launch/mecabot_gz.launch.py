from launch import LaunchDescription
from launch_ros.actions import Node # Node launch description
from launch.actions import IncludeLaunchDescription # Launch another launch file
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction # Launch delay timer
from ament_index_python import get_package_share_directory # Get package directory
import os # Joining paths
import xacro # XACRO utilities

def generate_launch_description():

  # Read XACRO file of the robot
  robot_desc = xacro.process_file(
    os.path.join(get_package_share_directory('mecabot_gz'), 
                '../../../../robot_desc/mecabot_gz.main.xacro')
  ).toprettyxml(indent='  ')

  # Run robot_state_publisher
  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
     'use_sim_time': True, 
     'robot_description': robot_desc
    }]
  )

  # Launch Gazebo simulator with empty world
  # TODO: Create the package's own world file
  gazebo = IncludeLaunchDescription(
    launch_description_source = PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('ros_gz_sim'),
                  'launch/gz_sim.launch.py'
      )
    ),
    launch_arguments = [{ 'gz_args', 'empty.sdf' }]
  )

  # Spawn robot in Gazebo from /robot_description
  spawner = Node(
    package='ros_gz_sim',
    executable='create',
    parameters=[{
      'name': 'mecabot_gz',
      'world': 'empty',
      'topic': '/robot_description',
      'z': 0.15
    }]
  )

  # Bridge Gazebo topics and services to ROS2
  ros_gz_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
      '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
      '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
      '/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
    ]
  )

  # Run rviz2
  rviz2 = Node(
    package='rviz2',
    executable='rviz2'
  )

  # Run encoder to odometry node
  enc_to_odom = Node(
    package='mecabot_gz',
    executable='encoder_to_odometry'
  )

  # Forward simulation time from gazebo
  sim_time_forward = Node(
    package='mecabot_gz',
    executable='sim_time_forward'
  )

  return LaunchDescription([
    ros_gz_bridge,
    sim_time_forward,
    TimerAction(
      period=3.0,
      actions=[gazebo]
    ),
    TimerAction(
      period=6.0,
      actions=[robot_state_publisher]
    ),
    TimerAction(
      period=7.0,
      actions=[
        spawner,
        rviz2,
        enc_to_odom
      ]
    ),
    
  ])