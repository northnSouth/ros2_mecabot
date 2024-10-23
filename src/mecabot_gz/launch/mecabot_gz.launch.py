from launch import LaunchDescription
from launch_ros.actions import Node # Node launch description
from launch.actions import ExecuteProcess # Duh
from launch.actions import TimerAction # Launch delay timer
from ament_index_python import get_package_share_directory # Get package directory
import os # Joining paths
import xacro # XACRO utilities
import math # Duh
from launch.substitutions import FindExecutable # So the logs will show where process binary is located

def generate_launch_description():

  # Read XACRO file of the robot
  robot_desc = xacro.process_file(
    os.path.join(get_package_share_directory('mecabot_gz'), 
                'robot_desc/mecabot_gz.main.xacro')
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
  gazebo_as_process = ExecuteProcess(
    cmd=[[
      FindExecutable(name="ros2"),
      " launch ",
      " ros_gz_sim ",
      " gz_sim.launch.py ",
      " gz_args:=empty.sdf "
    ]],
    shell=True
  )

  # Unpause Gazebo for ROS2 Controller activation
  gazebo_unpause = ExecuteProcess(
    cmd=[[
      FindExecutable(name="ros2"),
      " service call ",
      " /world/empty/control ",
      " ros_gz_interfaces/srv/ControlWorld ",
      " '{world_control: {pause: false}}' "
    ]],
    shell=True
  )

  # ROS2 Controller activation
  ros2c_activate = Node(
    package="controller_manager",
    executable="spawner",
    arguments=
    [
      "JSB", # joint_state_broadcaster
      "JTC" # joint_trajectory_controller
    ]
  )

  # Spawn robot in Gazebo from /robot_description
  bot_spawner = Node(
    package='ros_gz_sim',
    executable='create',
    parameters=[{
      'name': 'mecabot_gz',
      'world': 'empty',
      'topic': '/robot_description',
      'x': 5.25,
      'y': -2.3,
      'z': 0.2,
      'Y': math.pi / 2,
    }]
  )

  # Read arena xacro file
  arena_desc = xacro.process_file(
    os.path.join(get_package_share_directory('mecabot_gz'), 
                'world_desc/lks_arena.main.xacro')
  ).toprettyxml(indent='  ')

  # Spawn arena in Gazebo from file
  arena_spawner = Node(
    package='ros_gz_sim',
    executable='create',
    parameters=[{
      'name': 'arena',
      'world': 'empty',
      'string': arena_desc,
      'z': 0.018
    }]
  )

  # Bridge Gazebo topics and services to ROS2
  ros_gz_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
      '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
      '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
      '/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
      '/world/empty/control@ros_gz_interfaces/srv/ControlWorld'
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
    gazebo_as_process,
    TimerAction(
      period=1.0,
      actions=[
        ros_gz_bridge,
        sim_time_forward
      ]
    ),
    TimerAction(
      period=2.0,
      actions=[robot_state_publisher]
    ),
    TimerAction(
      period=3.0,
      actions=[arena_spawner]
    ),
    TimerAction(
      period=4.0,
      actions=[
        bot_spawner,
        rviz2,
        enc_to_odom,
        gazebo_unpause
      ]
    ),
    TimerAction(
      period=5.0,
      actions=[ros2c_activate]
    )
    
  ])