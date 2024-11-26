from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess 
from launch.actions import TimerAction 
from ament_index_python import get_package_share_directory 
import os 
import xacro 
import math
from launch.substitutions import FindExecutable 

package_name = 'mecabot_gz'

def generate_launch_description():

  robot_desc = xacro.process_file(
    os.path.join(get_package_share_directory(package_name), 
                'robot_desc/mecabot_gz.main.xacro')
  ).toprettyxml(indent='  ')

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
     'use_sim_time': True, 
     'robot_description': robot_desc
    }]
  )

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

  ros2c_activate = Node(
    package="controller_manager",
    executable="spawner",
    arguments=
    [
      "JSB",
      "velo_c"
    ]
  )

  bot_spawner = Node(
    package='ros_gz_sim',
    executable='create',
    parameters=[{
      'name': package_name,
      'world': 'empty',
      'topic': '/robot_description',
      'x': 5.25,
      'y': -2.3,
      'z': 0.2,
      'Y': math.pi / 2,
    }]
  )

  arena_desc = xacro.process_file(
    os.path.join(get_package_share_directory(package_name), 
                'world_desc/lks_arena.main.xacro')
  ).toprettyxml(indent='  ')

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

  ros_gz_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
      '/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
      '/world/empty/control@ros_gz_interfaces/srv/ControlWorld'
    ]
  )

  rviz2 = Node(
    package='rviz2',
    executable='rviz2'
  )

  trajectory_master = Node(
    package=package_name,
    executable='trajectory_master',
    parameters=[{'pid_kp': 8.0}]
  )

  sim_time_forward = Node(
    package=package_name,
    executable='sim_time_forward'
  )

  kinematics_control = Node(
    package=package_name,
    executable='kinematics_control',
    parameters=[{'speed_multiplier': 5.0}]
  )

  odometry_worker = Node(
    package=package_name,
    executable='odometry_worker'
  )

  directed_map = Node(
    package=package_name,
    executable='directed_map_broadcaster'
  )

  directed_pathfinder = Node(
    package=package_name,
    executable='directed_pathfinder'
  )

  return LaunchDescription([
    gazebo_as_process,
    TimerAction(
      period=1.0,
      actions=[
        ros_gz_bridge,
        sim_time_forward,
        arena_spawner
      ]
    ),
    TimerAction(
      period=2.0,
      actions=[robot_state_publisher]
    ),
    TimerAction(
      period=3.0,
      actions=[
        bot_spawner,
        rviz2,
        gazebo_unpause
      ]
    ),
    TimerAction(
      period=4.0,
      actions=[ros2c_activate]
    ),
    TimerAction(
      period=5.0,
      actions=[
        odometry_worker,
        trajectory_master,
        kinematics_control,
        directed_map,
        directed_pathfinder
      ]
    )
    
  ])