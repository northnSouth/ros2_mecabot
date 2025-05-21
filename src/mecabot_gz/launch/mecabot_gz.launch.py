import os 
import xacro 
import math
from launch import LaunchDescription, SomeEntitiesType
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory 
from launch.actions import ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.events.process import ProcessIO
from launch.event_handlers import OnProcessIO, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

package_name = 'mecabot_gz'

def generate_launch_description():

  robot_desc = xacro.process_file(
    os.path.join(get_package_share_directory(package_name), 'robot_desc/mecabot_gz.main.xacro')
    ).toprettyxml(indent='  ')

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
     'use_sim_time': True, 
     'robot_description': robot_desc
    }]
  )

  gazebo_launch_file = os.path.join(get_package_share_directory("ros_gz_sim"),'launch/gz_sim.launch.py')
  gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(gazebo_launch_file),
    launch_arguments={'gz_args': 'empty.sdf'}.items()
  )

  gazebo_unpause = ExecuteProcess(
    cmd=[[
      "ros2",
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
    os.path.join(get_package_share_directory(package_name), 'world_desc/lks_arena.main.xacro')
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

  # Rviz2
  rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz2', 'main_view.rviz')
  rviz2 = Node(
      package='rviz2',
      executable='rviz2',
      output='screen',
      arguments=['-d', rviz_config_file],
  )

  trajectory_master = Node(
    package=package_name,
    executable='trajectory_master',
    parameters=[{'pid_kp': 8.0}]
  )

  sim_time_forward = Node(
    package=package_name,
    executable='sim_time_forward',
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

  def on_matching_stdout(event: ProcessIO, matcher: str, result: SomeEntitiesType):
    for line in event.text.decode().splitlines():
      if matcher in line:
        return result

  return LaunchDescription([
    
    gazebo_launch, 
    ros_gz_bridge, 
    sim_time_forward,
    robot_state_publisher,
    directed_map,
    gazebo_unpause,

    RegisterEventHandler(
      OnProcessExit(
        target_action=gazebo_unpause,
        on_exit=[arena_spawner, bot_spawner]
      )
    ),

    RegisterEventHandler(
      OnProcessExit(
        target_action=bot_spawner,
        on_exit=ros2c_activate
      )
    ),

    RegisterEventHandler(
      OnProcessExit(
        target_action=ros2c_activate,
        on_exit=kinematics_control
      )
    ),

    RegisterEventHandler(
      OnProcessIO(
        target_action=kinematics_control,
        on_stderr=lambda event: on_matching_stdout(
          event, "Kinematics online", 
        [
          odometry_worker,
          trajectory_master,
          rviz2,
          directed_pathfinder
        ])
      )
    )
  ])