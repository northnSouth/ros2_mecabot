# ROS2 Mecabot: Autonomous Directed Graph Map-Based Mecanum Robot Simulation

This is a [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html) based mecanum drive robot project simulated in [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/) with features such as rotary encoder based odometry, PID kinematics control, directed graph map based motion, and A* pathfinding.

https://github.com/user-attachments/assets/3fea155b-344b-49d8-97cd-11f392b58dcf

## Installation

Make sure you have installed [ROS2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html), [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/), common C++ and Python toolchain and the dependencies listed in `package.xml` in your environment.

1. Clone the repository.
2. Change into the repository home directory, then source the environment.

```
source install/setup.bash
```

3. Build the package with `colcon`.

```
colcon build --symlink-install
```

Add this argument if you'd like to use `clang` for the workspace:

```
--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## Usage

*A few tips here, run the simulator at 0.01s step size otherwise the robot might rotate endlessly during autonomous mode, and set the `odometry_worker` node to a high priority during runtime to avoid latency in odometry calculations.*

There are multiple ways to move/interact with the robot:

* Using `/path_command` topic to have the robot find the shortest path and move from one landmark/frame in the map to another.
* Using `/trajectory_command` topic to move the robot in a certain simple trajectory in either absolute or relative coordinate reference.
* Publishing directly to `/cmd_vel` to control the robot with an open-loop fashion.
* To change speed, set `speed_multiplier` parameter of the `kinematics_control` node to a desired value. Default is 1.

### `/path_command`

This topic uses `std_msgs/String` interface with my custom format: `"start_frame end_frame"`

* `start_frame` should be the robot's current or closest frame.
* `end_frame` is the goal frame.

Example:

```
ros2 topic pub /path_command std_msgs/msg/String "{data: 'start end'}" --once
```

### `/trajectory_command`

This topic uses `std_msgs/String` interface with my custom format: `"trajectory/string_arguments/numeric_arguments"`

* `trajectory` type of trajectory or motion for the robot to execute. Available trajectory(s):
  * `point` moves robot to a certain coordinate with encoder feedback. Have one string argument with two options: Absolute `abs` and relative `rel` coordinate reference. When using `abs` the input coordinate will be calculated relative to the initial point of the `world` frame. When using `rel` the input coordinate will be calculated relative to the robot's current position which is relative to the `world` frame. It has three numeric arguments: `x,y,theta`, which are the input coordinates, `theta` uses radian.
* `string_arguments` string arguments that are trajectory specific, separated with comma `,` symbol.
* `numeric_arguments` numeric (float) arguments that are trajectory specific, separated with comma `,` symbol.

Example:

(Moves the robot to the input coordinate relative to the `world` frame.)

```
ros2 topic pub /trajectory_command std_msgs/msg/String "{data: 'point/abs/2,7,1.57'}"
```

### `/cmd_vel`

Compatible with the `teleop_twist_keyboard` executable.

## References

These are references and algorithm explanations used in the project.

* Rotary Encoder Based Odometry [Youtube: Odometry 101 for FIRST Tech Challenge Robots by DrBatanga](https://youtu.be/Av9ZMjS--gY?si=f_gCDkorZMCKaEdP) <br>
* Locomotion Kinematics [Youtube: How to Use Mecanum Wheels in 200 Seconds by Gavin Ford](https://youtu.be/gnSW2QpkGXQ?si=ZigLoyJ6pj0cPHlQ) <br>
* A* Pathfinding Algorithm [Youtube: A* Pathfinding (E01: algorithm explanation) by Sebastian Lague](https://youtu.be/-L-WgKMFuhE?si=UXPzRJ-5tqCyta3h)
