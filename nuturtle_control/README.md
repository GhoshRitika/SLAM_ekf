# NuturtleControl
* Ritika Ghosh

## Package Description
This package is responsible for launching the nodes that are control the robot both in the real world and simulation.

## Launch File
Allows the user to cmd_vel commands to the turtlebot (real or simulated, remote or local), receive odometry, and visualize everything in rviz. Rviz always displays a blue turtlebo corresponding to the real world robot and a red one when in simulation showing the expected turtlebot position.
### Launch Arguments
- cmd_src:
* `teleop` to start the turtlebot3_teleop teleop_twist_keyboard
* `circle` to start the circle node
* `none` start nothing, another node not started by this launchfile will publish the cmd_vel commands
- robot:
* `nusim` to start nusim simulator. This is the default. 
* `localhost` to run the nodes directly from the turtlebot3.
* `none` no additional node is run.
- use_rviz:
* `true` launches rviz with a configuration that enables seeing the robot model, tf frames, and the odometry. 
* `false` when used with the robot:=localhost option
### Simulation
`ros2 launch nuturtle_control start_robot.launch.xml cmd_src:=teleop robot:=nusim`
or
`ros2 launch nuturtle_control start_robot.launch.xml cmd_src:=circle robot:=nusim`
### Real World
`ros2 launch nuturtle_control start_robot.launch.xml cmd_src:=circle robot:=localhost`

## Parameters
*     wheel_radius (double): The radius of the wheels.
*     track_width (double): The distance between the wheels.
*     motor_cmd_max(int): The motors are provided commands in [-motor_cmd_max, motor_cmd_max].
*     motor_cmd_per_rad_sec(double): Each motor command "tick" is 0.024 rad/sec.
*     encoder_ticks_per_rad(double): The number of encoder "ticks" per radian.
*     body_id(string): The name of the body frame of the robot.
*     odom_id(string): The name of the odometry frame..
*     wheel_left(string): The name of the left wheel joint.
*     wheel_right(string): The name of the right wheel joint.
*     frequency (int):  Rate at which the cmd_vel topic is published.

The following is a video of the turtlebot robot driving in a circle, clockwise and counter clockwise multiple times and stops at its intial configuration.

1. [Turtlebot_simulation.mp4](https://user-images.githubusercontent.com/60728026/217688212-0d05f264-89fa-48e9-8433-4c95c98bdbc9.mp4)

2. [Turtlebot_realworld](https://user-images.githubusercontent.com/60728026/217689574-8d78a9b8-6175-47cf-8473-8954e2625b56.mp4)

<p>The robot stops at the close to the initial configuration, the odom msg at the time of stopping is:<br>
pose:<br>
  pose:<br>
    position:<br>
  	x: 0.04943881815180784<br>
  	y: 0.0024620670947768097<br>
  	z: 0.0<br>
	orientation:<br>
  	x: 0.0<br>
  	y: 0.0<br>
  	z: 0.08179099886342922<br>
  	w: 0.996649503338522</p>
