# Nuslam
* Ritika Ghosh
## Package Description:
This package publishes a transform from map to odom such that map to  base_footprint reflects the pose of the robot in the map frame. The EKF algorithm computes the pose of the robots and the pose of the landmarks relative to a map frame.
## Launch File
The `nuslam.launch.xml` launchfile launches the load_one.launch from nuturtle_description, the start_robot.launch.xml node, the slam node and the static broadcaster from nusim/world to map.
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
`ros2 launch nusim nuslam.launch.xml cmd_src:=teleop robot:=nusim`
or
`ros2 launch nusim nuslam.launch.xml cmd_src:=circle robot:=nusim`
### Real World
`ros2 launch nusim nuslam.launch.xml cmd_src:=circle robot:=localhost`
## Parameters:
*       wheel_radius (double): The radius of the wheels.
*       track_width (double): The distance between the wheels.
*       body_id(string): The name of the body frame of the robot.
*       odom_id(string): The name of the odometry frame..
*       wheel_left(string): The name of the left wheel joint.
*       wheel_right(string): The name of the right wheel joint.
*       sensor(bool): Depending on whether lidar or fake sensor is being used.

The following is an image showing the actual robot position, the estimated position, the map frame, the odom frame, and the paths of the real, odometry, and SLAM robots.

1. ![EKFwithNoise](https://user-images.githubusercontent.com/60728026/224211956-4987d0ea-dc35-46b8-b597-9c5b150dbde8.png)

2. ![EKFnoiseMany](https://user-images.githubusercontent.com/60728026/224212085-c52ddc29-45b8-49dc-84bd-e431790edb26.png)

The following is a video showing the actual and estimated robot position as well as the actual, estimated and odometry paths (2xfaster).


[Landmark_c.webm](https://github.com/GhoshRitika/SLAM_ekf/assets/60728026/74bff5cf-2bd3-4145-ad90-1bcdf864abdc)


![landmark4](https://user-images.githubusercontent.com/60728026/225818887-21deb855-d64b-4d93-8a4c-4d52f9f7a45c.png)

The final pose error between th actual robot position and odometry is:
    `x: 0.00404097`
    `y: 0.00929028`
    `z: 0`
The final pose error between th actual robot position and SLAM estimate is:
    `x: 0.008661882`
    `y: -0.005249775`
    `z: 0`