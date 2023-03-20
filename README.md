# ME495 Sensing, Navigation and Machine Learning For Robotics
* Ritika Ghosh
* Winter 2022
# Package List
This repository consists of several ROS packages
- [nuturtle_description](https://github.com/ME495-Navigation/nuturtle-Ritika521/tree/main/nuturtle_description) - Visualization of multiple turtle burger robots in rviz simultaneously.
- [nusim](https://github.com/ME495-Navigation/nuturtle-Ritika521/tree/main/nusim) - Creates a simulated environment for turtle robot.
- [nuturtle_control](https://github.com/ME495-Navigation/nuturtle-Ritika521/tree/main/nuturtle_control) - Controls the robot in real world and in simulation.
- [nuslam](https://github.com/ME495-Navigation/nuturtle-Ritika521/tree/main/nuslam) - Implements EKF to visualize the robot doing slam in simulation.

    [Turtlebot_simulation.mp4](https://user-images.githubusercontent.com/60728026/217688212-0d05f264-89fa-48e9-8433-4c95c98bdbc9.mp4)

    [Turtlebot_realworld](https://user-images.githubusercontent.com/60728026/217689574-8d78a9b8-6175-47cf-8473-8954e2625b56.mp4)

	![EKFwithNoise](https://user-images.githubusercontent.com/60728026/224211956-4987d0ea-dc35-46b8-b597-9c5b150dbde8.png)

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

The following is a video showing the actual and estimated robot position as well as the actual, estimated and odometry paths (2xfaster).


[landmarks4_C.webm](https://user-images.githubusercontent.com/60728026/225818625-985cc2ba-d4b9-455a-84d0-9d6699ee95e3.webm)


### References:
- https://www.cs.colostate.edu/~cs253/Spring19/Lecture/Stringstreams
- https://stackoverflow.com/questions/31815230/how-to-test-input-and-output-overloaded-operator-in-c-gtest
- https://answers.ros.org/question/282902/pass-parameters-to-xacro-from-launch-file-or-otherwise/
- https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html#robot-state-publisher
- https://www.theconstructsim.com/how-to-migrate-launch-files-in-xml-format-from-ros1-to-ros2/
- https://answers.ros.org/question/372752/static_transform_publisher-in-ros2-launch-file/
- https://stackoverflow.com/questions/70799228/how-to-cast-a-double-into-stdchronomilliseconds
- https://www.includehelp.com/stl/create-an-empty-vector-and-initialize-by-pushing-values.aspx
- https://answers.ros.org/question/354205/ros2-how-to-get-an-array-from-get_parameter/
- https://answers.ros.org/question/35246/add-markers-to-markerarray-in-c/
- https://github.com/ros2/design/blob/d3a35d7ea201721892993e85e28a5a223cdaa001/articles/151_roslaunch_xml.md 