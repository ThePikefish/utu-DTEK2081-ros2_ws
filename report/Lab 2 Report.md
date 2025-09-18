## Task 1 - rqt_graph
- [x] Installed required packages
- [x] Ran commands below in the workspace:

Terminal 1:
```
source /opt/ros/humble/setup.bash

ros2 run turtlesim turtlesim_node
```
Terminal 2:
```
source /opt/ros/humble/setup.bash

ros2 run turtlesim turtle_teleop_key
```
Terminal 3:
```
source /opt/ros/humble/setup.bash

ros2 run rqt_graph rqt_graph
```

- [x] In *rqt_graph*, change view to *Node/Topics*
  - The *Node/Topics* -view shows a more comprehensive view of the topic, highlighting its inheritance.
  - <img width="946" height="396" alt="image" src="https://github.com/user-attachments/assets/7e742c97-7e80-435b-98c6-33a52aa4c487" />
  - `/teleop_turtle` is publishing the move commands to the `/turtle1/cmd_vel` topic, which `/turtlesim` subscribes to. To get the absolute rotations right, the turtle gives feedback and results back to the `/teleop_turtle` with action topics.

- [x] Open the *Topic Monitor* plugin from `rqt`
  - `/turtle1/cmd_vel` is responsible for moving the turtlebot. From the *Topic Monitor*, we can see that the message is `geometry_msgs/msgs/Twist` with linear and angular double vectors.
  - <img width="850" height="478" alt="image" src="https://github.com/user-attachments/assets/3cbe88d7-ee46-4ce5-9307-409e3f490093" />

- [x] Open the *Plot* plugin from `rqt`
- [x] Add `/turtle1/pose/x` and `/turtle1/pose/y` to the plot
- [x] Move the turtlebot from `turtle_teleop_key` node
  - The position gets plotted
  - <img width="831" height="673" alt="image" src="https://github.com/user-attachments/assets/0f0a130d-52f2-4db6-a098-030c6dcdfcd4" />


## Task 2 - rviz
- [x] Previous example
- [x] In *rviz* select `base_link` for fixed frame
- [x] Add *Odometry*, *LaserScan*, *Camera* and *TF* and play with their parameters.
  - The *Odometry* shows an arrow on the robot. When the fixed frame is set to `odom`, it spawns multiple arrows that mark the robot's path. The visualisation can be easily configured, e.g. the frequency, amount and size.
  - The *LaserScan* shows a visualisation of the lidar sensor as shapes whose properties can be adjusted, e.g. the delay, size, shape and color.
  - The *TF* shows the transformations of the robot's frames (mainly the different parts of the robot). There, you can, for example, toggle some parts off and show their names to make the visualisation clearer.
  - The *Camera* gives a first-person view of the robot's camera with a few basic view settings.
- [x] Move the robot around the map with the `ros2 run turtlebot3_teleop teleop_keyboard` in another terminal.
  - First, this command gave an error about the model, but after exporting the model (and sourcing ROS) again according to the guide, it worked.
  - <img width="342" height="257" alt="image" src="https://github.com/user-attachments/assets/a05719c5-214d-4b63-ba6d-e1b343424d17" />
  - <img width="340" height="251" alt="image" src="https://github.com/user-attachments/assets/d3fa6018-c4cf-48e7-9733-f97bddc9df10" />
- [x] Changed parameters:
  - The *Covariance* had to be turned off on the *Odometry* so it doesn't block the view.
  - The fixed frame had to be set from `base_link` to `odom` for visualising the path of the robot.
- [x] Difference with *burger* and *waffle_pi*:
  - The *burger* did not seem to have a camera signal, but *waffle_pi* had it. Otherwise, the other visualisations and controls seemed to work similarly on both of them.
