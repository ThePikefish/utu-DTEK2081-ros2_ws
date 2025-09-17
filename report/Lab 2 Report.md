## Task 1
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
