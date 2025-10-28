## Task 1

- [x] Created a package by running
```
ros2 pkg create --build-type ament_python group2_navigation_stack \
  --license Apache-2.0 \
  --dependencies rclpy std_msgs sensor_msgs nav_msgs geometry_msgs \
                launch launch_ros gazebo_ros turtlebot3_description
```
- [x] Created a `launch` folder and a `worlds` folder
- [x] Modified `setup.py` and `package.xml`
- [x] Create a world `lab6.world` in Gazebo

Use `ros2 launch group2_navigation_stack lab6_gazebo.launch.py` to launch both the custom world and the turtlebot. 


First in every Terminal, ran commands below:
```
source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

colcon build --packages-select group2_navigation_stack
source install/setup.bash

terminal 1
```
ros2 launch group2_navigation_stack lab6_gazebo.launch.py
```

terminal 2
```
ros2 run group2_navigation_stack grid_mapper
```

terminal 3
```
ros2 run group2_navigation_stack waypoint_follower
```

<img width="851" height="869" alt="traj_on_map" src="https://github.com/user-attachments/assets/465dff12-9b85-4a30-acaf-b4ed79360dc7" />

<img width="1157" height="908" alt="twist_vs_time" src="https://github.com/user-attachments/assets/6da92aa0-76d2-4fb2-9c80-156936d55169" />




