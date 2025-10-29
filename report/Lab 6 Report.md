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

## Task 2

- [x] Created `build_graph_waypoints.py`, use the example code to create the corresponding adjacency matrix and weight matrix, and draw graph.
- [x] Ran the file `./build_graph_waypoints.py` and get the following results
<img width="1980" height="1320" alt="graph" src="https://github.com/user-attachments/assets/1987950f-ec7a-4389-932f-1ab395efa883" />
<img width="504" height="538" alt="task2" src="https://github.com/user-attachments/assets/6a462022-0262-4534-af6d-351408157a32" />
- [x] Created `dijkstra_path.py`, 

colcon build --packages-select group2_navigation_stack
source install/setup.bash

terminal 1
```
ros2 launch group2_navigation_stack lab6_gazebo.launch.py
```

terminal 2
```
ros2 run group2_navigation_stack dijkstra_path
```

Set the goal is the point 9, and start from point 0, the result is
<img width="862" height="50" alt="t2t1" src="https://github.com/user-attachments/assets/fd69f604-016a-4ad6-9d25-227bf101b999" />

In Gazebo, move the robot to the place near point 2 and point 3, the goal is still point 9, the result is 
<img width="799" height="49" alt="t2t2" src="https://github.com/user-attachments/assets/bc1c7aac-4203-4031-92b5-589cad7f5053" />

Set the goal is the point 8, and start from point 0, the result is
<img width="811" height="47" alt="t2t3" src="https://github.com/user-attachments/assets/b9ebd5d7-71be-4af8-aa23-04bd9a06afcb" />

The recorded viedos can be found in the .. folder.


