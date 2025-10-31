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
The grid_mapper.py will save the map to OUTPUT_DIR = os.path.expanduser('/home/ws/maps'). When run it, the directory maybe need to changed for the reason of permission requirement.

terminal 3
```
ros2 run group2_navigation_stack waypoint_follower
```

<img width="851" height="869" alt="traj_on_map" src="https://github.com/user-attachments/assets/465dff12-9b85-4a30-acaf-b4ed79360dc7" />

<img width="1157" height="908" alt="twist_vs_time" src="https://github.com/user-attachments/assets/6da92aa0-76d2-4fb2-9c80-156936d55169" />

## Task 2 - Dijkstra

- [x] Created `build_graph_waypoints.py`, use the example code to create the corresponding adjacency matrix and weight matrix, and draw graph.
- [x] Ran the file `./build_graph_waypoints.py` and get the following results
<img width="1980" height="1320" alt="graph" src="https://github.com/user-attachments/assets/1987950f-ec7a-4389-932f-1ab395efa883" />
<img width="504" height="538" alt="task2" src="https://github.com/user-attachments/assets/6a462022-0262-4534-af6d-351408157a32" />

- [x] Created `dijkstra_path.py` 

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

The recorded videos can be found in the lab6output folder.

## Task 3 - A*
- [x] Created `a_star_path.py`.
  - [x] Copy-pasted the `dijkstra_path.py` code for a starting point.
  - [x] Modified the naming and calls for a_star.
  - [x] Replaced `dijkstra` function with `a_star` function (main difference),
    - Using g_score similarly to `dist`
    - Using h_score as a heuristic by calculating Euclidean distance
    - f_score as their sum.
    - Similar neighbour exploration implementation, adjusted to A*.
```
colcon build --packages-select group2_navigation_stack
source install/setup.bash
```

Terminal 1
```
ros2 launch group2_navigation_stack lab6_gazebo.launch.py
```

Terminal 2
```
ros2 run group2_navigation_stack a_star_path
```

### Testing results with the same three paths as in task 2:

0 -> 9
```
[dijkstra_path]: Explored nodes: [0, 1, 2, 6, 3, 5, 4, 7, 8, 9]
[dijkstra_path]: Planned once using Dijkstra: 0 -> 1 -> 6 -> 5 -> 4 -> 9  (length=9.12 m)

[a_star_path]: Explored nodes: [0, 1, 2, 6, 5, 3, 4, 7, 8, 9]
[a_star_path]: Planned once using A*: 0 -> 1 -> 6 -> 5 -> 4 -> 9  (length=9.12 m)
```

3 -> 9
```
[dijkstra_path]: Explored nodes: [3, 2, 4, 5, 1, 0, 6, 9]
[dijkstra_path]: Planned once using Dijkstra: 3 -> 9  (length=4.84 m)

[a_star_path]: Explored nodes: [3, 2, 4, 5, 1, 6, 0, 9]
[a_star_path]: Planned once using A*: 3 -> 9  (length=4.84 m)
```

0 -> 8
```
[dijkstra_path]: Explored nodes: [0, 1, 2, 6, 3, 5, 4, 7, 8]
[dijkstra_path]: Planned once using Dijkstra: 0 -> 1 -> 6 -> 7 -> 8  (length=8.16 m)

[a_star_path]: Explored nodes: [0, 1, 2, 6, 5, 3, 4, 7, 8]
[a_star_path]: Planned once using A*: 0 -> 1 -> 6 -> 7 -> 8  (length=8.16 m)
```

Conclusion:
- All paths are the same, as both algorithms find the best path.
- There is a slight difference in exploration order. In these cases, when examining the grid map, the A* algorithm appears to prioritise better towards the goal as expected.
- The count of explored nodes stays the same. This is likely because the map is very simple. 

## Task 4 - Dijkstra through all waypoints
- [x] Created `extended_dijkstra_path.py`.
  - [x] Copy-pasted the `dijkstra_path.py` code for a starting point.
  - [x] Modified the naming and calls
  - [x] `plan_path()` replaced with `plan_next_path()`
    - Now also keeps track of visited and unvisited nodes.
    - Loop through all unvisited nodes with Dijkstra and select the nearest one.
  - [x] `self.planned` in `loop()` is not set back to False after a run (unless it's the last).
  - [x] Create `plot_trajectory()` which is called when the task is completed.
    - Plotting code is mostly adapted from `waypoint_follower.py`.
```
colcon build --packages-select group2_navigation_stack
source install/setup.bash
```

Terminal 1
```
ros2 launch group2_navigation_stack lab6_gazebo.launch.py
```

Terminal 2
```
ros2 run group2_navigation_stack grid_mapper
```

Terminal 3
```
ros2 run group2_navigation_stack extended_dijkstra_path
```

### Testing results with two different cycles:

Starting from node 0:
```
[INFO] [1761839816.780492277] [extended_dijkstra_path]: DijkstraPath ready. Move robot to a random pose, set GOAL_IDX.
[INFO] [1761839816.791937464] [extended_dijkstra_path]: Arrived at node 0. Unvisited: {1, 2, 3, 4, 5, 6, 7, 8, 9}
[INFO] [1761839816.793788072] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 1 with path: 0 -> 1  (length=1.43 m)
[INFO] [1761839824.742380193] [extended_dijkstra_path]: Reached node 1.
[INFO] [1761839824.791462453] [extended_dijkstra_path]: Arrived at node 1. Unvisited: {2, 3, 4, 5, 6, 7, 8, 9}
[INFO] [1761839824.793721468] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 2 with path: 1 -> 2  (length=1.40 m)
[INFO] [1761839833.291597176] [extended_dijkstra_path]: Reached node 2.
[INFO] [1761839833.341563881] [extended_dijkstra_path]: Arrived at node 2. Unvisited: {3, 4, 5, 6, 7, 8, 9}
[INFO] [1761839833.343042858] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 3 with path: 2 -> 3  (length=1.70 m)
[INFO] [1761839843.792465703] [extended_dijkstra_path]: Reached node 3.
[INFO] [1761839843.841770963] [extended_dijkstra_path]: Arrived at node 3. Unvisited: {4, 5, 6, 7, 8, 9}
[INFO] [1761839843.843122201] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 4 with path: 3 -> 4  (length=1.80 m)
[INFO] [1761839855.241079218] [extended_dijkstra_path]: Reached node 4.
[INFO] [1761839855.291276082] [extended_dijkstra_path]: Arrived at node 4. Unvisited: {5, 6, 7, 8, 9}
[INFO] [1761839855.292423857] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 5 with path: 4 -> 5  (length=1.30 m)
[INFO] [1761839863.991583520] [extended_dijkstra_path]: Reached node 5.
[INFO] [1761839864.046111406] [extended_dijkstra_path]: Arrived at node 5. Unvisited: {6, 7, 8, 9}
[INFO] [1761839864.048409328] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 6 with path: 5 -> 6  (length=1.50 m)
[INFO] [1761839873.391329361] [extended_dijkstra_path]: Reached node 6.
[INFO] [1761839873.441477368] [extended_dijkstra_path]: Arrived at node 6. Unvisited: {7, 8, 9}
[INFO] [1761839873.442951821] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 7 with path: 6 -> 7  (length=3.00 m)
[INFO] [1761839892.542057633] [extended_dijkstra_path]: Reached node 7.
[INFO] [1761839892.591230898] [extended_dijkstra_path]: Arrived at node 7. Unvisited: {8, 9}
[INFO] [1761839892.592521083] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 8 with path: 7 -> 8  (length=1.90 m)
[INFO] [1761839905.091601077] [extended_dijkstra_path]: Reached node 8.
[INFO] [1761839905.140933434] [extended_dijkstra_path]: Arrived at node 8. Unvisited: {9}
[INFO] [1761839905.141897900] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 9 with path: 8 -> 9  (length=1.50 m)
[INFO] [1761839914.191855683] [extended_dijkstra_path]: Reached node 9.
[INFO] [1761839914.241152865] [extended_dijkstra_path]: Arrived at node 9. Unvisited: set()
[INFO] [1761839914.241927886] [extended_dijkstra_path]: All nodes visited. DONE
[INFO] [1761839914.242604458] [extended_dijkstra_path]: Exploration order: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
```
<img width="842" height="843" alt="image" src="https://github.com/user-attachments/assets/ccf10a45-4d32-4a25-b361-354e4681d8d1" />

Starting from node 5:
```
[INFO] [1761841793.653379790] [extended_dijkstra_path]: DijkstraPath ready. Move robot to a random pose, set GOAL_IDX.
[INFO] [1761841794.765527487] [extended_dijkstra_path]: Arrived at node 5. Unvisited: {0, 1, 2, 3, 4, 6, 7, 8, 9}
[INFO] [1761841794.767204389] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 4 with path: 5 -> 4  (length=1.30 m)
[INFO] [1761841800.815085693] [extended_dijkstra_path]: Reached node 4.
[INFO] [1761841800.865049631] [extended_dijkstra_path]: Arrived at node 4. Unvisited: {0, 1, 2, 3, 6, 7, 8, 9}
[INFO] [1761841800.871556210] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 3 with path: 4 -> 3  (length=1.80 m)
[INFO] [1761841812.365188577] [extended_dijkstra_path]: Reached node 3.
[INFO] [1761841812.414870529] [extended_dijkstra_path]: Arrived at node 3. Unvisited: {0, 1, 2, 6, 7, 8, 9}
[INFO] [1761841812.416949285] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 2 with path: 3 -> 2  (length=1.70 m)
[INFO] [1761841823.514717416] [extended_dijkstra_path]: Reached node 2.
[INFO] [1761841823.564996906] [extended_dijkstra_path]: Arrived at node 2. Unvisited: {0, 1, 6, 7, 8, 9}
[INFO] [1761841823.566646210] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 1 with path: 2 -> 1  (length=1.40 m)
[INFO] [1761841831.819126711] [extended_dijkstra_path]: Reached node 1.
[INFO] [1761841831.865459001] [extended_dijkstra_path]: Arrived at node 1. Unvisited: {0, 6, 7, 8, 9}
[INFO] [1761841831.866894920] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 0 with path: 1 -> 0  (length=1.43 m)
[INFO] [1761841840.315207301] [extended_dijkstra_path]: Reached node 0.
[INFO] [1761841840.365185580] [extended_dijkstra_path]: Arrived at node 0. Unvisited: {6, 7, 8, 9}
[INFO] [1761841840.368490457] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 6 with path: 0 -> 1 -> 6  (length=3.26 m)
[INFO] [1761841862.515870532] [extended_dijkstra_path]: Reached node 6.
[INFO] [1761841862.565166938] [extended_dijkstra_path]: Arrived at node 6. Unvisited: {7, 8, 9}
[INFO] [1761841862.566843710] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 7 with path: 6 -> 7  (length=3.00 m)
[INFO] [1761841880.566214503] [extended_dijkstra_path]: Reached node 7.
[INFO] [1761841880.614819851] [extended_dijkstra_path]: Arrived at node 7. Unvisited: {8, 9}
[INFO] [1761841880.615678325] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 8 with path: 7 -> 8  (length=1.90 m)
[INFO] [1761841893.365874943] [extended_dijkstra_path]: Reached node 8.
[INFO] [1761841893.415173130] [extended_dijkstra_path]: Arrived at node 8. Unvisited: {9}
[INFO] [1761841893.416224370] [extended_dijkstra_path]: Planned Dijkstra to shortest unvisited node: 9 with path: 8 -> 9  (length=1.50 m)
[INFO] [1761841903.216573765] [extended_dijkstra_path]: Reached node 9.
[INFO] [1761841903.264692578] [extended_dijkstra_path]: Arrived at node 9. Unvisited: set()
[INFO] [1761841903.265310891] [extended_dijkstra_path]: All nodes visited. DONE
[INFO] [1761841903.265815977] [extended_dijkstra_path]: Exploration order: [5, 4, 3, 2, 1, 0, 6, 7, 8, 9]
```
<img width="841" height="847" alt="image" src="https://github.com/user-attachments/assets/b979efb1-e5f2-4338-83fe-a5d8676227ff" />
