- [x] Initialize terminals with with ros domain id of 2 and source
- [x] Play the provided rosbag on the backgroun with `ros2 bag play <ros bag folder name>`
## Task 1 - Feature detector
- [x] In `rviz2`:
    - [x] Fixed Frame to `base_scan`
    - [x] Add the *LaserScan* topic
    - [x] Change the topics *Reliability Policy* to `Best Effort`
    - [x] Change *Size (m)* to `0.05`
    - Now it looks like in the picture in the assignment
- [x] Create new ros2 package with `ros2 pkg create --build-type ament_python lab5_feature_detector`
- [x] Use the *Task1_feature_detector.py* as a starting point
    - */Scan* topic subscription
    - Publishes to topic */feature_scan*
- [x] Add entry point for the node in *setup.py*
- [x] Run `colcon build --packages-select lab5_feature_detector` and source
- [x] Run `ros2 run lab5_feature_detector feature_extracter`
- [x] In *rviz2*
    - [x] Add the *LaserScan* topic under */feature_scan* and change the same settings as earlier

(Picture)
Some of the points are missing in */feature_scan* when compared to */scan* from specific areas. They are being filltered out by the code in this part:
```
for r in msg.ranges:
            if r > 2.5 or r < 1:   # Feature extraction code here
                ranges.append(0.0)
            else:
                ranges.append(r)
```
If the distance check `if r > 2.5 or r < 1` is true, it's replaced with `0.0`.
Therefore the */feature_scan* only shows points between distances of 1.0 and 2.5 meters.

- [x] Add */front_scan* filter below the */feature_scan*
    - [x] Copy-paste the scan message creation from */feature_scan*
    - [x] Change the for loop (the filter) to check for angles
- [x] In *rviz2*
    - [x] Add the new topic similarly
    - [x] Adjust the visualization settings so each topic looks different

<img width="1033" height="741" alt="image" src="https://github.com/user-attachments/assets/d9d42b34-babc-40b1-bda3-bb046de1c618" />


- [x] Converted to points to plot-friendly format
- Used AI (gemini) to help with the polar-to-cartesian conversion
- [x] There was a problem with QoS mitchmatches so adjusted the policy

<img width="1033" height="741" alt="image" src="https://github.com/user-attachments/assets/5e9659a7-3199-4301-af51-ad971ee91d84" />


## Task 2 - Corner and line detection
- [x] Use the earliers codes as a starting point (cartesian conversion, overall structure, etc.)
- [x] *detect_corners* looks at a small window at time and iterates through the points and calls angle calculation on points
- [x] Convert to PointCloud(2) and publish
- Used AI (gemini) to help with understanding the math for calculating angles
<img width="1033" height="741" alt="image" src="https://github.com/user-attachments/assets/d4a9ee14-0e14-4aed-b867-0b44cad702a0" />



## Task 3 - Grid map 

- [x] Create new ros2 package with `ros2 pkg create --build-type ament_python lidar_grid_map`
- [x] Create a python file grid_mapper.py
    - [x] Create a new ROS node that subscribes to the /odom and /scan topics
    - [x] Use bresenham_points from example code
    - [x] Move the robot, and save the snapshot every 15s
    - [x] Store and save the final map to final_map.csv
- terminal 1, run `ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage4.launch.py`
  <img width="764" height="738" alt="lab5t3" src="https://github.com/user-attachments/assets/b215b639-9257-45e9-9633-5f9f4c68e621" />

- terminal 2, run `ros2 run lidar_grid_map grid_mapper`
- terminal 3, run `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
<img width="1633" height="922" alt="lab5t3b" src="https://github.com/user-attachments/assets/83fcd109-db9a-4ddc-9222-710dfc308300" />

## Task 4 - PF
- terminal 1, run `ros2 launch turtlebot3_gazebo empty_world.launch.py`, add a a coke can
<img width="1053" height="792" alt="lab5task4" src="https://github.com/user-attachments/assets/953b78e6-a093-4b5b-b02b-3da40d63b95e" />
 
- [x] Modify Task4_pfilter_template.py
    - [x] Change obstacle position
    - [x] Change def update_filter(self)
    - [x] In main(), add code to draw a plot and save to a png file

- terminal 2, run `./Task4_pfilter_template.py` (Before that install pfilter and modify #!/usr/bin/env python to #!/usr/bin/env python3)
- terminal 3, run `ros2 run teleop_twist_keyboard teleop_twist_keyboard`, and go straight
<img width="821" height="817" alt="pf_vs_odom" src="https://github.com/user-attachments/assets/aed3d942-9723-492e-a247-9a7d92e89917" />


