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

(Picture)

- [x] Plot...