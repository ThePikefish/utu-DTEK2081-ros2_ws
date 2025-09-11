
## Task 1 - Topics
___

### Tasks done

- Used the provided publisher/subscriber code examples as a starting point
- Edited message formats to `PoseStamped`
- Added `pose.position` iteration
- Ran commands below:

Terminal 1:
```
colcon build
source install/setup.bash

ros2 run pose_package publisher
```
Terminal 2:
```
source install/setup.bash

ros2 run pose_package subscriber
```

Terminal 3:
```
source install/setup.bash

ros2 topic list
ros2 topic echo Pose
ros2 topic hz Pose
ros2 topic info /Pose
```

### Results

- The `list` showed the /Pose
	- ![[Pasted image 20250904130428.png]]
- The `echo` showed the header and the pose updating
	- ![[Pasted image 20250904130406.png]]
- The `hz` showed the average rate of 1.000 with slight changes between the min and the max
	- ![[Pasted image 20250904130242.png]]
- The `info` showed the type of `PoseStamped` and 1 publisher and 1 subscription
	- ![[Pasted image 20250904130313.png]]
	- The `info` didn't seem to work without the `/` -symbol like the other commands did

![[Publisher and subscriber for Pose.png|%]]


## Task 2 - Custom messaging
___

### Tasks done

- Created the C++ `custom_interface` package and the format according to the guide
- Created a new ROS2 Python package `custom_pinterface` with:
```
ros2 pkg create --build-type ament_python custom_pinterface --license Apache-2.0 --dependencies rclpy std_msgs geometry_msgs
```
- Used the provided publisher/subscriber code examples as a starting point
- Changed msg type and formatting
- Added dependencies
- Ran commands bellow:

Terminal 1
```
colcon build
sb

ros2 run custom_pinterface publisher
```

Terminal 2
```
sb

ros2 run custom_pinterface subscriber
```
### Results

- The publisher and subscriber successfully send and receive the Person messages
![[Publisher and subscriber for Person interface.png|%]]


## Task 3 - Services
___

### Tasks done

- Created new package `service_example` according to the guide
- Created `CalculateDistance.srv` service to `custom_interface` package
- Created server and client nodes to the new package
- Added dependencies (e.g `geometry_msgs` and `builtin_interfaces` in `CMakeLists.txt`) and entry points
- Ran commands bellow:

Terminal 1
```
colcon build
sb

ros2 interface show custom_interface/srv/CalculateDistance

ros2 run service_example calculate_distance_server 
```

Terminal 2
```
sb

ros2 run service_example calculate_distance_client
```

### Results

- The server and the client successfully communicate with each other with client receiving the calculated distance
![[Pasted image 20250907155920.png]]

- The custom service
![[Pasted image 20250907160049.png]]


## Task 4 - Parameters
___

### Tasks done

- Added parameter deceleration for points as lists to `calculate_distance_client.py`
- Added parameter gets to the old point variables
- Added the node and the parameters to the `params.yaml` file which was created according to the guide
- Ran command below:

Terminal 1
```
colcon build
sb

ros2 run service_example calculate_distance_server
```

Terminal 2
```
sb

ros2 run service_example calculate_distance_client --ros-args --params-file src/service_example/config/params.yaml
```

### Results

- The server and the client successfully communicate with each other with client receiving the calculated distance based on the yaml file
![[Pasted image 20250907155920.png]]
- This was ran with different values in the yaml file to confirm it's indeed reacting to changes


## Task 5 - Actions
___
