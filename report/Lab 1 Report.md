
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
	- <img width="187" height="52" alt="Pasted image 20250904130428" src="https://github.com/user-attachments/assets/b6581817-4a6b-4b96-8db1-65005301e3cc" />

- The `echo` showed the header and the pose updating
	- <img width="213" height="273" alt="Pasted image 20250904130406" src="https://github.com/user-attachments/assets/04c036b6-7525-4a30-ad22-0a37f7b82b56" />

- The `hz` showed the average rate of 1.000 with slight changes between the min and the max
	- <img width="493" height="158" alt="Pasted image 20250904130242" src="https://github.com/user-attachments/assets/131a765a-4e59-4ecb-a63a-bc00212afe62" />

- The `info` showed the type of `PoseStamped` and 1 publisher and 1 subscription
	- <img width="334" height="51" alt="Pasted image 20250904130313" src="https://github.com/user-attachments/assets/8a84e520-3a3e-44d8-b629-f6cbb03a7ea3" />

	- The `info` didn't seem to work without the `/` -symbol like the other commands did

<img width="989" height="254" alt="Publisher and subscriber for Pose" src="https://github.com/user-attachments/assets/5f1a6b3d-c600-482b-a151-017bb39b0d51" />



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
- Ran commands below:

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
<img width="991" height="150" alt="Publisher and subscriber for Person interface" src="https://github.com/user-attachments/assets/afa46831-56e7-4590-8c0b-c1463d2d7851" />



## Task 3 - Services
___

### Tasks done

- Created new package `service_example` according to the guide
- Created `CalculateDistance.srv` service to `custom_interface` package
- Created server and client nodes to the new package
- Added dependencies (e.g `geometry_msgs` and `builtin_interfaces` in `CMakeLists.txt`) and entry points
- Ran commands below:

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
<img width="1035" height="117" alt="Pasted image 20250907155920" src="https://github.com/user-attachments/assets/d8d0b6a4-d0aa-4ab5-b4b1-dfcf1a48f274" />


- The custom service
<img width="254" height="211" alt="Pasted image 20250907160049" src="https://github.com/user-attachments/assets/caa1227f-c6fd-4ca7-9481-b48908cb4b02" />



## Task 4 - Parameters
___

### Tasks done

- Added parameter deceleration for points as lists to `calculate_distance_client.py`
- Added parameter gets to the old point variables
- Added the node and the parameters to the `params.yaml` file which was created according to the guide
- Ran commands below:

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
<img width="1035" height="117" alt="Pasted image 20250907155920" src="https://github.com/user-attachments/assets/f3ec7ff6-fa28-4111-aea7-59758b5d14ee" />

- This was ran with different values in the yaml file to confirm it's indeed reacting to changes


## Task 5 - Actions
___

### Tasks done
- Created a file `Countdown.action` in the package custom_inteface/action with the given format, and modify the CMakeLists.txt
- Created a package called countdown, ros2 pkg create --build-type ament_python countdown --dependencies rclpy custom_interface
- Created countdown_server.py and countdown_client.py, add code based on the fibonacci examples
- 


Terminal 1
```
colcon build
sb

ros2 run countdown countdown_server
```

Terminal 2
```
sb

ros2 run countdown countdown_client --ros-args -p start_from:=5
```

### Results

- The server and the client successfully communicate with each other with client send 5 and count down from it
![](task5_1.png)





