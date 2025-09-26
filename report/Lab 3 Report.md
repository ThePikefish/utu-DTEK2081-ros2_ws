## Task 1 - Basic image processing with OpenCV
- [x] Requirements
- Add default variable parameters for easy settings
- [x] Draw shapes and text using cv2 methods like in the examples
  - The locations and parameters are hard-coded to somewhere in the image
- [x] Draw date and time with `datetime`
- [x] Save the image with `cv2.imwrite(filename, img)`

Result:
<img width="686" height="430" alt="image" src="https://github.com/user-attachments/assets/d7204971-fd37-42f4-af2c-d4f7417881a0" />


## Task 2 - ROS2 image subscription
- [x] Add subscription to task2.py with `self.create_subscription` for type `CompressedImage` and hook to `compressed_image_callback()` (use the compressed version)
- [x] Convert `CompressedImage` to OpenCV format with `self.bridge.compressed_imgmsg_to_cv2(msg)`
- [x] Add features to video in `process_cv_image()`
  - [x] Border with `rectangle()`
  - [x] Background for the info with `rectangle()` and make it transparent with `addWeighted()`
  - [x] Infos with `putText()`
    - [x] Frame counter from iterating counter `processed_count`
    - [x] Size from `width` and `height`
    - [x] Timestamp from `self.time` which is `msg.header.stamp`, and formatting
    - [x] Elapsed time from `self.time - self.start_time`

Trying to run `python3 task2.py` results in this error:
```
A module that was compiled using NumPy 1.x cannot be run in
NumPy 2.2.6 as it may crash. To support both 1.x and 2.x
versions of NumPy, modules must be compiled with NumPy 2.0.
Some module may need to rebuild instead e.g. with 'pybind11>=2.12'.

If you are a user of the module, the easiest solution will be to
downgrade to 'numpy<2' or try to upgrade the affected module.
We expect that some modules will need time to support NumPy 2.
```
- Downgrade numpy with `pip install "numpy<2.0"` which results in this:
```
ERROR: pip's dependency resolver does not currently take into account all the packages that are installed. This behaviour is the source of the following dependency conflicts.
opencv-python 4.12.0.88 requires numpy<2.3.0,>=2; python_version >= "3.9", but you have numpy 1.26.4 which is incompatible.
Successfully installed numpy-1.26.4
```
- Ignoring the error as it seems to work now

Terminal 1:
```
ros2 bag play rosbags/lab2_rosbag
```
Terminal 2:
```
python3 task2.py
```

Result:
<img width="1469" height="832" alt="image" src="https://github.com/user-attachments/assets/79fa2518-feab-464b-bc5f-a2e9c5d30462" />
