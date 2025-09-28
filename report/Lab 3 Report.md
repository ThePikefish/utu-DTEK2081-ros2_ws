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


## Task 3 - Color filtering with RGB and HSV
RGB ranges:
```
ranges = {
  'red': ([0, 0, 220], [50, 50, 255]),
  'green': ([0, 200, 0], [50, 255, 50]),
  'blue': ([220, 0, 0], [255, 80, 80]),
  'yellow': ([0, 220, 200], [10, 255, 255]),
  'purple': ([150, 0, 150], [250, 10, 190])
}
```
HSV ranges:
```
ranges = {
  'red': [
    ([0, 50, 10], [10, 255, 255]),    # low reds
    ([170, 50, 10], [180, 255, 255])  # high reds
  ],
  'green': [
    ([40, 10, 10], [86, 255, 255])
  ],
  'blue': [
    ([94, 10, 10], [126, 255, 255])
  ],
  'yellow': [
    ([28, 10, 10], [32, 255, 255])
  ],
  'purple': [
    ([140, 10, 10], [160, 255, 255])
  ]
}
```

- [x] Convert lists to numpy arrays with `np.array()`
- [x] Mask with `cv2.inRange(image, lower_np, upper_np)` and `cv2.bitwise_and(image, image, mask=mask)`
- [x] Convert BGR to HSV with `cv2.cvtColor(image, cv2.COLOR_BGR2HSV)`

In most situations, it was easier to define the HSV ranges as they represent more understandable features of the colors. Also, e.g. the green color of the wheel doesn't align perfectly with the "green" of RGB, which led to some surprising results first. Hence, when trying to fit these filters to real-life scenarios, the understandable parameters of HSV work much better.

Run with:
```
python3 task3.py
```

Results:
<img width="1288" height="903" alt="image" src="https://github.com/user-attachments/assets/a5373a20-daec-4e3e-8de4-3d8e1533b586" />

<img width="770" height="935" alt="image" src="https://github.com/user-attachments/assets/5b681e49-ca04-4989-8488-98a3c007865f" />


## Task 4
- [x] Add subscription to task4.py with `self.create_subscription` for type `Image` and hook to `image_callback()` (There doesn't seem to be a compressed topic in the rosbag)
- [x] Similar steps as in tasks 2 and 3 for task4.py (conversions, masks, filter ranges, etc.)
- [x] For showing the grid:
```
top = np.hstack((original, red))
bottom = np.hstack((green, blue))
grid = np.vstack((top, bottom))
```

For filtering, HSV was selected for the same reason as stated in task 3. The parameters of HSV make more sense when considering colors.

Terminal 1:
```
ros2 bag play rosbags/lab3_2
```
Terminal 2:
```
python3 task4.py
```
<img width="982" height="706" alt="lab3task6" src="https://github.com/user-attachments/assets/8423e1cc-5fbc-4d4b-9396-c6865e287874" />

Result:
<img width="987" height="727" alt="image" src="https://github.com/user-attachments/assets/0e24f3d3-2175-47df-ac69-cc9595faf962" />


## Task 5
- [x] Use Reflection (Mirror) padding
- [x] Try two diﬀerent kernel sizes: 3x3 and 5x5. Bigger kernels blur more because they average over a larger area, losing more detail and softening edges.
- [x] Box blur uses uniform weights and can introduce blockiness and slight halos. Gaussian uses smoothly decaying weights, producing more natural smoothing with fewer artifacts. In the results, the 5×5 box blur shows chunkier petal edges and faint halos, but the 5×5 Gaussian (σ=1) has smoother transitions along the petals and a cleaner background.
- [x] Ran `python3 task5.py`

<img width="970" height="704" alt="lab3task5" src="https://github.com/user-attachments/assets/e32620ff-7336-4d36-860d-f12b6204288f" />

## Task 6
- [x] Created the sobel_x and sobel_y kernels
- [x] Ran `python3 task6.py`

<img width="982" height="706" alt="lab3task6" src="https://github.com/user-attachments/assets/08fa0479-9f6e-4805-ae35-0c6c21325fc8" />


## Task 7
- [x] Created the sobel_x and sobel_y kernels
- [x] Ran `python3 task6.py`


