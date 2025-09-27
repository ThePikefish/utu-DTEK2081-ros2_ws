#!/usr/bin/env python3
"""
Task 2: ROS2 Image Processing with RGB/HSV Filters - Student Template
- Subscribes to raw and/or compressed image topics
- Resizes images to 500px wide
- Applies RGB and HSV filters (red, green, blue)
- Displays results in a 2x2 grid
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time


class ImageBagProcessor(Node):
    def __init__(self):
        super().__init__('image_bag_processor')
        topic_name = '/image_raw'

        self.bridge = CvBridge()
        
        # Counters
        self.processed_count = 0
        self.compressed_count = 0
        self.start_time = time.time()

        # Define the QoS profile for best effort
        qos = QoSProfile(
            depth=1,  # Keep a shallow history
            reliability=QoSReliabilityPolicy.BEST_EFFORT  # Set to best effort
        )
        
        # TODO: (DONE) Create subscribers for EITHER compressed and uncompressed images
        # Hint: self.create_subscription()

        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            qos)
        
        self.subscription
        
        self.processed_count = 0

        self.get_logger().info("Image Bag Processor initialized")


    # USE ONLY ONE OF THE CALL BACKS DEPENDING ON THE TOPIC TYPE
    # (Using not compressed as there doesn't seem to be a compressed topic in the rosbag)
    
    # -----------------------------
    # Callbacks 
    # -----------------------------
    def compressed_image_callback(self, msg):
        """Process compressed image messages"""
        try:
            # TODO: (DONE) Convert ROS compressed image to OpenCV (BGR8)
            # Hint: self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

            self.compressed_count += 1
            
            # TODO: (DONE) Process the image
            processed = self.process_cv_image(cv_image)
            
            # Display every 5th frame
            if self.compressed_count % 5 == 0:
                cv2.imshow('Filtering Grid', processed)
                cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error (compressed): {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {e}')
    
    # Using this
    def image_callback(self, msg):
        """Process raw image messages"""
        try:
            # TODO: (DONE) Convert ROS raw image to OpenCV (BGR8)
            # Hint: self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            self.processed_count += 1
            
            # TODO: (DONE) Process the image
            processed = self.process_cv_image(cv_image)
            
            # Display every 5th frame
            if self.processed_count % 5 == 0:
                cv2.imshow('Filtering Grid', processed)
                cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error (raw): {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing raw image: {e}')
    
    # -----------------------------
    # Image Processing
    # -----------------------------
    def process_cv_image(self, cv_image):
        """Resize, apply RGB/HSV filters, and stack in 2x2 grid"""

        new_width = 500
        height, width, _ = cv_image.shape
        aspect_ratio = height / width
        new_height = int(new_width * aspect_ratio)
        
        # Resize the image
        frame = cv2.resize(cv_image, (new_width, new_height))

        # CHOSE WHICH ONE YOU ARE USING RGB OR HSV

        # TODO: Apply RGB filters
        #rgb_results = self.apply_rgb_filters(frame)

        # TODO: (DONE) Apply HSV filters
        hsv_results = self.apply_hsv_filters(frame)

        # TODO: Annotate the original image with text
        original = frame.copy()
        cv2.putText(original, text=f'Original', org=(20, 20), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=(255, 255, 255), thickness=1)

        # TODO: (DONE) Choose which filtered images to display in the grid

        red = hsv_results['red']['filtered']
        green = hsv_results['green']['filtered']
        blue = hsv_results['blue']['filtered']

        cv2.putText(red, text=f'Red filter', org=(20, 20), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=(255, 255, 255), thickness=1)
        cv2.putText(green, text=f'Green filter', org=(20, 20), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=(255, 255, 255), thickness=1)
        cv2.putText(blue, text=f'Blue filter', org=(20, 20), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=(255, 255, 255), thickness=1)

        
        # TODO: (DONE) 2 x 2 grid display of original and 3 filtered images
        # Hint: np.hstack(image1, image2)
        top = np.hstack((original, red))
        bottom = np.hstack((green, blue))
        grid = np.vstack((top, bottom))
        
        return grid
    
    # -----------------------------
    # Color Filter Functions
    # -----------------------------
    def apply_rgb_filters(self, image):
        """Apply RGB color filtering"""
        # TODO: Define BGR color ranges for red, green, and blue
        # Hint: ranges = {'red': ([low_B, low_G, low_R], [high_B, high_G, high_R]), ...}
        ranges = {}

        results = {}
        for color, (lower, upper) in ranges.items():
            # TODO: Convert lists to NumPy arrays
            # TODO: Create mask with cv2.inRange()
            # TODO: Apply cv2.bitwise_and() to extract the color region
            mask = None
            filtered = image  # Replace with masked image
            results[color] = {'mask': mask, 'filtered': filtered}
        
        return results

    # Using this
    def apply_hsv_filters(self, image):
        """Apply HSV color filtering"""
        # TODO: (DONE) Convert image from BGR to HSV using cv2.cvtColor()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # TODO: (DONE) Define HSV ranges for red, green, and blue
        # Hint: Red requires TWO ranges (0–10 and 170–180 hue)
        ranges = {
            'red': [
                ([0, 100, 50], [10, 255, 255]),    # low reds
                ([170, 50, 10], [180, 255, 255])  # high reds
            ],
            'green': [
                ([40, 50, 50], [86, 255, 255])
            ],
            'blue': [
                ([95, 50, 50], [126, 255, 255])
            ]
        }

        results = {}
        for color, bounds_list in ranges.items():
            mask = None
            for lower, upper in bounds_list:
                # TODO: (DONE) Convert lower/upper to NumPy arrays
                # TODO: (DONE) Create mask using cv2.inRange()
                # TODO: (DONE) Combine masks using cv2.bitwise_or()
                lower_np = np.array(lower)
                upper_np = np.array(upper)

                new_mask = cv2.inRange(hsv, lower_np, upper_np)
                if mask is None:
                    mask = new_mask
                else:
                    mask = cv2.bitwise_or(mask, new_mask)
                pass
            
            # TODO: Extract the color region using cv2.bitwise_and()
            filtered = cv2.bitwise_and(image, image, mask=mask)
            results[color] = {'mask': mask, 'filtered': filtered}
        
        return results
    

def main(args=None):
    rclpy.init(args=args)
    processor = ImageBagProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
