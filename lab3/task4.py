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
        
        # TODO: Create subscribers for EITHER compressed and uncompressed images
        # Hint: self.create_subscription()
        
        self.processed_count = 0

        self.get_logger().info("Image Bag Processor initialized")


    # USE ONLY ONE OF THE CALL BACKS DEPENDING ON THE TOPIC TYPE
    
    # -----------------------------
    # Callbacks 
    # -----------------------------
    def compressed_image_callback(self, msg):
        """Process compressed image messages"""
        try:
            # TODO: Convert ROS compressed image to OpenCV (BGR8)
            # Hint: self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            cv_image = None

            self.compressed_count += 1
            
            # TODO: Process the image
            processed = self.process_cv_image(cv_image)
            
            # Display every 5th frame
            if self.compressed_count % 5 == 0:
                cv2.imshow('Filtering Grid', processed)
                cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error (compressed): {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {e}')
    
    def image_callback(self, msg):
        """Process raw image messages"""
        try:
            # TODO: Convert ROS raw image to OpenCV (BGR8)
            # Hint: self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = None

            self.processed_count += 1
            
            # TODO: Process the image
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
        rgb_results = self.apply_rgb_filters(frame)

        # TODO: Apply HSV filters
        hsv_results = self.apply_hsv_filters(frame)

        # TODO: Annotate the original image with text
        original = frame.copy()

        # TODO: Choose which filtered images to display in the grid
        # TODO: 2 x 2 grid display of original and 3 filtered images
        # Hint: np.hstack(image1, image2)
        grid = None
        
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


    def apply_hsv_filters(self, image):
        """Apply HSV color filtering"""
        # TODO: Convert image from BGR to HSV using cv2.cvtColor()
        hsv = None

        # TODO: Define HSV ranges for red, green, and blue
        # Hint: Red requires TWO ranges (0–10 and 170–180 hue)
        ranges = {}

        results = {}
        for color, bounds_list in ranges.items():
            mask = None
            for lower, upper in bounds_list:
                # TODO: Convert lower/upper to NumPy arrays
                # TODO: Create mask using cv2.inRange()
                # TODO: Combine masks using cv2.bitwise_or()
                pass
            
            # TODO: Extract the color region using cv2.bitwise_and()
            filtered = image
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
