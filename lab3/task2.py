#!/usr/bin/env python3
"""
Task 2: ROS2 Image Processing from Bag - Student Template
Read images from a ROS bag and process them
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageBagProcessor(Node):
    def __init__(self):
        super().__init__('image_bag_processor')
        topic_name = 'camera/image_raw/compressed'
        self.bridge = CvBridge()

        # Define the QoS profile for best effort
        qos = QoSProfile(
            depth=1,  # Keep a shallow history
            reliability=QoSReliabilityPolicy.BEST_EFFORT  # Set to best effort
        )
        
        # TODO: (DONE) Create subscribers for EITHER compressed and uncompressed images
        # Hint: self.create_subscription()
        self.subscription = self.create_subscription(
            CompressedImage,
            topic_name,
            self.compressed_image_callback,
            qos)
        
        self.subscription
        
        self.processed_count = 0


    # USE ONLY ONE OF THE CALL BACKS DEPENDING ON THE TOPIC TYPE FROM THE ROSBAG
    # Use compressed
    def compressed_image_callback(self, msg):
        """Process compressed image messages"""
        try:
            # Get time
            self.time = msg.header.stamp
            self.time = float(str(self.time.sec) + '.' + str(self.time.nanosec)[:3])
            # Get start time
            if self.processed_count == 0:
                self.start_time = self.time
            
            self.processed_count += 1

            # TODO: (DONE) Convert ROS compressed image to OpenCV format
            # Hint: Use self.bridge.compressed_imgmsg_to_cv2()
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)  # Replace with conversion
            
            # TODO: Process the image (add shapes and text)
            processed_image = self.process_cv_image(cv_image)
            cv2.imshow("Image", processed_image)
            cv2.waitKey(1)

            # Save periodically OPTIONAL
            # if self.processed_count % 30 == 0:
            #     filename = ''
            #     cv2.imwrite(filename, processed_image)
            #     self.get_logger().info(f"Saved {filename}")

            
        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {e}')
    
    # (Not in use, because using compressed version)
    def image_callback(self, msg):
        """Process uncompressed image messages"""
        try:
            # TODO: Convert ROS image to OpenCV format
            # Hint: Use self.bridge.imgmsg_to_cv2()
            cv_image = None  # Replace with conversion
            
            # TODO: Process the image
            processed_image = self.process_cv_image(cv_image)

            # Save periodically OPTIONAL
            # if self.processed_count % 30 == 0:
            #     filename = ''
            #     cv2.imwrite(filename, processed_image)
            #     self.get_logger().info(f"Saved {filename}")
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def process_cv_image(self, cv_image):
        """Add annotations to the image"""
        # TODO: (DONE) Implement image processing
        # Add timestamp, frame counter, shapes, etc.
        
        # Get image dimensions
        height, width = cv_image.shape[:2]

        #time = round(float(str(time.sec) + '.' + str(time.nanosec)), 3)
        
        # TODO: (DONE) Add a rectangle border
        cv2.rectangle(cv_image, pt1=(0, 0), pt2=(width, height), color=(255, 0, 0), thickness=5)

        # Info rectangle
        cv_image_copy = cv_image.copy() # Copy for the semi transparent rectangle
        cv2.rectangle(cv_image, pt1=(0, 0), pt2=(width, 100), color=(20, 20, 20), thickness=-1)
        cv_image = cv2.addWeighted(cv_image, 0.4, cv_image_copy, 1 - 0.5, 0) # Copied image to rectangle with alpha
        
        # TODO: (DONE) Add frame counter text
        cv2.putText(cv_image, text=f'Frame: {self.processed_count} (Compressed)', org=(10, 20), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=(255, 255, 255), thickness=1)

        # Add size text
        cv2.putText(cv_image, text=f'Size: {width}x{height}', org=(10, 40), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=(255, 255, 255), thickness=1)

        # TODO: (DONE) Add timestamp (you can use self.get_clock().now())
        cv2.putText(cv_image, text=f'ROS time: {self.time}', org=(10, 60), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=(255, 255, 255), thickness=1)

        # Elapsed time
        elapsed_time = round(self.time - self.start_time, 1)
        cv2.putText(cv_image, text=f'Elapsed time: {elapsed_time}s', org=(10, 80), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1, color=(255, 255, 255), thickness=1)

        return cv_image

def main(args=None):
    rclpy.init(args=args)
    
    processor = ImageBagProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()