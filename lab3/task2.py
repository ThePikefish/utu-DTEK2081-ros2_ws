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
        self.bridge = CvBridge()

        # Define the QoS profile for best effort
        qos = QoSProfile(
            depth=1,  # Keep a shallow history
            reliability=QoSReliabilityPolicy.BEST_EFFORT  # Set to best effort
        )
        
        # TODO: Create subscribers for EITHER compressed and uncompressed images
        # Hint: self.create_subscription()
        
        self.processed_count = 0

    # USE ONLY ONE OF THE CALL BACKS DEPENDING ON THE TOPIC TYPE FROM THE ROSBAG
  
    def compressed_image_callback(self, msg):
        """Process compressed image messages"""
        try:
            # TODO: Convert ROS compressed image to OpenCV format
            # Hint: Use self.bridge.compressed_imgmsg_to_cv2()
            cv_image = None  # Replace with conversion
            
            # TODO: Process the image (add shapes and text)
            processed_image = self.process_cv_image(cv_image)

            # Save periodically OPTIONAL
            # if self.processed_count % 30 == 0:
            #     filename = ''
            #     cv2.imwrite(filename, processed_image)
            #     self.get_logger().info(f"Saved {filename}")
            
        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {e}')
    
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
        # TODO: Implement image processing
        # Add timestamp, frame counter, shapes, etc.
        
        # Get image dimensions
        height, width = cv_image.shape[:2]
        
        # TODO: Add a rectangle border
        
        # TODO: Add frame counter text
        
        # TODO: Add timestamp (you can use self.get_clock().now())
        
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