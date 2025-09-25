#!/usr/bin/env python3
"""
Task: ROS2 Image Processing with Canny Edge Detection - Student Template
- Subscribe to raw or compressed image topics
- Apply Gaussian blur, Sobel gradients, and Canny edge detection
- Display results in a 2x3 grid with colored labels
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CannyProcessor(Node):
    def __init__(self):
        super().__init__('canny_processor')
        self.bridge = CvBridge()
        self.processed_count = 0
        self.compressed_count = 0

        # Define the QoS profile for best effort
        qos = QoSProfile(
            depth=1,  # Keep a shallow history
            reliability=QoSReliabilityPolicy.BEST_EFFORT  # Set to best effort
        )

        # TODO: Create subscriptions for raw and/or compressed images
        # self.create_subscription(...)

    # -----------------------------
    # Callbacks
    # -----------------------------
    def image_callback(self, msg):
        try:
            # TODO: Convert ROS Image message to OpenCV format
            cv_image = None

            self.processed_count += 1
            processed = self.process_cv_image(cv_image)

            # Display the grid
            cv2.imshow("Canny Steps Grid", processed)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing raw image: {e}")

    def compressed_image_callback(self, msg):
        try:
            # TODO: Convert ROS CompressedImage message to OpenCV format
            cv_image = None

            self.compressed_count += 1
            processed = self.process_cv_image(cv_image)

            # Display the grid
            cv2.imshow("Canny Steps Grid", processed)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing compressed image: {e}")

    # -----------------------------
    # Image processing
    # -----------------------------
    def process_cv_image(self, image):
        """Resize, blur, apply Canny edge detection, stack grid with colored labels"""
        # TODO: Resize if necessary
        frame = image

        # TODO: Convert to grayscale
        gray = None

        # TODO: Apply Gaussian blur
        blurred = None

        # TODO: Compute Sobel gradients (X and Y)
        grad_x = None
        grad_y = None
        magnitude = None
        magnitude_bgr = None

        # TODO: Apply Canny edge detection
        edges = None
        edges_bgr = None

        # TODO: Convert gray and blurred to BGR for stacking
        gray_bgr = None
        blurred_bgr = None

        # -----------------------------
        # TODO: Add colored labels on each image
        # -----------------------------

        # -----------------------------
        # TODO: Stack images into 2x3 grid
        # Top row: Original | Grayscale | Blurred
        # Bottom row: Grad Mag | Canny | Canny (or placeholder)
        # -----------------------------
        grid = frame

        return grid


def main(args=None):
    rclpy.init(args=args)
    processor = CannyProcessor()
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
