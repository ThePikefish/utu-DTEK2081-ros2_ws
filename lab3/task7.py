#!/usr/bin/env python3
"""
Task 5: ROS2 Image Processing with Sobel Filters - Student Template
- Create 2x2 grid: Original, Sobel magnitude, Sobel X, Sobel Y
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor_sobel')
        self.bridge = CvBridge()

        # Counters
        self.processed_count = 0
        self.compressed_count = 0

        # Define the QoS profile for best effort
        qos = QoSProfile(
            depth=1,  # Keep a shallow history
            reliability=QoSReliabilityPolicy.BEST_EFFORT  # Set to best effort
        )

        # TODO: Create subscribers for raw and compressed images
        # self.create_subscription(...)

    # -----------------------------
    # Callbacks
    # -----------------------------
    def image_callback(self, msg):
        try:
            # TODO: Convert ROS image to OpenCV
            cv_image = None

            self.processed_count += 1
            processed = self.process_cv_image(cv_image)

            # TODO: Display 2x2 grid
            cv2.imshow("Sobel Grid", processed)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing raw image: {e}")

    def compressed_image_callback(self, msg):
        try:
            # TODO: Convert ROS compressed image to OpenCV
            cv_image = None

            self.compressed_count += 1
            processed = self.process_cv_image(cv_image)

            # TODO: Display 2x2 grid
            cv2.imshow("Sobel Grid", processed)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing compressed image: {e}")

    # -----------------------------
    # Image processing
    # -----------------------------
    def process_cv_image(self, image):
        """Apply Sobel filters and stack 2x2 grid"""
        # TODO: Resize image
        frame = image

        # TODO: Convert to grayscale
        gray = None

        # TODO: Apply cv2.Sobel in X and Y directions
        sobel_x = None
        sobel_y = None

        # TODO: Compute Sobel magnitude
        sobel_mag = None

        # TODO: Convert grayscale images to BGR for stacking
        sobel_x_bgr = None
        sobel_y_bgr = None
        sobel_mag_bgr = None

        # TODO: Annotate each image with cv2.putText
        # frame -> "Original", sobel_mag_bgr -> "Sobel Mag", sobel_x_bgr -> "Sobel X", sobel_y_bgr -> "Sobel Y"

        # TODO: Stack in 2x2 grid
        grid = frame

        return grid

def main(args=None):
    rclpy.init(args=args)
    processor = ImageProcessor()
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
