#!/usr/bin/env python3
"""
Task: ROS2 Harris Corner Detection - Student Template
- Students should implement:
    - Grayscale conversion
    - Harris corner detection
    - Heatmap visualization
    - Mark corners on original image
    - Stack images in a grid with labels
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class HarrisCornerStudent(Node):
    def __init__(self):
        super().__init__('harris_corner_student')
        self.bridge = CvBridge()

        # Define the QoS profile for best effort
        qos = QoSProfile(
            depth=1,  # Keep a shallow history
            reliability=QoSReliabilityPolicy.BEST_EFFORT  # Set to best effort
        )

        # TODO: Subscribe to raw and/or compressed image topics
        # self.image_sub = ...
        # self.compressed_sub = ...

    # -----------------------------
    # Callbacks
    # -----------------------------
    def image_callback(self, msg):
        try:
            # TODO: Convert ROS Image to OpenCV format
            cv_image = None

            processed = self.process_cv_image(cv_image)
            cv2.imshow("Harris Corners - Student", processed)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing raw image: {e}")

    def compressed_image_callback(self, msg):
        try:
            # TODO: Convert ROS CompressedImage to OpenCV format
            cv_image = None

            processed = self.process_cv_image(cv_image)
            cv2.imshow("Harris Corners - Student", processed)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing compressed image: {e}")

    # -----------------------------
    # Image Processing
    # -----------------------------
    def process_cv_image(self, image):
        # TODO: Resize image if needed
        frame = image

        # TODO: Convert to grayscale
        # Hint: cv.cvtColor(src, code[, dst[, dstCn]])
        gray = None

        # TODO: Harris corner detection
        # Hint: cv2.cornerHarris(src, blockSize, ksize, k[, dst[, borderType]])
        dst = None

        # TODO: Create a heatmap visualization of the Harris response
        # Hint 1. cv2.normalize(src, dst, alpha, beta, norm_type[, dtype])
        #      2. cv2.applyColorMap(src, colormap[, dst])
        heatmap = None

        # TODO: Mark corners on original image
        corners_img = frame

        # TODO: Add labels for each image (Original, Grayscale, Heatmap, Corners Overlay)
        # e.g., cv2.putText(...)

        # TODO: Stack images in a 2x2 grid for step-by-step visualization
        top = np.hstack((frame, gray)) if gray is not None else frame
        bottom = np.hstack((heatmap, corners_img)) if heatmap is not None else corners_img
        grid = np.vstack((top, bottom)) if gray is not None and heatmap is not None else frame

        return grid


def main(args=None):
    rclpy.init(args=args)
    node = HarrisCornerStudent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
