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
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, qos)
        self.subscription
        self.get_logger().info("Image Bag Processor initialized")

    # -----------------------------
    # Callbacks
    # -----------------------------
    def image_callback(self, msg):
        try:
            # TODO: Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

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
        new_width = 400
        height, width, _ = image.shape
        aspect_ratio = height / width
        new_height = int(new_width * aspect_ratio)
        frame = cv2.resize(image, (new_width, new_height))

        # TODO: Convert to grayscale
        # Hint: cv.cvtColor(src, code[, dst[, dstCn]])
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray32 = np.float32(gray)

        # TODO: Harris corner detection
        # Hint: cv2.cornerHarris(src, blockSize, ksize, k[, dst[, borderType]])
        dst = cv2.cornerHarris(gray32, 2, 3, 0.04)   # blockSize=2, ksize=3, k=0.04
        dst_vis = cv2.dilate(dst, None)  # Dilate to mark the corners

        # TODO: Create a heatmap visualization of the Harris response
        # Hint 1. cv2.normalize(src, dst, alpha, beta, norm_type[, dtype])
        #      2. cv2.applyColorMap(src, colormap[, dst])
        heatmap = cv2.normalize(dst_vis, None, 0, 255, cv2.NORM_MINMAX)
        heatmap = cv2.applyColorMap(heatmap.astype(np.uint8), cv2.COLORMAP_JET)
        
        # TODO: Mark corners on original image
        corners_img = frame.copy()
        corners_img[dst_vis > 0.02 * dst_vis.max()] = (0, 255, 0)
        
        gray_bgr = cv2.cvtColor(cv2.convertScaleAbs(gray), cv2.COLOR_GRAY2BGR)

        # Different parameters
        dst1 = cv2.cornerHarris(gray32, 2, 3, 0.06) # blockSize=2, ksize=3, k=0.06
        dst2 = cv2.cornerHarris(gray32, 5, 3, 0.04) # blockSize=5, ksize=3, k=0.04
        dst_vis1 = cv2.dilate(dst1, None)
        dst_vis2 = cv2.dilate(dst2, None)

        corners_img1 = frame.copy()
        corners_img1[dst_vis1 > 0.02 * dst_vis1.max()] = (0, 255, 0)

        corners_img2 = frame.copy()
        corners_img2[dst_vis2 > 0.02 * dst_vis2.max()] = (0, 255, 0)



        # TODO: Add labels for each image (Original, Grayscale, Heatmap, Corners Overlay)
        # e.g., cv2.putText(...)
        cv2.putText(frame, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(gray_bgr, "Grayscale", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cv2.putText(heatmap, "Harris Heatmap", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cv2.putText(corners_img, "Corners Overlay, b=2 ksize=3 k=0.04", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(corners_img1, "Corners Overlay, b=2 ksize=3 k=0.06", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(corners_img2, "Corners Overlay, b=5 ksize=3 k=0.04", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        
        # TODO: Stack images
        top    = np.hstack((frame, gray_bgr))        
        middle = np.hstack((heatmap, corners_img)) 
        bottom   = np.hstack((corners_img1, corners_img2)) 

        grid = np.vstack((top, middle, bottom))

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
