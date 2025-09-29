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

WIN = "Canny Grid"

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
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, qos)
        self.subscription

        # Canny edge detection adjustable parameters
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.createTrackbar('Low', WIN, 100, 500, lambda x: None)
        cv2.createTrackbar('High',  WIN, 200, 500, lambda x: None)        
        
        self.get_logger().info("Image Bag Processor initialized")

    # -----------------------------
    # Callbacks
    # -----------------------------
    def image_callback(self, msg):
        try:
            # TODO: Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            self.processed_count += 1
            processed = self.process_cv_image(cv_image)

            # Display the grid
            cv2.imshow(WIN, processed)
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
        new_width = 500
        height, width, _ = image.shape
        aspect_ratio = height / width
        new_height = int(new_width * aspect_ratio)
        frame = cv2.resize(image, (new_width, new_height))

        # TODO: Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # TODO: Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # TODO: Compute Sobel gradients (X and Y)
        sobel_x = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(blurred, cv2.CV_64F, 0, 1, ksize=3)
        sobel_mag = cv2.magnitude(sobel_x, sobel_y)

        # TODO: Apply Canny edge detection
        low  = cv2.getTrackbarPos('Low', WIN)
        high = cv2.getTrackbarPos('High', WIN)
        if high <= low:
            high = low + 1
        canny_adj = cv2.Canny(blurred, low, high)
        canny_fixed = cv2.Canny(blurred, 100, 200)

        sobel_mag_disp = cv2.normalize(sobel_mag, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        # TODO: Convert gray and blurred to BGR for stacking
        gray_bgr = cv2.cvtColor(cv2.convertScaleAbs(gray), cv2.COLOR_GRAY2BGR)
        blurred_bgr = cv2.cvtColor(cv2.convertScaleAbs(blurred), cv2.COLOR_GRAY2BGR)
        
        sobel_mag_bgr = cv2.cvtColor(sobel_mag_disp, cv2.COLOR_GRAY2BGR)
        canny_adj_bgr = cv2.cvtColor(cv2.convertScaleAbs(canny_adj), cv2.COLOR_GRAY2BGR)
        canny_fixed_bgr = cv2.cvtColor(cv2.convertScaleAbs(canny_fixed), cv2.COLOR_GRAY2BGR)

        # -----------------------------
        # TODO: Add colored labels on each image
        # -----------------------------
        cv2.putText(frame, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(gray_bgr, "Grayscale", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(blurred_bgr, "Blurred", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
        cv2.putText(sobel_mag_bgr, "Grad Mag", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.putText(canny_adj_bgr, "Canny Adj", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(canny_fixed_bgr, "Canny Fixed", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)

        # -----------------------------
        # TODO: Stack images into 2x3 grid
        # Top row: Original | Grayscale | Blurred
        # Bottom row: Grad Mag | Canny | Canny (or placeholder)
        # -----------------------------
        top    = np.hstack((frame, gray_bgr, blurred_bgr))
        bottom = np.hstack((sobel_mag_bgr, canny_adj_bgr, canny_fixed_bgr))
        grid = np.vstack((top, bottom))

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
