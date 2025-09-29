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
import time

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
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, qos)
        self.subscription
        self.get_logger().info("Image Bag Processor initialized")

    # -----------------------------
    # Callbacks
    # -----------------------------
    def image_callback(self, msg):
        try:
            # TODO: Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

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
        new_width = 500
        height, width, _ = image.shape
        aspect_ratio = height / width
        new_height = int(new_width * aspect_ratio)
        frame = cv2.resize(image, (new_width, new_height))

        # TODO: Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # TODO: Apply cv2.Sobel in X and Y directions
        sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)

        # TODO: Compute Sobel magnitude
        sobel_mag = cv2.magnitude(sobel_x, sobel_y)
        
        sobel_x_disp = cv2.convertScaleAbs(sobel_x)
        sobel_y_disp = cv2.convertScaleAbs(sobel_y)
        sobel_mag_disp = cv2.normalize(sobel_mag, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        # TODO: Convert grayscale images to BGR for stacking
        sobel_x_bgr = cv2.cvtColor(sobel_x_disp, cv2.COLOR_GRAY2BGR)
        sobel_y_bgr = cv2.cvtColor(sobel_y_disp, cv2.COLOR_GRAY2BGR)
        sobel_mag_bgr = cv2.cvtColor(sobel_mag_disp, cv2.COLOR_GRAY2BGR)

        # TODO: Annotate each image with cv2.putText
        # frame -> "Original", sobel_mag_bgr -> "Sobel Mag", sobel_x_bgr -> "Sobel X", sobel_y_bgr -> "Sobel Y"
        cv2.putText(frame, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(sobel_mag_bgr, "Sobel Mag", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(sobel_x_bgr, "Sobel X", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(sobel_y_bgr, "Sobel Y", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # TODO: Stack in 2x2 grid
        top    = np.hstack((frame, sobel_x_bgr))
        bottom = np.hstack((sobel_y_bgr, sobel_mag_bgr))
        grid   = np.vstack((top, bottom))

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
