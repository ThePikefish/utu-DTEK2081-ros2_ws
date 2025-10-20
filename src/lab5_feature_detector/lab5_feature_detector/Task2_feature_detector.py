#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, PointCloud
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Point32


class CornerLineDetector(Node):

    # Three point to use in the window for corner detection
    CORNER_INDICES = [0, 4, 8]
    # Minimum distance between detected corners (prevents detecting the same corner multiple times)
    CORNER_AREA_WINDOW = 20

    def __init__(self):
        super().__init__('corner_line_detector')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1) 
        self.scan_idx = 0

        # Subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_policy)
        
        # Publishers
        self.corner_publisher = self.create_publisher(
            PointCloud2, '/corner_points', qos_profile=qos_policy
        )
        self.line_publisher = self.create_publisher(
            PointCloud2, '/line_points', qos_profile=qos_policy
        )

        self.scan_data = None


    def scan_callback(self, msg: LaserScan):
        self.scan_data = msg

        # Convert to cartesian
        angles, ranges, x, y = self.polar_to_cartesian_coordinate(
            self.scan_data.ranges, self.scan_data.angle_min, self.scan_data.angle_max, self.scan_data.angle_increment
        )
        points_xy = np.stack((x, y), axis=1)

        # Detect corners
        corner_points = self.detect_corners(points_xy)

        # Detect lines TODO

        # Publish results
        self.publish_cloud2(self.corner_publisher, corner_points, msg.header)
        #self.publish_cloud(self.corner_publisher, corner_points, msg.header)
        # self.publish_cloud() TODO

        self.scan_idx += 1
        print("Publish corner PointCloud2 message idx", self.scan_idx)


    def polar_to_cartesian_coordinate(self, ranges, angle_min, angle_max, angle_increment):
        """Convert LaserScan polar data to Cartesian coordinates."""
        
        # Convert to numpy array
        ranges = np.array(ranges)
        
        # remember to clip invalid values (inf, nan), if needed
        # Replace inf and 0.0 with nan
        ranges[np.isinf(ranges)] = np.nan
        ranges[np.isnan(ranges)] = np.nan
        ranges[ranges == 0.0] = np.nan
        
        # Create an array of angles
        angles = np.linspace(angle_min, angle_max, len(ranges))
        
        # Convert to cartesian
        x_cart = ranges * np.cos(angles)
        y_cart = ranges * np.sin(angles)

        return angles, ranges, x_cart, y_cart
    

    def detect_corners(self, points_xy: np.ndarray):
        """"Detects 90 degree corners using three points"""
        possible_corners = []

        idx1, idx2, idx3 = self.CORNER_INDICES
        window_size = idx3 + 1

        # Slide window in the scann
        for i in range(len(points_xy) - window_size):
            p1 = points_xy[i + idx1]
            p2 = points_xy[i + idx2]
            p3 = points_xy[i + idx3]

            # Check for nan points
            if np.isnan(p1).any() or np.isnan(p2).any() or np.isnan(p3).any():
                continue

            # Calculate the angle
            angle_rad = self.calculate_angle(p1, p2, p3)
            if np.isnan(angle_rad):
                continue

            angle_deg = np.degrees(angle_rad)
            # How far from perfect 90 degree angle
            diff_from_90 = abs(angle_deg - 90.0)

            # Store possible corners (there is more than 4)
            possible_corners.append((diff_from_90, p2, i + idx2))
        
        # Sort the corners (smaller diff is better)
        possible_corners.sort(key=lambda x: x[0])

        final_corners = []
        final_corner_indices = []

        for diff, point, index in possible_corners:
            # Check if this corner is too close to one already added
            is_duplicate = False
            for final_idx in final_corner_indices:
                if abs(index - final_idx) < self.CORNER_AREA_WINDOW:
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                # This is a new, distinct corner
                final_corners.append(point)
                final_corner_indices.append(index)
                
            # Stop when 4 corners have been found
            if len(final_corners) == 4:
                break

        return final_corners

    
    def calculate_angle(self, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray):
        """Calculates the angle (in radians) at vertex p2"""
        v1 = p1 - p2 # Vector from p2 to p1
        v2 = p3 - p2 # Vector from p2 to p3

        # Used AI here for help with understanding, how to calculate the angle in practice
        dot_product = np.dot(v1, v2)
        mag_v1 = np.linalg.norm(v1)
        mag_v2 = np.linalg.norm(v2)

        # Avoid dividing by zero
        if mag_v1 == 0 or mag_v2 == 0:
            return np.nan
        
        cos_theta = dot_product / (mag_v1 * mag_v2)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)

        return np.arccos(cos_theta)
    

    def publish_cloud2(self, publisher: rclpy.publisher.Publisher, points_list: list, header):
        """Publish (x,y) points as PointCloud2 message."""

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Convert np.arrays to list of tuples
        points_xyz_list = []
        for point in points_list:
            points_xyz_list.append( (float(point[0]), float(point[1]), 0.0) )

        pc2_msg = point_cloud2.create_cloud(header, fields, points_xyz_list)

        publisher.publish(pc2_msg)

    def publish_cloud(self, publisher: rclpy.publisher.Publisher, 
                      points_list: list, header):
        """Converts a list of (x,y) points to a PointCloud message and publishes."""
        pc_msg = PointCloud()
        pc_msg.header = header # Use stamp and frame_id from original scan
        
        ros_points = []
        for point in points_list:
            p = Point32()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = 0.0 # Lidar is 2D
            ros_points.append(p)
            
        pc_msg.points = ros_points
        publisher.publish(pc_msg)



def main(args=None):
    rclpy.init(args=args)
    corner_line_detector = CornerLineDetector()
    rclpy.spin(corner_line_detector) 
    corner_line_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
