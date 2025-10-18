#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan

from matplotlib import pyplot as plt

class FeatureExtracter(Node):
    def __init__(self):
        super().__init__('feature_extracter')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1) 
        self.scan_idx = 0
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_policy)
        self.publisher_ = self.create_publisher(LaserScan, '/feature_scan', qos_profile=qos_policy)

        # create a new publisher for /front_scan
        self.front_publisher_ = self.create_publisher(LaserScan, '/front_scan', qos_profile=qos_policy)


    def scan_callback(self,msg):

        ranges = []
        for r in msg.ranges:
            if r > 2.5 or r < 1:   # Feature extraction code here
                ranges.append(0.0)
            else:
                ranges.append(r)
        ################################## 

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = msg.header.frame_id
        scan.angle_min = msg.angle_min
        scan.angle_max = msg.angle_max
        scan.angle_increment = msg.angle_increment
        scan.time_increment = msg.time_increment
        scan.range_min = msg.range_min
        scan.range_max = msg.range_max 
        scan.ranges = ranges
        self.publisher_.publish(scan)
        ################################## 

        # New front publisher

        front_ranges = []
        angle_min_rad = -np.deg2rad(60.0) # -60 degrees in radians
        angle_max_rad = np.deg2rad(60.0)  # +60 degrees in radians
        for i, r in enumerate(msg.ranges):
            current_angle = msg.angle_min + i * msg.angle_increment
            
            # Check if the angle is within the (-60, +60) degree range
            if current_angle >= angle_min_rad and current_angle <= angle_max_rad:
                front_ranges.append(r)
            else:
                front_ranges.append(0.0)
        
        front_scan = LaserScan()
        front_scan.header.stamp = msg.header.stamp
        front_scan.header.frame_id = msg.header.frame_id
        front_scan.angle_min = msg.angle_min
        front_scan.angle_max = msg.angle_max
        front_scan.angle_increment = msg.angle_increment
        front_scan.time_increment = msg.time_increment
        front_scan.range_min = msg.range_min
        front_scan.range_max = msg.range_max
        front_scan.ranges = front_ranges
        self.front_publisher_.publish(front_scan)


        self.scan_idx += 1
        print("Publish feature scan and front scan message idx", self.scan_idx)





def main(args=None):
    rclpy.init(args=args)
    feature_extracter = FeatureExtracter()
    rclpy.spin(feature_extracter) 
    feature_extracter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
