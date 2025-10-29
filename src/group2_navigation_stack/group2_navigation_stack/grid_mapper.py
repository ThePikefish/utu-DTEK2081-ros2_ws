#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import numpy as np
import matplotlib.pyplot as plt
import time
import os


RESOLUTION = 0.10
WIDTH = 160
HEIGHT = 160
ORIGIN_X = -8.0
ORIGIN_Y = -8.0
SAVE_HZ = 0.5
OUTPUT_DIR = os.path.expanduser('/home/ws/maps')

# Example code 
def bresenham_points(p0, p1):
    point_list = []  # We will fill this list with all points in between p0 and p1

    x0, y0 = p0[0], p0[1]
    x1, y1 = p1[0], p1[1]

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    if x0 < x1:
        sx = 1
    else:
        sx = -1

    if y0 < y1:
        sy = 1
    else:
        sy = -1

    err = dx - dy

    while True:
        if [x0, y0] != p0 and [x0, y0] != p1:  # exclude first and last
            point_list.append([x0, y0])
        if x0 == x1 and y0 == y1:
            break  # This means we have finished, so we break the loop

        e2 = 2 * err
        if e2 > -dy:
            # overshot in the y direction
            err = err - dy
            x0 = x0 + sx
        if e2 < dx:
            # overshot in the x direction
            err = err + dx
            y0 = y0 + sy

    return point_list


class GridMapper(Node):
    def __init__(self):
        super().__init__('grid_mapper')
        os.makedirs(OUTPUT_DIR, exist_ok=True)

        self.grid = np.full((HEIGHT, WIDTH), -1, dtype=np.int8)  # -1 unknown

        # Latest odom
        self.latest_odom = None

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        
        self.path_pub = self.create_publisher(String, '/grid_map_path', 1)
        self.create_timer(1.0 / SAVE_HZ, self.save_csv_timer)

        # self.create_timer(15.0, self.save_snapshot)


    # Helper
    def world_to_grid(self, x_w, y_w):
        gx = int(math.floor((x_w - ORIGIN_X) / RESOLUTION))
        gy = int(math.floor((y_w - ORIGIN_Y) / RESOLUTION))
        return gx, gy


    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def scan_callback(self, msg: LaserScan):
        if self.latest_odom is None:
            return

        # Robot pose in world
        px = self.latest_odom.pose.pose.position.x
        py = self.latest_odom.pose.pose.position.y
        q = self.latest_odom.pose.pose.orientation
        
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Robot grid coords
        grx, gry = self.world_to_grid(px, py)


        # Iterate rays
        angle = msg.angle_min
        for r in msg.ranges:
            angle_i = angle
            angle += msg.angle_increment

            if r is None or (not math.isfinite(r)) or r <= max(1e-3, msg.range_min):
                continue

            r_clamped = min(r, msg.range_max)

            # World endpoint
            theta_world = yaw + angle_i
            hit_x = px + r_clamped * math.cos(theta_world)
            hit_y = py + r_clamped * math.sin(theta_world)

            gx_hit, gy_hit = self.world_to_grid(hit_x, hit_y)

            path = bresenham_points([grx, gry], [gx_hit, gy_hit])

            if len(path) > 0:
                for (gx, gy) in path[:-1]:
                    try:
                        if self.grid[gy, gx] == 1:
                            continue
                        self.grid[gy, gx] = 0
                    except IndexError:
                        pass

            try:
                self.grid[gy_hit, gx_hit] = 1
            except IndexError:
                pass

    def save_csv_timer(self):
        try:
            ts = int(time.time())
            path = os.path.join(OUTPUT_DIR, f'gridmap_{ts}.csv')
            np.savetxt(path, self.grid, fmt='%d', delimiter=',')
            self.path_pub.publish(String(data=path))
            self.get_logger().info(f"CSV saved: {path}")

        except Exception as e:
            self.get_logger().warn(f"CSV save failed: {e}")
    



def main(args=None):
    rclpy.init(args=args)
    node = GridMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
