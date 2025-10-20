#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np
import matplotlib.pyplot as plt
import time
import os

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

        # Map parameters, map is 8m x 8m with 10cm resolution
        self.resolution = 0.10
        self.width = 80
        self.height = 80
        # Lower-left origin in world coords
        self.origin_x = -4.0
        self.origin_y = -4.0
        
        self.snapshot_period = 15.0  # Save a snapshot every N seconds
        self.output_dir = '.'

        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)  # -1 unknown

        # Latest odom
        self.latest_odom = None

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        
        self.create_timer(self.snapshot_period, self.save_snapshot)


    # Helper
    def world_to_grid(self, x_w, y_w):
        gx = int(math.floor((x_w - self.origin_x) / self.resolution))
        gy = int(math.floor((y_w - self.origin_y) / self.resolution))
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

    def save_snapshot(self):
        try:
            viz = np.where(self.grid == -1, 127,
                           np.where(self.grid == 0, 255, 0)).astype(np.uint8)
            plt.figure(figsize=(5, 5))
            plt.imshow(viz, cmap='gray', origin='lower',
                       extent=[self.origin_x,
                               self.origin_x + self.width * self.resolution,
                               self.origin_y,
                               self.origin_y + self.height * self.resolution])
            plt.xlabel('x (m)')
            plt.ylabel('y (m)')
            plt.title('Occupancy Grid Map')
            ts = int(time.time())
            fname = f'{self.output_dir}/grid_snapshot_{ts}.png'
            plt.savefig(fname, dpi=150, bbox_inches='tight')
            plt.close()
        except Exception as e:
            pass
    
    def save_grid_csv(self):
        try:
            path = os.path.join(self.output_dir, 'final_map.csv')
            np.savetxt(path, self.grid, fmt='%d', delimiter=',')
        except Exception:
            pass



def main(args=None):
    rclpy.init(args=args)
    node = GridMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save_grid_csv()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
