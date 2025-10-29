#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import os
import glob
import time
import numpy as np
import matplotlib.pyplot as plt

RESOLUTION = 0.10
WIDTH = 160
HEIGHT = 160
ORIGIN_X = -8.0
ORIGIN_Y = -8.0
OUTDIR = os.path.expanduser('/home/ws/maps')

WAYPOINTS = [
    (1.3, -0.6),
    (2.7, -0.6),
    (4.4, -0.6),
    (4.4, -2.4),
    (3.1, -2.4),
    (1.6, -2.4),
    (1.6, -5.4),
    (3.5, -5.4),
    (5.0, -5.4),
]
POS_TOL   = 0.20      # position tolerance (m)
YAW_TOL   = 0.15      # yaw tolerance (rad)
LIN_SPEED = 0.20      # linear speed (m/s)
ANG_SPEED = 0.80      # angular speed (rad/s)
CTRL_DT   = 0.05      # control period 20Hz



class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(String, '/grid_map_path', self.map_cb, 1) 
        self.timer = self.create_timer(CTRL_DT, self.loop)

        self.has_odom = False
        self.x = self.y = self.yaw = 0.0
        self.i = 0
        self.done = False
        
        self.t0 = self.get_clock().now()
        self.traj = []            # [(t,x,y,yaw), ...]
        self.cmd_log = []         # [(t,v,w), ...]
        self.map_csv_path = None
        os.makedirs(OUTDIR, exist_ok=True)

    def map_cb(self, msg: String):
        self.map_csv_path = msg.data

    def odom_cb(self, msg: Odometry):
        self.has_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        
        t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        self.traj.append((t, self.x, self.y, self.yaw))

    def loop(self):
        if not self.has_odom or self.done:
            return

        gx, gy = WAYPOINTS[self.i]
        dx, dy = gx - self.x, gy - self.y
        dist = math.hypot(dx, dy)
        target = math.atan2(dy, dx)
        yaw_err = ((target - self.yaw) + math.pi) % (2.0 * math.pi) - math.pi

        cmd = Twist()
        if abs(yaw_err) > YAW_TOL:
            # rotate
            cmd.angular.z = ANG_SPEED if yaw_err > 0 else -ANG_SPEED
        else:
            # move forward
            cmd.linear.x = LIN_SPEED

        self.pub.publish(cmd)
        
        t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        self.cmd_log.append((t, cmd.linear.x, cmd.angular.z))

        if dist < POS_TOL:
            self.i += 1
            if self.i >= len(WAYPOINTS):
                self.pub.publish(Twist())  # stop
                self.done = True
                self.plot_figures() 

    def plot_figures(self):
        path = self.map_csv_path
        if path is None:
            candidates = sorted(glob.glob(os.path.join(OUTDIR, 'gridmap_*.csv')))
            if candidates:
                path = candidates[-1]  # newest file
                self.get_logger().warn(f'No topic path; use latest file: {path}')
            else:
                self.get_logger().warn('No map CSV found; will plot trajectory only.')

        grid = None
        if path is not None:
            try:
                grid = np.loadtxt(path, delimiter=',')
            except Exception as e:
                self.get_logger().warn(f'Load map failed: {e}')

        xs = np.array([p[1] for p in self.traj])
        ys = np.array([p[2] for p in self.traj])

        plt.figure()
        if grid is not None:
            gx = ((xs - ORIGIN_X) / RESOLUTION).astype(int)
            gy = ((ys - ORIGIN_Y) / RESOLUTION).astype(int)
            plt.imshow(grid, origin='lower')
            plt.plot(gx, gy)
        else:
            plt.plot(xs, ys)
            plt.xlabel('x [m]'); plt.ylabel('y [m]')
        plt.title('Trajectory over grid map')
        p1 = os.path.join(OUTDIR, 'traj_on_map.png')
        plt.savefig(p1, dpi=200, bbox_inches='tight'); plt.close()
        self.get_logger().info(f'Saved {p1}')


        t = [c[0] for c in self.cmd_log]
        v = [c[1] for c in self.cmd_log]
        w = [c[2] for c in self.cmd_log]
        plt.figure()
        plt.plot(t, v, label='linear x')
        plt.plot(t, w, label='angular z')
        plt.xlabel('time [s]'); plt.ylabel('command'); plt.legend()
        plt.title('Twist commands vs time')
        p2 = os.path.join(OUTDIR, 'twist_vs_time.png')
        plt.savefig(p2, dpi=200, bbox_inches='tight'); plt.close()
        self.get_logger().info(f'Saved {p2}')

   



def main(args=None):
    rclpy.init(args=args)
    n = WaypointFollower()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
