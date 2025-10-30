#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import math
from queue import PriorityQueue
import matplotlib.pyplot as plt
import os

RESOLUTION = 0.10
WIDTH = 160
HEIGHT = 160
ORIGIN_X = -8.0
ORIGIN_Y = -8.0
OUTDIR = os.path.expanduser('/home/ws/maps')

NODES = np.array([
    [0.0,  0.0],   # 0
    [1.3, -0.6],   # 1
    [2.7, -0.6],   # 2
    [4.4, -0.6],   # 3
    [4.4, -2.4],   # 4
    [3.1, -2.4],   # 5
    [1.6, -2.4],   # 6
    [1.6, -5.4],   # 7
    [3.5, -5.4],   # 8
    [5.0, -5.4],   # 9
], dtype=float)

EDGES = [
    (0,1), (1,2), (2,3),
    (3,4), (4,5), (5,6),
    (6,7), (7,8), (8,9),
    (1,6), (3,9), (4,9),
]


POS_TOL   = 0.20 
YAW_TOL   = 0.15
LIN_SPEED = 0.20
ANG_SPEED = 0.80
CTRL_DT   = 0.05



def build_adj_and_weight(nodes, edges):
    n = len(nodes)
    A = np.zeros((n,n), dtype=int)
    W = np.full((n,n), np.inf, dtype=float)
    for i,j in edges:
        A[i,j]=A[j,i]=1
        d = float(np.linalg.norm(nodes[i]-nodes[j]))
        W[i,j]=W[j,i]=d
    np.fill_diagonal(W, 0.0)
    return A, W

def dijkstra(A, W, start, goal):
    n = A.shape[0]
    dist = [math.inf]*n
    prev = [-1]*n
    dist[start] = 0.0
    pq = PriorityQueue()  # (cost, node, path)
    pq.put((0.0, start, [start]))
    explored_nodes = []
    while not pq.empty():
        cost,u,path = pq.get()
        if cost > dist[u]:
            continue
        explored_nodes.append(u) # For tracking explored nodes
        if u == goal:
            return path, cost, explored_nodes
        for v, is_nb in enumerate(A[u]):
            if is_nb == 1:
                alt = cost + float(W[u,v])
                if alt < dist[v]:
                    dist[v]=alt; prev[v]=u
                    pq.put((alt, v, path+[v]))
    return [], math.inf, []

class ExtendedDijkstraPath(Node):
    def __init__(self):
        super().__init__('extended_dijkstra_path')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(String, '/grid_map_path', self.map_cb, 1) # For plotting
        self.timer = self.create_timer(CTRL_DT, self.loop)

        self.has_odom = False
        self.x=self.y=self.yaw=0.0

        self.A, self.W = build_adj_and_weight(NODES, EDGES)

        self.unvisited_nodes = set(range(len(NODES))) # All nodes are unvisited
        self.current_node_idx = -1
        self.current_goal_idx = -1
        self.full_path = [] # For logging the full order
        
        self.path_world = None   # [(x,y), ...]
        self.i = 0
        self.planned = False
        self.done = False

        self.t0 = self.get_clock().now()
        self.traj = []            # [(t,x,y,yaw), ...]
        self.map_csv_path = None
        os.makedirs(OUTDIR, exist_ok=True)

        self.get_logger().info("DijkstraPath ready. Move robot to a random pose, set GOAL_IDX.")

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

    def map_cb(self, msg: String):
        self.map_csv_path = msg.data
        

    def plan_next_path(self):
        # nearest waypoints
        if self.current_node_idx == -1:
            # First run
            dists = np.linalg.norm(NODES - np.array([self.x,self.y]), axis=1)
            start_idx = int(np.argmin(dists))
        else:
            # Arrived to another node
            start_idx = self.current_node_idx

        if start_idx in self.unvisited_nodes:
            # Arrived at new unvisited node
            self.unvisited_nodes.remove(start_idx)
            self.full_path.append(start_idx)
            self.get_logger().info(f"Arrived at node {start_idx}. "
                                   f"Unvisited: {self.unvisited_nodes}")

        if not self.unvisited_nodes:
            # All nodes visited
            self.get_logger().info(f"All nodes visited. DONE")
            self.get_logger().info(f"Exploration order: {self.full_path}")
            self.cmd_pub.publish(Twist())
            self.done = True
            self.plot_trajectory()
            return
        
        # Find nearest unvisited node
        best_cost = math.inf
        best_path = []
        best_goal_idx = -1
        for goal_idx in self.unvisited_nodes:
            path_nodes, cost, explored_nodes = dijkstra(self.A, self.W, start_idx, goal_idx)
            #self.get_logger().info(f"Explored nodes: {explored_nodes}")
            if cost < best_cost and cost != 0:
                best_cost = cost
                best_path = path_nodes
                best_goal_idx = goal_idx

        if not best_path:
            self.get_logger().error("No path found from node {} to {}.".format(start_idx, goal_idx))
            self.done = True
            self.cmd_pub.publish(Twist())
            return
        
        self.current_goal_idx = best_goal_idx

        self.path_world = [tuple(NODES[k]) for k in best_path]
        self.i = 1 # Start moving to the second node
        self.planned = True

        path_txt = " -> ".join(str(k) for k in best_path)
        self.get_logger().info(f"Planned Dijkstra to shortest unvisited node: {self.current_goal_idx} with path: {path_txt}  (length={best_cost:.2f} m)")

    def loop(self):
        if not self.has_odom or self.done:
            return

        if not self.planned:
            self.plan_next_path()
            if not self.planned:
                # Done = true
                return

        # follow
        gx, gy = self.path_world[self.i]
        dx, dy = gx - self.x, gy - self.y
        dist = math.hypot(dx, dy)
        target = math.atan2(dy, dx)
        yaw_err = ((target - self.yaw) + math.pi) % (2.0 * math.pi) - math.pi


        cmd = Twist()
        if abs(yaw_err) > YAW_TOL:
            cmd.angular.z = ANG_SPEED if yaw_err > 0 else -ANG_SPEED
        else:
            cmd.linear.x = LIN_SPEED
        
        self.cmd_pub.publish(cmd)

        if dist < POS_TOL:
            self.i += 1
            if self.i >= len(self.path_world):
                # Reached end of current path goal
                self.get_logger().info(f"Reached node {self.current_goal_idx}.")
                self.planned = False
                self.current_node_idx = self.current_goal_idx
                

    def plot_trajectory(self):
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


def main(args=None):
    rclpy.init(args=args)
    node = ExtendedDijkstraPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
