#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math
from queue import PriorityQueue


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

# Goal
#GOAL_IDX = 8
GOAL_IDX = 9

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


def a_star(A, W, NODES, start, goal):
    n = A.shape[0]

    # g_score: distance from start node. (Same as 'dist' in Dijikstra)
    # f_score: g + h
    # h_score: heuristic from goal

    g_score = [math.inf] * n
    prev = [-1] * n
    g_score[start] = 0.0

    # h_score for start node from goal
    h_start = float(np.linalg.norm(NODES[start] - NODES[goal]))

    f_start = g_score[start] + h_start

    pq = PriorityQueue()  # (f_score, g_score, node_index, path_list)
    pq.put((f_start, 0.0, start, [start]))
    
    while not pq.empty():
        # Get node with lowest f_score
        f_u, g_u, u, path = pq.get()

        if g_u > g_score[u]:
            continue
        if u == goal:
            return path, g_u  # Goal found
        
        # Explore neighbours
        for v, is_nb in enumerate(A[u]):
            if is_nb == 1:
                alt_g = g_u + float(W[u,v])

                # If new path better, update g_score and prev
                if alt_g < g_score[v]:
                    g_score[v]=alt_g; prev[v]=u

                    # Calculate heuristic and f_score for neighbor v
                    h_v = float(np.linalg.norm(NODES[v] - NODES[start]))
                    f_v = alt_g + h_v

                    pq.put((f_v, alt_g, v, path+[v]))

    # No path found
    return [], math.inf


class AStarPath(Node):
    def __init__(self):
        super().__init__('a_star_path')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(CTRL_DT, self.loop)

        self.has_odom = False
        self.x=self.y=self.yaw=0.0

        self.A, self.W = build_adj_and_weight(NODES, EDGES)

        
        self.path_world = None   # [(x,y), ...]
        self.i = 0
        self.planned = False
        self.done = False

        self.get_logger().info("AStarPath ready. Move robot to a random pose, set GOAL_IDX.")

    def odom_cb(self, msg: Odometry):
        self.has_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        

    def plan_path(self):
        # nearest waypoints
        dists = np.linalg.norm(NODES - np.array([self.x,self.y]), axis=1)
        start_idx = int(np.argmin(dists))

        path_nodes, total = a_star(self.A, self.W, NODES, start_idx, GOAL_IDX)

        if not path_nodes:
            self.get_logger().error("No path found from node {} to {}.".format(start_idx, GOAL_IDX))
            self.done = True
            return

        self.path_world = [tuple(NODES[k]) for k in path_nodes]
        self.i = 0
        self.planned = True

        path_txt = " -> ".join(str(k) for k in path_nodes)
        self.get_logger().info(f"Planned once using A*: {path_txt}  (length={total:.2f} m)")

    def loop(self):
        if not self.has_odom or self.done:
            return

        if not self.planned:
            self.plan_path()
            if not self.planned:
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
                self.cmd_pub.publish(Twist())
                self.get_logger().info("Goal reached. Done.")
                self.done = True

def main(args=None):
    rclpy.init(args=args)
    node = AStarPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
