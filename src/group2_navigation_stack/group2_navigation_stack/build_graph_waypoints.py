#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx

from queue import PriorityQueue
from math import inf

waypoints = np.array([
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

number_of_nodes = len(waypoints)

neighbor_pairs = [
    (0,1), (1,2), (2,3),
    (3,4), (4,5), (5,6),
    (6,7), (7,8), (8,9),
    (1,6), (3,9), (4,9),
]

G = nx.Graph()
G.add_edges_from(neighbor_pairs)
positions = {i: tuple(waypoints[i]) for i in range(number_of_nodes)}
plt.figure(figsize=(9, 6))
nx.draw(G, pos=positions, with_labels=True, node_size=600, node_color='lightblue', font_weight='bold')
plt.savefig('/home/ws/maps/graph.png', dpi=220)
plt.close()



# adjacency and weight matrices
adjacency_matrix = np.zeros((number_of_nodes, number_of_nodes), dtype=int)
weights = np.full((number_of_nodes, number_of_nodes), np.inf, dtype=float)

for i, j in neighbor_pairs:
    adjacency_matrix[i, j] = 1
    adjacency_matrix[j, i] = 1
    d = float(np.linalg.norm(waypoints[i] - waypoints[j]))  # real Euclidean distance
    weights[i, j] = d
    weights[j, i] = d
np.fill_diagonal(weights, 0.0)

print("Adjacency matrix A:\n", adjacency_matrix)
print("\nWeight matrix W (meters):\n", weights)


# Dijkstra's algorithm
start_node = 0
end_node   = 9
N = adjacency_matrix.shape[0]

dist = [inf]*N
prev = [-1]*N
dist[start_node] = 0.0

pq = PriorityQueue()
# Each element: (distance, node_id, path_to_node)
pq.put((0, start_node, [start_node]))

while not pq.empty():
    current_cost, current_node, current_path = pq.get()
    # print(current_path)

    if current_cost > dist[current_node]:
        continue

    if current_node == end_node:
        print("We have finished!")
        print("The path is: {}".format(current_path))
        print("The total cost is: {}".format(current_cost))
        break

    for neighbor_id, is_neighbor in enumerate(adjacency_matrix[current_node]):
        if is_neighbor == 1:
            alt = current_cost + float(weights[current_node][neighbor_id])
            if alt < dist[neighbor_id]:
                dist[neighbor_id] = alt
                prev[neighbor_id] = current_node
                pq.put((alt, neighbor_id, current_path + [neighbor_id])) 

