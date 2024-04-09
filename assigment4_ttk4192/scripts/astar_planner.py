import math
import numpy as np
import heapq


import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

VERMILLION = (213, 94, 0, 255)
BLUE_GREEN = (0, 158, 115, 255)
SKY_BLUE = (86, 180, 233, 255)
REDDISH_PURPLE = (204, 121, 167, 255)


class Graph:
    def __init__(self, image_file):
        img = Image.open(image_file)
        self.arr = np.array(img)
        self.start = None
        self.goal = None
        self.obstacle_map = np.zeros([self.arr.shape[0], self.arr.shape[1]])
        goal_color = np.array([255, 0, 0, 255])
        start_color = np.array([0, 255, 0, 255])
        obstacle_color = np.array([0, 0, 0, 255])
        self.visited_color = np.array(SKY_BLUE)

        self.map_changed = True
        self.nodes = []

        self.fig = plt.figure()
        self.img = plt.imshow(self.arr)

        for i in range(self.arr.shape[0]):
            for j in range(self.arr.shape[1]):
                pixel = self.arr[i][j]
                if np.array_equal(pixel, goal_color):
                    self.goal = [i, j]
                    self.nodes.append(tuple(self.goal))
                    self.arr[i][j] = np.array(VERMILLION)

                elif np.array_equal(pixel, start_color):
                    self.start = [i, j]
                    self.nodes.append(tuple(self.start))
                    self.arr[i][j] = np.array(BLUE_GREEN)

                elif np.array_equal(pixel, obstacle_color):
                    self.obstacle_map[i][j] = 1

                else:
                    self.nodes.append((i, j))

    def get_list_of_nodes(self):
        return self.nodes

    def get_neighbors(self, node):

        neighbors = []

        for x, y in [[-1, 0], [1, 0], [0, -1], [0, 1]]:
            try:
                potential_neighbor = [node[0] + x, node[1] + y]
                if potential_neighbor[0] < 0 or potential_neighbor[1] < 0:
                    continue
                if self.obstacle_map[potential_neighbor[0]][potential_neighbor[1]] == 0:
                    neighbors.append(tuple(potential_neighbor))

            except IndexError:
                continue

        return neighbors

    def add_visited_node(self, node):
        if node != tuple(self.start) and node != tuple(self.goal):
            self.arr[node[0]][node[1]] = self.visited_color

        # self.visualize_map()

    def visualize_map(self):
        self.img.set_data(self.arr)
        plt.pause(0.0001)

    def add_shortest_path(self, path):
        path_color = np.array(REDDISH_PURPLE)
        for node in path:
            if node != tuple(self.start) and node != tuple(self.goal):
                self.arr[node[0]][node[1]] = path_color
        self.visualize_map()
        plt.show()

    def get_start_node(self):
        return self.start

    def get_goal_node(self):
        return self.goal


# You should only need to use the following functions to implement A*:
# Graph(path_to_map) # path_to_map can, for instance, be 'maps/map1.png'
# graph.get_list_of_nodes()
# graph.get_neighbors(node)
# graph.get_start_node()
# graph.get_goal_node()

# For visualizing the algorithm use the following functions:
# graph.add_visited_node(node) # Use this when you are visiting a new node to update the visualization
# graph.add_shortest_path(path) # Use this to visualize the shortest path after you have found it.

def a_star(graph, heuristic_function=None):
    # Here you need to implement A*. The implementation should return the path from start to goal, in the form of a
    # sequential list of nodes in the path.

    dist = {}           # dictionary storing shortest distance to node u
    prev = {}           # dictionary storing parent of node u
    Q = []              # nodes that are not yet discovered
    Frontier = []       # nodes at frontier are candidate shortest distance nodes
    visited = 0         # number of nodes explored

    # initialize dist, prev and Q
    for node in graph.get_list_of_nodes():
        dist[node] = np.inf
        prev[node] = None

    dist[tuple(graph.get_start_node())] = 0                                                             # set distance to start node = 0
    heapq.heappush(Frontier,(dist[tuple(graph.get_start_node())], tuple(graph.get_start_node())))       # add start node to frontier
    Done = False

    while Frontier != [] and Done != True:
        u = heapq.heappop(Frontier)[1]  # next shortest distance
        graph.add_visited_node(u)       # update animation
        visited = visited + 1

        # update distance of neighbor nodes
        for neighbor in graph.get_neighbors(u):
            if tuple(neighbor) == tuple(graph.get_goal_node()): # goal node detected
                prev[neighbor] = u
                Done = True
                break
            new_cost = dist[u] + 1
            if dist[neighbor] > new_cost:
                dist[neighbor] = new_cost
                prev[neighbor] = u
                priority = new_cost + heuristic_function(graph, neighbor)
                heapq.heappush(Frontier,(priority,neighbor))

    graph.visualize_map()

    # Find shortest path
    path = []
    node = graph.get_goal_node()
    while tuple(node) != tuple(graph.get_start_node()):
        path.append(node)
        node = tuple(prev[tuple(node)])

    path.append(graph.get_start_node())

    return path, visited

def heuristic_euclidean(graph, node):
    goal = graph.get_goal_node()
    return math.sqrt((goal[0] - node[0])**2 + (goal[1] - node[1])**2)

def heuristic_manhattan(graph, node):
    goal = graph.get_goal_node()
    return abs(goal[0] - node[0]) + abs(goal[1] - node[1])

def heuristic_diagonal(graph, node):
    goal = graph.get_goal_node()
    return min(abs(goal[0] - node[0]), abs(goal[1] - node[1]))

if __name__ == "__main__":
    graph = Graph('maps/map5.png')
    path, visited = a_star(graph, heuristic_function=heuristic_manhattan)

    graph.add_shortest_path(path)
    print(visited)