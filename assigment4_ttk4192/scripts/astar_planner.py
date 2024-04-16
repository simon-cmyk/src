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
        imgmap = Image.open(image_file)
        self.arr = np.array(imgmap)
        self.start = None
        self.goal = None
        self.obstacle_map = np.zeros([self.arr.shape[0], self.arr.shape[1]])
        self.obstacle_map_inflated = np.zeros([self.arr.shape[0], self.arr.shape[1]])
        goal_color = np.array([255, 0, 0, 255])
        start_color = np.array([0, 255, 0, 255])
        obstacle_color = np.array([0, 0, 0, 255])
        self.visited_color = np.array(SKY_BLUE)

        self.map_changed = True
        self.nodes = []

        # self.fig = plt.figure()               # MNK 160424 Conflict with Hybrid A* plotting
        # self.imgmap = plt.imshow(self.arr)    # MNK 160424 Conflict with Hybrid A* plotting

        for i in range(self.arr.shape[0]):
            for j in range(self.arr.shape[1]):
                pixel = self.arr[i][j]

                if np.array_equal(pixel, obstacle_color):
                    self.obstacle_map[i][j] = 1

                else:
                    self.nodes.append((i, j))

        self.inflate_obstacles()


    def inflate_obstacles(self):
        safe_dist = 20    # inflation distance TODO: change to be parameter
        x_max = self.arr.shape[0]-1
        y_max = self.arr.shape[1]-1
        for i in range(self.arr.shape[0]):
            for j in range(self.arr.shape[1]):
                if self.obstacle_map[i][j] == 1:
                    x1 = max(0,i-safe_dist)
                    x2 = min(x_max,i+safe_dist)
                    y1 = max(0,j-safe_dist)
                    y2 = min(y_max,j+safe_dist)
                    self.obstacle_map_inflated[x1:x2+1,y1:y2+1] = np.ones([x2-x1+1, y2-y1+1])




    def get_list_of_nodes(self):
        return self.nodes

    def get_neighbors(self, node):

        neighbors = []

        # for x, y in [[-1, 0], [1, 0], [0, -1], [0, 1]]:
        for x, y in [[-1, 0], [1, 0], [0, -1], [0, 1], [-1,-1], [-1,1], [1,-1], [1,1]]:
            try:
                potential_neighbor = [node[0] + x, node[1] + y]
                if potential_neighbor[0] < 0 or potential_neighbor[1] < 0:
                    continue

                if self.obstacle_map_inflated[potential_neighbor[0]][potential_neighbor[1]] == 0:
                    neighbors.append(tuple(potential_neighbor))

            except IndexError:
                continue

        return neighbors

    def add_visited_node(self, node):
        if node != tuple(self.start) and node != tuple(self.goal):
            self.arr[node[0]][node[1]] = self.visited_color

        # self.visualize_map()

    def visualize_map(self):
        self.imgmap.set_data(self.arr)
        plt.pause(0.0001)

    def add_shortest_path(self, path):
        path_color = np.array(REDDISH_PURPLE)
        for node in path:
            if node != tuple(self.start) and node != tuple(self.goal):
                self.arr[node[0]][node[1]] = path_color

        plt.figure()
        plt.imshow(self.arr)
        plt.show()

    def get_start_node(self):
        return self.start

    def get_goal_node(self):
        return self.goal
    
    def set_start_node(self, node):
        self.start = node
        self.arr[node[0]][node[1]] = np.array(VERMILLION)

    def set_goal_node(self, node):
        self.goal = node
        self.arr[node[0]][node[1]] = np.array(VERMILLION)



def a_star(graph, start_node, goal_node, heuristic_function=None):

    dist = {}           # dictionary storing shortest distance to node u
    prev = {}           # dictionary storing parent of node u
    Q = []              # nodes that are not yet discovered
    Frontier = []       # nodes at frontier are candidate shortest distance nodes
    visited = 0         # number of nodes explored

    graph.set_start_node(start_node)
    graph.set_goal_node(goal_node)

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


    # Find shortest path
    path = []
    node = graph.get_goal_node()
    while tuple(node) != tuple(graph.get_start_node()):
        path.append(node)
        if prev[tuple(node)] == None:
            print("Failure: Goal is not reachable")
            return [], visited
        else:
            node = tuple(prev[tuple(node)])

    path.append(graph.get_start_node())

    graph.add_shortest_path(path)

    path = path[:-1:7] + [path[-1]] # Only keep every 7 point in path

    return path, visited

def mirror_coordinate(point, h, scale):
    return [scale*point[1], h-scale*point[0]]

def mirrior_plan(path, height, scale):
    """mirror path: i.e. origin of GNC is bottom-left while A* origin is top_left
       map scale is different: i.e. GNC in meters, A* in pixels => 1m = 100pxl"""
    reverse_path = []
    for point in path:
        reverse_path.insert(0, mirror_coordinate(point,height,scale))

    reverse_path = np.array(reverse_path)
    return reverse_path

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
    graph = Graph('maps/map_ttk4192CA4.png')
    start_node = [250,40]
    goal_node = [30,370]
    path, visited = a_star(graph,start_node,goal_node, heuristic_function=heuristic_euclidean)

    graph.add_shortest_path(path)
    print(visited)