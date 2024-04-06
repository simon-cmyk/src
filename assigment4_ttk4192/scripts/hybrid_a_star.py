import numpy as np
import matplotlib.pyplot as plt
import time
import argparse
import matplotlib.animation as animation
from math import pi, tan, sqrt
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection, LineCollection
from matplotlib.patches import Rectangle
from itertools import product
import argparse
from utils.grid import Grid_robplan
from utils.car import SimpleCar
from utils.environment import Environment_robplan
from utils.dubins_path import DubinsPath
from utils.astar import Astar
from utils.utils import plot_a_car, get_discretized_thetas, round_theta, same_point

""" ---------------------------------------------------------------
 Code for pathfinding using hybrid A-star 
 dependencies: Curve generator (Dubins path)
 Based on: Open source scripts in GitHub
 Date: 18.01.23
--------------------------------------------------------------------
"""

class HybridAstar:
    """ Hybrid A* search procedure. """
    #  def __init__(self, car, grid, reverse, unit_theta=pi/12, dt=1e-2, check_dubins=1):
    def __init__(self, car, grid, reverse, unit_theta, dt, check_dubins):
        self.car = car
        self.grid = grid
        self.reverse = reverse
        self.unit_theta = unit_theta
        self.dt = dt
        self.check_dubins = check_dubins

        self.start = self.car.start_pos
        self.goal = self.car.end_pos

        self.r = self.car.l / tan(self.car.max_phi)
        self.drive_steps = int(sqrt(2)*self.grid.cell_size/self.dt) + 1
        self.arc = self.drive_steps * self.dt
        self.phil = [-self.car.max_phi, 0, self.car.max_phi]
        self.ml = [1, -1]

        if reverse:
            self.comb = list(product(self.ml, self.phil))
        else:
            self.comb = list(product([1], self.phil))

        self.dubins = DubinsPath(self.car)
        self.astar = Astar(self.grid, self.goal[:2])

        self.w1 = 0.95 # 0.95 weight for astar heuristic
        self.w2 = 0.05 # 0.05 weight for simple heuristic
        self.w3 = 0.30 # 0.30 weight for extra cost of steering angle change
        self.w4 = 0.20 # 0.10 weight for extra cost of turning
        self.w5 = 0.50 # 2.00 weight for extra cost of reversing

        self.thetas = get_discretized_thetas(self.unit_theta)
    
    def construct_node(self, pos):
        """ Create node for a pos. """

        theta = pos[2]
        pt = pos[:2]

        theta = round_theta(theta % (2*pi), self.thetas)
        
        cell_id = self.grid.to_cell_id(pt)
        grid_pos = cell_id + [theta]

        node = Node(grid_pos, pos)

        return node
    
    def simple_heuristic(self, pos):
        """ Heuristic by Manhattan distance. """

        return abs(self.goal[0]-pos[0]) + abs(self.goal[1]-pos[1])
        
    def astar_heuristic(self, pos):
        """ Heuristic by standard astar. """

        h1 = self.astar.search_path(pos[:2]) * self.grid.cell_size
        h2 = self.simple_heuristic(pos[:2])
        
        return self.w1*h1 + self.w2*h2

    def get_children(self, node, heu, extra):
        """ Get successors from a state. """

        children = []
        for m, phi in self.comb:

            # don't go back
            if node.m and node.phi == phi and node.m*m == -1:
                continue

            if node.m and node.m == 1 and m == -1:
                continue

            pos = node.pos
            branch = [m, pos[:2]]

            for _ in range(self.drive_steps):
                pos = self.car.step(pos, phi, m)
                branch.append(pos[:2])

            # check safety of route-----------------------
            pos1 = node.pos if m == 1 else pos
            pos2 = pos if m == 1 else node.pos
            if phi == 0:
                safe = self.dubins.is_straight_route_safe(pos1, pos2)
            else:
                d, c, r = self.car.get_params(pos1, phi)
                safe = self.dubins.is_turning_route_safe(pos1, pos2, d, c, r)
            # --------------------------------------------
            
            if not safe:
                continue
            
            child = self.construct_node(pos)
            child.phi = phi
            child.m = m
            child.parent = node
            child.g = node.g + self.arc
            child.g_ = node.g_ + self.arc

            if extra:
                # extra cost for changing steering angle
                if phi != node.phi:
                    child.g += self.w3 * self.arc
                
                # extra cost for turning
                if phi != 0:
                    child.g += self.w4 * self.arc
                
                # extra cost for reverse
                if m == -1:
                    child.g += self.w5 * self.arc

            if heu == 0:
                child.f = child.g + self.simple_heuristic(child.pos)
            if heu == 1:
                child.f = child.g + self.astar_heuristic(child.pos)
            
            children.append([child, branch])

        return children
    
    def best_final_shot(self, open_, closed_, best, cost, d_route, n=10):
        """ Search best final shot in open set. """

        open_.sort(key=lambda x: x.f, reverse=False)

        for t in range(min(n, len(open_))):
            best_ = open_[t]
            solutions_ = self.dubins.find_tangents(best_.pos, self.goal)
            d_route_, cost_, valid_ = self.dubins.best_tangent(solutions_)
        
            if valid_ and cost_ + best_.g_ < cost + best.g_:
                best = best_
                cost = cost_
                d_route = d_route_
        
        if best in open_:
            open_.remove(best)
            closed_.append(best)
        
        return best, cost, d_route
    
    def backtracking(self, node):
        """ Backtracking the path. """

        route = []
        while node.parent:
            route.append((node.pos, node.phi, node.m))
            node = node.parent
        
        return list(reversed(route))
    
    def search_path(self, heu=1, extra=False):
        """ Hybrid A* pathfinding. """

        root = self.construct_node(self.start)
        root.g = float(0)
        root.g_ = float(0)
        
        if heu == 0:
            root.f = root.g + self.simple_heuristic(root.pos)
        if heu == 1:
            root.f = root.g + self.astar_heuristic(root.pos)

        closed_ = []
        open_ = [root]

        count = 0
        while open_:
            count += 1
            best = min(open_, key=lambda x: x.f)

            open_.remove(best)
            closed_.append(best)

            # check dubins path
            if count % self.check_dubins == 0:
                solutions = self.dubins.find_tangents(best.pos, self.goal)
                d_route, cost, valid = self.dubins.best_tangent(solutions)
                
                if valid:
                    best, cost, d_route = self.best_final_shot(open_, closed_, best, cost, d_route)
                    route = self.backtracking(best) + d_route
                    path = self.car.get_path(self.start, route)
                    cost += best.g_
                    print('Shortest path: {}'.format(round(cost, 2)))
                    print('Total iteration:', count)
                    
                    return path, closed_

            children = self.get_children(best, heu, extra)

            for child, branch in children:

                if child in closed_:
                    continue

                if child not in open_:
                    best.branches.append(branch)
                    open_.append(child)

                elif child.g < open_[open_.index(child)].g:
                    best.branches.append(branch)

                    c = open_[open_.index(child)]
                    p = c.parent
                    for b in p.branches:
                        if same_point(b[-1], c.pos[:2]):
                            p.branches.remove(b)
                            break
                    
                    open_.remove(child)
                    open_.append(child)

        return None, None

def plotstart(env, car):
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(-0.02,env.lx + 0.02)
    ax.set_ylim(-0.02, env.ly + 0.02)
    ax.set_aspect("equal")

    plt.grid(which='both')

    start_state = car.get_car_state(car.start_pos)
    end_state = car.get_car_state(car.end_pos)

    ax.plot(car.start_pos[0], car.start_pos[1], 'ro', markersize=6)
    ax = plot_a_car(ax, end_state.model)
    ax = plot_a_car(ax, start_state.model)

    for ob in env.obs:
        ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))

    plt.show()

def main_hybrid_a(heu,start_pos, end_pos, reverse, extra):
    l = 0.281
    start_pos, end_pos = WpToCarFrame(start_pos, l), WpToCarFrame(end_pos, l)
    tc = map_grid_robplan()
    print(start_pos)
    print(end_pos)

    env = Environment_robplan(tc.obs, lx=5.0, ly=2.9, safe_distance=0.01)
    car = SimpleCar(env, start_pos, end_pos, l, max_phi=pi/5)
    grid = Grid_robplan(env, cell_size=0.1)

    plotstart(env, car)

    hastar = HybridAstar(car, grid, reverse, unit_theta=pi/12, dt=1e-2, check_dubins=True)

    t = time.time()
    path, closed_ = hastar.search_path(heu, extra)
    print('Total time: {}s'.format(round(time.time()-t, 3)))

    if not path:
        print('No valid path!')
        return
    # a post-processing is required to have path list
    path = path[::5] + [path[-1]]
    branches = []
    bcolors = []
    for node in closed_:
        for b in node.branches:
            branches.append(b[1:])
            bcolors.append('y' if b[0] == 1 else 'b')

    xl, yl = [], []
    xl_np1,yl_np1=[],[]
    carl = []
    dt_s=int(25)  # samples for gazebo simulator
    for i in range(len(path)):
        xl.append(path[i].pos[0])
        yl.append(path[i].pos[1])
        carl.append(path[i].model[0])
        if i==0 or i==len(path):
            xl_np1.append(path[i].pos[0])
            yl_np1.append(path[i].pos[1])            
        elif dt_s*i<len(path):
            xl_np1.append(path[i*dt_s].pos[0])
            yl_np1.append(path[i*dt_s].pos[1])      
    # defining way-points
    xl_np=np.array(xl_np1)
    xl_np=xl_np
    yl_np=np.array(yl_np1)
    yl_np=yl_np
    global WAYPOINTS
    WAYPOINTS=np.column_stack([xl_np,yl_np])
    print(WAYPOINTS)
    
    start_state = car.get_car_state(car.start_pos)
    end_state = car.get_car_state(car.end_pos)

    # plot and animation
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(-0.02,env.lx + 0.02)
    ax.set_ylim(-0.02, env.ly + 0.02)
    ax.set_aspect("equal")

    plt.grid(which='both')

    
    for ob in env.obs:
        ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))
    
    ax.plot(car.start_pos[0], car.start_pos[1], 'ro', markersize=6)
    ax = plot_a_car(ax, end_state.model)
    ax = plot_a_car(ax, start_state.model)

    _branches = LineCollection(branches, color='b', alpha=0.8, linewidth=1)
    ax.add_collection(_branches)

    _carl = PatchCollection(carl[::20], color='m', alpha=0.1, zorder=3)
    ax.add_collection(_carl)
    ax.plot(xl, yl, color='whitesmoke', linewidth=2, zorder=3)
    _car = PatchCollection(path[-1].model, match_original=True, zorder=4)
    ax.add_collection(_car)

    _branches = LineCollection([], linewidth=1)
    ax.add_collection(_branches)

    _path, = ax.plot([], [], color='lime', linewidth=2)
    _carl = PatchCollection([])
    ax.add_collection(_carl)
    _path1, = ax.plot([], [], color='w', linewidth=2)
    _car = PatchCollection([])
    ax.add_collection(_car)
    
    frames = len(branches) + len(path) + 1

    def init():
        _branches.set_paths([])
        _path.set_data([], [])
        _carl.set_paths([])
        _path1.set_data([], [])
        _car.set_paths([])

        return _branches, _path, _carl, _path1, _car

    def animate(i):

        edgecolor = ['k']*5 + ['r']
        facecolor = ['y'] + ['k']*4 + ['r']

        if i < len(branches):
            _branches.set_paths(branches[:i+1])
            _branches.set_color(bcolors)
        
        else:
            _branches.set_paths(branches)

            j = i - len(branches)

            _path.set_data(xl[min(j, len(path)-1):], yl[min(j, len(path)-1):])

            sub_carl = carl[:min(j+1, len(path))]
            _carl.set_paths(sub_carl[::4])
            _carl.set_edgecolor('k')
            _carl.set_facecolor('m')
            _carl.set_alpha(0.1)
            _carl.set_zorder(3)

            _path1.set_data(xl[:min(j+1, len(path))], yl[:min(j+1, len(path))])
            _path1.set_zorder(3)

            _car.set_paths(path[min(j, len(path)-1)].model)
            _car.set_edgecolor(edgecolor)
            _car.set_facecolor(facecolor)
            _car.set_zorder(3)

        return _branches, _path, _carl, _path1, _car

    ani = animation.FuncAnimation(fig, animate, init_func=init, frames=frames,
                                  interval=1, repeat=True, blit=True)

    plt.show()

class Node:
    """ Hybrid A* tree node. """

    def __init__(self, grid_pos, pos):

        self.grid_pos = grid_pos
        self.pos = pos
        self.g = None
        self.g_ = None
        self.f = None
        self.parent = None
        self.phi = 0
        self.m = None
        self.branches = []

    def __eq__(self, other):

        return self.grid_pos == other.grid_pos
    
    def __hash__(self):

        return hash((self.grid_pos))

global WAYPOINTS # Siste er antagelse om pose
WAYPOINTS = [
                [0.30, 0.30, pi/2],
                [1.85, 0.40, 0],
                [3.00, 1.05, 0],
                [3.25, 2.45, pi/2],
                [4.70, 0.50, -pi/2],
                [0.80, 2.50, -pi],
                [3.60, 1.70, 0]
            ]

class map_grid_robplan:
    def __init__(self):
        # x, y, w, h. Not added everything
        self.obs = [
            [0.00, 0.00, 0.20, 0.20],
            [1.35, 0.55, 0.90, 0.30],
            [2.45, 0.55, 0.90, 0.30],
            [3.15, 2.70, 0.20, 0.20],
            [4.80, 0.30, 0.20, 0.20],
            [0.60, 2.40, 0.20, 0.20],
            [3.20, 1.85, 0.80, 0.50],
            [1.00, 1.50, 0.40, 0.80],
            [1.50, 1.50, 0.40, 0.80],
            [2.00, 1.50, 0.70, 0.80], 
            [0.00, -0.02, 5.00, 0.02],
            [-0.02, 0.00, 0.02, 2.90],
            [0.00, 2.90, 5.00, 0.02],
            [5.00, 0.00, 0.02, 2.90], 
            [0.00, 1.00, 0.5, 0.20],
            [3.00, 0.00, 2.00, 0.30],
        ]

    # p = argparse.ArgumentParser()
    # p.add_argument('-heu', type=int, default=1, help='heuristic type')  #A* heuristic
    # p.add_argument('-r', action='store_true', help='allow reverse or not')
    # p.add_argument('-e', action='store_true', help='add extra cost or not')
    # p.add_argument('-g', action='store_true', help='show grid or not')

def WpToCarFrame(wp, l): 
    # Waypoint for car is the backtires.
    wp[0] -= l/2 * np.cos(wp[2])
    wp[1] -= l/2 * np.sin(wp[2]) 
    return wp

if __name__ == '__main__':
    print("Executing hybrid A* algorithm")

    start_pos   = WAYPOINTS[1]       # Here defined initial position [x,y,angle]
    end_pos     = WAYPOINTS[2]       # Target point                  [x,y, angle]

    heu         = 1                    # Making use of all heuristics
    main_hybrid_a(heu, start_pos,end_pos, reverse=True, extra=True)


    
