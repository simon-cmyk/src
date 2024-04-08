import numpy as np
import time
from math import pi, tan, sqrt
from itertools import product
from utils.map import map_grid_robplan
from utils.grid import Grid_robplan
from utils.car import RoboCar
from utils.node import Node
from utils.environment import Environment_robplan
from utils.dubins_path import DubinsPath
from utils.astar import Astar
from utils.utils import animate_solution, get_discretized_thetas, plotstart, round_theta, same_point, WpToCarFrame

# TODO: Må fikse sånn at start og slutt vises i midten av roboten. 
# TODO: Teste med den.

""" ---------------------------------------------------------------
 Code adapted from the Course TTK4192 at NTNU
 dependenncies: Curve generator (Dubins path)
 Based on: Open source scripts in GitHub
 Date: 08.04.24
--------------------------------------------------------------------
"""

class HybridAstar:
    """ Hybrid A* search procedure. """
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

        if reverse: self.comb = list(product(self.ml, self.phil))
        else: self.comb = list(product([1], self.phil))

        self.dubins = DubinsPath(self.car)
        self.astar = Astar(self.grid, self.goal[:2])

        # All are weights for choosing heuristic. (default in the comments) 
        self.w1 = 0.95 # 0.95 astar heuristic
        self.w2 = 0.05 # 0.05 simple heuristic
        self.w3 = 0.30 # 0.30 extra cost of steering angle change
        self.w4 = 0.10 # 0.10 extra cost of turning
        self.w5 = 0.50 # 2.00 extra cost of reversing

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
        temp = self.astar.search_path(pos[:2])
        if temp is not None:
            h1 = self.astar.search_path(pos[:2]) * self.grid.cell_size
        else:
            h1 = 100
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

            pos1 = node.pos if m == 1 else pos
            pos2 = pos if m == 1 else node.pos
            if phi == 0:
                safe = self.dubins.is_straight_route_safe(pos1, pos2)
            else:
                d, c, r = self.car.get_params(pos1, phi)
                safe = self.dubins.is_turning_route_safe(pos1, pos2, d, c, r)
            
            if not safe:
                continue
            
            child = self.construct_node(pos)
            child.phi = phi
            child.m = m
            child.parent = node
            child.g = node.g + self.arc
            child.g_ = node.g_ + self.arc

            if extra:
                if phi != node.phi:
                    child.g += self.w3 * self.arc
                if phi != 0:
                    child.g += self.w4 * self.arc
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

def main_hybrid_a(heu,start_pos, end_pos, reverse, extra, visualize):
    l = 0.281
    w = 0.306
    max_phi = pi/4
    start_pos, end_pos = WpToCarFrame(start_pos, l), WpToCarFrame(end_pos, l)
    tc = map_grid_robplan()
  
    env = Environment_robplan(tc.obs, lx=5.0, ly=2.9, safe_distance=0.02)
    car = RoboCar(env, start_pos, end_pos, l, max_phi, w)
    grid = Grid_robplan(env, cell_size=0.1)

    if visualize == True:
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

    Wpts = np.array([list(p.pos) for p in path])
    carl = [p.model[0] for p in path]
    
    if visualize == True: 
        animate_solution(car, env, path, Wpts[:, 0], Wpts[:, 1], carl, grid.cell_size)

    return Wpts

global WAYPOINT 
WAYPOINT = [
                [0.30, 0.30, pi/2],
                [1.85, 0.35, 0],
                [3.00, 1.05, 0],
                [3.25, 2.48, pi],
                [4.65, 0.70, 0],
                [0.95, 2.60, pi],
                [3.60, 1.40, pi]
            ]

if __name__ == '__main__':
    print("Executing hybrid A* algorithm")

    heu         = 1 
    start_pos   = WAYPOINT[0]       # Here defined initial position [x,y,angle]
    end_pos     = WAYPOINT[1]       # Target point                  [x,y, angle]

    my_path1 = main_hybrid_a(heu, start_pos,end_pos, reverse=True, extra=True, visualize=True)

    start_pos   = WAYPOINT[1]      
    end_pos     = WAYPOINT[2]
    my_path2 = main_hybrid_a(heu, start_pos,end_pos, reverse=True, extra=True, visualize=True)
    
    start_pos   = WAYPOINT[2]      
    end_pos     = WAYPOINT[3]
    my_path3 = main_hybrid_a(heu, start_pos,end_pos, reverse=True, extra=True, visualize=True)
    
    start_pos   = WAYPOINT[3]      
    end_pos     = WAYPOINT[4]
    my_path4 = main_hybrid_a(heu, start_pos,end_pos, reverse=True, extra=True, visualize=True)

    start_pos   = WAYPOINT[4]      
    end_pos     = WAYPOINT[5]
    my_path5 = main_hybrid_a(heu, start_pos,end_pos, reverse=True, extra=True, visualize=True)

    start_pos   = WAYPOINT[5]      
    end_pos     = WAYPOINT[6]
    my_path2 = main_hybrid_a(heu, start_pos,end_pos, reverse=True, extra=True, visualize=True)

