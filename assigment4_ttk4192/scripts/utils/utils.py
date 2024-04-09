from math import cos, sin, atan2, pi, sqrt
from matplotlib import animation
from matplotlib.collections import LineCollection, PatchCollection
from matplotlib.patches import Rectangle
import numpy as np
import matplotlib.pyplot as plt


def transform(x, y, w, l, theta, id):
    """ Coordinates transform. """

    if id == 1:
        x_ = x + w*cos(theta) - l*sin(theta)
        y_ = y + w*sin(theta) + l*cos(theta)
    if id == 2:
        x_ = x + w*cos(theta) + l*sin(theta)
        y_ = y + w*sin(theta) - l*cos(theta)
    if id == 3:
        x_ = x - w*cos(theta) - l*sin(theta)
        y_ = y - w*sin(theta) + l*cos(theta)
    if id == 4:    
        x_ = x - w*cos(theta) + l*sin(theta)
        y_ = y - w*sin(theta) - l*cos(theta)
    
    return np.array([x_, y_])

def WpToCarFrame(wp, l): 
    # Waypoint for car is the backtires.
    wp[0] -= l/2 * np.cos(wp[2])
    wp[1] -= l/2 * np.sin(wp[2]) 
    return wp

def plot_a_car(ax, model):
    """ Plot a car model. """

    pc = PatchCollection(model, match_original=True, zorder=2)
    ax.add_collection(pc)

    return ax


def arc_length(pos1, pos2, r):
    """ Calculate the arc and chord length. """
    
    delta_theta = pos2[2] - pos1[2]

    if delta_theta != 0:
        arc = abs(delta_theta*r)
        chord = abs(2*r*sin(delta_theta/2))
    else:
        arc = distance(pos1[:2], pos2[:2])
        chord = arc
    
    return arc, chord


def directional_theta(vec1, vec2, d):
    """ Calculate the directional theta change. """
    
    theta = atan2(vec2[1], vec2[0]) - atan2(vec1[1], vec1[0])

    if theta < 0 and d == 1:
        theta += 2*pi
    elif theta > 0 and d == -1:
        theta -= 2*pi
    
    return theta


def distance(pt1, pt2):
    """ Distance of two points. """
    
    d = sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

    return d


def same_point(pt1, pt2, h=1e-2):
    """ Check two points are same within a samll error. """

    d = distance(pt1, pt2)

    return d < h


def round_theta(theta, thetas):
    """ Round theta to closest discretized value. """

    return min(thetas, key=lambda x: abs(x-theta) % (2*pi))


def get_discretized_thetas(unit_theta):
    """ Get all discretized theta values by unit value. """

    thetas = [0]

    while True:
        theta = thetas[-1] + unit_theta
        if theta > (2*pi - unit_theta):
            break
        
        thetas.append(theta)
    
    return thetas

def plotstart(env, car):
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(-0.02,env.lx + 0.02)
    ax.set_ylim(-0.02, env.ly + 0.02)
    ax.set_aspect("equal")

    plt.grid(which='both')

    start_state = car.get_car_state(car.start_pos)
    end_state = car.get_car_state(car.end_pos)

    ax = plot_a_car(ax, end_state.model)
    ax = plot_a_car(ax, start_state.model)
    WPS_t = WpToCarFrame(car.start_pos, 0)
    WPE_t = WpToCarFrame(car.end_pos, 0)
    ax.plot(WPS_t[0], WPS_t[1], 'bo', markersize=6, label="start")
    ax.plot(WPE_t[0], WPE_t[1], 'ro', markersize=6, label="end")

    for ob in env.obs:
        ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))
    plt.legend(loc="lower right")
    plt.show()

def animate_solution(car, env, path, xl, yl, carl, cell_size):
    start_state = car.get_car_state(car.start_pos)
    end_state = car.get_car_state(car.end_pos)

    # plot and animation
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(-0.02,env.lx + 0.02)
    ax.set_ylim(-0.02, env.ly + 0.02)
    ax.set_aspect("equal")

    ticksx = np.arange(0, env.lx, cell_size)
    ticksy = np.arange(0, env.ly, cell_size)
    ax.set_xticks(ticksx)
    ax.set_yticks(ticksy)
    labelx = [str(tick) if tick % 0.5 == 0 else "" for tick in ticksx]
    labely = [str(tick) if tick % 0.5 == 0 else "" for tick in ticksy]
    ax.set_xticklabels(labelx)
    ax.set_yticklabels(labely)
    ax.tick_params(length=1)
    plt.grid(which='both')
    
    for ob in env.obs:
        ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))
    
    ax = plot_a_car(ax, end_state.model)
    ax = plot_a_car(ax, start_state.model)
    WPS_t = WpToCarFrame(car.start_pos, 0)
    WPE_t = WpToCarFrame(car.end_pos, 0)
    ax.plot(WPS_t[0], WPS_t[1], 'bo', markersize=6, label="start")
    ax.plot(WPE_t[0], WPE_t[1], 'ro', markersize=6, label="end", zorder=5)
    plt.legend(loc="lower right")
    _carl = PatchCollection(carl[::20], match_original=True, alpha=0.1, zorder=3)
    ax.add_collection(_carl)
    ax.plot(xl, yl, color='whitesmoke', linewidth=2, zorder=3)
    _car = PatchCollection(path[-1].model, match_original=True, zorder=4, alpha=0.6, fc="gray", ec="black")
    ax.add_collection(_car)

    _branches = LineCollection([], linewidth=1)
    ax.add_collection(_branches)

    _path, = ax.plot([], [], color='violet', linewidth=4)
    _carl = PatchCollection([])
    ax.add_collection(_carl)
    _path1, = ax.plot([], [], color='w', linewidth=2)
    _car = PatchCollection([])
    ax.add_collection(_car)
    
    frames = len(path) + 1

    def init():
        _branches.set_paths([])
        _path.set_data([], [])
        _carl.set_paths([])
        _path1.set_data([], [])
        _car.set_paths([])

        return _branches, _path, _carl, _path1, _car

    def animate(i):

        edgecolor = ['k']*7
        facecolor = ['darkolivegreen'] * 7 

        j = i

        _path.set_data(xl[min(j, len(path)-1):], yl[min(j, len(path)-1):])

        sub_carl = carl[:min(j+1, len(path))]
        _carl.set_paths(sub_carl[::4])
        _carl.set_edgecolor('k')
        _carl.set_facecolor('k')
        _carl.set_alpha(0.1)
        _carl.set_zorder(3)

        _path1.set_data(xl[:min(j+1, len(path))], yl[:min(j+1, len(path))])
        _path1.set_zorder(3)

        _car.set_paths(path[min(j, len(path)-1)].model)
        _car.set_edgecolor(edgecolor)
        _car.set_facecolor(facecolor)
        _car.set_alpha(0.6)
        _car.set_zorder(3)

        return _branches, _path, _carl, _path1, _car

    ani = animation.FuncAnimation(fig, animate, init_func=init, frames=frames,
                                interval=1, repeat=True, blit=True)

    plt.show()