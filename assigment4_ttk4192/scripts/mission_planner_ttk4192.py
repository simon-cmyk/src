#!/usr/bin/env python3
import rospy
import os
import tf
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi, sqrt, atan2, tan, sin, cos
from os import system, name
import re
import fileinput
import sys
import argparse
import random
import matplotlib.animation as animation
from datetime import datetime
from matplotlib.collections import PatchCollection, LineCollection
from matplotlib.patches import Rectangle
from itertools import product
from utils.astar import Astar
from utils.utils import plot_a_car, get_discretized_thetas, round_theta, same_point
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import shutil
import copy
import time
from utils.environment import Environment_robplan
from utils.car import SimpleCar
from utils.grid import Grid_robplan
# Import here the packages used in your codes
from hybrid_a_star import HybridAstar, main_hybrid_a
from GNC import PosControl

""" ----------------------------------------------------------------------------------
Mission planner for Autonomos robots: TTK4192,NTNU. 
Date:20.03.23
characteristics: AI planning,GNC, hybrid A*, ROS.
robot: Turtlebot3
version: 1.1
""" 


# 1) Program here your AI planner (only one) ------------------------------------------------------------
"""
1) Temporal planner : Program a routine wich call the installed STP planner; 
2) Graph plan       : Use the graph-plan code provided in the lecture
3) other algorithm 
"""


# 2) Program here your path-finding algorithm (only one) --------------------------------------------------------------------
""" 
1) Hybrid A-star pathfinding : Use the algorithm provided in Assignment 1
2) A-star                    : Program your code
3) Other method
"""


        
#3) GNC module (path-followig and PID controller for the robot) ------------------------------
"""  Robot Guidance navigation and control module 
"""



#4) Program here the turtlebot actions (based in your PDDL domain)
"""
Turtlebot 3 actions-------------------------------------------------------------------------
"""

class TakePhoto:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Allow up to one second to connection
        rospy.sleep(1)

    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    def take_picture(self, img_title):
        if self.image_received:
            # Save an image
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False
        
def taking_photo_exe():
    # Initialize
    camera = TakePhoto()

    # Default value is 'photo.jpg'
    now = datetime.now()
    dt_string = now.strftime("%d%m%Y_%H%M%S")
    img_title = rospy.get_param('~image_title', 'photo'+dt_string+'.jpg')

    if camera.take_picture(img_title):
        rospy.loginfo("Saved image " + img_title)
    else:
        rospy.loginfo("No images received")
	#eog photo.jpg
    # Sleep to give the last log messages time to be sent

	# saving photo in a desired directory
    # file_source = '/home/miguel/catkin_ws/'
    # file_destination = '/home/miguel/catkin_ws/src/assigment4_ttk4192/scripts'
    # g='photo'+dt_string+'.jpg'

    # shutil.move(file_source + g, file_destination)
    rospy.sleep(1)

def move_robot_waypoint0_waypoint1():
    # This function executes Move Robot from 1 to 2
    # This function uses hybrid A-star
    a=0
    while a<3:
        print("Excuting Mr12")
        time.sleep(1)
        a=a+1
    print("Computing hybrid A* path")
	
    p = argparse.ArgumentParser()
    p.add_argument('-heu', type=int, default=1, help='heuristic type')
    p.add_argument('-r', action='store_true', help='allow reverse or not')
    p.add_argument('-e', action='store_true', help='add extra cost or not')
    p.add_argument('-g', action='store_true', help='show grid or not')
    args = p.parse_args()
    start_pos = [2, 2, 0]
    end_pos = [6, 6, 3*pi/4]
    main_hybrid_a(args.heu,start_pos,end_pos,args.r,args.e,args.g)
    print("Executing path following")
    turtlebot_move()


def Manipulate_OpenManipulator_x():
    print("Executing manipulate a weight")
    time.sleep(5)

def check_pump_picture_ir_waypoint0():
    a=0
    while a<3:
        print("Taking IR picture at waypoint0 ...")
        time.sleep(1)
        a=a+1
    time.sleep(5)

def check_seals_valve_picture_eo_waypoint0():
    a=0
    while a<3:
        print("Taking EO picture at waypoint0 ...")
        time.sleep(1)
        a=a+1
    time.sleep(5)

# Charging battery 
def charge_battery_waypoint0():
    print("chargin battert")
    time.sleep(5)

def move_robot(task):
    """
    Created by MNK as a general move_action between input positions given as string literals
    """
    startpos = task.split('_')[2]
    startpos = WPNS[startpos][0:3]
    goalpos = task.split('_')[3]
    goalpos = WPNS[goalpos][0:3]
    print("Excuting move from (%2f,%2f) to (%2f,%2f)" % (startpos[0], startpos[1], goalpos[0], goalpos[1]))

    # Get path from path planner
    print('Computing path using A*')
    # TODO: Add path planner
    heu = 1
    my_path1 = main_hybrid_a(heu, startpos, goalpos, reverse=True, extra=True, visualize=True)
    path = [startpos[0:2],goalpos[0:2]]

    # Move robot using motion controller
    print("Executing path following")
    # turtlebot_move(my_path1)
    PosControl(my_path1)




def take_picture(task):
    # Initialize
    waypoint = task.split('_')[2]
    theta = WPNS[waypoint][2]

    turn = turtle_turn(theta)       # adjust angle before taking photo
    taking_photo_exe()              # take picture

class turtle_turn():
    """
    Created by MNK as a method to orientate the robot before taking an image
    """
    def __init__(self, theta_sp):
        rospy.on_shutdown(self.stop)

        self.theta = 0.0
        self.theta_sp = theta_sp
        self.pid_theta = PID(0,0,0)  # initialization

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback) # subscribing to the odometer (return pos and vel of turtlebot)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)        # sending vehicle speed commands to turtlebot3
        self.vel = Twist()                                                     # vector3 linear, vector3 angular
        self.rate = rospy.Rate(10)                                             # update frequency of velocity commands


        self.turn_robot()
        self.stop()
        rospy.logwarn("Turning done.")

    def turn_robot(self):
        print("Executing Make a turn")
        time.sleep(1)

        # TODO: Turn robot
        self.pid_theta.setPID(1, 0, 0)     # P control while steering
        self.pid_theta.setPoint(self.theta_sp)
        rospy.logwarn("### PID: set target theta = " + str(self.theta_sp) + " ###")

        # Adjust orientation first
        while not rospy.is_shutdown():
            angular = self.pid_theta.update(self.theta)     # return calculated PID input
            if abs(angular) > 0.2:                          # turtlebot has not adjusted angle yet
                angular = angular/abs(angular)*0.2          # fixed input "magnitude" when angle error is large
            if abs(angular) < 0.01:                         # angle is within tolerance
                break
            self.vel.linear.x = 0               
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)                  # publish velocity commands to turtlebot3
            self.rate.sleep()

        self.stop()

    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)

    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.theta = yaw

def EucledianDist():
    """
    MNK - Function created to find eucledian distance between waypoints needed in AI planer
    DELETE to cleanup code when not needed anymore
    """
    for wpn1 in WPNS:
        for wpn2 in WPNS:
            if wpn1 != wpn2:

                dist = sqrt((WPNS[wpn1][0] - WPNS[wpn2][0])**2 + (WPNS[wpn1][1] - WPNS[wpn2][1])**2)
                print("Distance " + wpn1 + " to " + wpn2 + ": " + str(dist))

def move_sine():
    """
    Created by MNK to test creating a trajectory and making TB3 follow it
    """
    print("Initiated move sine trajectory")
    # Create trajectory
    curr_pos = [0.0,0.0]
    x,y = [], []
    for k in range(1000):
        x.append(curr_pos[0] + sin(k/100))
        y.append(curr_pos[1] + k/100)

    xl = np.array(x)
    yl = np.array(y)
    path = np.column_stack([xl,yl])

    # Move turtlebot
    turtlebot_move(path)

def odom_callback(msg):
    # Get (x, y, theta) specification from odometry topic
    quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

    robot_pos_theta = yaw
    robot_pos_x = msg.pose.pose.position.x
    robot_pos_y = msg.pose.pose.position.y

global robot_pos_x
global robot_pos_y
global robot_pos_theta


# Define list of global waypoints
global WPNS
WPNS = {'waypoint0': [0.40, 0.40, 0.0],
       'waypoint1':  [1.85, 0.35, 0.0],
       'waypoint2':  [3.00, 1.05, 0.0],
       'waypoint3':  [3.15, 2.60, 0.0],
       'waypoint4':  [4.70, 0.50, 0.0],
       'waypoint5':  [0.95, 2.50, pi],
       'waypoint6':  [3.60, 1.70, pi/2],
       'waypoint7':  [0.40, 0.40, 0.0],
       'waypoint8':  [0.75, 0.40, 0.0],
       'waypoint9':  [0.75, 1.25, 0.0],
       'waypoint10': [2.90, 1.25, 0.0]}

# 'waypoint0': [0.30, 0.30, 0.0],
# 'waypoint1':  [1.80, 0.45, pi/2],
# 'waypoint2':  [2.95, 0.95, 3*pi/2],



# Define the global varible: WAYPOINTS  Wpts=[[x_i,y_i]];
global WAYPOINTS
# WAYPOINTS = [
#                 [0.30, 0.30],
#                 [1.80, 0.45],
#                 [2.95, 0.95],
#                 [3.15, 2.60],
#                 [4.70, 0.50],
#                 [0.95, 2.50],
#                 [3.60, 1.70]
#             ]


# 5) Program here the main commands of your mission planning algorithm for turtlebot ---------------------------------
""" Main code 
"""
if __name__ == '__main__':
    try:
        print()
        print("************ TTK4192 - Assigment 4 **************************")
        print()
        print("AI planners: STP/GraphPlan/Other")
        print("Path-finding: Hybrid A-star/A*/other")
        print("GNC Controller: PID path-following")
        print("Robot: Turtlebot3 waffle-pi")
        print("date: 19.03.24")
        print()
        print("**************************************************************")
        print()
        print("Press Intro to start ...")
        input_t=input("")

        rospy.init_node('turtlebot_move', anonymous=False)
        odom_sub = rospy.Subscriber("odom", Odometry, odom_callback)



	# 5.1) Starting the AI Planner
       #Here you must run your AI planner module

        # 5.2) Reading the plan 
        # print("  ")
        # print("Reading the plan from AI planner")
        # print("  ")
        # plan_general=plan_general
        # print(plan_general[0])

        # 5.3) Start mission execution 
        # convert string into functions and executing
        # print("")
        # print("Starting mission execution")
        # Start simulations with battery = 100%
        # battery=100
        # task_finished=0
        # task_total=len(plan_general)
        # i_ini=0

        rospy.sleep(1)

        # PosControl([[1.4, 0.4],[0.8, 0.4]])

        task_total = ["move_robot_waypoint0_waypoint2",
                      "take_picture_waypoint2",
                      "move_robot_waypoint2_waypoint0"]

        for task in task_total:
            if 'move_robot' in task:
                move_robot(task)

            elif 'take_picture' in task:
                take_picture(task)
                rospy.sleep(3)



        print("")
        print("--------------------------------------")
        print("All tasks were performed successfully")
        time.sleep(5)  

    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
