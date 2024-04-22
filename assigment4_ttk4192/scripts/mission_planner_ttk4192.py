#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry
from math import pi
import matplotlib.animation as animation
from datetime import datetime
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import subprocess
import time
from hybrid_a_star import main_hybrid_a
from GNC import PosControl, turtle_turn
import actionlib
import control_msgs.msg
import trajectory_msgs.msg
from astar_planner import Graph, a_star, heuristic_euclidean, mirrior_plan

""" ----------------------------------------------------------------------------------
Mission planner for Autonomos robots: TTK4192, NTNU. 
Date: 26.04.24
characteristics: AI planning,GNC, hybrid A*, ROS.
robot: Turtlebot3
version: 1.1
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
        
def taking_photo(location):
    # Initialize
    camera = TakePhoto()

    # Default value is 'photo.jpg'
    now = datetime.now()
    dt_string = now.strftime("%d%m%Y_%H%M%S")
    img_title = rospy.get_param('~image_title', str(location)+'-'+dt_string+'.jpg')

    if camera.take_picture(img_title):
        rospy.loginfo("Saved image " + img_title)
    else:
        rospy.loginfo("No images received")

    rospy.sleep(1)

def manipulate_action(joints_pos, execution_time_secs=1):

    waypoint = task.split(' ')[4]
    theta = WPN_ORIENTATION[waypoint]

    rospy.logwarn("Executing manipulate an object at waypoint %s" % (waypoint))

    turtle_turn(theta)              # adjust angle before manipulating

    move_gripper(gripper_manipulate_pose)
    rospy.sleep(2)
    move_gripper(gripper_home_pose)
    
    turtle_turn(WPNS[waypoint][2])     

    rospy.logwarn("Action manipulate done.")

def move_gripper(joints_pos, execution_time_secs=1):

    goal_point = trajectory_msgs.msg.JointTrajectoryPoint(positions=joints_pos)
    goal_point.time_from_start.secs = execution_time_secs

    trajectory = trajectory_msgs.msg.JointTrajectory(joint_names=['joint1', 'joint2', 'joint3', 'joint4'], points=[goal_point])

    goal = control_msgs.msg.FollowJointTrajectoryGoal(trajectory=trajectory)

    action_client.wait_for_server()

    action_client.send_goal(goal)

    action_client.wait_for_result()

    return action_client.get_result()

def move_robot_action(task):
    """
    Inputs
        task:   move task including initial and gial waypoint

    Function will compute a path between waypoints using a path planner.
    Hybrid A* is default planner. Fall back planner A* is used in case primary planner fails
    """
    startpos = task.split(' ')[4]
    startpos = WPNS[startpos][0:3]
    goalpos = task.split(' ')[5]
    goalpos = WPNS[goalpos][0:3]
    rospy.logwarn("Excuting move from (%2f,%2f) to (%2f,%2f)" % (startpos[0], startpos[1], goalpos[0], goalpos[1]))

    # Get path from path planner
    try:    # Hybrid A* planner
        rospy.loginfo('Computing path using Hybrid A*')
        heu = 1
        path = main_hybrid_a(heu, startpos, goalpos, reverse=True, extra=True, visualize=True)
    except: # A* planner
        rospy.logwarn('Hybrid A* failed computing a path')
        rospy.loginfo('Computing path using A*')
        start_node = [int(289-100*startpos[1]), int(100*startpos[0])]
        goal_node = [int(289-100*goalpos[1]), int(100*goalpos[0])]
        path, visited = a_star(graph,start_node,goal_node, heuristic_function=heuristic_euclidean)   

        path = mirrior_plan(path, map_height, map_scale)

    # Move robot using motion controller
    rospy.loginfo("Executing path following \n")
    PosControl(path)
    
    rospy.logwarn("Action move done. \n")
    # turtle_turn(goalpos[2])  


def take_picture_action(task):
    """
    """
    # Initialize
    waypoint = task.split(' ')[4]
    theta = WPN_ORIENTATION[waypoint]

    rospy.logwarn("Executing take picture at waypoint %s" % (waypoint))

    turtle_turn(theta)              # adjust angle before taking photo
    taking_photo(waypoint)          # take picture
    turtle_turn(WPNS[waypoint][2])  





def odom_callback(msg):
    # Get (x, y, theta) specification from odometry topic
    quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

    robot_pos_theta = yaw
    robot_pos_x = msg.pose.pose.position.x
    robot_pos_y = msg.pose.pose.position.y


# global Robot pose updated in subscriber and accessible everywhere
global robot_pos_x          
global robot_pos_y
global robot_pos_theta

gripper_home_pose = [-0.0, -1.0, 0.3, 0.7]
gripper_manipulate_pose = [0.0, 0.0, 0.0, 0.0]

map_height = 2.89
map_scale = 0.01 

# Define list of global waypoints
global WPNS
WPNS = {'waypoint0': [0.40, 0.40, 0.0],
       'waypoint1':  [1.85, 0.30, 0.0],
       'waypoint2':  [3.00, 1.10, 0.0],
       'waypoint3':  [3.75, 2.60, pi],
       'waypoint4':  [4.55, 0.75, 0.0],
       'waypoint5':  [1.35, 2.60, pi],
       'waypoint6':  [3.60, 1.50, 0]
        }

global WPN_ORIENTATION
WPN_ORIENTATION = {'waypoint0': 0.0,
                   'waypoint1': pi/4,
                   'waypoint2': -3*pi/4,
                   'waypoint3': 0.0,
                   'waypoint4': 0.0,
                   'waypoint5': pi,
                   'waypoint6': pi/2}





# 5) Program here the main commands of your mission planning algorithm for turtlebot ---------------------------------
""" Main code 
"""
if __name__ == '__main__':
    try:
        print()
        print("************ TTK4192 - Assigment 4 **************************")
        print()
        print("AI planner: STP")
        print("Path-finding: Hybrid A-star/A*")
        print("GNC Controller: PID path-following")
        print("Robot: Turtlebot3 waffle-pi")
        print("date: 26.04.24")
        print()
        print("**************************************************************")
        print()
        print("Press Intro to start ...")
        _ =input("")

        rospy.init_node('turtlebot_move', anonymous=False)
        odom_sub = rospy.Subscriber("odom", Odometry, odom_callback)
        action_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)

        rospy.sleep(1)
        move_gripper(gripper_home_pose)

        graph = Graph('maps/map_ttk4192CA4.png')
    
        script_path = '/home/ntnu-itk/catkin_ws/src/AI-planning/run-planner/run_planner.sh'
        subprocess.run(['bash', script_path])


        with open('plan/tmp_sas_plan.1', 'r') as file:
            task_total = [task.strip() for task in file]

    
        rospy.loginfo('Done with calculating plan \n')
        # print(task_total)
        
        for task in task_total:
            if 'move_robot' in task:
                move_robot_action(task)

            elif 'check' in task:
                take_picture_action(task)
                rospy.sleep(3)

            elif 'manipulate' in task:
                manipulate_action(task)

        print("")
        print("--------------------------------------")
        print("All tasks were performed successfully")
        time.sleep(5)  

    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
