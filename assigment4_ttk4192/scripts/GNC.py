import rospy
import numpy as np
import tf
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi, sqrt, atan2, tan, sin, cos

def ssa(angle):
    """
    angle = ssa(angle) returns the smallest-signed angle in [ -pi, pi )
    """
    angle = (angle + np.pi) % (2 * np.pi) - np.pi
        
    return angle 

class PosControl():
    """
    Path-following module
    """
    def __init__(self,path):
        rospy.on_shutdown(self.stop)

        self.path = path

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback) # subscribing to the odometer (return pos and vel of turtlebot)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)        # sending vehicle speed commands to turtlebot3
        self.vel = Twist()                                                     # vector3 linear, vector3 angular
        self.rate = rospy.Rate(10)                                             # update frequency of velocity commands
        self.counter = 0                                                       # store trajectory point only at specific counter values
        self.trajectory = list()                                               # store history (trajectory driven by turtlebot3)
        rospy.sleep(1)

        self.kp = 0.1
        self.CONSTANT_ANGULAR_SPEED = 5
        self.CONSTANT_LINEAR_SPEED = 10
        self.CONSTANT_REVERSE_SPEED = 2.5

        self.MAX_LINEAR_SPEED = 0.2
        self.MIN_LINEAR_SPEED = 0.01
        self.MAX_ORIENTATION_SPEED = 0.2
        self.DISTANCE_THRESHOLD = 0.05
        self.ORIENTATION_THRESHOLD_LOW = 0.07
        self.ORIENTATION_THRESHOLD_HIGH = 0.3
        self.orientation_threshold = self.ORIENTATION_THRESHOLD_LOW

        # track a sequence of waypoints
        # for point in WAYPOINTS:                                                # global list of wpns (xi,yi)
        for point in self.path:
            self.move_to_point(point)
            # rospy.sleep(1)    # MNK 7.4.23 COMMENTED OUT
        self.stop()
        rospy.logwarn("Action done.")

        # plot trajectory
        data = np.array(self.trajectory)                                       # visualize driven trajectory
        np.savetxt('trajectory.csv', data, fmt='%f', delimiter=',')
        plt.plot(data[:,0],data[:,1])
        plt.show()

    def move_to_point(self, target_pos):

        self.destination = False
        self.dir = 1
        while (not self.destination):
            errorX = target_pos[0] - self.x
            errorY = target_pos[1] - self.y
            target_distance = sqrt(errorX**2 + errorY**2)
            target_theta = atan2(errorY, errorX)
            errorTheta = ssa(target_theta - self.theta)

            robot_direction = np.array([cos(self.theta), sin(self.theta)])
            target_direction = np.array([cos(target_theta), sin(target_theta)])

            if robot_direction @ target_direction < 0:
                target_theta = ssa(target_theta + np.pi)
                errorTheta = ssa(target_theta - self.theta) 
                target_direction = np.array([cos(target_theta), sin(target_theta)])
                self.dir = -1

            # angle error is to big (only adjust orientation until within threshold)
            if abs(errorTheta) > self.orientation_threshold and target_distance > self.DISTANCE_THRESHOLD:  # angle error is to big
                self.orientation_threshold = self.ORIENTATION_THRESHOLD_LOW

                # adjust orientation
                if target_theta > self.theta:
                    self.vel.angular.z = self.kp*abs(errorTheta)*self.CONSTANT_ANGULAR_SPEED
                else:
                    self.vel.angular.z = -self.kp*abs(errorTheta)*self.CONSTANT_ANGULAR_SPEED
                self.vel.linear.x = 0.0
                
            # move towards target position adjusting speed and orientation
            else:
                self.orientation_threshold = self.ORIENTATION_THRESHOLD_HIGH

                # adjust orientation
                if target_theta > self.theta:
                    self.vel.angular.z = 2*self.kp*abs(errorTheta)*self.CONSTANT_ANGULAR_SPEED
                else:
                    self.vel.angular.z = -2*self.kp*abs(errorTheta)*self.CONSTANT_ANGULAR_SPEED

                # adjust speed
                if target_distance > self.DISTANCE_THRESHOLD:
                    self.vel.linear.x = max(min(0.7*self.vel.linear.x+self.CONSTANT_LINEAR_SPEED*self.kp*target_distance, self.MAX_LINEAR_SPEED), self.MIN_LINEAR_SPEED)
                    self.vel.linear.x *= self.dir
                else:
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 0.0
                    self.destination = True


            if self.vel.angular.z > self.MAX_ORIENTATION_SPEED:                                   # limit orientation speed
                self.vel.angular.z = self.vel.angular.z/abs(self.vel.angular.z)*self.MAX_ORIENTATION_SPEED

            if self.vel.linear.x > self.MAX_LINEAR_SPEED:                                        # limit linear speed
                self.vel.linear.x = self.vel.linear.x/abs(self.vel.linear.x)*self.MAX_LINEAR_SPEED
                
            if self.dir == -1:
                self.vel.linear.x = self.vel.linear.x*self.CONSTANT_REVERSE_SPEED

            self.vel_pub.publish(self.vel)
            self.rate.sleep()

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
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Make messages saved and prompted in 5Hz rather than 100Hz
        self.counter += 1
        if self.counter == 20:
            self.counter = 0
            self.trajectory.append([self.x,self.y])

            
class PID:
    """
    Discrete PID control
    """
    def __init__(self, P=0.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=10, Integrator_min=-10):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        PI = 3.1415926535897
        self.error = self.set_point - current_value
        if self.error > pi:  # specific design for circular situation
            self.error = self.error - 2*pi
        elif self.error < -pi:
            self.error = self.error + 2*pi
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator = self.Integrator + self.error
        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min
        self.I_value = self.Integrator * self.Ki
        PID = self.P_value + self.I_value + self.D_value
        return PID

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
        self.Integrator = 0

    def setPID(self, set_P=0.0, set_I=0.0, set_D=0.0):
        self.Kp = set_P
        self.Ki = set_I
        self.Kd = set_D