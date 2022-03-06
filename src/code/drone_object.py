#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion


class Drone:
    # Initializer for the drone:
    def __init__(self, name, set_rate):
        # Initiate the drone via rospy with the node having the given name
        self.name = rospy.init_node(self.name)
        # Setting the rate to what is dictated by the second parameter
        self.rate = rospy.rate(set_rate)
        # Setting default location data
        self.x = 0
        self.y = 0
        self.z = 0
        self.theta = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        # Publishers and Subscribers will be set with the following set as standard
        self.odom_sub = rospy.Subscriber("/drone/gt_pose", Pose, self.odom)
        self.movement_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def odom(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z

        rot_q = msg.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_z(self):
        return self.z

    def get_roll(self):
        return self.roll

    def get_pitch(self):
        return self.pitch

    def get_theta(self):
        return self.theta
