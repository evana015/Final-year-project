#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion


class Drone:
    def __init__(self, name, set_rate, publishers, subscribers, x, y, z):
        self.name = name
        self.rate = rospy.rate(set_rate)
        self.publishers = [].append(publishers)
        self.subscribers = [].append(subscribers)
        self.x = x
        self.y = y
        self.z = z
        self.theta = 0
        self.roll = 0
        self.pitch = 0

    def initialize(self):
        return None

    def publish(self):
        return None

    def subscribe(self):
        return None

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