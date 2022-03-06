#!/usr/bin/env python
import rospy


class Drone:
    def __init__(self, name, set_rate, publishers, subscribers, x, y, z):
        self.name = name
        self.rate = rospy.rate(set_rate)
        self.publishers = [].append(publishers)
        self.subscribers = [].append(subscribers)
        self.x = x
        self.y = y
        self.z = z
