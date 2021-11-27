#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose


x = 0.0
y = 0.0
theta = 0.0


def newOdom(msg):
    global x
    global y
    global theta
    x = msg.position.x
    y = msg.position.y

    rot_q = msg.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


if __name__ == '__main__':
    rospy.init_node("speed_controller")

    sub = rospy.Subscriber("/drone/gt_pose", Pose, newOdom)
    rospy.sleep(5)
    print(x)
    print(y)
    rospy.spin()
