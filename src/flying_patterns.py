#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty
from geometry_msgs.msg import Point, Twist, Pose
from math import atan2


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


def move_to(goal_x, goal_y, margin):
    speed = Twist()

    r = rospy.Rate(4)

    goal = Point()
    goal.x = goal_x
    goal.y = goal_y

    hovering = False

    while not hovering:
        inc_x = goal.x - x
        inc_y = goal.y - y

        angle_to_goal = atan2(inc_y, inc_x)

        if (goal.x - margin) < x < (goal.x + margin) and \
                (goal.y - margin) < y < (goal.y + margin):
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            hovering = True
        elif abs(angle_to_goal - theta) > 0.05:
            if (angle_to_goal - theta) > 0:
                speed.linear.x = 0.0
                speed.angular.z = 0.3
            else:
                speed.linear.x = 0.0
                speed.angular.z = -0.3
        else:
            if abs(inc_x) < 0.5 and abs(inc_y) < 0.5:
                speed.linear.x = 0.1
                speed.angular.z = 0.0
            else:
                speed.linear.x = 0.5
                speed.angular.z = 0.0

        pub.publish(speed)
        r.sleep()




if __name__ == '__main__':
    rospy.init_node("speed_controller")

    # # TODO: learn how to import this
    # pub = rospy.Publisher("drone/takeoff", Empty, queue_size=1)  # node is publishing to the topic "takeoff" using
    # # empty type
    # rate = rospy.Rate(10)  # 10hz
    # ctrl_c = False
    # while not ctrl_c:
    #     connections = pub.get_num_connections()
    #     if connections > 0:
    #         pub.publish((Empty()))  # Publishes Empty "{}" to the takeoff rostopic
    #         ctrl_c = True
    #     else:
    #         rate.sleep()  # sleeps just long enough to maintain  the desired rate to loop through

    sub = rospy.Subscriber("/drone/gt_pose", Pose, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    move_to(5, 5, 0.05)
