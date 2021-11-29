#!/usr/bin/env python
import math

import rospy
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty
from geometry_msgs.msg import Point, Twist, Pose
from math import atan2, sin, cos, radians


def newOdom(msg):
    global x
    global y
    global theta
    x = msg.position.x
    y = msg.position.y

    rot_q = msg.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def turn(goal_x, goal_y, speed):
    inc_x = goal_x - x
    inc_y = goal_y - y
    if abs(inc_x) < 0.5 and abs(inc_y) < 0.5:  # edge case the drone misses the waypoint and cant turn around
        speed.linear.x = 0.0
        speed.angular.z = -0.2
    elif goal_x-x >= 0 and goal_y-y >= 0:  # x y
        if 1.5 < theta < 3:
            speed.linear.x = 0.0
            speed.angular.z = -0.2
        elif -1.5 < theta < 0:
            speed.linear.x = 0.0
            speed.angular.z = 0.2
        elif -3 < theta < -1.5:
            speed.linear.x = 0.0
            speed.angular.z = -0.2
    elif goal_x-x >= 0 and goal_y-y < 0:  # x -y
        if 0 < theta < 1.5:
            speed.linear.x = 0.0
            speed.angular.z = -0.2
        elif -3 < theta < -1.5:
            speed.linear.x = 0.0
            speed.angular.z = 0.2
        elif 1.5 <= theta < 3:
            speed.linear.x = 0.0
            speed.angular.z = -0.2
    elif goal_x-x < 0 and goal_y-y >= 0:  # -x y
        if -1.5 < theta < -3:
            speed.linear.x = 0.0
            speed.angular.z = -0.2
        elif 0 < theta < 1.5:
            speed.linear.x = 0.0
            speed.angular.z = 0.2
        elif -1.5 < theta < 0:
            speed.linear.x = 0.0
            speed.angular.z = -0.2
    else:  # -x -y
        if -1.5 < theta < 0:
            speed.linear.x = 0.0
            speed.angular.z = -0.2
        elif 1.5 < theta < 3:
            speed.linear.x = 0.0
            speed.angular.z = 0.2
        elif 0 < theta < 1.5:
            speed.linear.x = 0.0
            speed.angular.z = -0.2
    if speed.angular.z == 0.0:
        speed.linear.x = 0.0
        speed.angular.z = -0.2
    return speed


def move_to(goal_x, goal_y, margin):
    print("moving to waypoint(", goal_x, ",", goal_y, ")")
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
            rate.sleep()
            hovering = True
        elif abs(angle_to_goal - theta) > 0.1:
            speed = turn(goal_x, goal_y, speed)
        else:
            if abs(inc_x) < 0.5 and abs(inc_y) < 0.5:
                speed.linear.x = 0.2
                speed.angular.z = 0.0
            else:
                speed.linear.x = 0.5
                speed.angular.z = 0.0

        pub.publish(speed)
        r.sleep()


def pts(boundary_x, boundary_y):
    original_x = x
    original_y = y
    waypoint_x = x
    waypoint_y = original_y + boundary_y
    while waypoint_x < original_x + boundary_x:
        move_to(waypoint_x, waypoint_y, 0.1)  # using half a unit as a base margin of error
        waypoint_x += 1
        move_to(waypoint_x, waypoint_y, 0.1)  # move across by one unit to make a parallel movement next time
        if waypoint_y != original_y:
            waypoint_y = original_y
        else:
            waypoint_y = boundary_y


def ess(boundary_x, boundary_y):  # generate a sequence of waypoints that will be followed to conduct a ess
    original_x = x
    original_y = y
    waypoint_x = x
    waypoint_y = y + 1
    increment = 1
    hit_limit = False
    while not hit_limit:
        move_to(waypoint_x, waypoint_y, 0.1)  # 0 1 # 1 -1
        waypoint_x = waypoint_y
        move_to(waypoint_x, waypoint_y, 0.1)  # 1 1 # -1 -1
        increment += 1
        if waypoint_y > original_y:
            waypoint_y = waypoint_y - increment
        else:
            waypoint_y = waypoint_y + increment
        if waypoint_y > original_y + boundary_y or waypoint_x > original_x + boundary_x:
            hit_limit = True


def ss(radius):
    radians_needed = [0, radians(60), radians(120), radians(180), radians(240), radians(300)]
    original_x = x
    original_y = y
    n = 0
    for i in range(0, 3):  # 3 triangle movements to perform
        move_to(original_x + (radius * sin(radians_needed[n])),
                original_y + (radius * cos(radians_needed[n])), 0.1)
        move_to(original_x + (radius * sin(radians_needed[n + 1])),
                original_y + (radius * cos(radians_needed[n + 1])), 0.1)
        move_to(original_x, original_y, 0.1)
        n += 2


if __name__ == '__main__':
    x = 0.0
    y = 0.0
    theta = 0.0

    rospy.init_node("speed_controller")

    # TODO: learn how to import this
    pub = rospy.Publisher("drone/takeoff", Empty, queue_size=1)
    rate = rospy.Rate(10)
    ctrl_c = False
    while not ctrl_c:
        connections = pub.get_num_connections()
        if connections > 0:
            pub.publish((Empty()))
            ctrl_c = True
        else:
            rate.sleep()

    sub = rospy.Subscriber("/drone/gt_pose", Pose, newOdom)
    rate.sleep()  # sleep needed as previously it was reading as 0 0 as a first reading
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    ss(2)
