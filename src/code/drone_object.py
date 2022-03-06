#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, Twist, Point
from tf.transformations import euler_from_quaternion
from math import atan2, sin, cos, radians


class Drone:
    # Initializer for the drone:
    def __init__(self, name, set_rate):
        # Initiate the drone via rospy with the node having the given name
        self.name = rospy.init_node(name)
        # Setting the rate to what is dictated by the second parameter
        self.rate = rospy.Rate(set_rate)
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
        self.takeoff_pub = rospy.Publisher("drone/takeoff", Empty, queue_size=1)
        self.land_pub = rospy.Publisher("drone/land", Empty, queue_size=1)

    # odom is the method that is assigned to be callback for the drone's subscriber to the "/drone/gt_pose" topic
    # It takes a geometry_msgs.msg.Pose and breaks it down into the necessary components to update the drones location
    # data. This allows the drone to be self aware of its position within the gazebo simulation
    def odom(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z

        rot_q = msg.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    # takeoff_or_land behave as you would expect from the name
    # Specifically the drone will publish a message to the topic "drone/takeoff" or "drone/land" with an Empty msg
    # The method will take a "choice" parameter which will determine which publisher to use

    def takeoff_or_land(self, choice):
        ctrl_c = False
        if choice == "takeoff":
            choice = self.takeoff_pub
        else:
            choice = self.land_pub
        while not ctrl_c:
            connections = choice.get_num_connections()
            if connections > 0:
                self.choice.publish((Empty()))  # Publishes Empty "{}" to the takeoff rostopic
                ctrl_c = True
            else:
                self.rate.sleep()  # sleeps just long enough to maintain the desired rate to loop through

    def turn(self, goal_x, goal_y, speed):
        inc_x = goal_x - self.x
        inc_y = goal_y - self.y
        if abs(inc_x) < 0.5 and abs(inc_y) < 0.5:  # edge case the drone misses the waypoint and cant turn around
            speed.linear.x = 0.0
            speed.angular.z = -0.2
        elif goal_x - self.x >= 0 and goal_y - self.y >= 0:  # x y
            if 1.5 < self.theta < 3:
                speed.linear.x = 0.0
                speed.angular.z = -0.2
            elif -1.5 < self.theta < 0:
                speed.linear.x = 0.0
                speed.angular.z = 0.2
            elif -3 < self.theta < -1.5:
                speed.linear.x = 0.0
                speed.angular.z = 0.2
        elif goal_x - self.x >= 0 and goal_y - self.y < 0:  # x -y
            if 0 < self.theta < 1.5:
                speed.linear.x = 0.0
                speed.angular.z = -0.2
            elif -3 < self.theta < -1.5:
                speed.linear.x = 0.0
                speed.angular.z = 0.2
            elif 1.5 <= self.theta < 3:
                speed.linear.x = 0.0
                speed.angular.z = -0.2
        elif goal_x - self.x < 0 and goal_y - self.y >= 0:  # -x y
            if -1.5 < self.theta < -3:
                speed.linear.x = 0.0
                speed.angular.z = -0.2
            elif 0 < self.theta < 1.5:
                speed.linear.x = 0.0
                speed.angular.z = 0.2
            elif -1.5 < self.theta < 0:
                speed.linear.x = 0.0
                speed.angular.z = -0.2
        else:  # -x -y
            if -1.5 < self.theta < 0:
                speed.linear.x = 0.0
                speed.angular.z = -0.2
            elif 1.5 < self.theta < 3:
                speed.linear.x = 0.0
                speed.angular.z = 0.2
            elif 0 < self.theta < 1.5:
                speed.linear.x = 0.0
                speed.angular.z = -0.2
        if speed.angular.z == 0.0:
            speed.linear.x = 0.0
            speed.angular.z = -0.2
        return speed

    def move_to(self, goal_x, goal_y, margin):
        print("moving to waypoint(", goal_x, ",", goal_y, ")")
        speed = Twist()

        r = rospy.Rate(4)

        goal = Point()
        goal.x = goal_x
        goal.y = goal_y

        hovering = False
        while not hovering:
            inc_x = goal.x - self.x
            inc_y = goal.y - self.y

            angle_to_goal = atan2(inc_y, inc_x)
            if (goal.x - margin) < self.x < (goal.x + margin) and \
                    (goal.y - margin) < self.y < (goal.y + margin):
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                self.rate.sleep()
                hovering = True
            elif abs(angle_to_goal - self.theta) > 0.1:
                speed = self.turn(goal_x, goal_y, speed)
            else:
                if abs(inc_x) < 0.5 and abs(inc_y) < 0.5:
                    speed.linear.x = 0.2
                    speed.angular.z = 0.0
                else:
                    speed.linear.x = 0.5
                    speed.angular.z = 0.0

            self.movement_pub.publish(speed)
            r.sleep()


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


test_drone = Drone("Parrot", 10)
test_drone.takeoff_or_land("takeoff")
test_drone.move_to(4, 4, 0.5)
