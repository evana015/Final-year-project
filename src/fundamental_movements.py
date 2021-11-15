#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty  # importing ros and empty msgs to publish takeoff command
from geometry_msgs.msg import Twist


def takeoff():
    pub = rospy.Publisher("drone/takeoff", Empty, queue_size=1)  # node is publishing to the topic "takeoff" using
    # empty type
    rate = rospy.Rate(10)  # 10hz
    ctrl_c = False
    while not ctrl_c:
        connections = pub.get_num_connections()
        if connections > 0:
            pub.publish((Empty()))  # Publishes Empty "{}" to the takeoff rostopic
            ctrl_c = True
        else:
            rate.sleep()  # sleeps just long enough to maintain  the desired rate to loop through


def move_straight():
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=2)

    move_cmd = Twist()
    move_cmd.linear.x = 1.0
    stop_moving_cmd = Twist()
    stop_moving_cmd.linear.x = 0.0
    now = rospy.Time.now()
    rate = rospy.Rate(10)

    while rospy.Time.now() < now + rospy.Duration.from_sec(6):
        pub.publish(move_cmd)
        rate.sleep()
    ctrl_c = False
    while not ctrl_c:
        connections = pub.get_num_connections()
        if connections > 0:
            pub.publish(stop_moving_cmd)
            ctrl_c = True
        else:
            rate.sleep()


def land():
    pub = rospy.Publisher("drone/land", Empty, queue_size=1)
    rate = rospy.Rate(10)
    ctrl_c = False
    while not ctrl_c:
        connections = pub.get_num_connections()
        if connections > 0:
            pub.publish((Empty()))
            ctrl_c = True
        else:
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('drone',
                        anonymous=True)  # initiates the node and gives it a name to communicate with the ROS master
        takeoff()
        move_straight()
        land()
    except rospy.ROSInterruptException:
        pass
