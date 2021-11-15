#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty  # importing ros and empty msgs to publish takeoff command
from geometry_msgs.msg import Twist

def takeoff():
    pub = rospy.Publisher("drone/takeoff", Empty, queue_size=10)  # node is publishing to the topic "takeoff" using
    # empty type
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():  # Checks for a Ctrl-C or any other termination
        pub.publish(Empty())  # Publishes Empty "{}" to the takeoff rostopic
        rate.sleep()  # sleeps just long enough to maintain  the desired rate to loop through


def move_straight():
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    move_cmd = Twist()
    move_cmd.linear.x = 1.0

    now = rospy.Time.now()
    rate = rospy.Rate(10)

    while rospy.Time.now() < now + rospy.Duration.from_sec(6):
        pub.publish(move_cmd)
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('drone',
                        anonymous=True)  # initiates the node and gives it a name to communicate with the ROS master
        takeoff()
        move_straight()
    except rospy.ROSInterruptException:
        pass
