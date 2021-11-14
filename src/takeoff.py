#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty  # importing ros and empty msgs to publish takeoff command


def takeoff():
    pub = rospy.Publisher("drone/takeoff", Empty, queue_size=10)  # node is publishing to the topic "takeoff" using empty type
    rospy.init_node('drone',
                    anonymous=True)  # initiates the node and gives it a name to communicate with the ROS master
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():  # Checks for a Ctrl-C or any other termination
        pub.publish(Empty())    # Publishes Empty "{}" to the takeoff rostopic
        rate.sleep()    # sleeps just long enough to maintain  the desired rate to loop through


if __name__ == '__main__':
    try:
        takeoff()
    except rospy.ROSInterruptException:
        pass
