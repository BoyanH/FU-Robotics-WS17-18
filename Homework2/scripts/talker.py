#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16


def talker():
    steeringPublisher = rospy.Publisher('/manual_control/steering', Int16, queue_size=10)
    speedPublisher = rospy.Publisher('/manual_control/speed', Int16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    for i in range(4):  # 4 times do the following control
        rospy.loginfo("Steer straight")
        for y in range(11):  # 10hz rate, so for about 1.1 seconds
            steeringPublisher.publish(80)
            if (i == 0):
                speedPublisher.publish(-700)
            rate.sleep()

        rospy.loginfo("Steer right")
        for y in range(4): # 10hz rate, for about 0.7 second
            steeringPublisher.publish(30)
            rate.sleep()

    for i in range(5): # for another half a second, get the car straight
        steeringPublisher.publish(80)
        rate.sleep()

    rospy.loginfo("Stop")
    while not rospy.is_shutdown():
        speedPublisher.publish(0)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
