#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16


def talker():
    # create publishers for the 2 topics needed for the task
    steeringPublisher = rospy.Publisher('/manual_control/steering', Int16, queue_size=10)
    speedPublisher = rospy.Publisher('/manual_control/speed', Int16, queue_size=10)
    # init an anonymous node called squarecontroller (anonymous = track every instance individually, appends hash)
    rospy.init_node('squarecontroller', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # if it was a gas instead of speed topic, car would only accelerate on straights / while exiting a corner

    # we decided for some reason it makes more sense to first go straight and then corner, in case we were only
    # accelerating on straights; This is not the case so in order for the car to stop at the point where it beginned,
    # corner first would make more sense; In this case we would not needed the final for loop to get the car
    # straight as well but... yeah...

    for i in range(4):  # 4 times do the following control
        rospy.loginfo("Steer straight")
        for y in range(11):  # 10hz rate, so for about 1.1 seconds
            steeringPublisher.publish(80) # we found out experimentally straight is 80 deg angle for a reason
            if (i == 0):
                speedPublisher.publish(-700)
            rate.sleep()

        rospy.loginfo("Steer right")
        for y in range(4): # 10hz rate, for about 0.7 second
            steeringPublisher.publish(30)
            rate.sleep()

    # we need this, because wheels don't move instantly, also after steering was set to 80, car will still corner a little

    for i in range(5): # for another half a second, get the car straight
        steeringPublisher.publish(80)
        rate.sleep()

    # we only need to publish it once, but we don't want to kill the node (finish it)
    # straight away anyways
    rospy.loginfo("Stop")
    while not rospy.is_shutdown():
        speedPublisher.publish(0)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
