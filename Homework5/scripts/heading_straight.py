#!/usr/bin/env python

import rospy
import math
import numpy as np
from std_msgs.msg import Float32, Int16
# import matplotlib.pyplot as plt

experiment_started = False
squared_errors = []
times = []
Kp = 0.6


def get_calibrated_steering(angle):
    return 80


def yaw_callback(yaw_msg):
    global squared_errors

    rospy.loginfo(yaw_msg)
    desired_angle = 0  # we want to go straight
    current_angle = yaw_msg.data
    calibrated_angle = get_calibrated_steering(90)
    squared_error = (desired_angle - current_angle) ** 2
    squared_errors.append(squared_error)
    times.append(rospy.get_time())

    pub_steering.publish(Kp * (desired_angle - current_angle) + calibrated_angle)
    start_experiment()


def start_experiment():
    global experiment_started
    rate = rospy.Rate(10)  # 10hz

    if experiment_started:
        return

    experiment_started = True

    rospy.sleep(0.5)
    for i in range(100):
        rospy.loginfo(' go straight')
        pub_speed.publish(-100)
        rate.sleep()
    for i in range(10):
        pub_speed.publish(0)
        rate.sleep()

    # experiment finished, draw plot
    # draw_plot()

# def draw_plot():
#     figure = plt.figure()


# --- main ---
rospy.init_node("p_controller")

rospy.loginfo('started')

rospy.Subscriber("/model_car/yaw", Float32, yaw_callback, queue_size=100)

pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100)
pub_steering = rospy.Publisher("/manual_control/steering", Int16, queue_size=100)

rospy.spin()
