#!/usr/bin/env python

# --- imports ---
import rospy
from sensor_msgs.msg import LaserScan
import math
import sys
from std_msgs.msg import Int16


# --- global variables
scanSub = None
scan = None
alpha = 10
angle2 = None
steeringAngle = None


motorSteer = [0,30,60,90,120,150,179]
speed = 30
sleepTime = 3.0

lengthCar = None

# on car 104
# 90deg is right wall, 180deg is behind, 270 deg is right, 360 is front


angle_far = 30
angle_near = 120
# in radians
angle_between_measurements = (angle_near - angle_far) * math.pi / 180
dist_axles = 0.3
lookahead_distance = 0.5
desired_dist_wall = 0.3
inited = False
Kp = 2
initial_time = None



# --- definitions ---


def measure_distances(ranges):
    return ranges[angle_near], ranges[angle_far]


def measure_distance_and_angle(ranges):

    #get distances
    dist_l2, dist_r2 = measure_distances(ranges)

    # Law of cosines: a**2 = b**2 + c**2 - 2bc*cos(alpha)
    a = math.sqrt(dist_l2**2 + dist_r2**2 - 2*dist_l2*dist_r2*math.cos(angle_between_measurements))
    # Law of sines: sin(alpha)/a = sin(gamma)/c -> sin(alpha)*c/a = sin(gamma)
    angle_gamma = math.asin((math.sin(angle_between_measurements)* dist_r2) / a)

    distance_to_wall = math.sin(angle_gamma) * dist_l2
    theta_l2 = math.acos(distance_to_wall / dist_l2)

    curve_angle = theta_l2 - angle_between_measurements/2

    return distance_to_wall, - curve_angle


def get_delta_heading(scan_msg):
    ranges = scan_msg.ranges

    dist_to_wall, curve_angle = measure_distance_and_angle(ranges)
    center_axis_y = dist_to_wall + math.sin(curve_angle) * dist_axles
    # rospy.loginfo('curve angle: {}'.format(curve_angle * 180/math.pi))
    return math.atan((desired_dist_wall - center_axis_y) / lookahead_distance)


def scan_callback(scan_msg):
    global inited, initial_time

    if not inited:
        inited = True
        start_experiment()
        initial_time = rospy.get_time()
    elif rospy.get_time() - initial_time > 20:
        pub_speed.publish(0)


    calibrated_angle = get_calibrated_steering(90)
    delta_heading = get_delta_heading(scan_msg)
    delta_in_deg = delta_heading * 180/math.pi

    if math.isnan(delta_heading):
        return

    rospy.loginfo(delta_in_deg)
    pub_steer.publish(Kp * delta_in_deg + calibrated_angle)
    # rospy.loginfo('delta heading: {}'.format(delta_heading * 180/math.pi))
    # rospy.loginfo(get_delta_heading(scan_msg))

def start_experiment():
    rospy.sleep(1)
    for i in range(10):
        pub_unlock.publish(0)
        pub_speed.publish(-400)

def get_calibrated_steering(angle):
    return 100

########################################################################################################################
# Until here, everything works fine
# positive delta heading means curve left that amount, negative means curve right that amount
# when the car is at 40cm from the wall right to it and parallel, angle is nearly 0, so we are all fine





if __name__ == "__main__":
    rospy.init_node("driving_parallel")

    pub_steer = rospy.Publisher("/manual_control/steering", Int16, queue_size=100)
    pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100)
    pub_unlock = rospy.Publisher("/manual_control/stop_start", Int16, queue_size=100)
    rospy.Subscriber("scan", LaserScan, scan_callback, queue_size=100)
    rospy.spin()
