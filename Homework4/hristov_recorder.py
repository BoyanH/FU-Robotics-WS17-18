
#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan

LIDAR_TOPIC = '/scan'
current_angle = 179  # from 0, 30, 60, 90, 120, 150, 179
experiment_speed = -200

class SteeringCalibration:
    def __init__(self):
        rospy.Subscriber(LIDAR_TOPIC, LaserScan, self.on_new_scan_msg)
        self.steeringPublisher = rospy.Publisher('/manual_control/steering', Int16, queue_size=10)
        self.speedPublisher = rospy.Publisher('/manual_control/speed', Int16, queue_size=10)
        self.waiting_for_scan = True
        self.first_measurement_taken = False

        rospy.loginfo('taking measurement')

    def calibrate(self):
        pass

    def measure(self):
        pass

    def on_measurement_ready(self):
        if not self.first_measurement_taken:
            self.first_measurement_taken = True
            rospy.loginfo('measurement taken, now drive')
	    self.steeringPublisher.publish(current_angle)
	    rospy.sleep(1)
            self.speedPublisher.publish(experiment_speed)
            rospy.sleep(5)
            rospy.loginfo('stop')
            self.speedPublisher.publish(0) # stop
            rospy.loginfo('taking second measurement')
            self.waiting_for_scan = True
        else:
            rospy.loginfo('second measurement taken')


    def on_new_scan_msg(self, msg):
        if self.waiting_for_scan:
            self.waiting_for_scan = False
	    self.steeringPublisher.publish(current_angle)
            rospy.sleep(3)
            self.on_measurement_ready()


    def begin_scan(self):
        self.scanned_area = 0

def init_calibration():
    rospy.init_node('camera_processor', anonymous=True)
    steering_calibration = SteeringCalibration()
    # don't exit while node is not stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        init_calibration()
    except rospy.ROSInterruptException:
        pass
