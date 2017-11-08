#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2


class ImageProcessor:
    def __init__(self):
        self.cv_gray_scale_image = None
        self.bridge = CvBridge()
        rospy.Subscriber('/app/camera/rgb/image_raw', Image, self.on_new_image)
        self.gray_scale_publisher = rospy.Publisher('/app/camera/gray_scale/image_raw', Image, queue_size=10)

    def process_image(self, image_message):
        # convert image to openCv grayscale image
        self.cv_gray_scale_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="mono8")
        gray_scale_image_to_transport = self.bridge.cv2_to_imgmsg(self.cv_gray_scale_image)
        rospy.loginfo("Publishing grayscale")
        self.gray_scale_publisher.publish(gray_scale_image_to_transport)

        # in order to view the gray-scale image use:
        # rosrun image_view image_view image:=/app/camera/gray_scalimage_raw _do_dynamic_scaling:=true

        bi_gray_max = 255
        bi_gray_min = 245
        img_bw = cv2.threshold(self.cv_gray_scale_image, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY)[1]
        cv2.imwrite('bw_image.png', img_bw)

    def on_new_image(self, data):
        self.process_image(data)


def camera_processor():
    rospy.init_node('camera_processor', anonymous=True)
    image_processor = ImageProcessor()
    # don't exit while node is not stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        camera_processor()
    except rospy.ROSInterruptException:
        pass
