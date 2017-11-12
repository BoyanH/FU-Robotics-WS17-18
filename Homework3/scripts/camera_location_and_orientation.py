#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import os

gray_scal_topic = '/app/camera/gray_scale/image_raw'

class ImageProcessor:
    def __init__(self):
        self.cv_gray_scale_image = None
        self.bridge = CvBridge()
        rospy.Subscriber('/app/camera/rgb/image_raw', Image, self.on_new_image)
        self.gray_scale_publisher = rospy.Publisher(gray_scal_topic, Image, queue_size=10)
        self.image_processed = False

        # Flags:
        # keep reworking and publishing new grayscale image each time an image is received
        self.stream_gray_image = False
        self.use_adaptive_threshold = True

    def process_image(self, image_message):
        # convert image to openCv grayscale image
        self.cv_gray_scale_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="mono8")
        gray_scale_image_to_transport = self.bridge.cv2_to_imgmsg(self.cv_gray_scale_image)
        rospy.loginfo("Publishing gray-scale image to topic {}".format(gray_scal_topic))
        self.gray_scale_publisher.publish(gray_scale_image_to_transport)

        # in order to view the gray-scale image use:
        # rosrun image_view image_view image:=/app/camera/gray_scale/image_raw _do_dynamic_scaling:=true
        # for this to work, the flag stream_gray_image must be set to True

        if not self.use_adaptive_threshold:
            # the method we actually needed to use. Unfortunately our image had a really bright area on top-right
            # corner, so the next approach was needed

            bi_gray_max = 255 # use pure white for the values above the treshold
            bi_gray_min = 205
            img_bw = cv2.threshold(self.cv_gray_scale_image, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY)[1]
            bw_save_path = os.path.join(os.path.dirname(__file__), '../output/bw_image.png')
        else:
            # We need to determine the brightness of
            # our white pixel in relation to the nearby pixels. So we are no longer using a global threshold
            # but instead we are comparing how "white" a pixel is compared to its neighbours

            # In our case, we look at a 167x167 square and determine the mean (average) pixel brightness
            # in that area and compare it to the value of the middle pixel. If it is brighter, than it goes white
            # the -90 says it must be at least 90 points (from 0 to 255) brighter than the average to go white
            # We found the values 167 and -90 experimentally, a good starting point would be the size of a square +
            # half of the size between the squares horizontally (as the distance vertically gets shorter for the
            # further points due to perspective)
            img_bw = cv2.adaptiveThreshold(self.cv_gray_scale_image, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                   cv2.THRESH_BINARY, 167, -90)
            bw_save_path = os.path.join(os.path.dirname(__file__), '../output/bw_image_adaptive.png')

        cv2.imwrite(bw_save_path, img_bw)
        rospy.loginfo("Wrote bw image to {}".format(os.path.realpath(bw_save_path)))

    def on_new_image(self, data):
        if not self.image_processed or self.stream_gray_image:
            self.image_processed = True
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
