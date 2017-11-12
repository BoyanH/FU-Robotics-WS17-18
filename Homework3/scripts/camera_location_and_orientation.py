#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import os
from sklearn.cluster import KMeans
from copy import deepcopy


gray_scale_topic = '/app/camera/gray_scale/image_raw'
number_of_points = 6

class ImageProcessor:
    def __init__(self):
        self.cv_gray_scale_image = None
        self.bridge = CvBridge()
        rospy.Subscriber('/app/camera/rgb/image_raw', Image, self.on_new_image)
        self.gray_scale_publisher = rospy.Publisher(gray_scale_topic, Image, queue_size=10)
        self.image_processed = False

        # Flags:
        # keep reworking and publishing new grayscale image each time an image is received
        self.stream_gray_image = False
        self.use_adaptive_threshold = True

    @staticmethod
    def get_bottom_left_point(points):
        # this approach gets the lowest left point, though as we are missing some pixels in the bw image
        # a better approach would be to take the min_x and min_y as a point
        # point_judgement = list(map(lambda point: point[0] + (point[1] * -1), points))
        # min_idx = point_judgement.index(min(point_judgement))
        # return points[min_idx]

        min_x = min(list(map(lambda x: x[0], points)))
        max_y = max(list(map(lambda x: x[1], points)))
        return (min_x, max_y)

    def get_camera_location_and_orientation(self, image_data):
        bw_image = self.process_image(image_data)
        white_points_coordinates = []

        # we extract the coordinates of all white pixels from the image
        # into the white_points_coordinates array
        for row_idx, row in enumerate(bw_image):
            for col_idx, pixel in enumerate(row):
                if pixel == 255:
                                                    # x     , y
                    white_points_coordinates.append((col_idx, row_idx))

        # then we use k-means algorithm to cluster all points into clusters (group them)
        km = KMeans(n_clusters=number_of_points).fit(white_points_coordinates)
        points_per_cluster = [[] for x in range(number_of_points)]
        for idx, label in enumerate(km.labels_):
            points_per_cluster[label].append(white_points_coordinates[idx])

        # then we can extract the lowest-left point from each cluster
        # (in our case the min_x and max_y as some white pixels are missing on the image)
        square_coordinates = list(map(lambda cluster_points: ImageProcessor.get_bottom_left_point(cluster_points),
                                      points_per_cluster))
        # sort row-wise
        square_coordinates.sort(key = lambda x: x[0])
        square_coordinates.sort(key = lambda x: x[1])

        # for debugging purposes
        # save an image with the coordinates marked as gray points
        marked_img = deepcopy(bw_image)
        for point in square_coordinates:
            marked_img[point[1]][point[0]] = 127
        marked_save_path = os.path.join(os.path.dirname(__file__), '../output/marked_image.png')
        cv2.imwrite(marked_save_path, marked_img)

        rospy.loginfo(square_coordinates)

    def process_image(self, image_data):
        # convert image to openCv grayscale image
        self.cv_gray_scale_image = self.bridge.imgmsg_to_cv2(image_data, desired_encoding="mono8")
        gray_scale_image_to_transport = self.bridge.cv2_to_imgmsg(self.cv_gray_scale_image)
        rospy.loginfo("Publishing gray-scale image to topic {}".format(gray_scale_topic))
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
        return img_bw

    def on_new_image(self, data):
        if not self.image_processed or self.stream_gray_image:
            self.image_processed = True
            self.get_camera_location_and_orientation(data)


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
