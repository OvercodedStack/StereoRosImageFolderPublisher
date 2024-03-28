#!/usr/bin/env python3
from __future__ import print_function

import roslib
roslib.load_manifest('stereo_image_folder_publisher')

import sys
import os
from os import listdir
from os.path import isfile, join

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_folder_publisher:
    def __init__(self):
        self.__app_name = "stereo_image_folder_publisher"
        self._cv_bridge = CvBridge()


        # ROS Topics outputs for images
        self._topic_name_left = rospy.get_param('~topic_name_left', '/image_raw_left')
        rospy.loginfo("[%s] (topic_name) Publishing Images to topic  %s", self.__app_name, self._topic_name_left)
        self._image_publisher_left = rospy.Publisher(self._topic_name_left, Image, queue_size=1)

        self._topic_name_right = rospy.get_param('~topic_name_right', '/image_raw_right')
        rospy.loginfo("[%s] (topic_name) Publishing Images to topic  %s", self.__app_name, self._topic_name_right)
        self._image_publisher_right = rospy.Publisher(self._topic_name_right, Image, queue_size=1)

        #ROS params
        self._rate = rospy.get_param('~publish_rate', 30)
        rospy.loginfo("[%s] (publish_rate) Publish rate set to %s hz", self.__app_name, self._rate)

        self._sort_files = rospy.get_param('~sort_files', True)
        rospy.loginfo("[%s] (sort_files) Sort Files: %r", self.__app_name, self._sort_files)

        # ROS Camera topics
        self._frame_id_left = rospy.get_param('~frame_id_left', 'camera_left')
        rospy.loginfo("[%s] (frame_id) Frame ID set to  %s", self.__app_name, self._frame_id_left)

        self._frame_id_right = rospy.get_param('~frame_id_right', 'camera_right')
        rospy.loginfo("[%s] (frame_id) Frame ID set to  %s", self.__app_name, self._frame_id_right)

        self._loop = rospy.get_param('~loop', 1)
        rospy.loginfo("[%s] (loop) Loop  %d time(s) (set it -1 for infinite)", self.__app_name, self._loop)

        # ROSrun folder parameters
        self._image_folder_left = rospy.get_param('~image_folder_left', '')
        self._image_folder_right = rospy.get_param('~image_folder_right', '')

        self._sleep = rospy.get_param('~sleep', 0.0)
        rospy.sleep(self._sleep)
        rospy.loginfo("[%s] (sleep) Sleep %f seconds after each image", self.__app_name, self._sleep)
        
        #Section for loading the photos... Could use a dynamically loaded approach for times when pictures are incidentially loaded.         
        if self._image_folder_left == '' or not os.path.exists(self._image_folder_left) or not os.path.isdir(self._image_folder_left):
            rospy.logfatal("[%s] (image_folder) Invalid Image folder", self.__app_name)
            sys.exit(0)
        rospy.loginfo("[%s] Reading images from %s", self.__app_name, self._image_folder_left)

        if self._image_folder_right == '' or not os.path.exists(self._image_folder_right) or not os.path.isdir(self._image_folder_right):
            rospy.logfatal("[%s] (image_folder) Invalid Image folder", self.__app_name)
            sys.exit(0)

        rospy.loginfo("[%s] Reading images from %s", self.__app_name, self._image_folder_right)
        

    def run(self):
        ros_rate = rospy.Rate(self._rate)


        files_in_dir_left = [f for f in listdir(self._image_folder_left) if isfile(join(self._image_folder_left, f))]
        files_in_dir_right = [f for f in listdir(self._image_folder_right) if isfile(join(self._image_folder_right, f))]


        if self._sort_files:
            files_in_dir_left.sort()

        if self._sort_files:
            files_in_dir_right.sort()


        if (len(files_in_dir_left) != len(files_in_dir_right)):
            rospy.loginfo("[%s] Mismatched stereo image count: Left camera count:%d  right camera count: %d", self.__app_name, len(files_in_dir_left),len(files_in_dir_right))
            return

        try:
            while self._loop != 0:


                # for f in files_in_dir:
                for i in range(0,len(files_in_dir_left)):
                    if not rospy.is_shutdown():
                        left_image_stereo_filename   = join(self._image_folder_left, files_in_dir_left[i])
                        right_image_stereo_filename  = join(self._image_folder_right, files_in_dir_right[i])

                        # if isfile(join(self._image_folder, f)):
                        if(isfile(left_image_stereo_filename) and isfile(right_image_stereo_filename)):
                            cv_image_left = cv2.imread(left_image_stereo_filename)
                            cv_image_right= cv2.imread(right_image_stereo_filename)
                            
                            if (cv_image_left is not None and cv_image_right is not None):
                                ros_msg_left = self._cv_bridge.cv2_to_imgmsg(cv_image_left, "bgr8")
                                ros_msg_left.header.frame_id = self._frame_id
                                ros_msg_left.header.stamp = rospy.Time.now()
                                self._image_publisher_left.publish(ros_msg_left)
                                rospy.loginfo("[%s] Published %s", self.__app_name, left_image_stereo_filename)

                                ros_msg_right = self._cv_bridge.cv2_to_imgmsg(cv_image_right, "bgr8")
                                ros_msg_right.header.frame_id = self._frame_id
                                ros_msg_right.header.stamp = rospy.Time.now()
                                self._image_publisher_left.publish(ros_msg_left)
                                rospy.loginfo("[%s] Published %s", self.__app_name, right_image_stereo_filename)
                            else:
                                rospy.loginfo("[%s] Invalid image file %s", self.__app_name, left_image_stereo_filename)
                            ros_rate.sleep()
                    else:
                        return



                self._loop = self._loop - 1



        except CvBridgeError as e:
            rospy.logerr(e)


def main(args):
    rospy.init_node('stereo_image_folder_publisher', anonymous=True)

    image_publisher = image_folder_publisher()
    image_publisher.run()


if __name__ == '__main__':
    main(sys.argv)
