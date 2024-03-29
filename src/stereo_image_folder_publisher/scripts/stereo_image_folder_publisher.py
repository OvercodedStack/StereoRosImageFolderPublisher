#!/usr/bin/env python2
from __future__ import print_function

import roslib
roslib.load_manifest('stereo_image_folder_publisher')

import sys
import os
from os import listdir
from os.path import isfile, join

import re
import rospy
import cv2
import math


from sensor_msgs.msg import Image,CameraInfo,RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

class image_folder_publisher:
    def __init__(self):
        self.__app_name = "stereo_image_folder_publisher"
        self._cv_bridge = CvBridge()


        # ROS Topics outputs for images
        self._topic_name_left = rospy.get_param('~topic_name_left', '/image_raw_left')
        rospy.loginfo("[%s] (topic_name) Publishing Images to topic  %s", self.__app_name, self._topic_name_left)
         
        self._image_publisher_left = rospy.Publisher(self._topic_name_left+ "/image_rect", Image, queue_size=1)

        self._topic_name_right = rospy.get_param('~topic_name_right', '/image_raw_right')
        rospy.loginfo("[%s] (topic_name) Publishing Images to topic  %s", self.__app_name, self._topic_name_right)
         
        self._image_publisher_right = rospy.Publisher(self._topic_name_right+ "/image_rect", Image, queue_size=1)

        #ROS params
        self._rate = rospy.get_param('~publish_rate', 30)
        rospy.loginfo("[%s] (publish_rate) Publish rate set to %s hz", self.__app_name, self._rate)

        self._sort_files = rospy.get_param('~sort_files', True)
        rospy.loginfo("[%s] (sort_files) Sort Files: %r", self.__app_name, self._sort_files)



        # ROS Camera topics
        self._frame_id = rospy.get_param('~frame_id', 'camera_left')
        rospy.loginfo("[%s] (frame_id) Frame ID set to  %s", self.__app_name, self._frame_id)
        # self._frame_id_right = rospy.get_param('~frame_id_right', 'camera_right')
        # rospy.loginfo("[%s] (frame_id) Frame ID set to  %s", self.__app_name, self._frame_id_right)

        self._loop = rospy.get_param('~loop', 1)
        rospy.loginfo("[%s] (loop) Loop  %d time(s) (set it -1 for infinite)", self.__app_name, self._loop)


        #Enable camera mimicker
        self.enable_camera_simulation =rospy.get_param('~simulate_camera', False)
        self.camera_baseline=rospy.get_param('~stereo_baseline', 0.1)
        self.camera_fov = rospy.get_param('~simulated_camera_fov', 90)
        if self.enable_camera_simulation:
            self._camera_simulated_left  = rospy.Publisher(self._topic_name_left + "/camera_info", CameraInfo, queue_size=1)
            self._camera_simulated_right = rospy.Publisher(self._topic_name_right + "/camera_info", CameraInfo, queue_size=1)

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
     

    def generate_camera_info(self,image_name, baseline, first_camera):
        img = cv2.imread(image_name)
        h, w, = img.shape[:2]
        cx = w/2
        cy = h/2

        fx = w/(2*math.tan(self.camera_fov/2))
        fy = fx #h/(2*math.tan(self.camera_fov/2))

        Tx = 0
        Ty = 0
        
        if not (first_camera):
            Tx = -fx * baseline
            Ty = 0    
        
        #Camera information
        camera_info_msg = CameraInfo()
        camera_info_msg.height = h
        camera_info_msg.width  = w 
        camera_info_msg.distortion_model = ""
        camera_info_msg.D = []
        camera_info_msg.K = [fx,0.0,cx,0.0,fy,cy,0.0,0.0,1.0]
        camera_info_msg.R = [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0]
        camera_info_msg.P = [fx,0.0,cx,Tx,0.0,fy,cy,Ty,0.0,0.0,1.0,0.0]

        #ROI information...
        roi_msg = RegionOfInterest()
        roi_msg.x_offset = 0
        roi_msg.y_offset = 0
        roi_msg.height = 0
        roi_msg.width = 0
        roi_msg.do_rectify = False

        camera_info_msg.roi = roi_msg
        return camera_info_msg

    def run(self):
        ros_rate = rospy.Rate(self._rate)


        files_in_dir_left  = [f for f in listdir(self._image_folder_left) if isfile(join(self._image_folder_left, f))]
        files_in_dir_right = [f for f in listdir(self._image_folder_right) if isfile(join(self._image_folder_right, f))]


        if self._sort_files:
            files_in_dir_left.sort(key=lambda var:[int(x) if x.isdigit() else x for x in re.findall(r'[^0-9]|[0-9]+', var)])

        if self._sort_files:
            files_in_dir_right.sort(key=lambda var:[int(x) if x.isdigit() else x for x in re.findall(r'[^0-9]|[0-9]+', var)])


        if (len(files_in_dir_left) != len(files_in_dir_right)):
            rospy.loginfo("[%s] Mismatched stereo image count: Left camera count:%d  right camera count: %d", self.__app_name, len(files_in_dir_left),len(files_in_dir_right))
            return
        else:
            rospy.loginfo("[%s] Image count: Left camera count:%d  right camera count: %d", self.__app_name, len(files_in_dir_left),len(files_in_dir_right))


        try:
            img_calibration_left = join(self._image_folder_left, files_in_dir_left[0])
            calibration_data_left = self.generate_camera_info(img_calibration_left,0.1,True)

            img_calibration_right = join(self._image_folder_left, files_in_dir_right[0])
            calibration_data_right = self.generate_camera_info(img_calibration_right,self.camera_baseline,False)

            while self._loop != 0:
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


                                frame_time = rospy.Time.now()
                                ros_msg_left.header.stamp = frame_time
                                

                                ros_msg_right = self._cv_bridge.cv2_to_imgmsg(cv_image_right, "bgr8")
                                ros_msg_right.header.frame_id = self._frame_id
                                ros_msg_right.header.stamp = frame_time



                                #Camera calibration simulator. Outputs to the image topic + /camera_info
                                if (self.enable_camera_simulation):
                                    calibration_data_left.header.frame_id = self._frame_id
                                    calibration_data_left.header.stamp = frame_time

                                    calibration_data_right.header.frame_id = self._frame_id
                                    calibration_data_right.header.stamp = frame_time
                                    self._camera_simulated_left.publish(calibration_data_left)
                                    self._camera_simulated_right.publish(calibration_data_right)
                                self._image_publisher_left.publish(ros_msg_left)
                                self._image_publisher_right.publish(ros_msg_right)
                                rospy.loginfo("[%s] Published %s", self.__app_name, left_image_stereo_filename)
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
