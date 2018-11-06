#!/usr/bin/env python
"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs
CompressedImage. It converts the CompressedImage into a numpy.ndarray,
then detects and marks features in that image. It finally displays
and publishes the new image - again as CompressedImage topic.
"""
__author__ =  'Wang Tong'
# Python libs
import sys, time

# numpy and scipy
import numpy as np

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
import requests
# Ros Messages
from sensor_msgs.msg import CompressedImage, Image
# We do not use cv_bridge it does not support CompressedImage in python
from cv_bridge import CvBridge, CvBridgeError

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''

        # subscribed Topic
        self.subscriberTopic = "kinect2_sd_image_color_rect"
        self.subscriber = rospy.Subscriber("/kinect2/hd/image_color_rect",
            Image, self.callback)
        self.imageIndex = 0
        self.imageMaxIndex = 10
        self.imageUploadUrl = r"http://wangtong15.com:10001/BroadCast/"
        self.bridge = CvBridge()
        self.imageHeight = 80
        self.imageWidth = (int)(self.imageHeight * 517.0 / 375.0)


    def callback(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''

        #### direct conversion to CV2 ####
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
            cv_image = cv2.resize(cv_image, (self.imageWidth, self.imageHeight), interpolation = cv2.INTER_AREA)
            self.imageIndex = (self.imageIndex + 1) % self.imageMaxIndex
            self.imageName = "./{0}_{1}.jpeg".format(self.subscriberTopic, self.imageIndex)
            cv2.imwrite(self.imageName, cv_image)
            requests.request("POST", self.imageUploadUrl, files = {'image':open(self.imageName, "rb")})
        except CvBridgeError as e:
            print(e)
        except KeyboardInterrupt:
            print("KeyboardInterrupt")
        except:
            print("Other Error")



def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_broadcast', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
