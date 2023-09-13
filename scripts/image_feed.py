#!/usr/bin/env python3
from ast import Try
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import apriltag
import math

def callback(data):

    bridge_object = CvBridge()   
    try:
        img = bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)
        
    cv.imshow("Image Feed",img)
    cv.waitKey(1)

def listener():

    rospy.init_node('image_feed', anonymous=True)

    sub = rospy.Subscriber("/camera/image_raw", Image, callback)
    
    rospy.spin()

if __name__ == '__main__':

    listener()

   

    