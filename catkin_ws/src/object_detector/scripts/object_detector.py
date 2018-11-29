#!/usr/bin/python

import rospy
import cv2
import numpy as np
import argparse
import imutils
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

img = 0

def imageCallback(img_data):
    #store image data to use
    global img
    img = img_data

def object_detector():
    centroid_pub = rospy.Publisher('centroid_data', String, queue_size=10)
    rospy.init_node('object_detector', anonymous=True)
    
    rospy.Subscriber("/usb_cam/image_raw", Image, imageCallback)
    rospy.loginfo("Subscribed to image message")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #computation
        
        debug_str = "Node launched and subscribing from camera %s " %rospy.get_time()
        rospy.loginfo(debug_str)
        centroid_pub.publish(debug_str)
        rate.sleep()
        
def main():
    try:
        object_detector()
    except rospy.ROSInterruptException:
        pass
        
if __name__ == '__main__':
    main()
            