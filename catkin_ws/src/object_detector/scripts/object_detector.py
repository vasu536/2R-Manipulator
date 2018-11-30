#!/usr/bin/python

import rospy
import cv2
import numpy as np
import argparse
import imutils
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from std_msgs.msg import String

def boundingBoxCallback(bb_data):
    
    debug_str = "Subscribed to topic publishing Bounding boxes %s " %rospy.get_time()
    #rospy.loginfo(debug_str)
    #print(debug_str)
    global counter
    global bounding_boxes
    #bounding_box = 0
    bounding_boxes = bb_data.bounding_boxes
    bounding_boxes = bounding_boxes[0]
    if (counter < 5):
        print(counter)
        print(bounding_boxes)
    else:
        counter = counter+1        
        

def object_detector():
    global counter
    centroid_pub = rospy.Publisher('centroid_data', String, queue_size=10)
    rospy.init_node('object_detector', anonymous=True)
    
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, boundingBoxCallback)
    #rospy.loginfo("Subscribed to image message")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #computation
        
        debug_str = "Node launched and subscribing to bounding boxes message %s " %rospy.get_time()
        #rospy.loginfo(debug_str)
        #centroid_pub.publish(debug_str)
        #print(bounding_boxes)
        rate.sleep()
        
def main():
    try:
        object_detector()
    except rospy.ROSInterruptException:
        pass
        
if __name__ == '__main__':
    main()
            