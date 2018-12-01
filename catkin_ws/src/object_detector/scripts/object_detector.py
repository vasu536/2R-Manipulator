#!/usr/bin/python

import rospy
import cv2
import numpy as np
import math
from random import *
import argparse
import imutils
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from std_msgs.msg import String

counter = 0
counter_thresh = 5
bounding_boxes = 0
pixel_center_x_cumm = 0
pixel_center_y_cumm = 0
pixel_center_x = 0
pixel_center_y = 0

fx = 1455.9
fy = 1476.7
cx = 656.4961
cy = 352.9728

p_pixel = 0
p_camera_hom = 0
p_camera = 0

p_desired = 0
Z = 1

rot_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

def boundingBoxCallback(bb_data):
    
    debug_str = "Subscribed to topic publishing Bounding boxes %s " %rospy.get_time()
    global counter
    global counter_thresh
    global bounding_boxes
    global pixel_center_x_cumm
    global pixel_center_y_cumm
    global pixel_center_x
    global pixel_center_y
    #rospy.loginfo(debug_str)
    #print(debug_str)
    #bounding_box = 0
    bounding_boxes = bb_data.bounding_boxes
    bounding_boxes = bounding_boxes[0]
    if (counter < counter_thresh):
        #print(counter)
        #print(bounding_boxes.Class)
        pixel_center_x_cumm = pixel_center_x_cumm + math.floor((bounding_boxes.xmin + bounding_boxes.xmax)/2)
        pixel_center_y_cumm = pixel_center_y_cumm + math.floor((bounding_boxes.ymin + bounding_boxes.ymax)/2)
        counter = counter+1
    elif (counter == counter_thresh):
        #print("Calculating the average")
        pixel_center_x = math.floor(pixel_center_x_cumm/counter)
        pixel_center_y = math.floor(pixel_center_y_cumm/counter)
        counter = counter+1


def object_detector():
    
    centroid_pub = rospy.Publisher('centroid_data', Point, queue_size=10)
    rospy.init_node('object_detector', anonymous=True)
    
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, boundingBoxCallback)
    #rospy.loginfo("Subscribed to image message")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        #computation
        global counter
        global counter_thresh        
        global pixel_center_x
        global pixel_center_y
        global fx, fy, cx, cy, Z
        global p_pixel
        global p_camera_hom
        global p_camera
        global p_desired
        global rot_matrix
        
        p_desired = Point()
        p_desired.x = 0
        p_desired.y = 0
        p_desired.z = 1
        p_pixel = np.array([pixel_center_x, pixel_center_y, 1])
        print(p_pixel)
        if (counter == counter_thresh):
            p_camera_hom = np.array([0, 0, 1], dtype=np.float32)
            p_camera_hom[0] = (p_pixel[0] - cx)/fx
            p_camera_hom[1] = (p_pixel[1] - cy)/fy
            #print("Entering here!!!")
        #debug_str = "Node launched and subscribing to bounding boxes message %s " %rospy.get_time()
        #rospy.loginfo(debug_str)
        #centroid_pub.publish(debug_str)
        #print(p_camera_hom)
        #print("A random number", randint(1, 100))
        if (counter > counter_thresh):
            
            p_camera = np.array([p_camera_hom[0] * Z, p_camera_hom[1] * Z, Z])
            p_desired_array = (np.dot(rot_matrix, p_camera.transpose())).transpose()
        
            p_desired.x = p_desired_array[0]
            p_desired.y = p_desired_array[1]
            p_desired.z = 1
        centroid_pub.publish(p_desired)
        rate.sleep()
        
def main():
    try:
        object_detector()
    except rospy.ROSInterruptException:
        pass
        
if __name__ == '__main__':
    main()
            