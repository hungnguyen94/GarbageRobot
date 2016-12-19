#!/usr/bin/env python

from __future__ import print_function
import rospy
from oscar_garbage_classifier.srv import ClassifyImage
import RPi.GPIO as GPIO

def classify_image_client(image):
    rospy.wait_for_service('image_classify')
    image_classify = rospy.ServiceProxy('image_classify', ClassifyImage)
    result = image_classify(image)
    print('Result: %s' % result)
    return result.prediction

def usage():
    return "hey"

if __name__ == '__main__':
    import cv2
    from cv_bridge import CvBridge
    img = cv2.imread('/mnt/data/Development/ros/catkin_ws/images/coffee_cup_7.jpg')
    imgmsg = CvBridge().cv2_to_imgmsg(img)
    res = classify_image_client(imgmsg)



