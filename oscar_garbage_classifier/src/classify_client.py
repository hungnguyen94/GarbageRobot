#!/usr/bin/env python

from __future__ import print_function
import rospy
from oscar_garbage_classifier.srv import ClassifyImage

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
    cam = cv2.VideoCapture(0)
    ret, img = cam.read()
    imgmsg = CvBridge().cv2_to_imgmsg(img)
    classify_image_client(imgmsg)


