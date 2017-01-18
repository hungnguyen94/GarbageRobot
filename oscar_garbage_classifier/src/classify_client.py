#!/usr/bin/env python

from __future__ import print_function
import rospy
from oscar_garbage_classifier.srv import ClassifyImage

classes = ['bottles', 'cans', 'cups', 'other']
cam_index = 0

cups_pin = 36
pmd_pin = 38
other_pin = 40


def init_rpi_gpio():
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([cups_pin, pmd_pin, other_pin], GPIO.OUT)


def classify_image(image):
    """
    Create a request to the classify image service.
    :param image: Image to be classified.
    :return: Integer representing the predicted class.
    """
    rospy.wait_for_service('image_classify')
    image_classify = rospy.ServiceProxy('image_classify', ClassifyImage)
    result = image_classify(image)
    print('Result: %s' % result.prediction)
    return result.prediction


def switch_input(pin):
    GPIO.output(pin, GPIO.HIGH)
    GPIO.output(pin, GPIO.LOW)


def invoke_sorter(classification):
    if classification in ['bottles', 'cans']:
        switch_input(pmd_pin)
    elif classification in ['cups']:
        switch_input(cups_pin)
    else:
        switch_input(other_pin)


def usage():
    return ""

if __name__ == '__main__':
    import cv2
    from cv_bridge import CvBridge

    cam = cv2.VideoCapture(cam_index)
    cam.set(3, 1280)
    cam.set(4, 720)
    ret, img = cam.read()
    imgmsg = CvBridge().cv2_to_imgmsg(img)
    result = classify_image(imgmsg)
    # invoke_sorter(classes[result])
    # GPIO.cleanup()


