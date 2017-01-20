#!/usr/bin/env python
import ConfigParser
import rospy
from std_msgs.msg import String
from oscar.msg import gpio
from enum import Enum
import RPi.GPIO as GPIO
import cv2
from cv_bridge import CvBridge
import os

configPath= os.path.dirname(os.path.abspath(__file__)) + '/cfg/rpi_pins.cfg'
print configPath
pinDefs_input = {}
pinDefs_output = {}
sorting_output = {}

class State(Enum):
    driving = 0
    interacting = 1
    full = 2
    sorting = 3

currentState = State.driving


def GPIO_input_falling(pin):
    b = GPIO.input(pin)
    print "Pin {0} is now {1}".format(pin,b)
    if pin in pinDefs_input:
        pinDefs_input[pin](b)

def startROS():
    rospy.init_node('state_controller', anonymous=True)
    # rospy.Subscriber("sonar_data", Range, handle_sonar_message)
    rospy.on_shutdown(shutdown)
    rospy.spin()

def setupPins():
    GPIO.setmode(GPIO.BOARD)
    for keystr in pinDefs_input:
        key = int(keystr)
        GPIO.setup(key,GPIO.IN,pull_up_down=GPIO.PUD_UP)
	print "Set pin {0} to GPIO.IN".format(key)
        GPIO.add_event_detect(key,GPIO.FALLING,callback=GPIO_input_falling)
    for keystr in pinDefs_output:
        key = int(pinDefs_output[keystr])
	GPIO.setup(key,GPIO.OUT)
	print "Set pin {0} to GPIO.OUT".format(key)
	GPIO.output(key,0)
	print "Setting ping {0} to 0".format(key)

def readConfig():
    config = ConfigParser.ConfigParser()
    config.read(configPath)
    print  config.sections()
    for (name, pin_number) in config.items('GPIO_in'):
        pinDefs_input[pin_number] = name

    for (name, pin_number) in config.items('GPIO_out'):
        pinDefs_output[name] = pin_number

    
    sorting_output[0] = pinDefs_output["class_bottle"]
    sorting_output[1] = pinDefs_output["class_can"]
    sorting_output[2] = pinDefs_output["class_coffee"]
    sorting_output[3] = pinDefs_output["class_residual"]


if __name__ == '__main__':
    readConfig()
    setupPins()
    startROS()

def shutdown():
    GPIO.cleanup()

# ===============================


def start_user_interaction() :
    if currentState == State.full:
        return
    # stop driving
    currentstate = State.interacting

def start_sorting() :
    if currentState == State.full:
        return 
    if currentState == State.sorting:
        return
    else :
        currentState = State.sorting
    print "Started sorting"
    # set mouth leds to HIGH
    GPIO.output(pinDefs_output["mouth_leds"],True)
    print "Set pin {0} to 1".format(pinDefs_output["mouth_leds"])
    # turn sorting lights on
    GPIO.output(pinDefs_output["sorting_leds"],True)
    print "Set pin {0} to 1".format(pinDefs_output["sorting_leds"])
    # call the litter classification Service
    print "Starting classifing garbage"
    garbageID = classify_garbage()
    print "Garbage classified"
    # set the correct pin to high based on the result of the classification
    GPIO.output(sorting_output[garbageID],True)
    print "Set pin {0} to 1".format(sorting_output["garbageID"])
    sleep(0.5)
    print "Set pin {0} to 0".format(sorting_output["garbageID"])
    GPIO.output(sorting_output[garbageID],False)
    sleep(3)
    GPIO.output(pinDefs_output["sorting_leds"],False)    
    print "Set pin {0} to 0".format(pinDefs_output["mouth_leds"])
    GPIO.output(pinDefs_output["mouth_leds"],False)
    print "Set pin {0} to 0".format(pinDefs_output["sorting_leds"]) 

def mouth_sensor(level):
    if not level :
        start_sorting()

def stop_button(data):
    start_user_interaction()

def fill_level(data):
    if data.value :
        currentState = State.full
    elif currentState == State.full:
        currentState = State.interacting
# =============================================================
def classify_image(image):
    """
    Create a request to the classify image service.
    :param image: Image to be classified.
    :return: Integer representing the predicted class.
    """
    rospy.wait_for_service('image_classify')
    try:
        image_classify = rospy.ServiceProxy('image_classify', ClassifyImage)
        result = image_classify(image)
        return result.prediction
    except rospy.ServiceException, e:
        print "Service call image_classify failed: %s"%e

def classify_garbage():
    cam = cv2.VideoCapture(cam_index)
    cam.set(3, 1280)
    cam.set(4, 720)
    ret, img = cam.read()
    imgmsg = CvBridge().cv2_to_imgmsg(img)
    return classify_image(imgmsg)
