#!/usr/bin/env python
import ConfigParser
import rospy
from std_msgs.msg import String
from oscar.msg import gpio
from enum import Enum
import RPi.GPIO as GPIO
import pin_behaviour

configPath= 'cfg/rpi_pins.cfg'
pinDefs_input = {}
pinDefs_output = {}
sorting_output = {}
currentState = State.driving

class State(Enum):
    driving = 0
    interacting = 1
    full = 2

def GPIO_input_falling(pin):
    if pin in pinDefs_input:
        pinDefs_input[pin](False)
def GPIO_input_rising(pin):
    if pin in pinDefs_input:
        pinDefs_input[pin](True)

def startROS():
    rospy.init_node('state_controller', anonymous=True)
    rospy.Subscriber("sonar_data", Range, handle_sonar_message)
    rospy.spin()

def setupPins():
    GPIO.setmode(GPIO.BOARD)
    for key in pinDefs_input:
        GPIO.setup(key,GPIO.IN)
        GPIO.add_event_detect(key,GPIO.FALLING,callback=GPIO_input_falling)
        GPIO.add_event_detect(key,GPIO.RISING,callback=GPIO_input_rising)
    for key in pinDefs_output:
        GPIO.setup(key,GPIO.OUT)

def readConfig():
    config = ConfigParser.RawConfigParser()
    config.read(configPath)
    for (name, pin_number) in config.items(GPIO_in):
        pinDefs_input[pin_number] = name

    for (name, pin_number) in config.items(GPIO_out):
        pinDefs_output[name] = pin_number

    sorting_output[0] = sorting_output["class_can"]
    sorting_output[1] = sorting_output["class_bottle"]
    sorting_output[2] = sorting_output["class_coffee"]
    sorting_output[3] = sorting_output["class_residual"]


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
    if currentState != State.interacting:
        start_user_interaction()
    # set mouth leds to HIGH
    GPIO.output(pinDefs_output["mouth_leds"],True)
    # turn sorting lights on
    GPIO.output(pinDefs_output["sorting_leds"],True)
    # call the litter classification Service
    garbageID = classify_garbage()
    # set the correct pin to high based on the result of the classification
    GPIO.output(sorting_output[garbageID],True)
    sleep(0.5)
    GPIO.output(sorting_output[garbageID],False)
    sleep(3)
    GPIO.output(pinDefs_output["sorting_leds"],False)
    GPIO.output(pinDefs_output["mouth_leds"],False)




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

def classify_garbage():
    rospy.wait_for_service('image_classify')
    try:
        add_two_ints = rospy.ServiceProxy('image_classify', AddTwoInts)
        result = image_classify(image)(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call image_classify failed: %s"%e
