#!/usr/bin/env python
import ConfigParser
import rospy
from std_msgs.msg import String
from oscar.msg import gpio
from enum import Enum
import RPi.GPIO as GPIO

class State(Enum):
    driving = 0
    interacting = 1
    full = 2

currentState = State.driving
configPath= 'cfg/rpi_pins.cfg'
pinDefs_input = {}
pinDefs_output = {}

def start_user_interaction() :
    if currentState == State.full:
        return
    # stop driving
    
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
    # set the correct pin to high based on the result of the classification

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
            pinDefs_output[pin_number] = name

if __name__ == '__main__':
    readConfig()
    setupPins()
    startROS()
