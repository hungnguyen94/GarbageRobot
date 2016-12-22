#!/usr/bin/env python
from oscar.srv import *
import rospy
import RPi.GPIO as GPIO

def handle_set_gpio_mode(req):
    mode = GPIO.IN if req.mode==1  else GPIO.OUT
    rospy.loginfo("Requested received to set pin %s to mode %s", req.pinNumber, mode)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(req.pinNumber,mode)
    response = "Pin %s set to mode %s"%(req.pinNumber, mode)
    return SetGpioModeResponse(response)

def handle_set_gpio_value(req):
    rospy.loginfo("Requested received to set pin %s to %s", req.pinNumber, req.value)
    output = GPIO.HIGH if (req.value) else GPIO.LOW
    GPIO.output(req.pinNumber, output)
    response ="Pin %s set to %s" %(str(req.pinNumber), str(output))
    return SetGpioModeResponse(response)

def set_gpio_mode_server():
    rospy.init_node('set_gpio_mode_node')
    rospy.Service('set_gpio_mode', SetGpioMode, handle_set_gpio_mode)
    rospy.Service('set_gpio_value', SetGpioValue, handle_set_gpio_value)
    rospy.spin()

if __name__ == "__main__":
    set_gpio_mode_server()
