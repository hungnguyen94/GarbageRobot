#!/usr/bin/env python
from oscar.srv import *
import rospy

def handle_set_gpio_mode(req):
    mode = "INPUT" if req.mode==0  else "OUTPUT"
    rospy.loginfo("Requested received to set pin %s to mode %s", req.pinNumber, mode)
    # TODO actually set the gpio pin to desired mode
    response = "Pin %s set to mode %s"%(req.pinNumber, mode)
    return SetGpioModeResponse(response)

def handle_set_gpio_value(req):
    rospy.loginfo("Requested received to set pin %s to %s", req.pinNumber, req.value)
    # TODO actually set the gpio pin to desired value
    response ="Pin %s set to %s" %(str(req.pinNumber), str(req.value))
    return SetGpioModeResponse(response)

def set_gpio_mode_server():
    rospy.init_node('set_gpio_mode_node')
    rospy.Service('set_gpio_mode', SetGpioMode, handle_set_gpio_mode)
    rospy.Service('set_gpio_value', SetGpioValue, handle_set_gpio_value)
    rospy.spin()

if __name__ == "__main__":
    set_gpio_mode_server()
