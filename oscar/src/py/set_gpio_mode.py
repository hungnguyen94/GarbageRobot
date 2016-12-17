#!/usr/bin/env python
from oscar.srv import *
import rospy

def handle_set_gpio_mode(req):
    rospy.loginfo("Requested received to set pin %s to %s", req.pinNumber, req.mode)
    # TODO actually set the gpio pin to desired output
    response = "Pin %s set to %s"%(req.pinNumber, req.mode)
    return SetGpioModeResponse(response)

def set_gpio_mode_server():
    rospy.init_node('set_gpio_mode_node')
    s = rospy.Service('set_gpio_node', SetGpioMode, handle_set_gpio_mode)
    rospy.spin()

if __name__ == "__main__":
    set_gpio_mode_server()
