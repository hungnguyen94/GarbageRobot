#!/usr/bin/env python
from oscar.srv import *
import rospy

def set_gpio_mode(req):
    rospy.loginfo("Requested received to set ping %s to %s", req.pinNumber, req.mode)
    return SetGpioModeResponse("Request handled")

def set_gpio_mode_server():
    rospy.init_node('set_gpio_mode_node')
    s = rospy.Service('set_gpio_node', SetGpioMode, set_gpio_mode)
    rospy.spin()

if __name__ == "__main__":
    rospy.logerr("I will publish to the topic")
    rospy.loginfo("Starting node set_gpio_mode_node")
    set_gpio_mode_server()
