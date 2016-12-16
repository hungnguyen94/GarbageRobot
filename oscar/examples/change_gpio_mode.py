#!/usr/bin/env python

import sys
import rospy

def add_two_ints_client(x, y):
    rospy.wait_for_service('set_gpio_mode')
    try:
        service = rospy.ServiceProxy('set_gpio_mode', SetGpioMode)
        resp1 = service(x,y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [pinNumber mode]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        pin = int(sys.argv[1])
        mode = bool(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting setting pin %s to %s"%(pin, mode)
