#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from oscar_garbage_classifier.srv import Sort
from time import sleep

GPIO_available = False
cups_pin = 33
pmd_pin = 31
other_pin = 29


def init_GPIO():
    """
    Set GPIO available if running on RPi. Set to false if it can't be imported.
    Set GPIO mode to BOARD, and set pins to OUTPUT.
    """
    global GPIO_available
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup([cups_pin, pmd_pin, other_pin], GPIO.OUT)
        GPIO_available = True
    except RuntimeError:
        GPIO_available = False


def handle_service(request):
    """
    Invoke the sorting mechanism using the RPi GPIO pins.
    Integers represent the category.
    { 0: cups, 1: pmd, 2: other }
    :param request: Sort request containing the type in Sort.category, as an integer.
    :return: Boolean containing result.
    """
    switcher = {0: lambda: switch_pin(cups_pin),
                1: lambda: switch_pin(pmd_pin),
                2: lambda: switch_pin(other_pin)}
    # Call function in the switcher, with default value handling.
    return switcher.get(request.category, lambda: False)()


def switch_pin(pin):
    """
    Switch GPIO pins to invoke the sorter.
    :param pin: Pin number(using board mode)
    :return: True if successful
    """
    # Just print if gpio is not available.
    if GPIO_available:
        try:
            GPIO.output(pin, GPIO.HIGH)
            if GPIO.input(pin) != GPIO.HIGH:
                raise Exception("GPIO pin %s was set to %s but was %s" % (pin, GPIO.HIGH, GPIO.input(pin)))
            sleep(0.5)
            GPIO.output(pin, GPIO.LOW)
            if GPIO.input(pin) != GPIO.LOW:
                raise Exception("GPIO pin %s was set to %s but was %s" % (pin, GPIO.LOW, GPIO.input(pin)))
            return True
        except Exception as e:
            rospy.logerr("An error has occurred when setting GPIO pins: \n%s" % (str(e)))
            return False
    else:
        rospy.logdebug("Switching pin %s" % pin)
        return True


def sorter_service():
    """
    Register sorter service.
    """
    rospy.init_node('sorter_service')
    init_GPIO()
    s = rospy.Service('invoke_sorter', Sort, handle_service)
    rospy.loginfo('Sorter service ready.')
    rospy.spin()
    if GPIO_available:
        GPIO.cleanup()


if __name__ == '__main__':
    sorter_service()
