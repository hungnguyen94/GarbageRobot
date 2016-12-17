#!/usr/bin/env python
# license removed for brevity
import rospy
from oscar.msg import gpio
import std_msgs.msg

# Name of the topic to publish the pin values to
topic="gpio_events"

def talker():
    pub = rospy.Publisher(topic, gpio, queue_size=10)
    rospy.init_node('gpio_reader', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    seq = 0;
    pinNumber = 10; #TODO change to right pin
    newPinValue = True; #TODO change to right value
    while not rospy.is_shutdown():
        pub.publish(seq, rospy.Time.now(),pinNumber,newPinValue)
        seq+=1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
