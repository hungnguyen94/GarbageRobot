#!/usr/bin/env python
import rospy
import numpy
import copy
from std_msgs.msg import String
from sensor_msgs.msg import Range


publisher = None;
amount = 5
queues ={
    '/sonar_rv':[0]*amount,
    '/sonar_rm':[0]*amount,
    '/sonar_rav':[0]*amount,
    '/sonar_raa':[0]*amount,
    '/sonar_laa':[0]*amount,
    '/sonar_lav':[0]*amount,
    '/sonar_lm':[0]*amount,
    '/sonar_lv':[0]*amount
}
indices ={
    '/sonar_rv':0,
    '/sonar_rm':0,
    '/sonar_rav':0,
    '/sonar_raa':0,
    '/sonar_laa':0,
    '/sonar_lav':0,
    '/sonar_lm':0,
    '/sonar_lv':0
}



def medianFilter(data) :
    q = queues[data.header.frame_id]
    i = indices[data.header.frame_id]
    q[i] = data.range
    msg = copy.copy(data)
    msg.range = numpy.median(q)
    publisher.publish(msg)
    indices[data.header.frame_id] = (i + 1)%amount


def listener():
    rospy.init_node('sonar_filter', anonymous=True)
    rospy.Subscriber("range_data", Range, medianFilter)
    global publisher
    publisher = rospy.Publisher("sonar_data", Range, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()
