#!/usr/bin/env python

import rospy
from oscar_garbage_classifier.srv import Sort
import sys


def usage():
    return "%s [category]\n " \
           "Category must be represented as an integer. " \
           "Mapping: { 0: cups, 1: pmd, 2: other }" % (sys.argv[0])

if __name__ == '__main__':
    if len(sys.argv) != 2 or int(sys.argv[1]) not in range(3):
        print(usage())
        sys.exit(1)

    category = int(sys.argv[1])
    rospy.wait_for_service('invoke_sorter')
    invoke_sorter = rospy.ServiceProxy('invoke_sorter', Sort)
    print(invoke_sorter(category))
