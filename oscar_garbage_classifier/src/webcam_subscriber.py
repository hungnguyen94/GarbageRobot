#!/usr/bin/env python

from __future__ import print_function
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class image_subscriber: 

  def __init__(self):
    self.sub = rospy.Subscriber("image_topic", Image, self.callback, queue_size=1)
    self.cv_bridge = CvBridge()

  def callback(self, img): 
    try:
      cv_image = self.cv_bridge.imgmsg_to_cv2(img)
      b, g, r = cv2.split(cv_image)
      frame = cv2.merge([r, g, b])
      frame = cv2.fastNlMeansDenoisingColored(frame, h=5, hColor=7, templateWindowSize=5, searchWindowSize=7)

      cv2.imshow("Wall-A", frame)
      cv2.waitKey(1)
    except CvBridgeError as e:
      print(e)


def main(args): 
  img_sub = image_subscriber()
  rospy.init_node("image_subscriber", anonymous=True)
  try: 
    rospy.spin()
  except rospy.ROSInterruptException(), KeyboardInterrupt: 
    print("exiting image_subscriber")
  cv2.destroyAllWindows()


if __name__ == "__main__": 
  try: 
    main(sys.argv)
  except rospy.ROSInterruptException(): 
    pass
