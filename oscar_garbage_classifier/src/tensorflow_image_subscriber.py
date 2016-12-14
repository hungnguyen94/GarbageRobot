#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys
import cv2
import os.path
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
from tensorflow.models.image.imagenet.classify_image import NodeLookup, FLAGS
import numpy as np
from random import randrange


class tf_image_subscriber: 

  def __init__(self):
    # Creates graph from saved graph_def.pb.
    with tf.gfile.FastGFile(os.path.join(FLAGS.model_dir, 'classify_image_graph_def.pb'), 'rb') as f:
      graph_def = tf.GraphDef()
      graph_def.ParseFromString(f.read())
      _ = tf.import_graph_def(graph_def, name='')

    # Create tensorflow session
    self.tf_sess = tf.Session()
    # 'softmax:0': A tensor containing the normalized prediction across 1000 labels.
    self.softmax_tensor = self.tf_sess.graph.get_tensor_by_name('softmax:0')
    # Creates node ID --> English string lookup.
    self.node_lookup = NodeLookup()

    self.sub = rospy.Subscriber("image_topic", Image, self.callback, queue_size = 1)
    self.cv_bridge = CvBridge()


  def callback(self, img): 
    try:
      # Prevent congestion
      if randrange(100) > 5: 
        return
      # Convert image from ros imgmsg to opencv2 image. 
      frame = self.cv_bridge.imgmsg_to_cv2(img)

      # Run tensorflow session on frame. 
      predictions = self.tf_sess.run(self.softmax_tensor, {'DecodeJpeg:0': frame})
      predictions = np.squeeze(predictions)
      
      # Only get the top k predictions. 
      top_k = predictions.argsort()[-FLAGS.num_top_predictions:][::-1]

      # Lookup the human readable classification and draw it on the screen. 
      for i in xrange(len(top_k)): 
        node_id = top_k[i]  
        human_string = self.node_lookup.id_to_string(node_id)
        score = predictions[node_id]
        result = '%.5f: %s' % (score, human_string)
        # Don't output predictions with a confidence lower than this. 
        if score < 0.09: 
          break
        cv2.putText(frame, result, (20, 50+20*i), cv2.FONT_HERSHEY_PLAIN, 1.0, (255,150,0),2)
        # if i == 0: 
          # print("%s\n" % human_string)

      # Display the image. 
      cv2.imshow("Wall-E", frame)
      cv2.waitKey(1)
    except CvBridgeError as e:
      print(e)

  def on_close(self): 
    self.tf_sess.close()
    cv2.destroyAllWindows()


def main(args): 
  FLAGS.model_dir = '/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/models' # args[0]
  img_sub = tf_image_subscriber()
  rospy.init_node("tf_image_subscriber", anonymous=True)
  try: 
    rospy.spin()
  except rospy.ROSInterruptException(), KeyboardInterrupt: 
    print("exiting image_subscriber")
  cv2.destroyAllWindows()


if __name__ == "__main__": 
  main(sys.argv)
