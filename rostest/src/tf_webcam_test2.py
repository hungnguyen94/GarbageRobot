#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import tensorflow as tf
from tensorflow.models.image.imagenet.classify_image import NodeLookup, FLAGS
import os.path
import numpy as np
import time
import cv2

FLAGS.model_dir = '/mnt/data/Development/ros/models'
num_top_predictions = 7
camera_index = 0
camera_resolution = (1280, 720)
display_resolution = (640, 360)

def run_inference():
  """Runs inference on a webcam stream.
  Returns:
    Nothing
  """

  """Creates a graph from saved GraphDef file and returns a saver."""
  # Creates graph from saved graph_def.pb.
  with tf.gfile.FastGFile(os.path.join(FLAGS.model_dir, 'classify_image_graph_def.pb'), 'rb') as f:
    graph_def = tf.GraphDef()
    graph_def.ParseFromString(f.read())
    _ = tf.import_graph_def(graph_def, name='')

  # Creates node ID --> English string lookup.
  node_lookup = NodeLookup()

  cam = cv2.VideoCapture(camera_index)
  font = cv2.FONT_HERSHEY_COMPLEX_SMALL

  with tf.Session() as sess:
    # Some useful tensors:
    # 'softmax:0': A tensor containing the normalized prediction across
    #   1000 labels.
    # 'pool_3:0': A tensor containing the next-to-last layer containing 2048
    #   float description of the image.
    # 'DecodeJpeg/contents:0': A tensor containing a string providing JPEG
    #   encoding of the image.
    # Runs the softmax tensor by feeding the image_data as input to the graph.
    softmax_tensor = sess.graph.get_tensor_by_name('softmax:0')

    running = True
    while running: 
      ret, frame = cam.read()
      if not ret: 
        continue

      predictions = sess.run(softmax_tensor, feed_dict={'DecodeJpeg:0': frame})
      predictions = np.squeeze(predictions)

      top_k = predictions.argsort()[-num_top_predictions:][::-1]

      for i in xrange(num_top_predictions): 
        node_id = top_k[i]  
        human_string = node_lookup.id_to_string(node_id)
        score = predictions[node_id]
        result = '%.5f: %s' % (score, human_string)
        if score < 0.09: 
          break
        cv2.putText(frame, result, (20, 50+20*i), font, 1.0, (255,150,0),2)

      if cv2.waitKey(1) & 0xFF == ord('q'):
        break

      cv2.imshow('Video', frame)  

  cam.release()
  cv2.destroyAllWindows()
          

if __name__ == '__main__':
  run_inference()
