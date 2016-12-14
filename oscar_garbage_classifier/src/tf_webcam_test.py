#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import tensorflow as tf
from tensorflow.models.image.imagenet.classify_image import NodeLookup, FLAGS
import os.path
import numpy as np
import pygame
import pygame.camera as pycam
import time

FLAGS.model_dir = '/mnt/data/Development/ros/models'
num_top_predictions = 7
camera_index = 1
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

  pygame.init()
  pycam.init()
  target_camera = pycam.list_cameras()[camera_index]
  cam = pycam.Camera(target_camera, camera_resolution)
  cam.start()

  screen = pygame.display.set_mode(display_resolution)
  pygame.display.set_caption("Wall-E")
  myfont = pygame.font.SysFont("Comic Sans MS, Bold", 15)
  image = cam.get_image()
  pygame.display.get_surface().blit(pygame.transform.scale(image, display_resolution), (0, 0)) 

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
      time.sleep(0.1)
      cam.get_image(image)
      # Rotate image because resulting array has image rotated. 
      img_arr = pygame.surfarray.pixels3d(pygame.transform.rotate(image, 90))
      # image_data = pygame.image.tostring(image, "RGBA", True)
      # pil_img = Image.frombuffer("RGBA", (1280, 720), image_data)
      # img_arr = np.array(pil_img)[:, :, 0:3]

      # Blit image onto screen.
      screen.blit(pygame.transform.scale(image, display_resolution), (0, 0))

      predictions = sess.run(softmax_tensor, {'DecodeJpeg:0': img_arr})
      predictions = np.squeeze(predictions)

      top_k = predictions.argsort()[-num_top_predictions:][::-1]

      for i in xrange(num_top_predictions): 
        node_id = top_k[i]  
        human_string = node_lookup.id_to_string(node_id)
        score = predictions[node_id]
        result = '%.5f: %s' % (score, human_string)
        if score < 0.09: 
          break
        label = myfont.render(result, True, (255-20*i,255,20*i))
        screen.blit(label, (10, 10 + 20*i))

      # Refresh display
      pygame.display.flip()

      for event in pygame.event.get():
        if event.type == pygame.QUIT:
          running = False
          
  cam.stop()
  pygame.quit()

if __name__ == '__main__':
  run_inference()
