#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

from squeezenetv1_1 import SqueezeNet
import numpy as np
import tensorflow as tf
import cv2

camera_index = 1
camera_resolution = (1280, 720)

weights_file = '/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/models/squeezenet_webcam_weights.12-0.09206-0.97959.h5'
input_shape = (100, 100, 3)
classes = ['bottles', 'cans', 'cups', 'other']

def run_inference():
    """Runs inference on a webcam stream.
    Returns:
      Nothing
    """
    cam = cv2.VideoCapture(camera_index)
    cam.set(3, camera_resolution[0])
    cam.set(4, camera_resolution[1])
    graph = tf.Graph()

    with graph.as_default():
        model = SqueezeNet(len(classes), input_shape[0], input_shape[1], input_shape[2])
        model.load_weights(weights_file, by_name=True)

        running = True
        while running:
            ret, frame = cam.read()
            if not ret:
                continue

            height = frame.shape[0]
            width = frame.shape[1]
            offset = int(round(max(height, width) / 2.0))

            # Add borders to the images.
            padded_img = cv2.copyMakeBorder(frame, offset, offset, offset, offset, cv2.BORDER_CONSTANT)
            padded_height = padded_img.shape[0]
            padded_width = padded_img.shape[1]
            center_x = int(round(padded_width / 2.0))
            center_y = int(round(padded_height / 2.0))
            # Crop the square containing the full image.
            cropped_frame = padded_img[center_y - offset: center_y + offset, center_x - offset: center_x + offset]
            cv2.imwrite('/tmp/test.jpg', cropped_frame)

            resized_frame = cv2.resize(cv2.imread('/tmp/test.jpg'), (input_shape[0], input_shape[1]))
            # image = frame.astype('float')
            image = resized_frame.astype(np.float32)
            image = np.expand_dims(image, axis=0)
            res = model.predict(image)

            results = res[0]

            for i in xrange(len(results)):
                clazz = classes[i]
                text = '%.9f: %s' % (results[i], clazz)
                print(text)
                cv2.putText(cropped_frame, text, (20, 50+20*i), cv2.FONT_HERSHEY_PLAIN, 0.75, (255, 150, 0), 2)
            print("")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                running = False
                break

            cv2.imshow('Video', frame)

    cam.release()
    cv2.destroyAllWindows()
          

if __name__ == '__main__':
  run_inference()
