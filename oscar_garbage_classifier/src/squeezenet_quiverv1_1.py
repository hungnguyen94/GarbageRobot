#!/usr/bin/env python

import os
from quiver_engine import server
from squeezenetv1_1 import SqueezeNet

classes = ['bottles', 'cans', 'cups', 'other']

input_shape = (67, 67, 3)
# weights_file = '/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/models/squeezenet_webcam_weights.50-0.98438.h5'
weights_file = '/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/models/squeezenet_webcam_weights_67x67.83-loss_0.09716-acc_0.98438.h5'
model = SqueezeNet(len(classes), input_shape[0], input_shape[1], input_shape[2])
model.compile(loss="categorical_crossentropy", optimizer='adam', metrics=['accuracy'])
if os.path.isfile(weights_file):
    print('Loading weights: %s' % weights_file)
    model.load_weights(weights_file, by_name=True)
server.launch(model=model, classes=classes, temp_folder='/tmp/quiver', input_folder='/mnt/data/Development/ros/catkin_ws/images')
