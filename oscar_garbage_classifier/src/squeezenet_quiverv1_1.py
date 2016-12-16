#!/usr/bin/env python

import os
from quiver_engine import server
from squeezenetv1_1 import SqueezeNet

classes = [ 'coca_cola_bottles', 'fanta_bottle', 'cola_cans', 'fanta_cans', 'paper_coffee_cups', 'water_bottles']

input_shape = (33, 33, 3)
weights_file = './weights_v1.1.h5'
model = SqueezeNet(len(classes), input_shape=input_shape)
model.output
model.compile(loss="categorical_crossentropy", optimizer='adadelta', metrics=['accuracy'])
if os.path.isfile(weights_file):
    print('Loading weights: %s' % weights_file)
    model.load_weights(weights_file, by_name=True)
server.launch(model=model, classes=classes, temp_folder='/tmp/quiver', input_folder='/mnt/data/Development/ros/catkin_ws/images')
