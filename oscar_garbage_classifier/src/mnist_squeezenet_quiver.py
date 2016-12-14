#!/usr/bin/env python

import os
from quiver_engine import server
from squeezenet import SqueezeNet

classes = ['Zero', 'One', 'Two', 'Three', 'Four', 'Five', 'Six', 'Seven', 'Eight', 'Nine']

input_shape = (67, 67, 3)
weights_file = './mnist_weights.h5'
model = SqueezeNet(len(classes), input_shape=(input_shape))
model.output
model.compile(loss="categorical_crossentropy", optimizer='adadelta', metrics=['accuracy'])
if os.path.isfile(weights_file):
    print('Loading weights: %s' % weights_file)
    model.load_weights(weights_file, by_name=True)
server.launch(model=model, classes=classes, temp_folder='/tmp/quiver', input_folder='/mnt/data/Development/ros/catkin_ws/images')
