#!/usr/bin/env python

from squeezenet import get_squeezenet
from keras.datasets import mnist
from keras.utils.np_utils import to_categorical
import numpy as np
import os
import cv2
import random

images_dir = '/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/src/training/cleaned_images'
weights_file = './weights.h5'
nb_epoch = 1
batch_size = 64

classification_dict = {
    'coca_cola_bottles': 0,
    'fanta_bottle': 1,
    'cola_cans': 2,
    'fanta_cans': 3,
    'paper_coffee_cups': 4,
    'water_bottles': 5,
}


def load_image(img_path):
    # Load image with 3 channel colors
    # img = cv2.imread(img_path, flags=1)
    img = img_path
    # print img.shape

    # Crop image to a square
    height, width = img.shape
    offset = int(round(min(height, width) / 2.0))
    center_x = int(round(width / 2.0))
    center_y = int(round(height / 2.0))
    cropped_img = img[center_y - offset: center_y + offset, center_x - offset: center_x + offset]

    # Resize image to 227, 227 as Squeezenet only accepts this format.
    resized_image = cv2.resize(cropped_img, (227, 227)).astype(np.float32)
    # resized_image = np.expand_dims(resized_image, axis=0)
    return resized_image

# List comprehensions. Create a list of two tuples with (images, class).
print('Loading images..')
# images_classes = [(load_image(os.path.join(subdir, f)), [classification_dict[os.path.basename(subdir)]])
#                   for subdir, dirs, files in os.walk(images_dir)
#                   for f in files]

# Unzip to two lists.
# images, classes = zip(*images_classes)
[images, classes], [x, y] = mnist.load_data()
images = images[0:1000]
classes = classes[0:1000]

# images = [load_image(im) for im in images]
images = np.array([cv2.resize(cv2.cvtColor(im, cv2.COLOR_GRAY2RGB), (227, 227)) for im in images])
nr_classes = 10 #len(classification_dict)
# images = np.array(images)
# print images.shape
classes = to_categorical(classes, nb_classes=nr_classes)



print('Loading model..')
model = get_squeezenet(nr_classes, dim_ordering='tf')
model.compile(loss="categorical_crossentropy", optimizer='adam', metrics=['accuracy'])
if os.path.isfile(weights_file):
    print('Loading weights')
    model.load_weights(weights_file, by_name=True)

print('Fitting model')
model.fit(images, classes, batch_size=batch_size, nb_epoch=nb_epoch, shuffle=True, verbose=2)
# model.fit_generator(images_classes, batch_size, nb_epoch=nb_epoch, verbose=2)
print("Finished fitting model")

print('Saving weights')
model.save_weights(weights_file, overwrite=True)
print('Evaluating model')
score = model.evaluate(images, classes, verbose=1)
print('result: ')
print(score[0])
print(score[1])
