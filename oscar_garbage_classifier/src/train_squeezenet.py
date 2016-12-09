#!/usr/bin/env python

from squeezenet import get_squeezenet
import numpy as np
import os
import cv2

images_dir = '/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/src/training/images'
nb_epoch = 1
batch_size = 100


def load_image(img_path):
    # Load image with 3 channel colors
    img = cv2.imread(img_path, flags=1)

    # Crop image to a square
    height, width, depth = img.shape
    offset = min(height, width) / 2.0
    center_x, center_y = (width / 2.0, height / 2.0)
    cropped_img = img[center_y - offset: center_y + offset, center_x - offset: center_x + offset]

    # Resize image to 227, 227 as Squeezenet only accepts this format.
    resized_image = cv2.resize(cropped_img, (227, 227)).astype(np.float64)
    return resized_image

# List comprehensions. Create a list of two tuples with (images, class).
images_classes = [(load_image(os.path.join(subdir, f)), os.path.basename(subdir))
                  for subdir, dirs, files in os.walk(images_dir)
                  for f in files]
# Unzip to two lists.
images, classes = zip(*images_classes)
nr_classes = len(set(classes))
print('Loaded images..')

model = get_squeezenet(nr_classes, dim_ordering='tf')
model.compile(loss="categorical_crossentropy", optimizer="adam")
print("Fitting model")
model.fit(images, classes, batch_size=batch_size, nb_epoch=nb_epoch)
print("Finished fitting model")

print("Evaluating model")
score = model.evaluate(images, classes, verbose=1, show_accuracy=True)
print('result: ')
print(score)
