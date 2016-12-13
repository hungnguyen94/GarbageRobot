#!/usr/bin/env python

from __future__ import division
from squeezenet import SqueezeNet
from keras.utils.np_utils import to_categorical
from keras.optimizers import Adam
from keras.preprocessing.image import ImageDataGenerator
import numpy as np
import os
import cv2
import random

images_dir = './images'
weights_file = './weights.h5'
initial_epoch = 0
nb_epoch = 1
batch_size = 64
validation_split = 0.2 

class_mapping = {
    'coca_cola_bottles': 0,
    'fanta_bottle': 0,
    'cola_cans': 1, 
    'fanta_cans': 1,
    'paper_coffee_cups': 2,
    'water_bottles': 0
}
class_to_name = ['bottle', 'cans', 'coffee cup']

nb_classes = len(class_to_name)

def load_image(img_path):
    # Load image with 3 channel colors
    img = cv2.imread(img_path, flags=1)

    # Image needs to the resized to (227x227), but we want to maintain the aspect ratio.
    height = img.shape[0]
    width = img.shape[1]
    offset = int(round(max(height, width) / 2.0))

    # Add borders to the images.
    padded_img = cv2.copyMakeBorder(img, offset, offset, offset, offset, cv2.BORDER_CONSTANT)
    padded_height = padded_img.shape[0]
    padded_width = padded_img.shape[1]
    center_x = int(round(padded_width / 2.0))
    center_y = int(round(padded_height / 2.0))
    # Crop the square containing the full image.
    cropped_img = padded_img[center_y - offset: center_y + offset, center_x - offset: center_x + offset]

    # Resize image to 227, 227 as Squeezenet only accepts this format.
    resized_image = cv2.resize(cropped_img, (227, 227)).astype(np.float64)
    resized_image /= 255
    return resized_image

# List comprehension returns list of tuples (image_path, classification)
imgpaths_classes = [ (load_image(os.path.join(subdir, f)), class_mapping[os.path.basename(subdir)]) 
                        for subdir, dirs, files in os.walk(images_dir) 
                            for f in files if f.endswith('.jpg')][0:50]
# Randomize it. 
random.shuffle(imgpaths_classes)

images, classes = zip(*imgpaths_classes)
classes = to_categorical(classes, nb_classes=nb_classes)
images = np.asarray(images)

print('Loading model..')
model = SqueezeNet(nb_classes, input_shape=(227, 227, 3))
model.compile(loss="categorical_crossentropy", optimizer='adam', metrics=['accuracy', 'categorical_crossentropy'])
if os.path.isfile(weights_file):
    print('Loading weights: %s' % weights_file)
    model.load_weights(weights_file, by_name=True)

print('Fitting model')
model.fit(images, classes, batch_size=batch_size, nb_epoch=nb_epoch, verbose=1, validation_split=0.2, initial_epoch=0)
print("Finished fitting model")

print('Saving weights')
model.save_weights(weights_file, overwrite=True)
print('Evaluating model')
score = model.evaluate(images, classes, verbose=1)
print('result: %s' % score)

