#!/usr/bin/env python

from __future__ import division
from squeezenet import SqueezeNet
from keras.utils.np_utils import to_categorical
from keras.optimizers import Adam, SGD
from keras.datasets import mnist
import numpy as np
import os
import cv2
import random
import uuid

images_dir = './images'
weights_file = './mnist_weights.h5'
initial_epoch = 0
nb_epoch = 10
batch_size = 64
validation_split = 0.2 
input_shape = (67, 67, 3)

nb_classes = 10


def load_image(img):
    # Load image with 3 channel colors
    # img = cv2.imread(img_path, flags=1)

    # Convert to rgb
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    # cv2.imwrite('/tmp/test/%s.jpg' % name, img)
    # Image needs to the resized to (227x227), but we want to maintain the aspect ratio.
    # height = img.shape[0]
    # width = img.shape[1]
    # offset = int(round(max(height, width) / 2.0))

    # # Add borders to the images.
    # padded_img = cv2.copyMakeBorder(img, offset, offset, offset, offset, cv2.BORDER_CONSTANT)
    # padded_height = padded_img.shape[0]
    # padded_width = padded_img.shape[1]
    # center_x = int(round(padded_width / 2.0))
    # center_y = int(round(padded_height / 2.0))
    # # Crop the square containing the full image.
    # cropped_img = padded_img[center_y - offset: center_y + offset, center_x - offset: center_x + offset]

    # Resize image to 227, 227 as Squeezenet only accepts this format.
    resized_image = cv2.resize(img, (input_shape[0], input_shape[1])).astype('float32')
    return resized_image

(X_train, y_train), (X_test, y_test) = mnist.load_data()

X_train = X_train.reshape(X_train.shape[0], 28, 28, 1)
X_test = X_test.reshape(X_test.shape[0], 28, 28, 1)

X_train = X_train.astype('float32')
X_test = X_test.astype('float32')
X_train /= 255
X_test /= 255
X_train = np.asarray([load_image(img) for img in X_train])
X_test = np.asarray([load_image(img) for img in X_test])

Y_train = to_categorical(y_train, nb_classes)
Y_test = to_categorical(y_test, nb_classes)

# classes = to_categorical(classes, nb_classes=nr_classes)

print('Loading model..')
model = SqueezeNet(nb_classes, input_shape=input_shape)
model.compile(loss="categorical_crossentropy", optimizer='adam', metrics=['accuracy'])
if os.path.isfile(weights_file):
    print('Loading weights: %s' % weights_file)
    model.load_weights(weights_file, by_name=True)

print('Fitting model')
model.fit(X_train, Y_train, batch_size=batch_size, nb_epoch=nb_epoch, verbose=1, validation_split=0.2, initial_epoch=0)
print("Finished fitting model")

print('Saving weights')
model.save_weights(weights_file, overwrite=True)
print('Evaluating model')

score = model.evaluate(X_test, Y_test, verbose=1)
print('result: %s' % score)

