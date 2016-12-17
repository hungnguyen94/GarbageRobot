#!/usr/bin/env python

from __future__ import division
from squeezenetv1_1 import SqueezeNet
from keras.optimizers import Adam
from keras.preprocessing.image import ImageDataGenerator
import numpy as np
import os

training_dir = './images'
val_dir = './validation'
weights_file = './weights_v1.1.h5'
initial_epoch = 0
nb_epoch = 1
batch_size = 64
samples_per_epoch = 2600
nb_val_samples = 300
input_shape = (227, 227, 3)
width = input_shape[0]
height = input_shape[1]

classes = ['coca_cola_bottles', 'fanta_bottle', 'cola_cans', 'fanta_cans', 'paper_coffee_cups', 'water_bottles']

nb_classes = len(classes)

train_datagen = ImageDataGenerator(
        rescale=1./255, 
        shear_range=0.2,
        width_shift_range=0.1,
        height_shift_range=0.1,
        fill_mode='constant')

test_datagen = ImageDataGenerator(
        rescale=1./255, 
        fill_mode='constant')

train_generator = train_datagen.flow_from_directory(training_dir, 
        target_size=(width, height),
        batch_size=batch_size,
        class_mode='categorical', 
        classes=classes)
print('train datagen class indices: \n%s' % train_generator.class_indices)

val_generator = test_datagen.flow_from_directory(val_dir, 
        target_size=(width, height),
        batch_size=batch_size,
        class_mode='categorical', 
        classes=classes)
print('val datagen class indices: \n%s' % val_generator.class_indices)

print('Loading model..')
model = SqueezeNet(nb_classes, input_shape=input_shape)
# adam = Adam(lr=0.005)
model.compile(loss="categorical_crossentropy", optimizer='adam', metrics=['accuracy', 'categorical_crossentropy'])
if os.path.isfile(weights_file):
    print('Loading weights: %s' % weights_file)
    model.load_weights(weights_file, by_name=True)

print('Fitting model')
model.fit_generator(train_generator, 
        samples_per_epoch=samples_per_epoch, 
        validation_data=val_generator, 
        nb_val_samples=nb_val_samples, 
        nb_epoch=nb_epoch, 
        verbose=1, 
        initial_epoch=initial_epoch)
print("Finished fitting model")

print('Saving weights')
model.save_weights(weights_file, overwrite=True)
# print('Evaluating model')
#score = model.evaluate_generator(val_generator, val_samples=nb_val_samples)
#print('result: %s' % score)

