#!/usr/bin/env python

from __future__ import division
from squeezenetv1_1 import SqueezeNet
from keras.optimizers import Adam, RMSprop, Nadam
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import ModelCheckpoint, TensorBoard
from keras.metrics import top_k_categorical_accuracy
import keras.backend as K
import os
import random

# SqueezeNet for detecting whether the sorter is occupied

initial_epoch = 0
nb_epoch = 100
batch_size = 50
samples_per_epoch = 700
nb_val_samples = 80

input_shape = (100, 100, 3)
width = input_shape[0]
height = input_shape[1]
channels = input_shape[2]

training_dir = '../images/training_detector'
val_dir = '../images/validation_detector'
save_weights_file = '../models/squeezenet_detector_v1.1_weights_%s_epochs_%sx%s.h5' % (nb_epoch, width, height)
start_weights_file = '' #save_weights_file
weights_target = "../models/squeezenet_detector_weights_%sx%s.{epoch:02d}-loss_{val_loss:.5f}-acc_{val_acc:.5f}.h5" % (width, height)

classes = ['occupied', 'empty']
nb_classes = len(classes)

train_datagen = ImageDataGenerator(
        rescale=1./255,
        shear_range=0.20,
        width_shift_range=0.15,
        rotation_range=20.,
        height_shift_range=0.15,
        fill_mode='constant')

test_datagen = ImageDataGenerator(
        rescale=1./255,
        fill_mode='constant')

train_generator = train_datagen.flow_from_directory(training_dir,
                                                    target_size=(width, height),
                                                    batch_size=batch_size,
                                                    class_mode='categorical',
                                                    classes=classes)

def train_generator3():
    while True:
        yield train_generator.next()


print('train datagen class indices: \n%s' % train_generator.class_indices)

val_generator = test_datagen.flow_from_directory(val_dir,
                                                 target_size=(width, height),
                                                 batch_size=batch_size,
                                                 class_mode='categorical',
                                                 classes=classes)

def val_generator3():
    while True:
        yield val_generator.next()
        # yield val_generator2.next()

print('val datagen class indices: \n%s' % val_generator.class_indices)

def top_1_categorical_accuracy(y_true, y_pred):
    k = 1
    return top_k_categorical_accuracy(y_true, y_pred, k=k)

checkpoint = ModelCheckpoint(weights_target, monitor='val_loss',
                             verbose=1, save_best_only=True,
                             save_weights_only=False, mode='auto')

tensorboard = TensorBoard(log_dir='../logs', histogram_freq=5, write_graph=True, write_images=False)

print('Loading model..')
model = SqueezeNet(nb_classes, width, height, channels)
adam = Adam(lr=0.005)
rmsprop = RMSprop(lr=0.005)
model.compile(loss='categorical_crossentropy', optimizer='rmsprop', metrics=[top_1_categorical_accuracy, 'accuracy', 'precision', 'recall', 'categorical_crossentropy', 'binary_crossentropy'])
if os.path.isfile(start_weights_file):
        print('Loading weights: %s' % start_weights_file)
        model.load_weights(start_weights_file, by_name=True)

print('Fitting model')
model.fit_generator(train_generator3(),
                    samples_per_epoch=samples_per_epoch,
                    validation_data=val_generator3(),
                    callbacks=[checkpoint, tensorboard],
                    nb_val_samples=nb_val_samples,
                    nb_epoch=nb_epoch,
                    verbose=1,
                    initial_epoch=initial_epoch)
print("Finished fitting model")

print('Saving weights')
model.save_weights(save_weights_file, overwrite=True)
print('Done')
# print('Evaluating model')
#score = model.evaluate_generator(val_generator, val_samples=nb_val_samples)
#print('result: %s' % score)

