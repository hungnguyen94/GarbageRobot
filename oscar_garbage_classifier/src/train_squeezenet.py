#!/usr/bin/env python

from __future__ import division
from squeezenetv1_1 import SqueezeNet
from keras.optimizers import Adam, RMSprop, Nadam
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import ModelCheckpoint
import os

training_dir = '../images/training_webcam_cv2_resized'
training_dir2 = '../images/training_webcam_resized'
val_dir = '../images/validation_webcam_cv2_resized'
val_dir2 = '../images/validation_webcam_resized'
start_weights_file = ''
save_weights_file = '../models/squeezenet_webcam_v1.1_weights_30_epochs_80x80.h5'
weights_target = "../models/squeezenet_webcam_weights_80x80.{epoch:02d}-loss_{val_loss:.5f}-acc_{val_acc:.5f}.h5"

initial_epoch = 0
nb_epoch = 30
batch_size = 80
samples_per_epoch = 600
nb_val_samples = 60

input_shape = (224, 224, 3)
width = input_shape[0]
height = input_shape[1]
channels = input_shape[2]

classes = ['bottles', 'cans', 'cups', 'other']
# classes = ['water_bottles', 'coca_cola_bottles', 'fanta_bottles', 'cola_cans', 'fanta_cans', 'paper_coffee_cups']

nb_classes = len(classes)

train_datagen = ImageDataGenerator(
        rescale=1./255,
        shear_range=0.05,
        width_shift_range=0.05,
        rotation_range=1.,
        height_shift_range=0.05,
        fill_mode='constant')

test_datagen = ImageDataGenerator(
        rescale=1./255,
        fill_mode='constant')

train_generator = train_datagen.flow_from_directory(training_dir,
                                                    target_size=(width, height),
                                                    batch_size=batch_size,
                                                    class_mode='categorical',
                                                    classes=classes)
train_generator2 = train_datagen.flow_from_directory(training_dir2,
                                                     target_size=(width, height),
                                                     batch_size=batch_size,
                                                     class_mode='categorical',
                                                     classes=classes)
def train_generator3():
    while True:
        yield train_generator.next()
        yield train_generator2.next()


print('train datagen class indices: \n%s' % train_generator.class_indices)

val_generator = test_datagen.flow_from_directory(val_dir,
                                                 target_size=(width, height),
                                                 batch_size=batch_size,
                                                 class_mode='categorical',
                                                 classes=classes)

val_generator2 = test_datagen.flow_from_directory(val_dir2,
                                                 target_size=(width, height),
                                                 batch_size=batch_size,
                                                 class_mode='categorical',
                                                 classes=classes)
print('val datagen class indices: \n%s' % val_generator.class_indices)
def val_generator3():
    while True:
        yield val_generator.next()
        yield val_generator2.next()

checkpoint = ModelCheckpoint(weights_target, monitor='val_loss',
                             verbose=1, save_best_only=True,
                             save_weights_only=True, mode='auto')

print('Loading model..')
model = SqueezeNet(nb_classes, width, height, channels)
adam = Adam(lr=0.004)
rmsprop = RMSprop(lr=0.005)
model.compile(loss="categorical_crossentropy", optimizer=adam, metrics=['accuracy', 'categorical_crossentropy', 'binary_crossentropy'])
if os.path.isfile(start_weights_file):
        print('Loading weights: %s' % start_weights_file)
        model.load_weights(start_weights_file, by_name=True)

print('Fitting model')
model.fit_generator(train_generator3(),
                    samples_per_epoch=samples_per_epoch,
                    validation_data=val_generator3(),
                    callbacks=[checkpoint],
                    nb_val_samples=nb_val_samples,
                    nb_epoch=nb_epoch,
                    verbose=1,
                    initial_epoch=initial_epoch)
print("Finished fitting model")

print('Saving weights')
model.save_weights(save_weights_file, overwrite=True)
# print('Evaluating model')
#score = model.evaluate_generator(val_generator, val_samples=nb_val_samples)
#print('result: %s' % score)

