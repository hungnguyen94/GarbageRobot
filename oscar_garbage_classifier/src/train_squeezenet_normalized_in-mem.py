#!/usr/bin/env python

from __future__ import division
from squeezenetv1_1_res import SqueezeNet
from keras.optimizers import Adam, RMSprop, Nadam
from keras.preprocessing.image import ImageDataGenerator, load_img, img_to_array
from keras.callbacks import ModelCheckpoint, TensorBoard
from keras.metrics import top_k_categorical_accuracy
from keras.models import load_model
from keras.utils.np_utils import to_categorical
import keras.backend as K
import os
import numpy as np


initial_epoch = 00
nb_epoch = 100
batch_size = 64
samples_per_epoch = 1706
nb_val_samples = 359

input_shape = (224, 224, 3)
width = input_shape[0]
height = input_shape[1]
channels = input_shape[2]

training_dir = '../images/3feb-training-3feb'
val_dir = '../images/3feb-validation-3feb'
save_weights_file = '../models/mean_centered-12feb/squeezenet_v1.1_12feb-res_weights_%s_epochs_%sx%s.h5' % (nb_epoch, width, height)
start_weights_file = ''#save_weights_file
weights_target = "../models/mean_centered-12feb/squeezenet-res-12feb-_webcam_weights_%sx%s.{epoch:03d}-loss_{val_loss:.5f}-acc_{val_acc:.5f}.h5" % (width, height)

classes = ['bottles', 'cans', 'cups', 'cups_wrong', 'other']
nb_classes = len(classes)

# should be 224x224x3
images_mean = img_to_array(load_img('../meanpil.jpg', target_size=(width, height))) / 255.
images_std = img_to_array(load_img('../stdpil.jpg', target_size=(width, height))) / 255.


def process_image(im_path):
    img = img_to_array(load_img(im_path, target_size=(width, height))) / 255.
    img -= images_mean
    #  img /= (images_std + 1e-7)
    return img

def load_images(dir):
    image_paths, image_classes = zip(*[(os.path.join(folder, f), os.path.basename(folder))
                                       for folder, subf, files in os.walk(dir)
                                       for f in files])
    x = np.asarray([process_image(im) for im in image_paths])
    y = to_categorical([classes.index(clazz) for clazz in image_classes], nb_classes)
    return x, y

X_train, Y_train = load_images(training_dir)
X_val, Y_val = load_images(val_dir)


train_datagen = ImageDataGenerator(
        shear_range=0.15,
        width_shift_range=0.05,
        rotation_range=1.,
        height_shift_range=0.05,
        horizontal_flip=False,
        vertical_flip=True,
        # featurewise_center=True,
        # featurewise_std_normalization=True,
        fill_mode='constant')

test_datagen = ImageDataGenerator(
        fill_mode='constant',
        # featurewise_center=True,
        # featurewise_std_normalization=True
)

# train_datagen.fit(X_train)
# test_datagen.fit(X_train)

train_generator = train_datagen.flow(X_train, Y_train, batch_size=batch_size)
val_generator = test_datagen.flow(X_val, Y_val, batch_size=batch_size)



checkpoint = ModelCheckpoint(weights_target, monitor='val_acc',
                             verbose=1, save_best_only=True,
                             save_weights_only=True, mode='auto')

tensorboard = TensorBoard(log_dir='../logs/normalized-mean_12feb', histogram_freq=3, write_graph=True, write_images=False)

print('Loading model..')
model = SqueezeNet(nb_classes, width, height, channels)
adam = Adam(lr=0.005, decay=0.00004)
rmsprop = RMSprop(lr=0.005)
model.compile(loss='categorical_crossentropy', optimizer=adam, metrics=['accuracy', 'precision', 'recall', 'categorical_crossentropy', 'binary_crossentropy'])
if os.path.isfile(start_weights_file):
        print('Loading weights: %s' % start_weights_file)
        model.load_weights(start_weights_file, by_name=True)
        # model = load_model(start_weights_file)

print('Fitting model')
model.fit_generator(train_generator,
                    samples_per_epoch=samples_per_epoch,
                    validation_data=val_generator,
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

