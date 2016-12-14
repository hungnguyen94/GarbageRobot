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

batch_size = 64
val_samples = 2600

input_shape = (33, 33, 3)
width = input_shape[0]
height = input_shape[1]

classes = ['coca_cola_bottles', 'fanta_bottle', 'cola_cans', 'fanta_cans', 'paper_coffee_cups', 'water_bottles']
class_map = {
        'coca_cola_bottles': 'bottle', 
        'fanta_bottle': 'bottle', 
        'cola_cans': 'cans', 
        'fanta_cans': 'cans', 
        'paper_coffee_cups': 'cups', 
        'water_bottles': 'bottle'
        }

nb_classes = len(classes)

train_datagen = ImageDataGenerator(
        rescale=1./255, 
        fill_mode='constant')

train_generator = train_datagen.flow_from_directory('./images', 
        target_size=(width, height),
        batch_size=batch_size,
        class_mode='categorical', 
        classes=classes)
print('train datagen class indices: \n%s' % train_generator.class_indices)

print('Loading model..')
model = SqueezeNet(nb_classes, input_shape=input_shape)
# adam = Adam(lr=0.005)
model.compile(loss="categorical_crossentropy", optimizer='adam', metrics=['accuracy', 'categorical_crossentropy'])
if os.path.isfile(weights_file):
    print('Loading weights: %s' % weights_file)
    model.load_weights(weights_file, by_name=True)

print('Predicting model')
# results = model.predict_generator(train_generator, val_samples=val_samples)
correct = 0
total = 0
for batch in train_generator: 
    imgs, labels = batch
    results = model.predict(imgs)
    for i in xrange(len(imgs)):
        result = results[i]
        real_index = labels[i].argsort()[-1:][0]
        real_label = classes[real_index]
        top_index = result.argsort()[-1:][0]
        predicted_label = classes[top_index]
        confidence = result[top_index]
        predicted_label = class_map[predicted_label]
        real_label = class_map[real_label]
        print('Predicted label: %0.2f \t%s\n     Real label: \t%s' % (confidence, predicted_label, real_label))
        if real_label == predicted_label: 
            correct += 1
        total += 1
        print('Corrent: %s/%s' % (correct, total))

results = 1
print('results: %s' % results)

