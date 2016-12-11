#!/usr/bin/env python

from squeezenet import get_squeezenet
from keras.utils.np_utils import to_categorical
import numpy as np
import os
import cv2

images_dir = '/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/src/training/cleaned_images'
weights_file = './weights.h5'
# weights_file = '/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/models/squeezenet_weights_tf_dim_ordering_tf_kernels.h5'
nb_epoch = 1
batch_size = 32

decode = ['coca_cola_bottles', 'fanta_bottle', 'cola_cans', 'fanta_cans', 'paper_coffee_cups', 'water_bottles']
# decode = []
# with open('/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/src/classes.txt', 'r') as list_:
    # for line in list_:
        # decode.append(line.rstrip('\n'))

def load_image(img_path):
    # Load image with 3 channel colors
    img = cv2.imread(img_path, flags=1)

    # Crop image to a square
    height, width, depth = img.shape
    offset = int(round(min(height, width) / 2.0))
    center_x = int(round(width / 2.0))
    center_y = int(round(height / 2.0))
    cropped_img = img[center_y - offset: center_y + offset, center_x - offset: center_x + offset]

    # Resize image to (227, 227), as Squeezenet only accepts this format.
    resized_image = cv2.resize(cropped_img, (227, 227)) #.astype(np.float32)
    return resized_image

# List comprehensions. Create a list of two tuples with (images, class).
print('Loading images..')
paths = [os.path.join(subdir, f)
                  for subdir, dirs, files in os.walk(images_dir)
                  for f in files]

images = [load_image(path) for path in paths]
nr_classes = len(decode)
images = np.array(images)

print('Loading model..')
model = get_squeezenet(nr_classes, dim_ordering='tf')
model.compile(loss="categorical_crossentropy", optimizer="adam")
if os.path.isfile(weights_file):
    print('Loading weights...')
    model.load_weights(weights_file)

print("Classifying images...")
# predictions = model.predict(images, batch_size=100, verbose=1)
# print('Predicted %s images' % len(predictions))
for i in xrange(len(images)):
    img = np.expand_dims(images[i], axis=0)
    res = model.predict(img)
    results = res[0].argsort()[-5:][::-1]
    print('%s: ' % paths[i])
    for j in xrange(len(results)):
        result = decode[results[j]]
        text = '%.3f: %s' % (res[0][results[j]], result)
        print(text)

    # confidences = predictions[i].argsort()[-5:][::-1]
    # result_classes = [(decode_dict[c]) for c in confidences]
    # prediction = model.predict(img)
    # print('%s. prediction: %s' % (i, prediction))
    # result_classes = [(decode[j], prediction[j]) for j in xrange(len(prediction))]
    # path = paths[i]
    # print('%s: %s ' % (path, result_classes))


