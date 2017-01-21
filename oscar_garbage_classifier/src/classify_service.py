#!/usr/bin/env python

from __future__ import division, print_function
from squeezenetv1_1 import SqueezeNet
import tensorflow as tf
import keras.backend as K
from oscar_garbage_classifier.srv import ClassifyImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import rospy
import copy
import os

weights = rospy.get_param("squeezenet_classifier_weightsfile",
                          os.path.dirname(os.path.abspath(__file__)) + '/../models/squeezenet_webcam_weights_300x300.103-loss_0.00108-acc_1.00000.h5')
classes = rospy.get_param('classifier_classes', ['bottles', 'cans', 'cups', 'other'])
categories = ['cups', 'pmd', 'other']
class_to_category_index = {0: 1, # Bottles to pmd
                           1: 1, # cans to pmd
                           2: 0, # cups to cups
                           3: 2} # other to other

input_width = rospy.get_param('classifier_image_width', 300)
input_height = rospy.get_param('classifier_image_height', 300)

sq_graph = tf.Graph()
sq_sess = tf.Session(graph=sq_graph)
K.set_session(sq_sess)

squeezenet = None
rotation_matrix = cv2.getRotationMatrix2D((input_width/2, input_height/2), 90, 1)


def load_squeezenet():
    """
    Load SqueezeNet in sq_graph. Loads the weights from the weights variable.
    The SqueezeNet model is loaded in the squeezenet global variable.

    :return: SqueezeNet Model
    """
    with sq_graph.as_default():
        global squeezenet
        squeezenet = SqueezeNet(len(classes), input_width, input_height, 3)
        squeezenet.load_weights(weights, by_name=True)
        return squeezenet


def classify(img):
    """
    Classify the image using SqueezeNet v1.1 convNet.
    The convNet is trained to output 4 classes
    Images are classified in 3 categories: bottles, cans and cups.

    :param img: Input image with shape (1, 227, 227, 3)
    :return: Integer representing 1 of 3 classes.
    """
    with sq_graph.as_default():
        if squeezenet is None:
            load_squeezenet()
        result = squeezenet.predict(img)[0]

    top = result.argsort()[-1::][0]
    rospy.loginfo('Top result of %d classes: %s with confidence %s' % (len(classes), classes[top], result[top]))

    return top


def preprocess_image(img):
    """
    Preprocess the image before passing it to SqueezeNet.
    Resize to (227, 227, 3) and maintain aspect ratio by adding black borders.
    Add an extra dimension because SqueezeNet expects an array of images as input.

    :param img: RGB image with shape (height, width, 3)
    :return: Numpy array containing image with shape (1, 227, 227, 3)
    """

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

    # Resize image to 300, 300 as Squeezenet only accepts this format.
    resized_image = cv2.resize(cropped_img, (input_width, input_height)).astype('float32')
    resized_image /= 255
    
    # Rotate image 90 degrees
    image = cv2.warpAffine(resized_image, rotation_matrix, (input_width, input_height))
    aux = copy.copy(image)
    image[:, :, 0] = aux[:, :, 2]
    image[:, :, 2] = aux[:, :, 0]

    image = np.expand_dims(image, axis=0)
    return image


def handle_service(request):
    """
    Handle the request to classify an image. Converts the requested image to a cv2 image.
    Preprocesses the image and runs it through the model.
    :param request: ClassifyImage request.
    :return: An integer representing one of the classes.
    """

    img = CvBridge().imgmsg_to_cv2(request.msg)
    img = preprocess_image(img)
    result = classify(img)
    category = categories[result]
    rospy.loginfo('Classified as %s' % classes[result])
    rospy.loginfo('Category is %s' % categories[category])
    publish_result(result)
    return category


def image_classify_service():
    rospy.init_node('image_classify_service')
    s = rospy.Service('image_classify', ClassifyImage, handle_service)
    rospy.loginfo('Image classify service ready.')
    rospy.spin()


if __name__ == '__main__':
    if squeezenet is None:
        load_squeezenet()
    image_classify_service()

