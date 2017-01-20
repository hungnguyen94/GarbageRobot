#!/usr/bin/env python

from __future__ import division
import numpy as np
import rospy
import cv2
import tensorflow as tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from squeezenetv1_1 import SqueezeNet
import keras.backend as K
import classify_client
import copy

class ClassifierSubscriber:
    def __init__(self):
        self.weights_file = '/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/models/squeezenet_webcam_weights_300x300.103-loss_0.00108-acc_1.00000.h5'
        self.detector_weights_file = '/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/models/squeezenet_detector_weights_100x100.50-loss_0.00011-acc_1.00000.h5'
        self.detect_input_shape = (100, 100, 3)
        self.model = None
        self.cv_bridge = CvBridge()
        # self.classes = ['bottles', 'cans', 'cups', 'other']
        self.detector_classes = ['empty', 'occupied']

        self.detect_graph = tf.Graph()
        self.detect_sess = tf.Session(graph=self.detect_graph)

        self.rotation_matrix = cv2.getRotationMatrix2D((self.detect_input_shape[0]/2, self.detect_input_shape[1]/2), 90, 1)

        K.set_session(self.detect_sess)
        with self.detect_graph.as_default():
            self.model = SqueezeNet(2, self.detect_input_shape[0], self.detect_input_shape[1], self.detect_input_shape[2])
            self.model.load_weights(self.detector_weights_file, by_name=True)

        self.sub = rospy.Subscriber("image_topic", Image, self.classify)


    def classify(self, img):
        """
        Callback function that classifies the image.
        :param img:  Image to be classified
        :return:  Class of the image.
        """

        # frame = misc.imread('/mnt/data/Development/keras-squeezenet/images/cat.jpeg', mode='RGB')
        frame = self.cv_bridge.imgmsg_to_cv2(img) #, desired_encoding='8UC3')

        height = frame.shape[0]
        width = frame.shape[1]
        offset = int(round(max(height, width) / 2.0))

        # Add borders to the images.
        padded_img = cv2.copyMakeBorder(frame, offset, offset, offset, offset, cv2.BORDER_CONSTANT)
        padded_height = padded_img.shape[0]
        padded_width = padded_img.shape[1]
        center_x = int(round(padded_width / 2.0))
        center_y = int(round(padded_height / 2.0))
        # Crop the square containing the full image.
        cropped_frame = padded_img[center_y - offset: center_y + offset, center_x - offset: center_x + offset]
        resized_frame = cv2.resize(cropped_frame, (self.detect_input_shape[0], self.detect_input_shape[1]))

        rotated_frame = cv2.warpAffine(resized_frame, self.rotation_matrix, (self.detect_input_shape[0], self.detect_input_shape[1]))
        fr = rotated_frame
        image = fr.astype(np.float64) / float(255)
        # image /= 255.

        # Change BGR to RGB
        aux = copy.copy(image)
        image[:, :, 0] = aux[:, :, 2]
        image[:, :, 2] = aux[:, :, 0]

        image = np.expand_dims(image, axis=0)

        with self.detect_graph.as_default():
            res = self.model.predict(image)

        results = res[0]

        if results[1] > 0.50:
            # Img is rotated and resized in the service.
            class_result = classify_client.classify_image(img)
            print(class_result)
            # classify_client.invoke_sorter(classify_client.classes[class_result])
            # print("hoi")

        for i in xrange(len(results)):
            clazz = self.detector_classes[i]
            text = '%.9f: %s' % (results[i], clazz)
            print(text)
        print("")



if __name__ == '__main__':

    print("")
    img_sub = ClassifierSubscriber()
    rospy.init_node("classifier_subscriber", anonymous=True)

    try:
        rospy.spin()
    except rospy.ROSInterruptException(), KeyboardInterrupt:
        print("exiting classifier_subscriber")
    cv2.destroyAllWindows()

    # model = get_squeezenet(1000, dim_ordering='tf')
    # model.compile(loss="categorical_crossentropy", optimizer="adam")
    # model.load_weights('../model/squeezenet_weights_tf_dim_ordering_tf_kernels.h5', by_name=True)
    #
    #
    # start = time.time()
    # im = misc.imread('../images/cat.jpeg')
    #
    # im = misc.imresize(im, (227, 227)).astype(np.float32)
    # aux = copy.copy(im)
    # im[:, :, 0] = aux[:, :, 2]
    # im[:, :, 2] = aux[:, :, 0]
    #
    # # Remove image mean
    # im[:, :, 0] -= 104.006
    # im[:, :, 1] -= 116.669
    # im[:, :, 2] -= 122.679
    #
    # # im = np.transpose(im, (2, 0, 1))
    # im = np.expand_dims(im, axis=0)
    #
    # res = model.predict(im)
    # classes = []
    # with open('classes.txt', 'r') as list_:
    #     for line in list_:
    #         classes.append(line.rstrip('\n'))
    # duration = time.time() - start
    # print "{} s to get output".format(duration)
    #
    # print 'class: ' + classes[np.argmax(res[0])] + ' acc: ' + str(res[0][np.argmax(res[0])])