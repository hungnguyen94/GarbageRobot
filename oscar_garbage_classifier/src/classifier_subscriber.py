#!/usr/bin/env python

from __future__ import division
import numpy as np
import rospy
import cv2
import tensorflow as tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from squeezenetv1_1 import SqueezeNet
import keras.backend as K
from oscar_garbage_classifier.srv import ClassifyImage, Sort
import copy
import time
import rospkg

class ClassifierSubscriber:
    def __init__(self):
     	rp = rospkg.RosPack()
	path = rp.get_path('oscar_garbage_classifier')
        self.detector_weights_file = path + '/models/squeezenet_detector_weights_100x100.h5'
        self.detect_input_shape = (100, 100, 3)
        self.model = None
        self.cv_bridge = CvBridge()
        self.detector_classes = ['occupied', 'empty']

        self.detect_graph = tf.Graph()
        self.detect_sess = tf.Session(graph=self.detect_graph)

        self.rotation_matrix = cv2.getRotationMatrix2D((self.detect_input_shape[0]/2, self.detect_input_shape[1]/2), 90, 1)

        K.set_session(self.detect_sess)
        with self.detect_graph.as_default():
            self.model = SqueezeNet(2, self.detect_input_shape[0], self.detect_input_shape[1], self.detect_input_shape[2])
            self.model.load_weights(self.detector_weights_file, by_name=True)

        self.sub = rospy.Subscriber("image_topic", Image, self.detect)
        self.invoke_timeout = time.time()
        self.pub = rospy.Publisher('oscar_eating', Empty, queue_size=10)

    def process_image(self, imgmsg):
        """
        Function to process the image in the correct format for the classifier.
        :param img: Img message
        :return: Image
        """
        frame = self.cv_bridge.imgmsg_to_cv2(imgmsg) #, desired_encoding='8UC3')
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
        fr = resized_frame.astype('float32')
        image = fr.astype('float32')
        image /= 255.

        # Rotate image 90 degrees
        image = cv2.warpAffine(image, self.rotation_matrix, (self.detect_input_shape[0], self.detect_input_shape[1]))

        # Change BGR to RGB
        aux = copy.copy(image)
        image[:, :, 0] = aux[:, :, 2]
        image[:, :, 2] = aux[:, :, 0]

        image = np.expand_dims(image, axis=0)
        return image

    def detect(self, imgmsg):
        """
        Callback function that classifies the image.
        :param imgmsg:  Image to be classified
        :return:  Class of the image.
        """
        image = self.process_image(imgmsg)
        with self.detect_graph.as_default():
            res = self.model.predict(image)

        results = res[0]

        # If sorter is occupied, run the classifier
        if results[0] > 0.15:
            if time.time() >= self.invoke_timeout:
		self.pub.publish(Empty())
                self.invoke_timeout = time.time() + 15
                # Img is rotated and resized in the service.
                rospy.wait_for_service('image_classify')
                image_classify = rospy.ServiceProxy('image_classify', ClassifyImage)
                class_result = image_classify(imgmsg)
                print(class_result)
                rospy.wait_for_service('sort_srv')
                invoke_sorter = rospy.ServiceProxy('sort_srv', Sort)
                sort_result = invoke_sorter(class_result.prediction)
                print(sort_result)
                self.invoke_timeout = time.time() + 7
            else:
                print("Occupied but busy")
	else: 
 	    print("Empty")

        for i in xrange(len(results)):
            clazz = self.detector_classes[i]
            text = '%.9f: %s' % (results[i], clazz)
            print(text)
        print("")


if __name__ == '__main__':

    print("")
    img_sub = ClassifierSubscriber()
    rospy.init_node("classifier_subscriber", anonymous=False)

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
