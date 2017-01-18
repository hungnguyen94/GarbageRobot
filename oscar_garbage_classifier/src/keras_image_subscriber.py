#!/usr/bin/env python

from __future__ import division
import numpy as np
import rospy
import cv2
import tensorflow as tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from squeezenetv1_1 import SqueezeNet
import copy

class KerasImageSubscriber:
    def __init__(self):
        self.weights_file = '/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/models/squeezenet_webcam_weights_300x300.103-loss_0.00108-acc_1.00000.h5'
        self.input_shape = (300, 300, 3)
        self.model = None
        self.cv_bridge = CvBridge()
        self.classes = ['bottles', 'cans', 'cups', 'other']
        self.graph = tf.Graph()
        self.rotation_matrix = cv2.getRotationMatrix2D((self.input_shape[0]/2, self.input_shape[1]/2), 90, 1)

        with self.graph.as_default():
            self.model = SqueezeNet(len(self.classes), self.input_shape[0], self.input_shape[1], self.input_shape[2])
            self.model.load_weights(self.weights_file, by_name=True)

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

        # ts = int(time.time())
        # cv2.imwrite('/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/images/training_webcam_cv2_output/bottles/bottle-%s.jpg' % (ts), cropped_frame)
        # print("write img")

        # print "Frame: "
        # print type(frame)
        # print frame

        resized_frame = cv2.resize(cropped_frame, (self.input_shape[0], self.input_shape[1]))

        rotated_frame = cv2.warpAffine(resized_frame, self.rotation_matrix, (self.input_shape[0], self.input_shape[1]))
        # image = frame.astype('float')
        fr = rotated_frame
        image = fr.astype(np.float64) / float(255)
        # image /= 255.

        aux = copy.copy(image)
        image[:, :, 0] = aux[:, :, 2]
        image[:, :, 2] = aux[:, :, 0]
        # cv2.imwrite('/tmp/test2.jpg', image)
        # print "Image: "
        # print type(image)
        # print image


        # Remove image mean
        # image[:, :, 0] -= 104.006
        # image[:, :, 1] -= 116.669
        # image[:, :, 2] -= 122.679


        image = np.expand_dims(image, axis=0)

        with self.graph.as_default():
            res = self.model.predict(image)

        results = res[0]

        for i in xrange(len(results)):
            clazz = self.classes[i]
            text = '%.9f: %s' % (results[i], clazz)
            print(text)
            cv2.putText(fr, text, (20, 50+20*i), cv2.FONT_HERSHEY_PLAIN, 0.75, (255, 150, 0), 2)
        print("")

        cv2.imshow("Keras", fr)
        cv2.waitKey(1)


if __name__ == '__main__':

    print("")
    img_sub = KerasImageSubscriber()
    rospy.init_node("tf_image_subscriber", anonymous=True)

    try:
        rospy.spin()
    except rospy.ROSInterruptException(), KeyboardInterrupt:
        print("exiting image_subscriber")
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
