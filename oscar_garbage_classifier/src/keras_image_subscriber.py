#!/usr/bin/env python

import copy
import numpy as np
import rospy
import cv2
from scipy import misc
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from squeezenet import get_squeezenet
from random import randrange

class KerasImageSubscriber:

    def __init__(self):
        # self.model = get_squeezenet(1000, dim_ordering='tf')
        # self.model.compile(loss="categorical_crossentropy", optimizer="adam")
        # self.model.load_weights('/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/models/squeezenet_weights_tf_dim_ordering_tf_kernels.h5', by_name=True)

        # self.model = get_squeezenet(1000, dim_ordering='tf')
        # model.compile(loss="categorical_crossentropy", optimizer="adam")
        # self.model.load_weights('/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/models/squeezenet_weights_tf_dim_ordering_tf_kernels.h5', by_name=True)
        # self.init_model()

        self.model = None
        self.cv_bridge = CvBridge()
        self.classes = []
        with open('/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/src/classes.txt', 'r') as list_:
            for line in list_:
                self.classes.append(line.rstrip('\n'))
        self.sub = rospy.Subscriber("image_topic", Image, self.classify)


    def classify(self, img):
        """
        Callback function that classifies the image.
        :param img:  Image to be classified
        :return:  Class of the image.
        """

        # Convert image from ros imgmsg to opencv2 image.
        # image = self.cv_bridge.imgmsg_to_cv2(img)

        # if randrange(100) > 50:
        #     return

        if self.model is None:
            self.model = get_squeezenet(1000, dim_ordering='tf')
            self.model.compile(loss="categorical_crossentropy", optimizer="adam")
            self.model.load_weights('/mnt/data/Development/ros/catkin_ws/src/oscar_garbage_classifier/models/squeezenet_weights_tf_dim_ordering_tf_kernels.h5', by_name=True)

        # frame = misc.imread('/mnt/data/Development/keras-squeezenet/images/cat.jpeg', mode='RGB')
        frame = self.cv_bridge.imgmsg_to_cv2(img) #, desired_encoding='8UC3')

        height, width, depth = frame.shape
        offset = min(height, width) / 2.0
        centerX, centerY = (width / 2.0, height / 2.0)

        cropped_frame = frame[centerY - offset: centerY + offset, centerX - offset: centerX + offset]

        print "Frame: "
        print type(frame)
        print frame

        image = cv2.resize(cropped_frame, (227, 227)).astype(np.float64)

        # aux = copy.copy(image)
        # image[:, :, 0] = aux[:, :, 2]
        # image[:, :, 2] = aux[:, :, 0]
        print "Image: "
        print type(image)
        print image


        # Remove image mean
        # image[:, :, 0] -= 104.006
        # image[:, :, 1] -= 116.669
        # image[:, :, 2] -= 122.679


        # frame2 = copy.copy(image)
        frame2 = cv2.resize(frame, (227, 227)) #.astype(np.int8)
        print "Frame2: "
        print type(frame2)
        print frame2

        # image = np.transpose(image, (2, 0, 1))
        image = np.expand_dims(image, axis=0)

        model = self.model

        res = model.predict(image)

        # classification_result = self.classes[np.argmax(res[0])]
        results = res[0].argsort()[-5:][::-1]

        for i in xrange(len(results)):
            result = self.classes[results[i]]
            text = '%.3f: %s' % (res[0][results[i]], result[9:])
            # print(text)
            cv2.putText(cropped_frame, text, (20, 50+20*i), cv2.FONT_HERSHEY_PLAIN, 0.75, (255,150,0),2)

        # text = 'class: ' + classification_result + ' acc: ' + str(res[0][np.argmax(res[0])])
        # print text

        # result = self.classes[np.argmax(res[0])]
        # print result
        # cv2.putText(frame, text, (20, 50+20), cv2.FONT_HERSHEY_PLAIN, 1.0, (255,150,0),2)
        #
        cv2.imshow("Keras", cropped_frame)
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
