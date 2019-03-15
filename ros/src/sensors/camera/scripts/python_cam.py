#!/usr/bin/env python
#
# By Neil Nie
# (c) 2018, All Rights Reserved

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time


class python_cam(object):

    def __init__(self):

        self.bridge = CvBridge()

        self.numpy_type_to_cvtype = {'uint8': '8U', 'int8': '8S', 'uint16': '16U', 
        								'int16': '16S', 'int32': '32S', 'float32': '32F',
                                        'float64': '64F'}
        self.numpy_type_to_cvtype.update(dict((v, k) for (k, v) in self.numpy_type_to_cvtype.items()))

        rospy.init_node('python_cam')

        self.publisher = rospy.Publisher('/cv_camera_node/image_raw', data_class=Image, queue_size=5)

        cap = cv2.VideoCapture(0)

        # Find OpenCV version
        # With webcam get(CV_CAP_PROP_FPS) does not work.
        # Let's see for ourselves.

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

        fps = cap.get(cv2.CAP_PROP_FPS)
        print "Frames per second using video.get(cv2.CAP_PROP_FPS) : {0}".format(fps)

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            start_time = time.time()

            ret, frame = cap.read()
            print("--- %s seconds ---" % (time.time() - start_time))

            # Our operations on the frame come here
            color = frame # cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            img_msg = self.cv_to_imgmsg(color, "bgr8")

            self.publisher.publish(img_msg)


            rate.sleep()

    def cv_to_imgmsg(self, cvim, encoding="passthrough"):
        """
        Convert an OpenCV :cpp:type:`cv::Mat` type to a ROS sensor_msgs::Image message.

        :param cvim:      An OpenCV :cpp:type:`cv::Mat`
        :param encoding:  The encoding of the image data, one of the following strings:

           * ``"passthrough"``
           * one of the standard strings in sensor_msgs/image_encodings.h

        :rtype:           A sensor_msgs.msg.Image message
        :raises CvBridgeError: when the ``cvim`` has a type that is incompatible with ``encoding``

        If encoding is ``"passthrough"``, then the message has the same encoding as the image's OpenCV type.
        Otherwise desired_encoding must be one of the standard image encodings

        This function returns a sensor_msgs::Image message on success, or raises :exc:`cv_bridge.CvBridgeError` on failure.
        """

        img_msg = Image()
        img_msg.height = cvim.shape[0]
        img_msg.width = cvim.shape[1]
        if len(cvim.shape) < 3:
            cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, 1)
        else:
            cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, cvim.shape[2])
        if encoding == "passthrough":
            img_msg.encoding = cv_type
        else:
            img_msg.encoding = encoding

        img_msg.data = cvim.tostring()
        img_msg.step = len(img_msg.data) // img_msg.height

        return img_msg

    def dtype_with_channels_to_cvtype2(self, dtype, n_channels):
        return '%sC%d' % (self.numpy_type_to_cvtype[dtype.name], n_channels)


if __name__ == "__main__":

    try:
        python_cam()
    except rospy.ROSInterruptException:
        pass