#!/usr/bin/env python
#
# By Neil Nie
# (c) 2018, All Rights Reserved

import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CameraSelect(object):

    def __init__(self):

        self.camera_select = 0
        self.current_frame = None
        self.bridge = CvBridge()

        rospy.init_node('camera_select')

        rospy.Subscriber('/cv_camera_node/camera_select', Int8, callback=self.camera_input_select_callback)
        rospy.Subscriber('/cv_camera_node/image_raw', Image, callback=self.real_camera_update_callback, queue_size=8)
        rospy.Subscriber('/cv_camera_node/image_sim', Image, callback=self.sim_camera_update_callback, queue_size=8)
        rospy.Subscriber('/carla/ego_vehicle/camera/rgb/front/image_color', Image,
                         callback=self.carla_rgb_camera_update_callback, queue_size=8)

        self.publisher = rospy.Publisher('/vehicle/sensor/camera/front/image_color', data_class=Image, queue_size=5)
        rate = rospy.Rate(24)

        while not rospy.is_shutdown():

            if self.current_frame is not None:
                self.publisher.publish(self.current_frame)

            rate.sleep()

            # cv_camera callback
    def real_camera_update_callback(self, data):

        if self.camera_select == 0:
            self.current_frame = data

            # sim_camera callback
    def sim_camera_update_callback(self, data):

        if self.camera_select == 1:
            self.current_frame = data

            # carla simulator rgb camera callback
    def carla_rgb_camera_update_callback(self, data):

        if self.camera_select == 2:
            self.current_frame = data

        # camera input select callback
    def camera_input_select_callback(self, data):
        if self.camera_select != data.data:
            rospy.loginfo("Camera Input Switched " + str(data.data))
        self.camera_select = data.data


if __name__ == "__main__":

    try:
        CameraSelect()
    except rospy.ROSInterruptException:
        pass