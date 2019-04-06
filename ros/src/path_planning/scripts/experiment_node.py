#!/usr/bin/env python
#
# By Neil Nie
# (c) 2018, All Rights Reserved

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2


class ExperimentNode(object):

    def __init__(self):

        self.bridge = CvBridge()

        rospy.init_node('experiment_node')

        rospy.Subscriber("/zed/point_cloud/cloud_registered", PointCloud2, callback=self.point_cloud_callback)
        rospy.Subscriber('/zed/rgb/image_raw_color', Image, callback=self.real_camera_update_callback, queue_size=8)

        rospy.spin()

            # cv_camera callback
    def real_camera_update_callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            raise e

        print(cv_image.shape)

    def point_cloud_callback(self, data):

        print("point cloud")
        print(data.height)
        print(data.width)
        print(len(data.data))
        print(type(data.data))
        print(data.point_step)
        print("------------")


if __name__ == "__main__":

    try:
        ExperimentNode()
    except rospy.ROSInterruptException:
        pass