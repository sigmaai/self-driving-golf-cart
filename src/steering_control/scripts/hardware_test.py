#!/usr/bin/python


import pandas as pd
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class HardwareTest():

    def __init__(self):

        rospy.init_node('hardware_test')

        self.bridge = CvBridge()

        # load data path
        self.data_path = "/home/neil/dataset/steering/test/"
        self.values = pd.read_csv(self.data_path + "labels.csv")
        count = 1

        self.scale_factor = 15

        self.image_pub =rospy.Publisher('/cv_camera_node/image_raw', Image, queue_size=5)
        self.steering_pub = rospy.Publisher('/vehicle/dbw/steering_cmds/', Float32, queue_size=5)
        rate = rospy.Rate(15)

        while not rospy.is_shutdown():
            angle = self.values["steering_angle"][count]
            angle = angle * int(self.scale_factor)
            print(angle)
            self.steering_pub.publish(angle)

            image = cv2.imread(self.data_path + "center/" + str(self.values["frame_id"][count]) + ".jpg")
            # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image_data = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.image_pub.publish(image_data)
            count += 1
            rate.sleep()


if __name__ == "__main__":

    try:
        HardwareTest()
    except rospy.ROSInterruptException:
        pass

