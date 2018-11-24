#!/usr/bin/python
#
# Results video generator Udacity Challenge 2
# Original By: Comma.ai and Chris Gundling
# Revised and used by Neil Nie
#

import numpy as np
from PIL import Image as PILImage
from PIL import ImageFont
from PIL import ImageDraw
import cv2

# ROS
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError


class Visualization():

    def apply_visualization(self, image, accel):

        background = PILImage.fromarray(np.uint8(image))
        if accel < -1:
            sw = PILImage.open("/home/neil/Workspace/self-driving-golf-cart/ros/src/cruise_control/scripts/resources/stop.png")
            sw = sw.resize((80, 80), PILImage.ANTIALIAS)
            background.paste(sw, (10, 38), sw)

        draw = ImageDraw.Draw(background)
        font = ImageFont.truetype("/home/neil/Workspace/self-driving-golf-cart/ros/src/steering_control/scripts/resources/FiraMono-Medium.otf", 20)
        draw.text((40, 420), str(round(accel, 3)), (255, 255, 255), font=font)
        draw.text((10, 10), "Cruise Control Running... v1.0.0", (255, 255, 255), font=font)
        text = ""
        if accel < 0:
            text = 'Slow'
        elif accel > 0:
            text = 'Speed Up'
        draw.text((40, 380), text, (255, 255, 255), font=font)
        steering_img = cv2.resize(np.array(background), (640, 480))
        return steering_img

    def image_update_callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            raise e

        self.current_frame = cv_image

    def accel_cmds_callback(self, data):

        self.accel_cmds = data.data

    def __init__(self):

        rospy.init_node('cruise_control_visualization_node')

        self.current_frame = None
        self.bridge = CvBridge()
        self.accel_cmds = 0.0

        # Please note that the visualization node listens to either the raw
        # camera input or the simulated camera input. Please change this
        # setting in the launch file (/launch/cruise_control.launch)
        # or specify this parameter in your command line input.
        simulation = rospy.get_param("/cruise_control_node/simulation")
        if (simulation):
            rospy.logwarn("You are in simulation mode. If this is unintentional, please quite the program immediately")
            rospy.Subscriber('/cv_camera_node/image_sim', Image, callback=self.image_update_callback, queue_size=8)
        else:
            rospy.Subscriber('/cv_camera_node/image_raw', Image, callback=self.image_update_callback, queue_size=8)

        rospy.Subscriber('/vehicle/dbw/cruise_cmds', Float32, callback=self.accel_cmds_callback)

        visual_pub = rospy.Publisher('/visual/cruise_control/accel_img', Image, queue_size=5)
        rate = rospy.Rate(15)

        while not rospy.is_shutdown():

            if self.current_frame is not None:
                image = self.apply_visualization(image=self.current_frame, accel=self.accel_cmds)
                img_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                visual_pub.publish(img_msg)

            rate.sleep()


if __name__ == "__main__":

    try:
        Visualization()
    except rospy.ROSInterruptException:
        pass