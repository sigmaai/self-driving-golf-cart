#!/usr/bin/env python
#
# By Neil Nie
# (c) 2018, All Rights Reserved

import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import Joy


class CameraSelect(object):

    def __init__(self):

        self.select = 0
        #
        rospy.init_node('camera_select')

        rospy.Subscriber('/sensor/joystick/joy', Joy, callback=self.joystick_input_callback, queue_size=5)
        self.publisher = rospy.Publisher('/camera_node/camera_select', data_class=Int8, queue_size=5)
        rate = rospy.Rate(24)

        while not rospy.is_shutdown():

            data = Int8()
            data.data = self.select
            self.publisher.publish(data)

            rate.sleep()

    def joystick_input_callback(self, data):

        buttons = data.buttons
        if buttons[3] == 1 and buttons[7] == 1 and self.select < 2:
            self.select = self.select + 1
        elif buttons[3] == 1 and buttons[7] == 1 and self.select == 2:
            self.select = 0


if __name__ == "__main__":

    try:
        CameraSelect()
    except rospy.ROSInterruptException:
        pass