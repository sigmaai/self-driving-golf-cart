#!/usr/bin/env python
#
# By Neil Nie
# (c) 2018, All Rights Reserved

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy


class InputSwitch(object):

    def __init__(self):

        self.enabled = False
        #
        rospy.init_node('input_switch')

        rospy.Subscriber('/sensor/joystick/joy', Joy, callback=self.joystick_input_callback, queue_size=5)
        self.publisher = rospy.Publisher('/sensor/joystick/enabled', data_class=Bool, queue_size=5)
        # self.publisher = rospy.Publisher('/sensor/joystick/rqt_enabled', data_class=Bool, queue_size=5)
        rate = rospy.Rate(24)

        while not rospy.is_shutdown():

            data = Bool()
            data.data = self.enabled
            self.publisher.publish(data)

            rate.sleep()

    def joystick_input_callback(self, data):

        buttons = data.buttons
        # rospy.loginfo(data.buttons)
        if buttons[0] == 1 and buttons[7] == 1:
            self.enabled = not self.enabled


if __name__ == "__main__":

    try:
        InputSwitch()
    except rospy.ROSInterruptException:
        pass