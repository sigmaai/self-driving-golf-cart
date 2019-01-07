#!/usr/bin/env python
#
# By Neil Nie
# (c) 2018, All Rights Reserved

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy


class SelectiveOutput(object):

    def __init__(self):

        self.output = None
        rospy.init_node('selective_output')

        rospy.Subscriber('/sensor/joystick/joy', Joy, callback=self.joystick_input_callback, queue_size=5)
        self.publisher = rospy.Publisher('/sensor/joystick/left_stick_x', data_class=Float32, queue_size=5)

        rate = rospy.Rate(24)

        while not rospy.is_shutdown():

            if self.output is not None:
                data = Float32()
                data.data = self.output
                self.publisher.publish(data)

            rate.sleep()

    def joystick_input_callback(self, data):

        inputs = data.axes
        self.output = inputs[0]
        # rospy.loginfo(self.output)


if __name__ == "__main__":

    try:
        InputSwitch()
    except rospy.ROSInterruptException:
        pass