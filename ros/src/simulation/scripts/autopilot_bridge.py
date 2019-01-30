#!/usr/bin/python

import rospy
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDrive


class AutoPilotBridge():

    def __init__(self):

        rospy.init_node("self_driving_system_bridge_node")

        self.steering_cmd = None
        self.speed_cmd = None

        rospy.Subscriber('/vehicle/dbw/steering_cmds/', Float32, callback=self.steering_cmd_callback)
        rospy.Subscriber('/vehicle/dbw/cruise_cmds', Float32, callback=self.speed_cmd_callback)

        ackermann_pub = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDrive)

        rate = rospy.Rate(30)

        rospy.loginfo("self-driving system bridge started")

        while not rospy.is_shutdown():

            if self.steering_cmd is not None and self.speed_cmd is not None:
                ackermann_msg = AckermannDrive()

                ackermann_msg.steering_angle = -1.0 * float(self.steering_cmd)
                ackermann_msg.speed = 5 + float(self.speed_cmd)

                ackermann_pub.publish(ackermann_msg)

            rate.sleep()

    def steering_cmd_callback(self, data):

        self.steering_cmd = data.data

    def speed_cmd_callback(self, data):

        self.speed_cmd = data.data


if __name__ == "__main__":

    try:
        AutoPilotBridge()
    except rospy.ROSInitException:
        pass