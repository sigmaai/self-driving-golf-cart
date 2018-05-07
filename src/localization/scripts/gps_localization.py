#!/usr/bin/env python
#
# basic localization script for locating
# the robot in rviz based on GPS data
#
# (C) Yongyang Nie, 2018
#

import rospy
import tf
import geodesy.props
import geodesy.utm
import geodesy.wu_point
import geodesy
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
import math


def navsatfix_callback(msg):

    point = geodesy.utm.fromLatLong(msg.longitude, msg.latitude).toPoint()
    point.x = point.x - 696400.1
    point.y = point.y - 4713254.8

    while not rospy.is_shutdown():
        br.sendTransform((point.x - 696400.1, point.y - 4713254.8, 0),
                         (0, 0, 0, 1),
                         rospy.Time.now(),
                         "base_link",
                         "local_map")
        rate.sleep()

if __name__ == '__main__':

    rospy.init_node('gps_localization')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    rospy.Subscriber('/sensor/gps/fix', NavSatFix, callback=navsatfix_callback, queue_size=5)
    rospy.spin()

    #
    # 1. use geodesy to convert LatLong to point
    # 2. subtract the UTM static transformation from it
    # 3. send that transformation
    #
    #




