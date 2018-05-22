#!/usr/bin/env python

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
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose2D


class GPSLocalization():

    def __init__(self):

        rospy.init_node('gps_localization')
        self.trans = tf.TransformBroadcaster()

        self._x = 0
        self._y = 0
        self.gps_msg = None

        rospy.Subscriber('/sensor/gps/fix', NavSatFix, callback=self.navsatfix_callback, queue_size=5)

        # publishes the vehicle position after UTM transformation
        # as well as scaling. The Pose2D being published is the
        # same as the map/vehicle transformation that's being sent.

        self.map_pose_pub = rospy.Publisher('/localization/pose/map', Pose2D, queue_size=5)
        # self.vehicle_pose_pub = rospy.Publisher('/localization/pose/vehicle', Pose2D, queue_size=5)
        self.rate = rospy.Rate(30.0)

        while not rospy.is_shutdown():

            self.trans.sendTransform((self._x, self._y, 0), (0, 0, 0, 1), rospy.Time.now(), "base_link", "local_map")

            map_pose = Pose2D()
            map_pose.x = self._x
            map_pose.y = self._y
            self.map_pose_pub.publish(map_pose)

            self.rate.sleep()

        # 1. use geodesy to convert LatLong to point
        # 2. subtract the UTM static transformation from it
        # 3. send that transformation

    def navsatfix_callback(self, msg):

        if msg.status.status == -1:
            print("no data received")
            
        else:
            point = geodesy.utm.fromLatLong(msg.latitude, msg.longitude).toPoint()
            self._x = float(point.x) - 696400.1
            self._y = float(point.y) - 4713254.8
            self.gps_msg = msg
            

if __name__ == '__main__':

    try:
        GPSLocalization()
    except rospy.ROSInterruptException:
        pass




