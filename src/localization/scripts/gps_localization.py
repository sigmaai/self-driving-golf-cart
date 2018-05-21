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


class GPSLocalization():

    def __init__(self):

        rospy.init_node('gps_localization')
        self.br = tf.TransformBroadcaster()

        self.x = 0
        self.y = 0
        self.gps_msg = None

        rospy.Subscriber('/sensor/gps/fix', NavSatFix, callback=self.navsatfix_callback, queue_size=5)
        self.loc_coord_pub = rospy.Publisher('/localization/local_coordinate/fix/', NavSatFix, queue_size=5)

        self.rate = rospy.Rate(30.0)

        while not rospy.is_shutdown():
            self.br.sendTransform((self.x, self.y, 0),
                             (0, 0, 0, 1),
                             rospy.Time.now(),
                             "base_link",
                             "local_map")
            msg = self.gps_msg

            self.loc_coord_pub.publish()

            self.rate.sleep()

        # 1. use geodesy to convert LatLong to point
        # 2. subtract the UTM static transformation from it
        # 3. send that transformation

    def navsatfix_callback(self, msg):

        if msg.status.status == -1:
            print("no data received")
            
        else:
            # point = geodesy.utm.fromLatLong(42.546895, -72.609342).toPoint()
            point = geodesy.utm.fromLatLong(msg.latitude, msg.longitude).toPoint()
            self.x = float(point.x) - 696400.1
            self.y = float(point.y) - 4713254.8
            self.gps_msg = msg
            

if __name__ == '__main__':

    try:
        GPSLocalization()
    except rospy.ROSInterruptException:
        pass




