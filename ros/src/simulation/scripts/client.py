#!/usr/bin/env python

#
# Copyright (c) 2018 Intel Labs.
#
# authors: Bernd Gassmann (bernd.gassmann@intel.com)
#
"""
Entry point for carla simulator ROS bridge
"""

import rospy
import sys
import glob
import os

try:
    sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


import carla

from ros_carla_bridge import CarlaRosBridge

def main():
    """
    main function for carla simulator ROS bridge
    maintaiing the communication client and the CarlaRosBridge objects
    """
    rospy.init_node("carla_client", anonymous=True)

    params = rospy.get_param('carla')
    host = params['host']
    port = params['port']

    rospy.loginfo("Trying to connect to {host}:{port}".format(
        host=host, port=port))

    try:
        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(2.0)

        carla_world = carla_client.get_world()

        rospy.loginfo("Connected")

        carla_ros_bridge = CarlaRosBridge(carla_world=carla_client.get_world(), params=params)
        carla_ros_bridge.run()
        carla_ros_bridge = None

        rospy.logdebug("Delete world and client")
        del carla_world
        del carla_client

    finally:
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
