#! /usr/bin/env python

# MIT License. Must include author and license in your work.

# Copyright (c) 2017-2018 Yongyang Nie

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Run a semantic segmentation node.
This ROS node uses the object detector class to run detection.
"""

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
import reeds_shepp_path_planning as rs
from nav_msgs.msg import OccupancyGrid
import hybrid_a_star

XY_GRID_RESOLUTION = 5.0  # [m]
YAW_GRID_RESOLUTION = np.deg2rad(20.0)  # [rad]

class HASNode:

    def __init__(self):

        rospy.init_node('has_node')

        # ROS subscribers
        rospy.Subscriber("/grid_map_local", OccupancyGrid, callback=self.grid_map_callback, queue_size=2)

        self.ox = []
        self.oy = []

        rate = rospy.Rate(30)

        # Running ROS loop
        rospy.loginfo(" ")

        self.start = [100.0, 5.0, np.deg2rad(90.0)]
        self.goal = [100.0, 30.0, np.deg2rad(90.0)]
        self.path = None

        while not rospy.is_shutdown():

            if len(self.ox) != 0 and len(self.oy) != 0:

                plt.cla()
                plt.plot(self.ox, self.oy, ".k")
                rs.plot_arrow(self.start[0], self.start[1], self.start[2], fc='g')
                rs.plot_arrow(self.goal[0], self.goal[1], self.goal[2])

                self.path = hybrid_a_star.hybrid_a_star_planning(self.start, self.goal, self.ox, self.oy,
                                                                 XY_GRID_RESOLUTION,
                                                                 YAW_GRID_RESOLUTION)
                # x = self.path.xlist
                # y = self.path.ylist
                # yaw = self.path.yawlist

                plt.grid(True)
                # plt.plot(x, y, "-r", label="Hybrid A* path")
                plt.pause(0.001)

            rate.sleep()

    def grid_map_callback(self, data):

        meta_data = data.info

        ox = []
        oy = []

        print("height: " + str(meta_data.height))
        print("width: " + str(meta_data.width))

        for row in range(0, meta_data.height):

            for col in range(0, meta_data.width):

                occupancy_data = data.data[row * meta_data.width + col]

                if occupancy_data > 90:
                    oy.append(col)
                    ox.append(row)

        for i in range(0, 200, 2):
            ox.append(i)
            oy.append(0.0)
        for i in range(0, 250, 2):
            ox.append(200)
            oy.append(i)
        for i in range(0, 200 + 1, 2):
            ox.append(i)
            oy.append(250)
        for i in range(0, 250 + 1, 2):
            ox.append(0.0)
            oy.append(i)

        self.ox = ox
        self.oy = oy


if __name__ == "__main__":

    try:
        HASNode()
    except rospy.ROSInterruptException:
        pass
