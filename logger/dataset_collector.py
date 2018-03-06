#
# ------------- Data set collector -------------
# data_collector | Created by Neil Nie & Michael Meng
#      Main file of the self driving car
# (c) Neil Nie 2017, Please refer to the license
# ----------------------------------------------
#

import serial
from imutils.video import VideoStream
import helper
import os
import sys
import cv2
import numpy as np
from termcolor import colored
import time
import logger.configs as configs


data_points = []


def video_loop():

    while True:

        # -----------------------------------------------------
        # Check to see if the user closed the window
        if cv2.getWindowProperty(windowName, 0) < 0:
            break

        left_frame = vid_left.read()
        center_frame = vid_center.read()
        right_frame = vid_right.read()

        cv2.imwrite(configs.dataset_path + "/left/" + str(count) + ".png", left_frame)
        cv2.imwrite(configs.dataset_path + "/center/" + str(count) + ".png", center_frame)
        cv2.imwrite(configs.dataset_path + "/right/" + str(count) + ".png", right_frame)

        steering_angle = read_serial()
        count = count + 1
        row = [str(count),
               "/left/" + str(count) + ".png",
               "/center/" + str(count) + ".png",
               "/right/" + str(count) + ".png",
               str(steering_angle)]
        data_points.append(row)

        vid_buffer = np.concatenate((left_frame, center_frame, right_frame), axis=1)
        cv2.imshow(windowName, vid_buffer)

        key = cv2.waitKey(10)

        if key == 27:  # ESC key
            cv2.destroyAllWindows()
            print(colored("saving recorded data..."))
            print(colored("-------------------------", "green"))
            print(colored("Thank you! Program ended.", "green"))
            print(colored("-------------------------", "green"))
            break


def read_serial(port):

    s1.flushInput()

    while True:

        if s1.inWaiting() > 0:
        	value = ""
        	while ord(s1.read(1)) is not "b":
        		s1.read(1)
        	while ord(s1.read(1)) is not "e":
        		value = value + ord(s1.read(1))
            else:
            	print("invalid serial input")

            return value
        else:
        	return 0

def setup_dirs():

    if not os.path.exists(configs.dataset_path + "/right/"):
        os.makedirs(configs.dataset_path + "/right/")
    if not os.path.exists(configs.dataset_path + "/center/"):
        os.makedirs(configs.dataset_path + "/center/")
    if not os.path.exists(configs.dataset_path + "/left/"):
        os.makedirs(configs.dataset_path + "/left/")


if __name__ == '__main__':

    setup_dirs()
	s1 = serial.Serial(0, 9600)
    count = 0

    vid_left = VideoStream(src=configs.left_vid_src).start()
    vid_center = VideoStream(src=configs.cent_vid_src).start()
    vid_right = VideoStream(src=configs.right_vid_src).start()

    windowName = "data logger"
    cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(windowName, configs.default_img_size[1] * 3, 480)
    cv2.moveWindow(windowName, 0, 0)
    cv2.setWindowTitle(windowName, "data logger")

    var = input(colored("Enter 1 to start, 0 to Quite", "green"))
    if var == 0:
        print(colored("program begins", "green"))
        video_loop()
    else:
        print(colored("Program ended.... Thank you!", "green"))
        sys.exit(0)






