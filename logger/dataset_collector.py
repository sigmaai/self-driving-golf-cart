#
# ------------- Data set collector -------------
# data_collector | Created by Neil Nie & Michael Meng
#      Main file of the self driving car
# (c) Neil Nie 2017, Please refer to the license
# ----------------------------------------------
#


from imutils.video import VideoStream
import helper
import os
import sys
import cv2
import numpy as np
from termcolor import colored
import time
import data_collection.configs as configs


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


if __name__ == '__main__':

    vid_left = VideoStream(src=configs.left_vid_src).start()
    vid_center = VideoStream(src=configs.cent_vid_src).start()
    vid_right = VideoStream(src=configs.right_vid_src).start()

    windowName = "car detection"
    cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(windowName, configs.default_img_size[1] * 3, 480)
    cv2.moveWindow(windowName, 0, 0)
    cv2.setWindowTitle(windowName, "car detection")

    var = input(colored("Enter 1 to start, 0 to Quite", "green"))
    if var == 0:
        print(colored("program begins", "green"))
        video_loop()
    else:
        print(colored("Program ended.... Thank you!", "green"))
        sys.exit(0)






