#
# ------------------- AGC ----------------------
# Drive.py | Created by Neil Nie & Michael Meng
#      Main file of the self driving car
# (c) Neil Nie 2017, Please refer to the license
# ----------------------------------------------
#

from steering.steering_predictor import SteeringPredictor
from steering.mc import MC
from cruise.cruise_predictor import CruisePredictor
from cruise.cruise_controller import CC
from semantic_segmentation.segmentor import Segmentor

from termcolor import colored
import os
import cv2
import numpy as np
import configs.configs as configs
from path_planning.gps import GPS 
from path_planning.global_path import GlobalPathPlanner


def get_destination():
    var = input(colored("Please enter your destination:", "blue"))
    return str(var)


def get_serial_port():
    var = input(colored("Please enter serial port number: ", "blue"))
    return int(var)


if __name__ == '__main__':

    if configs.verbose:
        print(colored("configs: ", "blue"))
        print(colored("steering factor: {}".format(configs.st_fac), "blue"))
        print(colored("image size: {}".format(configs.default_img_size), "blue"))
        print(colored("segmentation size: {}".format(configs.segmentation_size), "blue"))
        print(colored("-----------------------------", "blue"))

    # initialize all objects
    segmentor = Segmentor("ENET")               # init segmentor
    steering_predictor = SteeringPredictor()    # init steering predictor
    c_controller = CC()                         # init cruise controller
    #c_predictor = CruisePredictor()              # init cruise predictor

    if configs.default_st_port:                 # check for serial setting
        print(colored("using system default serial port", "blue"))
        mc = MC()
    else:
        print(colored("---------------------", "blue"))
        os.system("ls /dev/ttyUSB*")
        st_port = get_serial_port()
        mc = MC(st_port)
        print(colored("---------------------", "blue"))

    # initiate path planner, including GPS and Google Maps API
    # enabling navigation
    if configs.navigation:
        gps = GPS()
        start = gps.query_gps_location()
        destination = get_destination()
        gp_planner = GlobalPathPlanner()
        directions = gp_planner.direction(start, destination)
        print(directions)


    # OpenCV main loop

    cap = cv2.VideoCapture("nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)720, height=(int)576,format=(string)I420, framerate=(fraction)1/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")

    if cap.isOpened():

        windowName = "car detection"
        cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(windowName, 1280, 480)
        cv2.moveWindow(windowName, 0, 0)
        cv2.setWindowTitle(windowName, "car detection")

        print(colored("program begins", "blue"))

        while True:

            # -----------------------------------------------------
            # Check to see if the user closed the window
            if cv2.getWindowProperty(windowName, 0) < 0:
                break
            ret_val, image = cap.read()

            # -----------------------------------------------------
            # run detection network
            # no image preprocessing required for any detector
            angle, steering_img = steering_predictor.predict_steering(image)
            mc.turn(configs.st_fac * angle)

            # if configs.cruise:
            #    cruise_visual = c_predictor.predict_speed(image)
            #    buff1 = np.concatenate((steering_img, cruise_visual), axis=1)
            # else:
            steering_img = cv2.resize(steering_img, (640, 480))

            # buff1 = np.concatenate((steering_img, steering_img), axis=1)      # not running detection
                                                                                # showing steering image buffer
            if configs.segmentation:                                            # running segmentation
                result, visual = segmentor.semantic_segmentation(image)
                visual = cv2.resize(visual, (640, 480))
            else:                                                               # not running segmentation
                image = cv2.resize(image, (640, 480))
                visual = image                                                  # show original images

            buff1 = np.concatenate((steering_img, visual), axis=1)
            #vidBuf = np.concatenate((buff1, buff2), axis=0)
            vidBuf = buff1
            # show the stuff
            # -----------------------------------------------------

            cv2.imshow(windowName, vidBuf)
            key = cv2.waitKey(10)

            if key == 27:  # ESC key
                cv2.destroyAllWindows()
                print(colored("-------------------------", "blue"))
                print(colored("Thank you! Program ended.", "blue"))
                print(colored("-------------------------", "blue"))
                break

    else:
        print("Fatal error, camera is not open")

