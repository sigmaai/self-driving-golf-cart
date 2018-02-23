#
# ------------------- AGC ----------------------
# Drive.py | Created by Neil Nie & Michael Meng
#      Main file of the self driving car
# (c) Neil Nie 2017, Please refer to the license
# ----------------------------------------------
#
#from steering.rambo import Rambo
from steering.steering_predictor import SteeringPredictor
from steering.mc import MC
from cruise.cruise_predictor import CruisePredictor
from cruise.sc import SC
from semantic_segmentation.segmentor import Segmentor
from semantic_segmentation.segmentation_analyzer import SegAnalyzer
from path_planning.global_path import GlobalPathPlanner
import configs.configs as configs
from path_planning.gps import GPS
from termcolor import colored
import time
import os
import cv2
import numpy as np

drive_continue = True
start_time = 0


def get_destination():
    var = input(colored("Please enter your destination:", "blue"))
    return str(var)


def get_serial_port():
    var = input(colored("Please enter serial port number: ", "blue"))
    return int(var)


def welcome():
    print(colored("Welcome to the self-driving golf cart program", "green"))
    print(colored("By Michael Meng & Neil Nie", "green"))
    print(colored("Thanks for driving with us", "green"))
    print(colored("v.0.2.1", "green"))
    print(colored("---------------------------------------------", "green"))


if __name__ == '__main__':

    welcome()

    if configs.verbose:
        print(colored("configs: ", "blue"))
        print(colored("steering factor: {}".format(configs.st_fac), "blue"))
        print(colored("image size: {}".format(configs.default_img_size), "blue"))
        print(colored("segmentation size: {}".format(configs.segmentation_size), "blue"))
        print(colored("-----------------------------", "blue"))

    # initialize all objects
    segmentor = Segmentor("ENET")               # init segmentor
    seg_analyzer = SegAnalyzer(0.05)            # init seg analyzer
 #   steering_predictor = Rambo("steering/final_model.hdf5", "steering/X_train_mean.npy")
    steering_predictor = SteeringPredictor()    # init steering predictor
    # c_predictor = CruisePredictor()           # init cruise predictor

    # initiate path planner, including GPS and Google Maps API
    # enabling navigation
    if configs.navigation:
        print(colored("------ GPS-------", "blue"))
        os.system("ls /dev/ttyUSB*")
        gps_port = get_serial_port()
        gps = GPS(gps_port)
        start = gps.query_gps_location()
        print("start: " + str(start))
        # destination = get_destination()
        # gp_planner = GlobalPathPlanner()
        # directions = gp_planner.direction(start, destination)
        # print(directions)

    if configs.default_st_port:                 # check for serial setting
        print(colored("using system default serial ports", "blue"))
        mc = MC()
        c_controller = SC()
    else:
        print(colored("------steering-------", "blue"))
        os.system("ls /dev/ttyUSB*")
        st_port = get_serial_port()
        mc = MC(st_port)

        print(colored("-------cruise-------", "blue"))
        os.system("ls /dev/ttyUSB*")
        cc_port = get_serial_port()                     # get serial ports
        c_controller = SC(cc_port)                      # init CC controller
        print(colored("---------------------", "blue"))

    # OpenCV main loop

    cap = cv2.VideoCapture(configs.CV_CAP_STR)

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
           # angle = -steering_predictor.predict(cv2.cvtColor(cv2.resize(image,(256, 192)),cv2.COLOR_BGR2GRAY))
           # steering_img = steering_predictor.post_process_image(cv2.resize(image,(320, 160)),angle)
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
          #      result, visual = segmentor.semantic_segmentation(image)
                result, visual = steering_img, steering_img
                visual = cv2.resize(visual, (640, 480))
                # speed = seg_analyzer.analyze_image(result)
                speed = 1
                if speed == 0:
                    print(colored("STOP!", "red"))
            #        c_controller.drive(-1)
                    #time.sleep(2)
            #    else:
            #        c_controller.drive(1)

            else:                                                               # not running segmentation
                image = cv2.resize(image, (640, 480))
                visual = image                                                  # show original image
                
            buff1 = np.concatenate((steering_img, visual), axis=1)
            # vidBuf = np.concatenate((buff1, buff2), axis=0)
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
        

