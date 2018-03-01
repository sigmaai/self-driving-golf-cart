#
# ------------------- AGC ----------------------
# Drive.py | Created by Neil Nie & Michael Meng
#      Main file of the self driving car
# (c) Neil Nie 2017, Please refer to the license
# ----------------------------------------------
#

# from steering.rambo import Rambo
from steering.autumn import AutumnModel
from steering.steering_predictor import SteeringPredictor
from steering.mc import MC
from cruise.cruise_predictor import CruisePredictor
from cruise.sc import SC
from semantic_segmentation.segmentor import Segmentor
from semantic_segmentation.segmentation_analyzer import SegAnalyzer
from path_planning.global_path import GlobalPathPlanner
import configs.configs as configs
from path_planning.gps import GPS

from imutils.video import VideoStream
import helper
import os
import cv2
import numpy as np
from termcolor import colored
import time

drive_continue = True
start_time = 0


def init_ml():

    # steering_predictor = SteeringPredictor()      # init steering predictor
    # steering_predictor = Rambo("steering/final_model.hdf5", "steering/X_train_mean.npy")
    steering_predictor = AutumnModel(configs.cnn_graph, configs.lstm_json, configs.cnn_weights, configs.lstm_weights)
    helper.steering_init_response(steering_predictor.model)
    c_predictor = None # CruisePredictor()          # init cruise predictor
    segmentor = Segmentor("ENET")                   # init segmentor
    seg_analyzer = SegAnalyzer(0.05)                # init seg analyzer

    return steering_predictor, c_predictor, segmentor, seg_analyzer


def init_nav():

    print(colored("------ GPS-------", "blue"))
    os.system("ls /dev/ttyUSB*")
    gps_port = helper.get_serial_port()
    gps = GPS(gps_port)
    start = gps.query_gps_location()
    print("start: " + str(start))
    destination = helper.get_destination()
    gp_planner = GlobalPathPlanner()
    directions = gp_planner.direction(start, destination)
    print(directions)


if __name__ == '__main__':

    helper.welcome()

    if configs.verbose:
        helper.print_configs()

    steering_predictor, c_predictor, segmentor, seg_analyzer = init_ml()

    if configs.navigation:
        init_nav()

    if configs.default_st_port:                 # check for serial setting
        print(colored("using system default serial ports", "blue"))
        mc = MC()
        c_controller = SC()
    else:
        print(colored("------ serial ports -------", "blue"))
        print(colored("-------- steering ---------", "blue"))
        os.system("ls /dev/ttyUSB*")
        st_port = helper.get_serial_port()
        mc = MC(st_port)

        print(colored("--------- cruise ---------", "blue"))
        os.system("ls /dev/ttyUSB*")
        cc_port = helper.get_serial_port()                     # get serial ports
        c_controller = SC(cc_port)                      # init CC controller
        print(colored("--------------------------", "blue"))


    # ----------------------- OpenCV main loop ------------------------
    # -----------------------------------------------------------------

    cap = cv2.VideoCapture(configs.CV_CAP_STR)

    vid_left = VideoStream(src=configs.left_vid_src).start()
    vid_center = VideoStream(src=configs.cent_vid_src).start()
    vid_right = VideoStream(src=configs.right_vid_src).start()

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

            left_frame = vid_left.read()
            center_frame = vid_center.read()
            right_frame = vid_right.read()

            ret_val, image = cap.read()

            # --------------------------- steering ---------------------------
            #
            # -------------------rambo-----------------------------
            # angle = -steering_predictor.predict(cv2.cvtColor(cv2.resize(image,(256, 192)),cv2.COLOR_BGR2GRAY))
            # steering_img = steering_predictor.post_process_image(cv2.resize(image,(320, 160)),angle)
            # -------------------own-------------------------------
            angle, steering_img = steering_predictor.predict_steering(image)
            # --------------------autumn---------------------------
            # angle = steering_predictor.predict(image)
            # steering_img = steering_predictor.post_process_image(image=cv2.resize(image, (320, 160)), angle=angle)
            # -----------------------------------------------------
            steering_img = cv2.resize(steering_img, (640, 480))
            mc.turn(configs.st_fac * angle)

            # ------------------------- segmentation -------------------------

            if cv2.waitKey(33) == ord('a'):
                c_controller.drive(1)
                visual = image
            else:
                result, visual = segmentor.semantic_segmentation(image, visualize=False)
                speed = seg_analyzer.analyze_image(result)
                if speed == 0:
                    print(colored("STOP!", "red"))
                    c_controller.drive(-1)
                    time.sleep(2)
                else:
                    c_controller.drive(1)

            # ------------------------- ------------ -------------------------

            image = cv2.resize(image, (640, 480))

            buff1 = np.concatenate((steering_img, visual), axis=1)
            # vidBuf = np.concatenate((buff1, buff2), axis=0)
            vidBuf = buff1

            # ------------------ show the stuff -------------------
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
        

