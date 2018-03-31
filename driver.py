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
from localization.gps import GPS

from imutils.video import VideoStream
import helper
import os
import cv2
import numpy as np
from steering.auto_pilot import AutoPilot as AP
from steering.rambo import Rambo
from termcolor import colored
import time


class Driver:

    def __init__(self, steering_model, cc=False, seg_vis=False, obj_det=False, det_vis=False, gps=False):

        self.steering_model = steering_model
        self.cc = cc
        self.seg_vis = seg_vis
        self.obj_det = seg_vis
        self.det_vis = det_vis
        self.gps = gps

        self.steering_predictor, self.c_predictor, self.segmentor, self.seg_analyzer = self.init_ml_models()

        if self.gps:
            self.init_nav()

        if configs.default_st_port:  # check for serial setting
            print(colored("using system default serial ports", "blue"))
            self.mc = MC(0)
            self.c_controller = SC(1)
        else:
            print(colored("------ serial ports -------", "blue"))
            print(colored("-------- steering ---------", "blue"))
            os.system("ls /dev/ttyUSB*")
            st_port = helper.get_serial_port()
            self.mc = MC(st_port)

            print(colored("--------- cruise ---------", "blue"))
            os.system("ls /dev/ttyUSB*")
            cc_port = helper.get_serial_port()  # get serial ports
            self.c_controller = SC(cc_port)     # init CC controller
            print(colored("--------------------------", "blue"))

    def init_ml_models(self):

        if self.steering_model == "Own":
            steering_predictor = SteeringPredictor()  # init steering predictor
        elif self.steering_model == "Komanda":
            steering_predictor = None
        elif self.steering_model == "Rambo":
            steering_predictor = Rambo("steering/final_model.hdf5", "steering/X_train_mean.npy")
        elif self.steering_model == "Autumn":
            steering_predictor = AutumnModel(configs.cnn_graph, configs.lstm_json, configs.cnn_weights, configs.lstm_weights)
        elif self.steering_model == "AutoPilot":
            steering_predictor = None
        else:
            raise Exception("Unknown model type: " + self.steering_model + ". Please enter a valid model type")

        if self.cc:
            c_predictor = CruisePredictor()          # init cruise predictor
        else:
            c_predictor = None

        segmentor = Segmentor("ENET")  # init segmentor
        seg_analyzer = SegAnalyzer(0.05)  # init seg analyzer

        return steering_predictor, c_predictor, segmentor, seg_analyzer

    def init_nav(self):

        print(colored("------ GPS-------", "blue"))
        os.system("ls /dev/ttyUSB*")
        gps_port = helper.get_serial_port()
        self.gps = GPS(gps_port)

        start = self.gps.query_gps_location()
        print("start: " + str(start))
        destination = helper.get_destination()

        self.gp_planner = GlobalPathPlanner()
        directions = self.gp_planner.direction(start, destination)
        print(directions)

    def drive(self):

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

                # --------------------------- steering --------------------
                #
                if self.steering_model == "Rambo":
                    resize = cv2.resize(image,(256, 192))
                    angle = -1 * self.steering_predictor.predict(cv2.cvtColor(resize), cv2.COLOR_BGR2GRAY)
                    steering_img = self.steering_predictor.post_process_image(cv2.resize(image,(320, 160)), angle)

                elif self.steering_model == "Own":
                    angle, steering_img = self.steering_predictor.predict_steering(image)

                elif self.steering_model == "Autumn":
                    angle = self.steering_predictor.predict(image)
                    steering_img = self.steering_predictor.post_process_image(image=cv2.resize(image, (320, 160)), angle=angle)
                else:
                    raise Exception("Not implemented: " + self.steering_model + ". Please enter a valid model type")

                # -------------- execute steering commands ----------------
                steering_img = cv2.resize(steering_img, (640, 480))
                self.mc.turn(configs.st_fac * angle)


                # ------------------ segmentation -------------------------

                if cv2.waitKey(33) == ord('a'):
                    speed = 1
                    visual = cv2.resize(image, (640, 480))
                else:
                    result, visual = self.segmentor.semantic_segmentation(image, visualize=self.seg_vis)
                    speed = self.seg_analyzer.analyze_image(result)

                if speed == 0:
                    print(colored("STOP!", "red"))
                    self.c_controller.drive(-1)
                    time.sleep(2)
                else:
                    self.c_controller.drive(1)

                # ------------------------- ------------ -------------------

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


