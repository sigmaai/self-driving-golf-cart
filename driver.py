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
import steering.visualization.visualization as steering_visualization
from steering.auto_pilot import AutoPilot as AP
from steering.rambo import Rambo

from cruise.cruise_predictor import CruisePredictor
from cruise.sc import SC

from semantic_segmentation.segmentor import Segmentor
from semantic_segmentation.segmentation_analyzer import SegAnalyzer

from detection.object.object_detector import ObjectDetector
from path_planning.global_path import GlobalPathPlanner
import configs.configs as configs
from localization.gps import GPS
from gui.info_screen import InfoScreen

from time import process_time
import helper
import os
import cv2
import numpy as np
from termcolor import colored
import time


class Driver:

    def __init__(self, steering_model, cruise_control=False, seg_vis=False, obj_det=False, gps=False):

        self.steering_model = steering_model
        self.cruise_control = cruise_control
        self.seg_vis = seg_vis
        self.obj_det = obj_det
        self.gps = gps

        # initialize some private variable
        self.__steering_heatmap = False
        self.__disable_stopping = False
        self.__fps = 0.0
        self.__info_screen = InfoScreen()

        self.steering_predictor, \
        self.c_predictor, \
        self.segmentor, \
        self.seg_analyzer, \
        self.object_detector = self.init_ml_models()

        if self.gps:
            self.init_navigation()

        if configs.default_st_port:  # check for serial setting
            print(colored("using system default serial ports", "yellow"))
            self.mc = MC(0)
            self.c_controller = SC(1)
        else:
            print(colored("------ serial ports -------", "yellow"))
            print(colored("-------- steering ---------", "yellow"))
            os.system("ls /dev/ttyAMC*")
            st_port = helper.get_serial_port()
            self.mc = MC(st_port)

            print(colored("--------- cruise ---------", "yellow"))
            os.system("ls /dev/ttyAMC*")
            cc_port = helper.get_serial_port()  # get serial ports
            self.c_controller = SC(cc_port)     # init CC controller
            print(colored("--------------------------", "yellow"))

    def init_ml_models(self):

        # steering models
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

        # cruise control
        if self.cruise_control:
            c_predictor = CruisePredictor()          # init cruise predictor
        else:
            c_predictor = None

        # segmentation
        segmentor = Segmentor("ENET")       # init segmentor
        seg_analyzer = SegAnalyzer(0.05)    # init seg analyzer

        # detection -- initialize yolo object detection
        if self.obj_det:
            detector = ObjectDetector()
        else:
            detector = None

        return steering_predictor, c_predictor, segmentor, seg_analyzer, detector

    def init_navigation(self):

        print(colored("------ GPS-------", "yellow"))
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

        cap = cv2.VideoCapture(0)

        if cap.isOpened():

            windowName = "self driving car... running"
            cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(windowName, 1100, 470)
            cv2.moveWindow(windowName, 0, 0)
            cv2.setWindowTitle(windowName, "self driving car")

            print(colored("[INFO]: program begins", "yellow"))

            while True:

                t = process_time()

                # -----------------------------------------------------
                # Check to see if the user closed the window
                if cv2.getWindowProperty(windowName, 0) < 0:
                    break
                ret_val, image = cap.read()
                image = cv2.resize(image, (640, 480))

                # ------------------ segmentation -------------------------
                # press down on "a" key to disable stopping
                if cv2.waitKey(33) == ord('a'):
                    self.__disable_stopping = not self.__disable_stopping

                result, seg_visual = self.segmentor.semantic_segmentation(image, visualize=self.seg_vis)
                speed, size = self.seg_analyzer.analyze_image(result)

                if self.__disable_stopping or speed is not 0:
                    self.c_controller.drive(1)
                else:
                    self.c_controller.drive(-1)
                    # time.sleep(2)

                # ------------------ detection ---------------------------
                # for detection visualization, I draw bounding boxes on top of
                # the steering visualizations. I want to condense several visualization
                # results into just one frame.
                if self.obj_det:
                    det_visualization = self.object_detector.detect_object(image, visualize=True)
                else:
                    det_visualization = image

                # --------------------------- steering --------------------
                if self.steering_model == "Rambo":
                    resize = cv2.cvtColor(cv2.resize(image, (256, 192))), cv2.COLOR_BGR2GRAY
                    angle = -1 * self.steering_predictor.predict(resize)
                elif self.steering_model == "Own":
                    angle = self.steering_predictor.predict(image)
                elif self.steering_model == "Autumn":
                    angle = self.steering_predictor.predict(image)
                else:
                    raise Exception("Not implemented: " + self.steering_model + ". Please enter a valid model type")

                # visualizing steering class activation
                # checks for the key if the user pressed "h"
                if cv2.waitKey(104) == ord('h'):
                    self.__steering_heatmap = not self.__steering_heatmap
                if self.__steering_heatmap:
                    str_act_vis = steering_visualization.visualize_class_activation_map(self.steering_predictor.cnn, image)
                else:
                    str_act_vis = det_visualization

                steering_img = steering_visualization.visualize_line(str_act_vis, speed_ms=0, angle_steers=angle, color=(0, 0, 255))

                # ------------ execute steering commands -------------
                self.mc.turn(configs.st_fac * angle)

                # ------------------ show the stuff ------------------

                str_screen = self.__info_screen.draw_steering_info_screen(angle=angle / 10, fps=self.__fps)
                cruise_screen = self.__info_screen.draw_cruise_info_screen(speed=speed, stopping_disabled=self.__disable_stopping, obj_size=size)

                steering_img = np.vstack((steering_img, str_screen))
                seg_visual = np.vstack((seg_visual, cruise_screen))
                vid_buffer = np.concatenate((steering_img, seg_visual), axis=1)

                elapsed_time = process_time() - t
                self.__fps = 1 / elapsed_time

                cv2.imshow(windowName, vid_buffer)

                key = cv2.waitKey(10)
                if key == 27:           # ESC key
                    cv2.destroyAllWindows()
                    print(colored("[INFO]: -------------------------", "yellow"))
                    print(colored("[INFO]: Thank you! Program ended.", "yellow"))
                    print(colored("[INFO]: -------------------------", "yellow"))
                    break

