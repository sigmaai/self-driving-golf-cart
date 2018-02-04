#
# ------------------- AGC ----------------------
# Drive.py | Created by Neil Nie & Michael Meng
#      Main file of the self driving car
# (c) Neil Nie 2017, Please refer to the license
# ----------------------------------------------
#


import os
from steering.steering_predictor import SteeringPredictor
from steering.mc import MC
from detection.vehicle.vehicle_detector import VehicleDetector
from semantic_segmentation.segmentor import Segmentor
import cv2
import numpy as np
import configs.configs as configs
from path_planning.gps import GPS 
from path_planning.global_path import GlobalPathPlanner


def get_destination():
    var = input("Please enter your destination:")
    return str(var)


def get_serial_port():
    var = input("Please enter serial port number")
    return int(var)


if __name__ == '__main__':

    # initialize all objects
    segmentor = Segmentor("SGN")                # init segmentor
    vehicle_detector = VehicleDetector()        # init vechicle detector
    steering_predictor = SteeringPredictor()    # init steering predictor

    if configs.default_st_port:                 # check for serial setting
        mc = MC()
    else:
        print("---------------------")
        os.system("ls /dev/ttyUSB*")
        st_port = get_serial_port()
        mc = MC(st_port)
        print("---------------------")

    # initiate path planner, including GPS and Google Maps API
    if configs.navigation:                      # enabling navigation
        gps = GPS()
        start = gps.query_gps_location()
        destination = get_destination()
        gp_planner = GlobalPathPlanner()
        directions = gp_planner.direction(start, destination)
        print(directions)

    # OpenCV main loop

    cap = cv2.VideoCapture("nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480,format=(string)I420, framerate=(fraction)1/1 ! nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")

    if cap.isOpened():

        windowName = "car detection"
        cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(windowName, 1280, 960)
        cv2.moveWindow(windowName, 0, 0)
        cv2.setWindowTitle(windowName, "car detection")

        while True:
            # 	time.sleep(0.5)
            # -----------------------------------------------------
            # Check to see if the user closed the window
            if cv2.getWindowProperty(windowName, 0) < 0:
                break
            ret_val, image = cap.read()

            # -----------------------------------------------------
            # run detection network
            # no image preprocessing required for any detector
            if configs.detection:
                detection_img, out_boxes, out_scores, out_classes = vehicle_detector.detect_vechicle(image)
            
            angle, steering_img = steering_predictor.predict_steering(image)
            mc.turn(configs.st_fac * angle)

            if configs.detection:
                detection_img = cv2.resize(detection_img, (640, 480))
                buff1 = np.concatenate((steering_img, detection_img), axis=1)
            else:
                buff1 = np.concatenate((steering_img, steering_img), axis=1)    # not running detection
                                                                                # showing steering image buffer

            if configs.segmentation:                                            # running segmentation
                segment_result = segmentor.segment_road(image)
                buff2 = np.concatenate((segment_result, segment_result))
            else:                                                               # not running segmentation
                buff2 = np.concatenate((image, image))                          # show original images

            vidBuf = np.concatenate((buff1, buff2), axis=0)

            # show the stuff
            # -----------------------------------------------------

            cv2.imshow(windowName, vidBuf)
            key = cv2.waitKey(10)
            if key == 27:  # ESC key
                cv2.destroyAllWindows()
                print("-------------------------")
                print("Thank you! Program ended.")
                print("-------------------------")
                break

    else:
        print("Fatal error, camera is not open")

