#
# Drive.py | Created by Neil Nie & Michael Meng
# Main file of the self driving car
# (c) Neil Nie 2017, Please refer to the license

from steering.steering_predictor import SteeringPredictor
from detection.vehicle.vehicle_detector import VehicleDetector
import road_segmentation
import cv2
import numpy as np


if __name__ == '__main__':

    # initiate all detectors
    vehicle_detector = VehicleDetector()
    steering_predictor = SteeringPredictor()

    # OpenCV main loop
    cap = cv2.VideoCapture("nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)480, height=(int)640,format=(string)I420, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")
    image = cv2.imread("")

    if cap.isOpened():

        windowName = "car detection"
        cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(windowName, 700, 720)
        cv2.moveWindow(windowName, 0, 0)
        cv2.setWindowTitle(windowName, "car detection")

        while True:

            # -----------------------------------------------------
            # Check to see if the user closed the window
            if cv2.getWindowProperty(windowName, 0) < 0:
                # This will fail if the user closed the window; get printed to the console
                break
            ret_val, image = cap.read()

            # -----------------------------------------------------
            # run the network and detection
            print(image.shape)

    else:
        print("Fatal error, camera is not open")
