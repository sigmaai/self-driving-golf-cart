#
# Drive.py | Created by Neil Nie & Michael Meng
# Main file of the self driving car
# (c) Neil Nie 2017, Please refer to the license
#
import time
from steering.steering_predictor import SteeringPredictor
from steering.mc import MC
from detection.vehicle.vehicle_detector import VehicleDetector
# from road_segmentation.road_segmentationroad_segmentor import RoadSegmentor
import cv2
import numpy as np
from path_planning.gps import GPS 
from path_planning.global_path import GlobalPathPlanner

l = 30


def get_destination():

    var = input("Please enter your destination:")
    return str(var)


if __name__ == '__main__':

    # initiate all detectors
    vehicle_detector = VehicleDetector()
    steering_predictor = SteeringPredictor()
    motor_controller = MC()
    
    # initiate path planner, including GPS and Google Maps API
    #gps = GPS()
    #start = gps.query_gps_location()
    #destination = get_destination()
    #gp_planner = GlobalPathPlanner()
    #directions = gp_planner.direction(start, destination)
    #print(directions)
    

    # OpenCV main loop
    
    
    cap = cv2.VideoCapture("nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480,format=(string)I420, framerate=(fraction)1/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")

    if cap.isOpened():

        windowName = "car detection"
        cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(windowName, 1280, 480)
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
            # run detecion network
            # no image preprocessing required for any detector
            # detection_img, out_boxes, out_scores, out_classes = vehicle_detector.detect_vechicle(image)
            
            angle, steering_img = steering_predictor.predict_steering(image)
            motor_controller.turn(l * angle)
            print('turning ' + str(l * angle))
            time.sleep(0.2) 
            #print(motor_controller.pos()) 
            #detection_img = cv2.resize(detection_img, (640, 480))
            vidBuf = np.concatenate((steering_img, steering_img), axis=1)
            displayBuf = vidBuf

            # show the stuff
            # -----------------------------------------------------

            cv2.imshow(windowName, displayBuf)
            key = cv2.waitKey(10)
            if key == 27:  # ESC key
                cv2.destroyAllWindows()
                break


    else:
        print("Fatal error, camera is not open")

