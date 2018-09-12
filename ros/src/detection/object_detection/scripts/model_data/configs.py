# -----------------------------------
# config.py, configerations for vechicle detection
# (c) Neil Nie, 2017
# All Rights Reserved.
# -----------------------------------

model_path = "/home/neil/Workspace/self-driving-golf-cart/ros/src/detection/object_detection/scripts/model_data/yolo.h5"
anchors_path = "/home/neil/Workspace/self-driving-golf-cart/ros/src/detection/object_detection/scripts/model_data/yolo_anchors.txt"
classes_path = "/home/neil/Workspace/self-driving-golf-cart/ros/src/detection/object_detection/scripts/model_data/coco_classes.txt"
font_path = '/home/neil/Workspace/self-driving-golf-cart/ros/src/detection/object_detection/font/FiraMono-Medium.otf'

t_model_path = "./yolo.h5"
t_anchors_path = "./model_data/yolo_anchors.txt"
t_classes_path = "./model_data/pascal_classes.txt"

score_threshold = 0.5
iou_threshold = 0.5
test_dataset = "/Volumes/Personal_Drive/Datasets/udacity-driving/validation/"
width = 416
height = 416