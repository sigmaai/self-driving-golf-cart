# -----------------------------------
# config.py, configerations for vechicle detection
# (c) Neil Nie, 2017
# All Rights Reserved.
# -----------------------------------

model_path = "./tiny-yolo.h5"
anchors_path = "./model_data/yolo_anchors.txt"
classes_path = "./model_data/pascal_classes.txt"
test_path = "./images"
output_path = "./output"
score_threshold = 0.5
iou_threshold = 0.5
test_dataset = "/Volumes/Personal_Drive/Datasets/udacity-driving/validation/"
width = 480
height = 640