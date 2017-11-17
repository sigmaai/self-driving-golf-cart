import sys
import os

model_path = "./yolo_keras.h5"
anchors_path = "./configs/yolo_keras_anchors.txt"
classes_path = "./configs/coco_classes.txt"
test_path = "./images"
output_path = "./output"
score_threshold = 0.5
iou_threshold = 0.5
test_dataset = "/Volumes/Personal_Drive/Datasets/udacity-driving/validation/"
width = 480
height = 640