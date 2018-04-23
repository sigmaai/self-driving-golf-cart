#
# config.py: configuration for semantic segmentation
# (c) Neil Nie, 2017
# All Rights Reserved.
#

batch_size = 8
img_height = 360
img_width = 640
learning_rate = 1e-4
nb_epoch = 10

data_path = "/Volumes/Personal_Drive/Datasets/CityScapes/"
# model_path = "./semantic_segmentation/enet-c-v1-3.h5"
model_path = "./weights/enet-c-v1-3.h5"
infer_model_path = "./semantic_segmentation/weights/enet-c-v1-3.h5"
test_dataset = "/Volumes/Personal_Drive/Datasets/CityScapes/"
