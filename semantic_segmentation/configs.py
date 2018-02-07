#
# config.py: configuration for semantic segmentation
# (c) Neil Nie, 2017
# All Rights Reserved.
#


batch_size = 8
img_height = 512
img_width = 512
learning_rate = 1e-4
nb_epoch = 10

data_path = "/Volumes/Personal_Drive/Datasets/CityScapes/"
model_path = "./road_segmentation/segmentation-fcn-2.h5"
test_dataset = "/Volumes/Personal_Drive/Datasets/udacity-driving/steering_validation/"