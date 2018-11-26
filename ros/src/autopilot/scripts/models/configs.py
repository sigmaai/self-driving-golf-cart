#
# config file for the steering module
# (c) Yongyang Nie, 2018. All Rights Reserved
# Self-driving car project.
#

# DATASET PATH
dir = "/Volumes/Personal_Drive/Datasets/udacity-driving/dataset-2/"
dir3 = "/Volumes/Personal_Drive/Datasets/udacity-driving/dataset-3/"
dir2 = "/Volumes/Personal_Drive/Datasets/udacity-driving/dataset-1/"
# ---------------------------------------------------------------------------------

dataset_dirs = ["/home/neil/dataset/steering/train/part-1",
                "/home/neil/dataset/steering/train/part-2",
                "/home/neil/dataset/steering/train/part-3",
                "/home/neil/dataset/steering/train/part-4",
                "/home/neil/dataset/steering/train/part-5"
                ]
val_dir = "/home/neil/dataset/steering/val/"

# ---------------------------------------------------------------------------------

model_path = './str-cai-self-v3.h5'

BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
BLUE =  (  0,   0, 255)
GREEN = (  0, 255,   0)
RED =   (255,   0,   0)

# -----------------------------------------

IMG_WIDTH = 224
IMG_HEIGHT = 224
CHANNELS = 3
LENGTH = 64

VAL_DIR = '/home/neil/dataset/steering/test/'
CC_VAL_DIR = '/home/neil/dataset/udacity/'

MODEL_PATH = ""

LOG_PATH = './logs/64_rgb'