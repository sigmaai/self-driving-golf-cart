#
# config file for the steering module
# (c) Yongyang Nie, 2018. All Rights Reserved
# Self-driving car project.
#

# DATASET PATH
dir = "/Volumes/Personal_Drive/Datasets/udacity-driving/dataset-2/"
dir3 = "/Volumes/Personal_Drive/Datasets/udacity-driving/dataset-3/"
dir2 = "/Volumes/Personal_Drive/Datasets/udacity-driving/dataset-1/"
# val_dir = "/Volumes/Personal_Drive/Datasets/udacity-driving/steering_validation/"
# ---------------------------------------------------------------------------------

dataset_dirs = ["/home/neil/dataset/steering/train/part-1",
                "/home/neil/dataset/steering/train/part-2",
                "/home/neil/dataset/steering/train/part-3",
                "/home/neil/dataset/steering/train/part-4",
                "/home/neil/dataset/steering/train/part-5"
                ]
val_dir = "/home/neil/dataset/steering/val/"
# ---------------------------------------------------------------------------------

# TRAINING
image_width = 320
image_height = 160
load_weights = True

model_path = './str-cai-self-v3.h5'
# model_path = "./weights/own/trained-cai-v6.h5"
# model_path = "/Users/yongyangnie/Developer/self-driving-golf-cart/steering/weights/own/trained-cai-v6.h5"
# model_path = "./steering/weights/training/str-cai-self-v2.h5"

train_model_path = "./str-cai-self-v1.h5"

BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
BLUE =  (  0,   0, 255)
GREEN = (  0, 255,   0)
RED =   (255,   0,   0)

# -----------------------------------------

# define some constants

# RNNs are typically trained using (truncated) backprop through time. SEQ_LEN here is the length of BPTT.
# Batch size specifies the number of sequence fragments used in a sigle optimization step.
# (Actually we can use variable SEQ_LEN and BATCH_SIZE, they are set to constants only for simplicity).
# LEFT_CONTEXT is the number of extra frames from the past that we append to the left of our input sequence.
# We need to do it because 3D convolution with "VALID" padding "eats" frames from the left, decreasing
# the sequence length.One should be careful here to maintain the model's causality.

SEQ_LEN = 10
BATCH_SIZE = 4
LEFT_CONTEXT = 5

# These are the input image parameters.
HEIGHT = 480
WIDTH = 640
CHANNELS = 3

# The parameters of the LSTM that keeps the model state.
RNN_SIZE = 32
RNN_PROJ = 32

KEEP_PROB_TRAIN = 0.25

NUM_EPOCHS = 20

CSV_HEADER = "index, timestamp, width, height, frame_id, filename, angle, torque, speed, lat, long, alt".split(",")
OUTPUTS = CSV_HEADER[-6:-3]  # angle,torque,speed
OUTPUT_DIM = len(OUTPUTS)  # predict all features: steering angle, torque and vehicle speed

DS_PATH = "/Volumes/Personal_Drive/Datasets/udacity-driving/dataset-1/"
TRAIN_DS_PATH = "/Volumes/Personal_Drive/Datasets/udacity-driving/dataset-1/mac_train_center_inter.csv"
TEST_DS_PATH = "/Volumes/Personal_Drive/Datasets/udacity-driving/dataset-2/interpolated.csv"