#
# training utils
# helper methods for training deep learning models
# By Neil Nie
# (c) Yongyang Nie, 2018. All Rights Reserved
# Contact: contact@neilnie.com
#

import cv2
import numpy as np
import pandas as pd
import os
import configs


# -----------------------------------------------------------------------------------

class BatchGenerator(object):

    def __init__(self, sequence, seq_len, batch_size):
        self.sequence = sequence
        self.seq_len = seq_len
        self.batch_size = batch_size
        chunk_size = 1 + (len(sequence) - 1) / batch_size
        self.indices = [(i * chunk_size) % len(sequence) for i in range(batch_size)]

    def next(self):
        while True:
            output = []

            for i in range(self.batch_size):

                idx = int(self.indices[i])
                left_pad = self.sequence[idx - configs.LEFT_CONTEXT:idx]

                if len(left_pad) < configs.LEFT_CONTEXT:
                    left_pad = [self.sequence[0]] * (
                                configs.LEFT_CONTEXT - len(left_pad)) + left_pad
                assert len(left_pad) == configs.LEFT_CONTEXT

                leftover = len(self.sequence) - idx

                if leftover >= self.seq_len:
                    result = self.sequence[idx:idx + self.seq_len]
                else:
                    result = self.sequence[idx:] + self.sequence[:self.seq_len - leftover]
                assert len(result) == self.seq_len

                self.indices[i] = (idx + self.seq_len) % len(self.sequence)
                images, targets = zip(*result)
                images_left_pad, _ = zip(*left_pad)
                output.append((np.stack(images_left_pad + images), np.stack(targets)))

            output = list(zip(*output))
            output = [np.stack(output[0]), np.stack(output[1])]

            return output


def read_csv(filename):

    with open(filename, 'r') as f:

        lines = [ln.strip().split(",")[-7:-3] for ln in f.readlines()]
        lines = map(lambda x: (x[0], np.float32(x[1:])), lines)  # imagefile, outputs
        return lines


def process_csv(filename, val=5):

    print("processing csv files, please wait...")

    sum_f = np.float128([0.0] * configs.OUTPUT_DIM)
    sum_sq_f = np.float128([0.0] * configs.OUTPUT_DIM)
    lines = read_csv(filename)

    # leave val% for validation
    train_seq = []
    valid_seq = []
    count = 0

    for ln in lines:

        if count < configs.SEQ_LEN * configs.BATCH_SIZE * (100 - val):
            train_seq.append(ln)
            sum_f += ln[1]
            sum_sq_f += ln[1] * ln[1]
        else:
            valid_seq.append(ln)
        count += 1
        count %= configs.SEQ_LEN * configs.BATCH_SIZE * 100

    mean = sum_f / len(train_seq)
    var = sum_sq_f / len(train_seq) - mean * mean
    std = np.sqrt(var)
    print("train seq length: " + str(len(train_seq)), "val sequence length: " + str(len(valid_seq)))
    print("mean: " + str(mean), "standard deviation: " + str(
        std))  # we will need these statistics to normalize the outputs (and ground truth inputs)

    print("finished processing csv files!")

    return (train_seq, valid_seq), (mean, std)


# -----------------------------------------------------------------------------------


# --------------HELPER-METHODS-------------- #

def random_flip(image, steering_angle):
    """
    Randomly flipt the image left <-> right, and adjust the steering angle.
    """
    if np.random.rand() < 0.5:
        image = cv2.flip(image, 1)
        steering_angle = -steering_angle
    return image, steering_angle


def load_image(image_file):

    img = cv2.imread(image_file)
    img = cv2.resize(img, (
    configs.image_width, configs.image_height))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return img


def rotate(img):

    row, col, channel = img.shape
    angle = np.random.uniform(-15, 15)
    rotation_point = (row / 2, col / 2)
    rotation_matrix = cv2.getRotationMatrix2D(rotation_point, angle, 1)
    rotated_img = cv2.warpAffine(img, rotation_matrix, (col, row))
    return rotated_img


def blur(img):
    r_int = np.random.randint(0, 2)
    odd_size = 2 * r_int + 1
    return cv2.GaussianBlur(img, (odd_size, odd_size), 0)


def random_shadow(image):
    """
    Generates and adds random shadow
    """
    # (x1, y1) and (x2, y2) forms a line
    # xm, ym gives all the locations of the image
    x1, y1 = configs.image_width * np.random.rand(), 0
    x2, y2 = configs.image_width * np.random.rand(), configs.image_height
    xm, ym = np.mgrid[0:configs.image_height, 0:configs.image_width]

    # mathematically speaking, we want to set 1 below the line and zero otherwise
    # Our coordinate is up side down.  So, the above the line:
    # (ym-y1)/(xm-x1) > (y2-y1)/(x2-x1)
    # as x2 == x1 causes zero-division problem, we'll write it in the below form:
    # (ym-y1)*(x2-x1) - (y2-y1)*(xm-x1) > 0
    mask = np.zeros_like(image[:, :, 1])
    mask[(ym - y1) * (x2 - x1) - (y2 - y1) * (xm - x1) > 0] = 1

    # choose which side should have shadow and adjust saturation
    cond = mask == np.random.randint(2)
    s_ratio = np.random.uniform(low=0.2, high=0.5)

    # adjust Saturation in HLS(Hue, Light, Saturation)
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    hls[:, :, 1][cond] = hls[:, :, 1][cond] * s_ratio
    return cv2.cvtColor(hls, cv2.COLOR_HLS2RGB)


def random_translate(image, steering_angle, range_x, range_y):
    """
    Randomly shift the image virtially and horizontally (translation).
    """
    trans_x = range_x * (np.random.rand() - 0.5)
    trans_y = range_y * (np.random.rand() - 0.5)
    steering_angle += trans_x * 0.002
    trans_m = np.float32([[1, 0, trans_x], [0, 1, trans_y]])
    height, width = image.shape[:2]
    image = cv2.warpAffine(image, trans_m, (width, height))
    return image, steering_angle


def random_brightness(image):
    """
    Randomly adjust brightness of the image.
    """
    # HSV (Hue, Saturation, Value) is also called HSB ('B' for Brightness).
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    ratio = 1.0 + 0.4 * (np.random.rand() - 0.5)
    hsv[:, :, 2] = hsv[:, :, 2] * ratio
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)


def augument(img_path, steering_angle):
    """
    Generate an augumented image and adjust steering angle.
    (The steering angle is associated with the center image)
    """
    image = load_image(img_path)

    a = np.random.randint(0, 3, [1, 5]).astype('bool')[0]
    if a[0] == 1:
        image = image ## no random shadow
    if a[1] == 1:
        image = blur(image)
        image = image
    if a[2] == 1:
        image = random_brightness(image)
    if a[3] == 1:
        image, steering_angle = random_flip(image, steering_angle)
    if a[4] == 1:
        image, steering_angle = random_translate(image, steering_angle, 10, 10)
    return image, steering_angle



# ----------------------------------------------------
# gather the self-collected dataset
# return a numpy array with all the datat and its path
# ----------------------------------------------------
def process_self_dataset(dirs):

    return_labels = None

    for path in dirs:

        labels = pd.read_csv(path + "/labels.csv").values
        for i in range(1, len(labels)):
            # add the dataset dir to the image path
            labels[i][0] = path + labels[i][0]

        if return_labels is None:
            return_labels = labels
        else:
            return_labels = np.concatenate((labels, return_labels), axis=0)

        print("%s finished loading", path)

    print(return_labels.shape)
    return return_labels


# ----------------------------------------------------
# gather the udacity dataset
# return a numpy array with all the datat and its path
# ----------------------------------------------------
def preprocess_udacity_dataset(dir1, dir2):

    data1 = pd.read_csv(dir1 + "interpolated.csv").values
    data2 = pd.read_csv(dir2 + "interpolated.csv").values
    # data3 = pd.read_csv(dir3 + "center_interpolated.csv").values

    print("begin processing dataset 1")
    labels1 = np.array([data1[1]])
    labels1[0][5] = dir1 + labels1[0][5]
    for i in range(0, len(data1)):
        if data1[i][4] == "center_camera":
            item = np.array([data1[i]])
            item[0][5] = dir1 + item[0][5]
            labels1 = np.concatenate((labels1, item), axis=0)

    print("dataset 1 processing completed")
    print("begin processing dataset 2")

    labels2 = np.array([data2[1]])
    labels2[0][5] = dir2 + labels2[0][5]
    for i in range(0, len(data2)):
        if data2[i][4] == "center_camera":
            item = np.array([data2[i]])
            item[0][5] = dir2 + item[0][5]
            labels2 = np.concatenate((labels2, item), axis=0)

    print("dataset 2 processing completed")
    return np.concatenate((labels1, labels2), axis=0)


def udacity_batch_generator(data, batch_size, is_training):
    """
    Generate training image give image paths and associated steering angles
    """
    images = np.empty([batch_size, configs.image_height, configs.image_width, 3])
    steers = np.empty(batch_size)

    while True:
        i = 0
        for index in np.random.permutation(data.shape[0]):
            center_path = str(data[index][5])
            steering_angle = data[index][6]

            # argumentation
            if is_training and np.random.rand() < 0.6:
                image, steering_angle = augument(center_path, steering_angle)
            else:
                image = load_image(center_path)
                # add the image and steering angle to the batch
            images[i] = image
            steers[i] = steering_angle
            i += 1
            if i == batch_size:
                break

        yield images, steers


def self_batch_generator(data, batch_size, is_training):
    """
    Generate training image give image paths and associated steering angles
    """
    images = np.empty([batch_size, configs.image_height, configs.image_width, 3])
    steers = np.empty(batch_size)

    while True:
        i = 0
        for index in np.random.permutation(data.shape[0]):
            center_path = str(data[index][0])
            steering_angle = data[index][2]

            if os.path.isfile(center_path):
                # argumentation
                if is_training and np.random.rand() < 0.6:
                    image, steering_angle = augument(center_path, steering_angle)
                else:
                    image = load_image(center_path)
                    # add the image and steering angle to the batch

                images[i] = image
                steers[i] = steering_angle
            else:
                center_path = str(data[2][0])
                steering_angle = data[2][2]
                image = load_image(center_path)
                # add the image and steering angle to the batch
                images[i] = image
                steers[i] = steering_angle
            i += 1
            if i == batch_size:
                break

        yield images, steers


def validation_generator(data, batch_size):
    """
    Generate training image give image paths and associated steering angles
    """
    images = np.empty([batch_size, configs.image_height, configs.image_width, 3])
    steers = np.empty(batch_size)

    while True:
        i = 0
        for index in np.random.permutation(data.shape[0]):
            center_path = str(data[index][0])
            steering_angle = data[index][2]

            if os.path.isfile(center_path):
                image = load_image(center_path)
                images[i] = image
                steers[i] = steering_angle
            else:
                center_path = str(data[2][0])
                steering_angle = data[2][2]
                image = load_image(center_path)
                # add the image and steering angle to the batch
                images[i] = image
                steers[i] = steering_angle
            i += 1
            if i == batch_size:
                break

        yield images, steers



