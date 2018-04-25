import cv2
import numpy as np
import pandas as pd
from src.steering_control.scripts.weights.training import configs as configs
import os.path

INPUT_SHAPE = (configs.image_height, configs.image_width, 3)

# --------------HELPER-METHODS-------------- #


def resize(image):
    """
    Resize the image to the input shape used by the network model
    """
    return cv2.resize(image, (configs.image_width, configs.image_height), cv2.INTER_AREA)


def rgb2yuv(image):
    """
    Convert the image from RGB to YUV (This is what the NVIDIA model does)
    """
    return cv2.cvtColor(image, cv2.COLOR_RGB2YUV)


def preprocess(image):
    """
    Combine all preprocess functions into one
    """
    # image = crop(image)
    # image = image[53: 160, 0:320]
    image = cv2.resize(image, (configs.image_width, configs.image_height))
    # image = rgb2yuv(image)
    return image


def random_flip(image, steering_angle):
    """
    Randomly flipt the image left <-> right, and adjust the steering angle.
    """
    if np.random.rand() < 0.5:
        image = cv2.flip(image, 1)
        steering_angle = -steering_angle
    return image, steering_angle


def load_image(image_file):
    """
    Load RGB images from a file
    """
    img = cv2.imread(image_file)
    # removed the cropping part
    #img = img[0:480, 0:640]
    img = cv2.resize(img, (configs.image_width, configs.image_height))
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


# --------------MAIN-METHODS-------------- #

def gather_self_dataset(dirs):

    return_labels = None

    for path in dirs:

        labels = pd.read_csv(path + "/labels.csv").values
        for i in range(len(labels)):
            # add the dataset dir to the image path
            labels[i][2] = path + labels[i][2]

        if return_labels is None:
            return_labels = labels
        else:
            return_labels = np.concatenate((labels, return_labels), axis=0)

        print("one iteration finished")

    print(return_labels.shape)
    return return_labels


def preprocess_dataset(dir1, dir2):

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


def batch_generator(data, batch_size, is_training):
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
            center_path = str(data[index][2])
            steering_angle = data[index][4]

            if os.path.isfile(center_path):
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
            else:
                print(center_path)
                center_path = str(data[0][2])
                steering_angle = data[0][4]
                image = load_image(center_path)
                # add the image and steering angle to the batch
                images[i] = image
                steers[i] = steering_angle

        yield images, steers


def validation_generator(dir, data, batch_size):
    """
    Generate training image give image paths and associated steering angles
    """
    images = np.empty([batch_size, configs.image_height, configs.image_width, 3])
    steers = np.empty(batch_size)

    while True:
        i = 0
        for index in np.random.permutation(data.shape[0]):

            path = dir + "center/" + str(data["frame_id"][i]) + ".jpg"
            steering_angle = data["steering_angle"][1]

            image = load_image(path)
            images[i] = image
            steers[i] = steering_angle
            i += 1
            if i == batch_size:
                break

        yield images, steers
