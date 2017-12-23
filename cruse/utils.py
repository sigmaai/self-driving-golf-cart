import cv2
import numpy as np
import configs
import os


# --------------HELPER-METHODS-------------- #

def resize(image):
    """
    Resize the image to the input shape used by the network model
    """
    return cv2.resize(image, (configs.image_width, configs.image_heighte), cv2.INTER_AREA)


def rgb2yuv(image):
    """
    Convert the image from RGB to YUV (This is what the NVIDIA model does)
    """
    return cv2.cvtColor(image, cv2.COLOR_RGB2YUV)


def random_flip(image, steering_angle):
    """
    Randomly flipt the image left <-> right, and adjust the steering angle.
    """
    if np.random.rand() < 0.5:
        image = cv2.flip(image, 1)
        steering_angle = -steering_angle
    return image, steering_angle


def bgr_rgb(img):
    b, g, r = cv2.split(img)  # get b,g,r
    img = cv2.merge([r, g, b])  # switch it to rgb
    return img


def load_image(image_file):
    """
    Load RGB images from a file
    """
    img = cv2.imread(image_file)
    img = cv2.resize(img, (512, 512))
    return bgr_rgb(img)


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


def augument(img_path):
    """
    Generate an augumented image and adjust steering angle.
    (The steering angle is associated with the center image)
    """
    image = load_image(img_path)

    a = np.random.randint(0, 3, [1, 5]).astype('bool')[0]
    if a[0] == 1:
        image = random_shadow(image)
    if a[1] == 1:
        image = blur(image)
    if a[2] == 1:
        image = random_brightness(image)

    return image


def batch_generator(data, batch_size, augmentation=False):
    """
    Generate training image give image paths and associated steering angles
    """
    images = np.empty([batch_size, configs.image_height, configs.image_width, configs.image_channel])
    throttles = np.empty(batch_size)

    while True:
        i = 0
        for index in np.random.permutation(data.shape[0]):

            image_path = str(data[index][2])
            throttle = data[index][0]

            # argumentation
            if augmentation and np.random.rand() < 0.6:
                image = augument(configs.data_path + image_path + ".jpeg")
            else:
                image = load_image(configs.data_path + image_path + ".jpeg")
                # add the image and steering angle to the batch
            images[i] = image
            throttles[i] = throttle
            i += 1
            if i == batch_size:
                break

        yield images, throttles