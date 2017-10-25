import cv2, os
import numpy as np
import matplotlib.image as mpimg

IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS = 480, 640, 3
INPUT_SHAPE = (IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS)


def bgr_rgb(img):
    b, g, r = cv2.split(img)  # get b,g,r
    img = cv2.merge([r, g, b])  # switch it to rgb
    return img


def load_image(dir, image_file):
    """
    Load RGB images from a file
    """
    img = cv2.imread(dir + str(image_file))
    return bgr_rgb(img)


def random_flip(image, steering_angle):
    """
    Randomly flipt the image left <-> right, and adjust the steering angle.
    """
    if np.random.rand() < 0.5:
        image = cv2.flip(image, 1)
        steering_angle = -steering_angle
    return image, steering_angle

def rotate(img):
    row, col, channel = img.shape
    angle = np.random.uniform(-15, 15)
    rotation_point = (row / 2, col / 2)
    rotation_matrix = cv2.getRotationMatrix2D(rotation_point, angle, 1)
    rotated_img = cv2.warpAffine(img, rotation_matrix, (col, row))
    return rotated_img


def gamma(img):
    gamma = np.random.uniform(0.5, 1.2)
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
    new_img = cv2.LUT(img, table)
    return new_img


def blur(img):
    r_int = np.random.randint(0, 2)
    odd_size = 2 * r_int + 1
    return cv2.GaussianBlur(img, (odd_size, odd_size), 0)


def augument(dir, img_path, steering_angle):
    """
    Generate an augumented image and adjust steering angle.
    (The steering angle is associated with the center image)
    """
    image = load_image(dir, img_path)
    
    a = np.random.randint(0, 3, [1, 4]).astype('bool')[0]
    if a[0] == 1:
        image, steering_angle = random_flip(image, steering_angle)
    if a[1] == 1:
        image = blur(image)
    if a[2] == 1:
        image = gamma(image)
    if a[3] == 1:
        image, steering_angle = random_flip(image, steering_angle)
    return image, steering_angle

def batch_generator(dir, data, batch_size, is_training):
    """
    Generate training image give image paths and associated steering angles
    """
    images = np.empty([batch_size, IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS])
    steers = np.empty(batch_size)

    while True:
        i = 0
        for index in np.random.permutation(data.shape[0]):
            center_path = data[index][5]
            steering_angle = data[index][6]
            # argumentation
            if is_training and np.random.rand() < 0.6:
                image, steering_angle = augument(dir, center_path, steering_angle)
            else:
                image = load_image(dir, center_path)
            # add the image and steering angle to the batch
            images[i] = image
            steers[i] = steering_angle
            i += 1
            if i == batch_size:
                break
        yield images, steers