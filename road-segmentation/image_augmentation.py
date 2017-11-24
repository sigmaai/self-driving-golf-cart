"""
Fairly basic set of tools for data augmentation on images.
"""

import numpy as np
import random
import cv2
from os.path import join, expanduser
import matplotlib.pyplot as plt


def perform_augmentation(batch_x, batch_y):
    """
    Perform basic data augmentation on image batches.
    
    Parameters
    ----------
    batch_x: ndarray of shape (b, h, w, c)
        Batch of images in RGB format, values in [0, 255]
    batch_y: ndarray of shape (b, h, w, c)
        Batch of ground truth with road segmentation
        
    Returns
    -------
    batch_x_aug, batch_y_aug: two ndarray of shape (b, h, w, c)
        Augmented batches
    """
    def mirror(x):
        return x[:, ::-1, :]

    def augment_in_hsv_space(x_hsv):
        print(x_hsv.shape)
        x_hsv = np.float32(cv2.cvtColor(x_hsv, cv2.COLOR_RGB2HSV))
        x_hsv[:, :, 0] = x_hsv[:, :, 0] * random.uniform(0.9, 1.1)   # change hue
        x_hsv[:, :, 1] = x_hsv[:, :, 1] * random.uniform(0.5, 2.0)   # change saturation
        x_hsv[:, :, 2] = x_hsv[:, :, 2] * random.uniform(0.5, 2.0)   # change brightness
        x_hsv = np.uint8(np.clip(x_hsv, 0, 255))
        return cv2.cvtColor(x_hsv, cv2.COLOR_HSV2RGB)

    batch_x_aug = np.copy(batch_x)
    batch_y_aug = np.copy(batch_y)

    for b in range(batch_x_aug.shape[0]):

        # Random mirroring
        should_mirror = random.choice([True, False])
        if should_mirror:
            batch_x_aug[b] = mirror(batch_x[b])
            batch_y_aug[b] = mirror(batch_y[b])

        # Random change in image values (hue, saturation, brightness)
        print(batch_x_aug[b])
        batch_x_aug[b] = augment_in_hsv_space(batch_x_aug[b])

    return batch_x_aug, batch_y_aug

