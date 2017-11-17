import cv2
import configs
import numpy as np
import os
import random


def prepare_dataset(path):

    inputs = os.listdir(path)
    imgs = os.listdir(path)

    for i in range(len(imgs)):
        imgs[i] = imgs[i][:-11] + "_road" + imgs[i][-11:]

    return inputs, imgs


def load_image(path):

    img = cv2.imread(path)
    img = cv2.resize(img, (configs.img_width, configs.img_height))
    return img

def train_generator(inputs, masks, batch_size):

    batch_images = np.zeros((batch_size, configs.img_height, configs.img_width, 3))
    batch_masks = np.zeros((batch_size, configs.img_height, configs.img_width, 1))
    while 1:

        combined = list(zip(inputs, masks))
        random.shuffle(combined)

        inputs[:], masks[:] = zip(*combined)

        for i_batch in range(batch_size):

            i_line = np.random.randint(len(inputs))
            img = cv2.cvtColor(load_image(configs.data_path + inputs[i_line]), cv2.COLOR_BGR2RGB)
            img_mask = cv2.cvtColor(load_image(configs.mask_path + masks[i_line]), cv2.COLOR_BGR2RGB)

            batch_images[i_batch] = img
            batch_masks[i_batch] = np.reshape(img_mask, (np.shape(img_mask)[0], np.shape(img_mask)[1], 1))
        yield batch_images, batch_masks
