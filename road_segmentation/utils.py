# Copyright 2015 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

import re
import cv2
import configs
import numpy as np
import os
import random
import scipy.misc
from glob import glob


def bc_img(img, s = 1.0, m = 0.0):
    img = img.astype(np.int)
    img = img * s + m
    img[img > 255] = 255
    img[img < 0] = 0
    img = img.astype(np.uint8)
    return img


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


def gen_batch_function(data_folder, image_shape, batch_size):
    """
    Generate function to create batches of training data
    :param data_folder: Path to folder that contains all the datasets
    :param image_shape: Tuple - Shape of image
    :return:
    """

    image_paths = glob(os.path.join(data_folder, 'image_2', '*.png'))
    label_paths = {
        re.sub(r'_(lane|road)_', '_', os.path.basename(path)): path
        for path in glob(
        os.path.join(data_folder, 'gt_image_2', '*_road_*.png')
        )
    }

    background_color = np.array([255, 0, 0])
    sideroad_color = np.array([0, 0, 0])
    road_color = np.array([255, 0, 255])

    while 1:

        random.shuffle(image_paths)

        images = []
        gt_images = []

        for image_file in image_paths[0: batch_size]:

            gt_image_file = label_paths[os.path.basename(image_file)]

            image = scipy.misc.imread(image_file)
            gt_image = scipy.misc.imread(gt_image_file)

            image = scipy.misc.imresize(image, image_shape)
            gt_image = scipy.misc.imresize(gt_image, image_shape)
            gt_bg = np.all(gt_image == background_color, axis=2)
            gt_bg = gt_bg.reshape(*gt_bg.shape, 1)
            gt_image = np.concatenate((gt_bg, np.invert(gt_bg)), axis=2)

            images.append(image)
            gt_images.append(gt_image)

        yield np.array(images), np.array(gt_images)

