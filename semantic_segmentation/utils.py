# 
# utilities for semantic segmentation
# autonomous golf cart project
# (c) Yongyang Nie, Michael Meng
# ==============================================================================
#

import re
import cv2
import semantic_segmentation.configs as configs
#import configs
import numpy as np
import os
import scipy.misc
from glob import glob
from collections import namedtuple
import time


Label = namedtuple('Label', [

    'name'        , # The identifier of this label, e.g. 'car', 'person', ... .
                    # We use them to uniquely name a class

    'id'          , # An integer ID that is associated with this label.
                    # The IDs are used to represent the label in ground truth images
                    # An ID of -1 means that this label does not have an ID and thus
                    # is ignored when creating ground truth images (e.g. license plate).
                    # Do not modify these IDs, since exactly these IDs are expected by the
                    # evaluation server.

    'trainId'     , # Feel free to modify these IDs as suitable for your method. Then create
                    # ground truth images with train IDs, using the tools provided in the
                    # 'preparation' folder. However, make sure to validate or submit results
                    # to our evaluation server using the regular IDs above!
                    # For trainIds, multiple labels might have the same ID. Then, these labels
                    # are mapped to the same class in the ground truth images. For the inverse
                    # mapping, we use the label that is defined first in the list below.
                    # For example, mapping all void-type classes to the same ID in training,
                    # might make sense for some approaches.
                    # Max value is 255!

    'category'    , # The name of the category that this label belongs to

    'categoryId'  , # The ID of this category. Used to create ground truth images
                    # on category level.

    'hasInstances', # Whether this label distinguishes between single instances or not

    'ignoreInEval', # Whether pixels having this class as ground truth label are ignored
                    # during evaluations or not

    'color'       , # The color of this label
    ])

# (NOTE! this is taken from the official Cityscapes scripts:)
labels = [
    #       name                     id    trainId   category            catId     hasInstances   ignoreInEval   color
    Label(  'ego vehicle'          ,  1 ,      19 , 'void'            , 0       , False        , True         , (0, 0, 0) ),
    Label(  'ground'               ,  6 ,      19 , 'void'            , 0       , False        , True         , (81, 0, 81) ),
    Label(  'road'                 ,  7 ,       0 , 'flat'            , 1       , False        , False        , (128, 64, 128) ),
    Label(  'sidewalk'             ,  8 ,       1 , 'flat'            , 1       , False        , False        , (244, 35, 232) ),
    Label(  'building'             , 11 ,       2 , 'construction'    , 2       , False        , False        , (70, 70, 70) ),
    Label(  'traffic sign'         , 20 ,       7 , 'object'          , 3       , False        , False        , (220, 220,  0) ),
    Label(  'vegetation'           , 21 ,       8 , 'nature'          , 4       , False        , False        , (107, 142, 35) ),
    Label(  'terrain'              , 22 ,       9 , 'nature'          , 4       , False        , False        , (152, 251, 152) ),
    Label(  'sky'                  , 23 ,       10 , 'sky'             , 5       , False        , False        ,(70, 130, 180) ),

    Label(  'person'               , 24 ,       11 , 'human'           , 6       , True         , False        , (220, 20, 60) ),
    Label(  'car'                  , 26 ,       13 , 'vehicle'         , 7       , True         , False        , (0, 0, 142) ),
    Label(  'truck'                , 27 ,       14 , 'vehicle'         , 7       , True         , False        , (0, 0, 70) ),
    Label(  'bicycle'              , 33 ,       18 , 'vehicle'         , 7       , True         , False        , (119, 11, 32) ),

    # Label(  'dynamic'              ,  5 ,      19 , 'void'            , 0       , False        , True         , (111, 74,  0) ),
    # Label(  'rider'                , 25 ,       12 , 'human'           , 6       , True         , False        , (255, 0, 0) ),
    # Label(  'guard rail'           , 14 ,      19 , 'construction'    , 2       , False        , True         , (180, 165,180) ),
    # Label(  'out of roi'           ,  3 ,      19 , 'void'            , 0       , False        , True         , (0,  0,  0) ),
    # Label(  'rectification border' ,  2 ,      19 , 'void'            , 0       , False        , True         , (0,  0,  0) ),
    # Label(  'unlabeled'            ,  0 ,      19 , 'void'            , 0       , False        , True         , (0, 0, 0) ),
    # Label(  'static'               ,  4 ,      19 , 'void'            , 0       , False        , True         , (0,  0,  0) ),
    # Label(  'bus'                  , 28 ,       15 , 'vehicle'         , 7       , True         , False        , (0, 60, 100) ),
    # Label(  'motorcycle'           , 32 ,       17 , 'vehicle'         , 7       , True         , False        , (0,  0, 230) ),
    # Label(  'traffic light'        , 19 ,       6 , 'object'          , 3       , False        , False        , (250, 170, 30) ),
    # Label(  'wall'                 , 12 ,       3 , 'construction'    , 2       , False        , False        , (102, 102, 156) ),
    # Label(  'parking'              ,  9 ,      19 , 'flat'            , 1       , False        , True         , (250, 170, 160) ),
    # Label(  'fence'                , 13 ,       4 , 'construction'    , 2       , False        , False        , (190, 153,153) ),
    # Label(  'license plate'        , -1 ,       -1 , 'vehicle'         , 7       , False        , True         , (0, 0, 142) ),
    # Label(  'caravan'              , 29 ,      19 , 'vehicle'         , 7       , True         , True         , (  0,  0, 90) ),
    # Label(  'trailer'              , 30 ,      19 , 'vehicle'         , 7       , True         , True         , (  0,  0,110) ),
    # Label(  'train'                , 31 ,       16 , 'vehicle'         , 7       , True         , False        , (  0, 80,100) ),
    # Label(  'bridge'               , 15 ,      19 , 'construction'    , 2       , False        , True         , (150,100,100) ),
    # Label(  'tunnel'               , 16 ,      19 , 'construction'    , 2       , False        , True         , (150,120, 90) ),
    # Label(  'pole'                 , 17 ,        5 , 'object'          , 3       , False        , False        , (153,153,153) ),
    # Label(  'polegroup'            , 18 ,      19 , 'object'          , 3       , False        , True         , (153,153,153) ),
    # Label(  'rail track'           , 10 ,      19 , 'flat'            , 1       , False        , True         , (230,150,140) ),
]


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
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (configs.img_width, configs.img_height))
    return img


def convert_rgb_to_class(image):

    outputs = []

    for i in range(len(labels)):

        label = labels[i]

        color = np.array(label[7], dtype=np.uint8)
        # objects found in the frame.
        mask = cv2.inRange(image, color, color)

        # divide each pixel by 255
        mask = np.true_divide(mask, 255)

        if len(outputs) == 0:
            outputs = mask
        else:
            outputs = np.dstack((outputs, mask))

    return outputs


def convert_class_to_rgb(image_labels, threshold=0.05):

    # convert any pixel > threshold to 1
    # convert any pixel < threshold to 0
    # then use bitwise_and

    output = np.zeros((configs.img_height, configs.img_width, 3), dtype=np.uint8)

    for i in range(len(labels)):

        if i != 44:
            split = image_labels[:, :, i]
            split[split > threshold] = 1
            split[split < threshold] = 0
            split[:] *= 255
            split = split.astype(np.uint8)
            color = labels[i][7]

            bg = np.zeros((configs.img_height, configs.img_width, 3), dtype=np.uint8)
            bg[:, :, 0].fill(color[0])
            bg[:, :, 1].fill(color[1])
            bg[:, :, 2].fill(color[2])

            res = cv2.bitwise_and(bg, bg, mask=split)

            # plt.imshow(np.hstack([bg, res]))
            # plt.show()

            output = cv2.addWeighted(output, 1.0, res, 1.0, 0)

    return output


def validation_generator(labels, batch_size):

    batch_images = np.zeros((batch_size, configs.img_height, configs.img_width, 3))
    batch_masks = np.zeros((batch_size, configs.img_height, configs.img_width, 3))

    while 1:

        for index in np.random.permutation(len(labels)):

            label = labels[index]
            image = load_image(configs.data_path + "leftImg8bit/val/" + label[1])
            gt_image = load_image(configs.data_path + "gtFine/val/" + label[2])

            batch_images[index] = image
            batch_masks[index] = gt_image

        yield batch_images, batch_masks



def train_generator(ls, batch_size):

    batch_images = np.zeros((batch_size, configs.img_height, configs.img_width, 3))
    batch_masks = np.zeros((batch_size, configs.img_height, configs.img_width, len(labels)))

    while 1:
        i = 0
        for index in np.random.permutation(len(ls)):

            label = ls[index]
            image = load_image(configs.data_path + label[0])
            gt_image = load_image(configs.data_path + label[1])

            batch_images[i] = image
            batch_masks[i] = convert_rgb_to_class(gt_image)
            i += 1
            if i == batch_size:
                break

        yield batch_images, batch_masks


if __name__ == "__main__":

    img = load_image("./testing_imgs/test.png")
    print(img.shape)
    array = convert_rgb_to_class(img)
    image = convert_class_to_rgb(array)
