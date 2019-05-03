#!/usr/bin/python

import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from keras.utils import to_categorical
import utils
import cv2

from models.icnet_fusion_old import ICNet_o
import configs

#### Test ####

# define global variables
model_type = 'cross_fusion'
n_classes = 34
checkpoint_path = '/home/neil/Workspace/semantic-segmentation/monodepth/models/cityscape/model_cityscapes.data-00000-of-00001'
model_path = 'icnet_' + model_type + '_v1_030_0.861.h5'
test_csv_path = "./new_val_labels.csv"

# ====== Model ======
net = ICNet_o(width=configs.img_width, height=configs.img_height, n_classes=34, weight_path="output/" + model_path,
            mode=model_type)
print(net.model.summary())


def test_fusion():

    labels = pd.read_csv(test_csv_path).values

    total_score = 0

    # ====== running... ======
    for i in range(len(labels)):

        test_img_path = labels[i][0]
        test_gt_path = labels[i][1]
        test_depth_path = labels[i][2]

        # ======== Testing ========
        x = cv2.resize(cv2.imread(test_img_path, 1), (configs.img_width, configs.img_height))
        x_depth = cv2.resize(cv2.imread(test_depth_path, 1), (configs.img_width, configs.img_height))
        gt = cv2.imread(test_gt_path, 0)
        gt = to_categorical(cv2.resize(gt, (gt.shape[1] // 4, gt.shape[0] // 4)), n_classes)

        x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)

        X_color = np.zeros((1, configs.img_height, configs.img_width, 3), dtype='float32')
        X_depth = np.zeros((1, configs.img_height, configs.img_width, 3), dtype='float32')

        X_color[0, :, :, :] = x
        X_depth[0, :, :, :] = x_depth

        prediction = net.model.predict([X_color, X_depth])[0]
        prediction = convert_to_binary_classification(cv2.resize(prediction, (prediction.shape[1] * 2, prediction.shape[0] * 2)))

        total_u = 0.0
        total_i = 0.0

        for j in range(n_classes):

            # plt.imshow(gt[:, :, j], cmap='gray')
            # plt.show()
            # plt.imshow(prediction[:, :, j], cmap='gray')
            # plt.show()
            # print(j)

            intersection = np.logical_and(gt[:, :, j], prediction[:, :, j])
            union = np.logical_or(gt[:, :, j], prediction[:, :, j])

            total_i = total_i + float(np.sum(intersection))
            total_u = total_u + float(np.sum(union))

        # print(total_i / total_u)
        total_score = total_score + (total_i / total_u)

    print("overall iou score")
    print(total_score / len(labels))


def convert_to_binary_classification(image_labels, threshold=0.80):

    # convert any pixel > threshold to 1
    # convert any pixel < threshold to 0
    # then use bitwise_and

    output = np.zeros((configs.img_height / 2, configs.img_width / 2, 34), dtype=np.uint8)

    for i in range(34):

        split = image_labels[:, :, i]
        split[split > threshold] = 1
        split[split < threshold] = 0
        split = split.astype(np.uint8)

        output[:, :, i] = split

    return output


if __name__ == "__main__":

    test_fusion()

