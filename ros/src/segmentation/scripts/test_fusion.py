#!/usr/bin/python

import argparse
import time
import numpy as np
import matplotlib.pyplot as plt
from monodepth.monodepth_runner import monodepth_runner
import utils
import cv2

from models.icnet_fusion import ICNet
import configs

#### Test ####

# define global variables
model_type = 'cross_fusion'
checkpoint_path = '/home/neil/Workspace/semantic-segmentation/monodepth/models/cityscape/model_cityscapes.data-00000-of-00001'
model_path = 'icnet_' + model_type + '_030_0.869.h5'
test_img_path = "./testing_imgs/731.png"

# ==== create monodepth runner ====
depth_runner = monodepth_runner(checkpoint_path)

# ====== Model ======
net = ICNet(width=configs.img_width, height=configs.img_height, n_classes=34, weight_path="output/" + model_path,
            mode=model_type)
print(net.model.summary())


def visualization_result(y, mid):
    y = cv2.resize(y, (configs.img_width / 2, configs.img_height / 2))
    image = utils.convert_class_to_rgb(y, threshold=0.50)
    viz = cv2.addWeighted(mid, 0.8, image, 0.8, 0)
    plt.figure(1)
    plt.imshow(viz)
    plt.show()

    cv2.imwrite('seg_result_overlay.png', cv2.resize(cv2.cvtColor(viz, cv2.COLOR_RGB2BGR), (1024, 512)))


def test_fusion():

    # ======== Testing ========
    x = cv2.resize(cv2.imread(test_img_path, 1), (configs.img_width, configs.img_height))
    x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)
    x_depth = depth_runner.run_depth(image_path=test_img_path, out_height=configs.img_height,
                                     out_width=configs.img_width)
    x_depth = np.dstack((x_depth, x_depth, x_depth))

    mid = cv2.resize(x, (configs.img_width / 2, configs.img_height / 2))

    X_color = np.zeros((1, configs.img_height, configs.img_width, 3), dtype='float32')
    X_depth = np.zeros((1, configs.img_height, configs.img_width, 3), dtype='float32')

    X_color[0, :, :, :] = x
    X_depth[0, :, :, :] = x_depth

    y = net.model.predict([X_color, X_depth], verbose=1)[0]

    # ====== running... ======
    start_time = time.time()
    for i in range(10):
        y = net.model.predict([X_color, X_depth])[0]

    duration = time.time() - start_time
    print('Generated segmentations in %s seconds -- %s FPS' % (duration / 10, 1.0 / (duration / 10)))

    visualization_result(y=y, mid=mid)


def test_early_fusion():

    # ======== Testing ========
    x = cv2.resize(cv2.imread(test_img_path, 1), (configs.img_width, configs.img_height))
    x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)
    x_depth = depth_runner.run_depth(image_path=test_img_path, out_height=configs.img_height,
                                     out_width=configs.img_width)
    x_depth = np.dstack((x_depth, x_depth, x_depth))

    plt.imshow(x_depth)
    plt.show()

    mid = cv2.resize(x, (configs.img_width / 2, configs.img_height / 2))
    x = np.array([np.concatenate((x, x_depth), axis=2)])

    y = net.model.predict(x)[0]

    # ===== running... =====
    start_time = time.time()
    for i in range():
        y = net.model.predict(x)[0]

    duration = time.time() - start_time
    print('Generated segmentations in %s seconds -- %s FPS' % (duration / 10, 1.0 / (duration / 10)))

    visualization_result(y=y, mid=mid)


if __name__ == "__main__":

    if model_type == 'mid_fusion' or model_type == 'cross_fusion':
        test_fusion()
    elif model_type == 'early_fusion':
        test_early_fusion()