#
# Semantic Segmentator
# Class for driving image segmentation
# Integrated in drive.py. For more
# information see README.md.
# ------------------------------------
# Neil Nie & Michael Meng
# (c) Yongyang Nie 2018


import semantic_segmentation.models.enet_naive_upsampling.model as enet
import semantic_segmentation.utils as utils
import semantic_segmentation.configs as configs
from keras.models import load_model
import os
import cv2
import numpy as np
from PIL import Image

class Segmentor:

    def __init__(self, type):

        if type == "ENET":
            self.model = enet.build(len(utils.labels), configs.img_height, configs.img_width)
        elif type == "ICNET":
            self.model = enet.build(len(utils.labels), configs.img_height, configs.img_width)

        self.model.load_weights(configs.infer_model_path)

    def semantic_segmentation(self, image, visualize=False):

        # parameters
        # image: input image
        # visualize: whether to visualize the segmentation results
        # return
        # output: output of ConvNet
        # img_pred: visualization

        image = cv2.resize(image, (640,360))
        output = self.model.predict(np.array([image]))[0]
        if visualize:
            im_mask = utils.convert_class_to_rgb(output)
        else:
            im_mask = image

        # img_pred = cv2.cvtColor(im_mask, cv2.COLOR_RGB2BGR)
        img_pred = cv2.addWeighted(im_mask, 0.8, image, 0.8, 0)
        img_pred = cv2.resize(img_pred, (640, 480))
        return output, img_pred
