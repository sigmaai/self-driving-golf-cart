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
        self.backgrounds = self.load_color_backgrounds()

    @staticmethod
    def load_color_backgrounds():

        backgrounds = []

        for i in range(len(utils.labels)):
            color = utils.labels[i][7]

            bg = np.zeros((configs.img_height, configs.img_width, 3), dtype=np.uint8)
            bg[:, :, 0].fill(color[0])
            bg[:, :, 1].fill(color[1])
            bg[:, :, 2].fill(color[2])
            backgrounds.append(bg)

        return backgrounds

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
            im_mask = self.convert_class_to_rgb(output)
        else:
            im_mask = image

        img_pred = cv2.addWeighted(im_mask, 0.8, image, 0.8, 0)
        img_pred = cv2.resize(img_pred, (640, 480))
        return output, img_pred

    def convert_class_to_rgb(self, image_labels, threshold=0.05):

        # convert any pixel > threshold to 1
        # convert any pixel < threshold to 0
        # then use bitwise_and

        output = np.zeros((configs.img_height, configs.img_width, 3), dtype=np.uint8)

        for i in range(len(utils.labels)):

            split = image_labels[:, :, i]
            split[split > threshold] = 1
            split[split < threshold] = 0
            split[:] *= 255
            split = split.astype(np.uint8)

            bg = self.backgrounds[i].copy()
            res = cv2.bitwise_and(bg, bg, mask=split)
            output = cv2.addWeighted(output, 1.0, res, 1.0, 0)

        return output



