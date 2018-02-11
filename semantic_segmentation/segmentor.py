#
# Semantic Segmentator
# Class for driving image segmentation
# Integrated in drive.py. For more
# information see README.md.
# ------------------------------------
# Neil Nie & Michael Meng
# (c) Yongyang Nie 2018


import semantic_segmentation.models.enet_naive_upsampling.model as enet
import semantic_segmentation.models.icnet.model as icnet
import semantic_segmentation.utils as utils
import semantic_segmentation.configs as configs
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

        self.model.load_weights(configs.model_path)

        print("-----------------")
        print("segmentor created")
        print("-----------------")

    def semantic_segmentation(self, image):
    
        image = cv2.resize(image, (640,360))
        output = self.model.predict(np.array([image]))[0]
        im_mask = utils.convert_class_to_rgb(output)
        img_pred = cv2.addWeighted(im_mask, 0.8, image, 0.8, 0)
        # img_pred = cv2.cvtColor(img_pred, cv2.COLOR_RGB2BGR)
        return output, img_pred
