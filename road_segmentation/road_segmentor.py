import model as m
import utils
import configs
import os
from keras.optimizers import Adam
import cv2
import pandas as pd
import numpy as np
from keras.metrics import binary_accuracy
from PIL import Image

class Segmentor:

    def __init__(self, type):

        if type == "SGN":
            self.model = m.segnet(nb_classes=2, input_height=configs.img_height, input_width=configs.img_width)
        elif type == "FCN":
            self.model = m.fcn_model()
        self.model.load_weight(configs.model_path)

        print("-----------------")
        print("segmentor created")
        print("-----------------")

    def segment_road(self, image):

        im_mask = self.model.predict(np.array([image]))[0][:, :, 1]
        im_mask = np.array(255 * im_mask, dtype=np.uint8)
        im_mask = cv2.cvtColor(im_mask, cv2.COLOR_GRAY2RGB)
        ret, mask = cv2.threshold(im_mask, 100, 255, cv2.THRESH_BINARY)
        mask[:, :, 1:3] = 0 * mask[:, :, 1:3]

        img_pred = cv2.addWeighted(mask, 1.0, image, 1.0, 0)
        
        return  img_pred