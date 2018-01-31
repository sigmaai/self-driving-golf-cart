# Cruse controller -- predictor class for speed control
# By Neil Nie & Michael Meng
# Jan, 2018
# Copyright (c), All rights reserved

import numpy as np
import cv2
import cruse.configs as configs
import cruse.model as model

class CrusePredictor:

    def __init__(self):

        self.cnn = model.nvidia_network()
        self.cnn.load_weights(configs.weight_path)
        print("cruse control model loaded, ready")

    def predict_speed(self, image):

        input = cv2.resize(image, (configs.img_w, configs.img_h))
        prediction = self.cnn.predict(input)
        return prediction