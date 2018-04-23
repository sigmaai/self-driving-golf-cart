#
# Cruse controller -- predictor class for speed control
# By Neil Nie & Michael Meng
# Jan, 2018
# Copyright (c), All rights reserved
#

import cv2
import cruise.configs as configs
import cruise.model as model

class CruisePredictor:

    def __init__(self):

        self.cnn = model.nvidia_network(input_shape=(configs.img_w, configs.img_h, 3))
        self.cnn.load_weights(configs.weight_path)
        print("cruise control model loaded, ready")
        print("----------------------------------")

    def predict_speed(self, image):

        input = cv2.resize(image, (configs.img_w, configs.img_h))
        prediction = self.cnn.predict(input)

        return prediction

    def visualization(self, image):

        return image
