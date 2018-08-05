"""
Steering predictor class
integrates the Inception3D model with some
helper methods that stream lines the steering
control node.

(c) Yongyang Nie, 2018
All Rights Reserved.
"""

from models.i3d import Inception3D
import numpy as np
import models.configs as configs
import cv2


# TODO: must test implementation

class SteeringPredictor:

    def __init__(self, weight_path, model_type):

        """
        Constructor for SteeringPredictor class
        :param weight_path:
        :param model_type:
        """

        self.model = Inception3D(input_shape=(configs.LENGTH, configs.IMG_HEIGHT, configs.IMG_WIDTH, 3), weights_path=weight_path)
        self.inputs = []
        self.model_type = model_type

    def predict(self, image):

        """
        predict the steering angle given the image. Only return results if
        self.input >= configs.length
        :param image:   input image
        :return:        steering angle
        """

        image = cv2.resize(image, (configs.IMG_WIDTH, configs.IMG_HEIGHT))

        if len(self.inputs) < configs.LENGTH:
            self.inputs.append(image)

        if len(self.inputs) == configs.LENGTH:
            prediction = self.model.model.predict(np.array([self.inputs]))[0][0]
            self.inputs.pop(0)
            return prediction

        if len(self.inputs) > configs.LENGTH:
            raise ValueError("Input length can't be longer than network input length")

        return 0.0