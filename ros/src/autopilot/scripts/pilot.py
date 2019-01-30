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
import cv2


class Pilot:

    def __init__(self, weight_path, model_type, input_length, img_height, img_width):

        """
        Constructor for SteeringPredictor class
        :param weight_path:
        :param model_type:
        """

        self.img_height = img_height
        self.img_width = img_width
        self.length = input_length
        self.model = Inception3D(input_shape=(input_length, self.img_height, self.img_width, 3), weights_path=weight_path)
        self.inputs = []
        self.model_type = model_type

    def predict(self, image):

        """
        predict the steering angle given the image. Only return results if
        self.input >= configs.length
        :param image:   input image
        :return:        steering angle
        """

        image = cv2.resize(image, (self.img_width, self.img_height))

        if len(self.inputs) < self.length:
            self.inputs.append(image)

        if len(self.inputs) == self.length:
            prediction = self.model.model.predict(np.array([self.inputs]))[0]
            self.inputs.pop(0)

            return prediction

        if len(self.inputs) > self.length:
            raise ValueError("Input length can't be longer than network input length")

        return [0.0, 0.0]