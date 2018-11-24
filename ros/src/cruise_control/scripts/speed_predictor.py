#
# Speed Predictor Class. Called by the Cruise Control Node.
# (Currently only using deep learning techniques, very
# incomplete. Working on other implementations...)
#
# Developed By Neil Nie
# (c) Yongyang Nie, 2018, All Rights Reserved
# Contact: contact@neilnie.com
#

from models.i3d import Inception3D
import numpy as np
import cv2


class SpeedPredictor:

    def __init__(self, weight_path, model_type, input_size, input_length):

        """
        Constructor for SteeringPredictor class
        :param weight_path:
        :param model_type:
        """

        self.input_shape = input_size
        self.input_length = input_length
        self.model = Inception3D(input_shape=(input_length, input_size[0], input_size[1], 3), weights_path=weight_path)
        self.inputs = []
        self.model_type = model_type

    def predict(self, image):

        """
        predict the steering angle given the image. Only return results if
        self.input >= configs.length
        :param image:   input image
        :return:        steering angle
        """

        image = cv2.resize(image, self.input_shape)

        if len(self.inputs) < self.input_length:
            self.inputs.append(image)

        if len(self.inputs) == self.input_length:
            prediction = self.model.model.predict(np.array([self.inputs]))[0][0]
            self.inputs.pop(0)
            return prediction

        if len(self.inputs) > self.input_length:
            raise ValueError("Input length can't be longer than network input length")

        return 0.0