
from i3d import i3d
import numpy as np
import configs
import cv2


input_shape = (244, 244, 3)


class SteeringPredictor:

    def __init__(self, weight_path, model_type):

        '''
        Constructor for SteeringPredictor class
        :param weight_path:
        :param model_type:
        '''

        self.model = i3d(input_shape=input_shape, weights_path=weight_path)
        self.inputs = []
        self.model_type = model_type

    def predict(self, image):

        '''
        predict the steering angle given the image. Only return results if
        self.input >= configs.length
        :param image:   input image
        :return:        steering angle
        '''

        image = cv2.resize(image, (configs.IMG_WIDTH, configs.IMG_HEIGHT))

        if len(self.inputs) < configs.LENGTH:
            self.inputs.append(image)
            return 0.0

        if len(self.inputs) == configs.LENGTH:
            input_array = np.array([input])
            prediction = self.model.model.predict(input_array)[0][0]
            self.inputs.pop(0)

            return prediction

        if len(self.inputs) > configs.LENGTH:
            raise ValueError("Input length can't be longer than network input length")