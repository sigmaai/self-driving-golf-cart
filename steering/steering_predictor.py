
import numpy as np
import cv2
import steering.configs as configs
import steering.models as models


class SteeringPredictor:

    def __init__(self):

        self.cnn = models.commaai_model()
        self.cnn.load_weights(configs.model_path)
        print("---------------------")
        print("steering model loaded")

    def predict(self, image):

        input = cv2.resize(image, (320, 160))
        predicted_angle = self.cnn.predict(np.array([input]))[0][0]

        return predicted_angle

