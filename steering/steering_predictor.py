
import numpy as np
import cv2
from PIL import Image
import steering.configs as configs
import steering.model as model


class SteeringPredictor:

    def overlayer_steering(self, image, angle):
        
        background = Image.fromarray(np.uint8(image))
        sw = Image.open("./steering/sw.png")
        sw = sw.rotate(angle*180/np.pi)
        background.paste(sw, (0, 0), sw)
        
        return np.array(background)

    def __init__(self):

        self.cnn = model.commaai_model()
        self.cnn.load_weights(configs.model_path)
        print("steering model loaded")

    def predict_steering(self, image):

        input = cv2.resize(image, (320, 160))
        predicted_steers = self.cnn.predict(np.array([input]))[0][0]
        overlay_img = self.overlayer_steering(image, predicted_steers)
        return predicted_steers, overlay_img

