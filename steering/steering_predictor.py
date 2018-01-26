
import numpy as np
import cv2
from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw
import steering.configs as configs
import steering.model as model


class SteeringPredictor:

    def post_process_image(self, image, angle):
        
        background = Image.fromarray(np.uint8(image))
        sw = Image.open("./steering/resources/sw.png")
        sw = sw.rotate(angle*180/np.pi)
        sw = sw.resize((80, 80), Image.ANTIALIAS)
        background.paste(sw, (10, 10), sw)

        draw = ImageDraw.Draw(background)
        font = ImageFont.truetype("./steering/resources/FiraMono-Medium.otf", 16)
        draw.text((80, 200), str(round(angle, 3)), (255, 255, 255), font=font)
        
        return np.array(background)


    def __init__(self):

        self.cnn = model.commaai_model()
        self.cnn.load_weights(configs.model_path)
        print("steering model loaded")

    def predict_steering(self, image):

        input = cv2.resize(image, (320, 160))
        predicted_angle = self.cnn.predict(np.array([input]))[0][0]
        overlay_img = self.post_process_image(image, predicted_angle)

        return predicted_angle, overlay_img

