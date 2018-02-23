import numpy as np
from collections import deque
from keras.models import load_model
from keras.preprocessing.image import load_img, img_to_array
from skimage.exposure import rescale_intensity
from scipy import misc
from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw


class Rambo(object):
    def __init__(self,
                 model_path,
                 X_train_mean_path):

        self.model = load_model(model_path)
        self.model.compile(optimizer="adam", loss="mse")
        self.X_mean = np.load(X_train_mean_path)
        self.mean_angle = np.array([-0.004179079])
        self.img0 = None
        self.state = deque(maxlen=2)

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


    def predict(self, img):
#        img_path = 'test.jpg'
#        misc.imsave(img_path, img)
#        img1 = load_img(img_path, grayscale=True, target_size=(192, 256))
#        img1 = img_to_array(img1)
        img1 = img_to_array(img)

        if self.img0 is None:
            self.img0 = img1
            return self.mean_angle[0]

        elif len(self.state) < 1:
            img = img1 - self.img0
            img = rescale_intensity(img, in_range=(-255, 255), out_range=(0, 255))
            img = np.array(img, dtype=np.uint8) # to replicate initial model
            self.state.append(img)
            self.img0 = img1

            return self.mean_angle[0]

        else:
            img = img1 - self.img0
            img = rescale_intensity(img, in_range=(-255, 255), out_range=(0, 255))
            img = np.array(img, dtype=np.uint8) # to replicate initial model
            self.state.append(img)
            self.img0 = img1

            X = np.concatenate(self.state, axis=-1)
            X = X[:,:,::-1]
            X = np.expand_dims(X, axis=0)
            X = X.astype('float32')
            X -= self.X_mean
            X /= 255.0
            return self.model.predict(X)[0][0]
