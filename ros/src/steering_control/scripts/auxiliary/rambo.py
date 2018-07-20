#
# Rambo model predictor class
# Udacity self-driving challenge 2
# (c) Yongyang Nie, 2018
# All Rights Reserved
#

import glob
import argparse
import numpy as np
from collections import deque
from keras.models import load_model
from keras.preprocessing.image import load_img, img_to_array
from skimage.exposure import rescale_intensity
from scipy import misc
import cv2


class Rambo(object):

    def __init__(self, model_path, X_train_mean_path):

        self.model = load_model(model_path)
        self.model.compile(optimizer="adam", loss="mse")
        self.X_mean = np.load(X_train_mean_path)
        print(self.X_mean)
        print(self.X_mean.shape)
        self.mean_angle = np.array([-0.004179079])
        print(self.mean_angle)
        self.img0 = None
        self.state = deque(maxlen=2)
        self.num = 0

    def predict_image(self, img):

        img1 = cv2.resize(img_to_array(img),(256,192))
        if (self.num < 20):
           misc.imsave("a"+str(self.num)+".jpg", img1)
        img1 = cv2.cvtColor(img1,cv2.COLOR_RGB2GRAY)
        img1 = np.expand_dims(img1, axis = -1)


        self.num += 1

        if self.img0 is None:
            self.img0 = img1
            return self.mean_angle[0]

        img = img1 - self.img0
        img = rescale_intensity(img, in_range=(-255, 255), out_range=(0, 255))
        img = np.array(img, dtype=np.uint8) # to replicate initial model
        self.state.append(img)
        self.img0 = img1

        if len(self.state) < 1:
            return self.mean_angle[0]

        else:
            X = np.concatenate(self.state, axis=-1)
            X = X[:,:,::-1]
            if (self.num < 20):
               misc.imsave("a"+str(self.num)+".jpg", X[:,:,0])
               misc.imsave("b"+str(self.num)+".jpg", X[:,:,1])
            X = np.expand_dims(X, axis=0)
            X = X.astype('float32')
            X -= self.X_mean
            X /= 255.0
            return self.model.predict(X)[0][0]

    def predict_path(self, img_path):

        self.num += 1

        img1 = load_img(img_path, grayscale=True, target_size=(192, 256))
        img1 = img_to_array(img1)

        if self.img0 is None:
            self.img0 = img1
            return self.mean_angle

        elif len(self.state) < 1:
            img = img1 - self.img0
            img = rescale_intensity(img, in_range=(-255, 255), out_range=(0, 255))
            img = np.array(img, dtype=np.uint8)  # to replicate initial model
            self.state.append(img)
            self.img0 = img1

            return self.mean_angle

        else:
            img = img1 - self.img0
            img = rescale_intensity(img, in_range=(-255, 255), out_range=(0, 255))
            img = np.array(img, dtype=np.uint8)  # to replicate initial model
            self.state.append(img)
            self.img0 = img1

            X = np.concatenate(self.state, axis=-1)
            print(X.shape)
            X = X[:, :, ::-1]
            X = np.expand_dims(X, axis=0)
            X = X.astype('float32')
            print(X.shape)
            X /= 255.0
            X -= self.X_mean
            misc.imsave("a" + str(self.num) + ".jpg", X[0][:, :, 0])
            misc.imsave("b" + str(self.num) + ".jpg", X[0][:, :, 1])
            return self.model.predict(X)[0]