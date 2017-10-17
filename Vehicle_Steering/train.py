import pandas as pd
import os, sys
import cv2
import numpy as np
import matplotlib.pyplot as plt

from keras.models import Model, Sequential
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.layers.convolutional import Conv2D, Conv2DTranspose, UpSampling2D
from keras.layers.pooling import AveragePooling2D, MaxPooling2D
from keras.layers.pooling import GlobalAveragePooling2D
from keras.layers import Input
from keras.layers.normalization import BatchNormalization
from keras.regularizers import l2
from keras.utils import plot_model
from keras.optimizers import Adam
from keras.models import load_model
import keras as K

def rotate(img):
    row, col, channel = img.shape
    angle = np.random.uniform(-15, 15)
    rotation_point = (row / 2, col / 2)
    rotation_matrix = cv2.getRotationMatrix2D(rotation_point, angle, 1)
    rotated_img = cv2.warpAffine(img, rotation_matrix, (col, row))
    return rotated_img


def gamma(img):
    gamma = np.random.uniform(0.5, 1.2)
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
    new_img = cv2.LUT(img, table)
    return new_img


def blur(img):
    r_int = np.random.randint(0, 2)
    odd_size = 2 * r_int + 1
    return cv2.GaussianBlur(img, (odd_size, odd_size), 0)

def random_transform(img):
    # There are total of 3 transformation
    # I will create an boolean array of 3 elements [ 0 or 1]
    a = np.random.randint(0, 2, [1, 3]).astype('bool')[0]
    if a[0] == 1:
        img = rotate(img)
    if a[1] == 1:
        img = blur(img)
    if a[2] == 1:
        img = gamma(img)
    return img

def random_number(length):
    i_line = np.random.randint(2000)
    i_line = i_line+length-2000
    while i_line % 3 != 0:
        i_line = np.random.randint(2000)
        i_line = i_line+length-2000
    return i_line

def generate_train_batch(data, batch_size = 16):

    img_rows = 480
    img_cols = 640
    
    batch_images = np.zeros((batch_size, img_rows, img_cols, 3))
    angles = np.zeros((batch_size, 1))
    while 1:
        for i_batch in range(batch_size):

            n = random_number(len(data))
            if n % 3 != 0:
                n = random_number(len(data))
            
            file_name = steering_labels.iloc[n]["filename"]
            img_bgr = cv2.imread("/home/ubuntu/dataset/udacity-driving/" + file_name)
            b,g,r = cv2.split(img_bgr)       # get b,g,r
            rgb_img = cv2.merge([r,g,b])     # switch it to rgb
            
            f =  float(steering_labels.iloc[n]["angle"])
    
            i = 2
            if i == 0:
                rgb_img = cv2.flip(rgb_img, 1)
                f = f * -1.0
            if i == 1:
                rgb_img = random_transform(rgb_img)
    
            batch_images[i_batch] = rgb_img
            angles[i_batch] = f
            
        yield batch_images, angles

def create_nvidia_model1():
    model = Sequential()

    model.add(MaxPooling2D(pool_size=(2, 2), strides=None, padding='valid', data_format=None, input_shape=(480, 640, 3)))
    model.add(Conv2D(24, (5, 5), padding="same", strides = 2))
    model.add(Activation('relu'))
    model.add(Conv2D(36, (3, 3), padding="same", strides = 2))
    model.add(Activation('relu'))
    model.add(Conv2D(48, (3, 3), padding="same", strides = 2))
    model.add(Activation('relu'))
    model.add(Conv2D(64, (3, 3), padding="same", strides = 2))
    model.add(Activation('relu'))
    model.add(Conv2D(64, (3, 3), padding="same", strides = 2))
    model.add(Flatten())
    model.add(Activation('relu'))
    model.add(Dense(256))
    model.add(Activation('relu'))
    model.add(Dense(128))
    model.add(Activation('relu'))
    model.add(Dense(64))
    model.add(Activation('relu'))
    model.add(Dense(1))
    adam = Adam(lr=1e-4)
    model.compile(optimizer=adam, loss="mse")

    print('Model is created and compiled..')
    return model
    
if __name__ == "__main__":

    steering_labels = pd.read_csv("/home/ubuntu/dataset/udacity-driving/interpolated.csv")
    print(steering_labels.shape)
    steering_labels.head()
    
    model = create_nvidia_model1()
    model.summary()
    
    model.fit_generator(generate_train_batch(steering_labels, 1), samples_per_epoch=10000, nb_epoch=5, validation_data=generate_train_batch(steering_labels, 1), nb_val_samples=500)

    model.save('trained-v12.h5')
