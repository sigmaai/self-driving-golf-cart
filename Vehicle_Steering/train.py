import pandas as pd
import cv2
import numpy as np
import train_util

from keras.models import Model, Sequential
from keras.layers.core import Dense, Activation, Flatten
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import AveragePooling2D, MaxPooling2D
from keras.layers import Input
from keras.layers.normalization import BatchNormalization
from keras.optimizers import Adam
from keras.models import load_model
import keras as K


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

    dir = "/Volumes/Personal_Drive/Datasets/Udacity_Self-Driving-Car/dataset/"

    steering_labels = pd.read_csv(dir + "interpolated.csv")
    print(steering_labels.shape)
    steering_labels.head()
    
    model = create_nvidia_model1()
    model.summary()


    model.fit_generator(train_util.batch_generator(dir, steering_labels.values, 8, True),
                        samples_per_epoch=1,
                        nb_epoch=5,
                        validation_data=train_util.batch_generator(dir, steering_labels.values, 8, False),
                        nb_val_samples=500)

    model.save('trained-v13.h5')
