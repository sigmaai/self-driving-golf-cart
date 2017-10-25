import pandas as pd
import cv2
import numpy as np
import train_util

from keras.models import Model, Sequential
from keras.layers.core import Dense, Activation, Flatten
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import AveragePooling2D, MaxPooling2D
from keras.layers import Input, Lambda
from keras.layers.normalization import BatchNormalization
from keras.optimizers import Adam
from keras.models import load_model
import keras as K


def create_nvidia_model1():
    
    model = Sequential()

    model.add(Conv2D(24, (5, 5), padding="same", strides = 2, input_shape=(480, 640, 3)))
    model.add(Activation('relu'))
    model.add(Conv2D(36, (5, 5), padding="same", strides = 2))
    model.add(Activation('relu'))
    model.add(Conv2D(48, (5, 5), padding="same", strides = 2))
    model.add(Activation('relu'))
    model.add(Conv2D(64, (3, 3), padding="same", strides = 2))
    model.add(Activation('relu'))
    model.add(Conv2D(64, (3, 3), padding="same", strides = 2))
    model.add(Flatten())
    model.add(Activation('relu'))
    model.add(Dense(512))
    model.add(Activation('relu'))
    model.add(Dense(256))
    model.add(Activation('relu'))
    model.add(Dense(128))
    model.add(Activation('relu'))
    model.add(Dense(1))
    adam = Adam(lr=1e-4)
    model.compile(optimizer=adam, loss="mse")

    print('Model is created and compiled..')
    return model


if __name__ == "__main__":

    dir = "/home/ubuntu/dataset/udacity-driving-testing-ds/"

    #steering_label1 = pd.read_csv("/home/ubuntu/dataset/udacity-driving/interpolated.csv")
    steering_label2 = pd.read_csv("/home/ubuntu/dataset/udacity-driving-testing-ds/interpolated.csv")
    #whole = [steering_label1, steering_label2]
    #steering_labels = pd.concat(whole)
    steering_labels = steering_label2
    print(steering_labels.shape)
    
    model = create_nvidia_model1()
    #model = load_model("./trained2-v4.h5")
    model.summary()

    model.fit_generator(train_util.batch_generator(dir, steering_labels.values, 4, True),
                        steps_per_epoch=2000, epochs=15, verbose=1,
                        validation_data=train_util.batch_generator(dir, steering_labels.values, 2, False),
                        validation_steps=200)

    model.save('trained2-v4.h5')
