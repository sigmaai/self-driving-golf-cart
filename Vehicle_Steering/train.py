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

    #model.add(MaxPooling2D(pool_size=(2, 2), strides=None, padding='valid', data_format=None, ))
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
    model.compile(optimizer=adam, loss=root_mean_squared_error)

    print('Model is created and compiled..')
    return model

def root_mean_squared_error(y_true, y_pred):
        return K.backend.sqrt(K.backend.mean(K.backend.square(y_pred - y_true), axis=-1)) 

def clean_steering_label(steering_labels):
    
    center_labels = np.array([steering_labels[2]])
    print("input length")
    print(len(steering_labels))
    for i in range(2, len(steering_labels)):
        if (i%10000) == 0:
            print(".")
        if (i+1) % 3 == 0:
            item = np.array([steering_labels[i]])
            center_labels = np.concatenate((center_labels, item), axis=0)
    return center_labels
            

if __name__ == "__main__":

    dir = "/home/ubuntu/dataset/udacity-driving-testing-ds/"

    labels = pd.read_csv("/home/ubuntu/dataset/udacity-driving-testing-ds/interpolated.csv")
    
    center_labels = clean_steering_label(labels.values)
    print(center_labels.shape)
    for i in range(0, 10):
        print(center_labels[i])
        
    model = create_nvidia_model1()
    model.summary()

    training_gen = train_util.batch_generator(dir, center_labels, 2, True)
    validation_gen = train_util.batch_generator(dir, center_labels, 2, False)
    
    model.fit_generator(training_gen,
                        steps_per_epoch=10000, epochs=10, verbose=1,
                        validation_data=validation_gen,
                        validation_steps=200)

    model.save('trained5-v1.h5')
