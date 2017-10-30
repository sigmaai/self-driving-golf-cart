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


def create_nvidia_model():
    
    model = Sequential()

    #model.add(MaxPooling2D(pool_size=(2, 2), strides=None, padding='valid', data_format=None, ))
    model.add(Conv2D(24, (5, 5), padding="same", strides = 2, input_shape=(128, 128, 3)))
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
        
        if steering_labels[i][4] == "center_camera":
            item = np.array([steering_labels[i]])
            center_labels = np.concatenate((center_labels, item), axis=0)
        if (i%10000) == 0:
            print(".")
    print(center_labels.shape)
    return center_labels
            

if __name__ == "__main__":

    dir = "/home/ubuntu/dataset/udacity-driving-testing-ds/"
    val_dir = "/home/ubuntu/dataset/small-testing-ds/"
    
    labels = pd.read_csv("/home/ubuntu/dataset/udacity-driving-testing-ds/interpolated.csv")
    val_labels = pd.read_csv(val_dir + "interpolated.csv")
    center_labels = clean_steering_label(labels.values)
    print(center_labels.shape)
        
    model = create_nvidia_model()
    model.load_weights("./trained5-v2.h5")
    model.summary()

    training_gen = train_util.batch_generator(dir, center_labels, 4, True)
    validation_gen = train_util.validation_generator(val_dir, val_labels, 2)
    
    model.fit_generator(training_gen,
                        steps_per_epoch=3000, epochs=15, verbose=1,
                        validation_data=validation_gen,
                        validation_steps=600)

    model.save('trained5-v3.h5')

