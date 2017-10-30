import pandas as pd
import os, sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import densenet

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
import train_util


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


dir = "/home/ubuntu/dataset/udacity-driving-testing-ds/"
labels = pd.read_csv("/home/ubuntu/dataset/udacity-driving-testing-ds/interpolated.csv")
center_labels = clean_steering_label(labels.values)
print(center_labels.shape)

training_gen = train_util.batch_generator(dir, center_labels, 1, True)
validation_gen = train_util.batch_generator(dir, center_labels, 1, False)

nb_classes = 1
img_dim = (480, 640, 3)
depth = 19
nb_dense_block = 3
growth_rate = 12
nb_filter = -1
dropout_rate = 0.0 # 0.0 for data augmentation

model = densenet.DenseNet(img_dim, classes=nb_classes, depth=depth, nb_dense_block=nb_dense_block,
                          growth_rate=growth_rate, nb_filter=nb_filter, dropout_rate=dropout_rate, weights=None)
adam = Adam(lr=1e-4)
model.compile(optimizer=adam, loss=root_mean_squared_error)
model.summary()
model.fit_generator(training_gen, 
                    steps_per_epoch=8000, epochs=15, verbose=1, 
                    validation_data=validation_gen, 
                    validation_steps=500)
