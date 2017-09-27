import pandas as pd
import os, sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
from keras.models import load_model

from keras.models import Model, Sequential
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.layers.convolutional import Conv2D, Conv2DTranspose, UpSampling2D
from keras.layers.pooling import AveragePooling2D, MaxPooling2D
from keras.layers.pooling import GlobalAveragePooling2D
from keras.layers import Input
from keras.layers.normalization import BatchNormalization
from keras.regularizers import l2
from keras.utils.data_utils import get_file
from keras.engine.topology import get_source_inputs
from keras.applications.imagenet_utils import _obtain_input_shape
from keras.applications.imagenet_utils import decode_predictions
from keras.utils import plot_model
from keras.optimizers import Adam
import keras as K

steering_labels = None

def generate_train_batch(data, batch_size = 16):

    img_rows = 480
    img_cols = 640
    
    batch_images = np.zeros((batch_size, img_rows, img_cols, 3))
    angles = np.zeros((batch_size, 1))
    while 1:
        for i_batch in range(batch_size):
            i_line = np.random.randint(30000)
            
            file_name = steering_labels.iloc[i_line]["filename"]
            img_bgr = cv2.imread("/home/ubuntu/dataset/udacity-driving/" + file_name)
            
            b,g,r = cv2.split(img_bgr)       # get b,g,r
            rgb_img = cv2.merge([r,g,b])     # switch it to rgb
            
            f =  float(steering_labels.iloc[i_line]["angle"]) * 57.2958 #float(* 180.00 / 3.14159265359 )
    
            batch_images[i_batch] = rgb_img
            angles[i_batch] = f
        yield batch_images, angles

def root_mean_squared_error(y_true, y_pred):
        return K.backend.sqrt(K.backend.mean(K.backend.square(y_pred - y_true), axis=-1)) 
    
if __name__ == "__main__":

    steering_labels = pd.read_csv("/home/ubuntu/dataset/udacity-driving/interpolated.csv")
    print(steering_labels.shape)
    steering_labels.head()
    
    # frame size
    nrows = 480
    ncols = 640

    # model start here
    model = Sequential()

    model.add(MaxPooling2D(pool_size=(2, 2), strides=None, padding='valid', input_shape=(480, 640, 3)))
    model.add(BatchNormalization(epsilon=0.001, axis=1))
    model.add(Conv2D(12,(5,5),padding='valid', activation='relu', strides=(2,2)))
    model.add(Conv2D(12,(5,5),padding='valid', activation='relu', strides=(2,2)))
    model.add(Conv2D(24,(5,5),padding='valid', activation='relu', strides=(2,2)))
    model.add(Conv2D(24,(3,3),padding='valid', activation='relu', strides=(1,1)))
    model.add(Conv2D(48,(3,3),padding='valid', activation='relu', strides=(1,1)))
    model.add(Flatten())
    model.add(Dense(500, activation='relu'))
    model.add(Dense(100, activation='relu'))
    model.add(Dense(50, activation='relu'))
    model.add(Dense(10, activation='relu'))
    model.add(Dense(1, activation=None))

    model.summary()
    
    adam = Adam(lr=0.0001)
    model.compile(loss=root_mean_squared_error,
              optimizer=adam,
              metrics=["accuracy"])

    generator = generate_train_batch(steering_labels, 1)
    history = model.fit_generator(generator, steps_per_epoch=10000, epochs=10, verbose=1)

    model.save('trained.h5')
