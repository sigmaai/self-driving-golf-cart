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
            
            f =  float(steering_labels.iloc[i_line]["angle"]) #* 57.2958 #float(* 180.00 / 3.14159265359 )
    
            batch_images[i_batch] = rgb_img
            angles[i_batch] = f
        yield batch_images, angles

def create_nvidia_model1():
    model = Sequential()

    model.add(Conv2D(24, 5, 5, subsample=(2, 2), border_mode="same", input_shape=(480, 640, 3)))
    model.add(Activation('relu'))
    model.add(Conv2D(36, 5, 5, subsample=(2, 2), border_mode="same"))
    model.add(Activation('relu'))
    model.add(Conv2D(48, 5, 5, subsample=(2, 2), border_mode="same"))
    model.add(Activation('relu'))
    model.add(Conv2D(64, 3, 3, subsample=(2, 2), border_mode="same"))
    model.add(Activation('relu'))
    model.add(Conv2D(64, 3, 3, subsample=(2, 2), border_mode="same"))
    model.add(Flatten())
    model.add(Activation('relu'))
    model.add(Dense(200))
    model.add(Activation('relu'))
    model.add(Dense(50))
    model.add(Activation('relu'))
    model.add(Dense(10))
    model.add(Activation('relu'))
    model.add(Dense(1))

    model.compile(optimizer="adam", loss="mse")

    print('Model is created and compiled..')
    return model
    
if __name__ == "__main__":

    steering_labels = pd.read_csv("/home/ubuntu/dataset/udacity-driving/interpolated.csv")
    print(steering_labels.shape)
    steering_labels.head()
    
    model = create_nvidia_model1()
    model.summary()
    # model start here
    generator = generate_train_batch(steering_labels, 1)
    history = model.fit_generator(generator, steps_per_epoch=10000, epochs=5, verbose=1)

    model.save('trained-v3.h5')
