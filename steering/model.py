from keras.models import Model, Sequential
from keras.layers.core import Dense, Activation, Flatten
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import AveragePooling2D, MaxPooling2D
from keras.layers import Input, Lambda, Dropout, ELU
from keras.layers.normalization import BatchNormalization
from keras.optimizers import Adam
from keras.models import load_model
import keras as K

def nvidia_network():
    
    model = Sequential()

    model.add(Conv2D(24, (5, 5), padding="same", strides = 2, input_shape=(256, 256, 3)))
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

def small_vgg_network():

    model = Sequential()
    model.add(Conv2D(32, (3, 3), activation='relu', padding='same', input_shape=(256, 256, 3)))
    model.add(MaxPooling2D((2, 2), strides=(2, 2)))
    model.add(Dropout(0.25))
    model.add(Conv2D(64, (3, 3), activation='relu', padding='same'))
    model.add(MaxPooling2D((2, 2), strides=(2, 2)))
    model.add(Dropout(0.25))
    model.add(Conv2D(128, (3, 3), activation='relu', padding='same'))
    model.add(MaxPooling2D((2, 2), strides=(2, 2)))
    model.add(Dropout(0.5))

    model.add(Flatten())
    model.add(Dense(512))
    model.add(Activation('relu'))
    model.add(Dense(256))
    model.add(Activation('relu'))
    model.add(Dense(1))

    adam = Adam(lr=1e-4)
    model.compile(optimizer=adam, loss=root_mean_squared_error)
    
    return model

def commaai_model(time_len=1):
    
    ch, row, col = 3, 160, 320  # camera format
    model = Sequential()
    model.add(Lambda(lambda x: x/127.5 - 1., input_shape=(row, col, ch), output_shape=(row, col, ch)))
    model.add(Conv2D(16, (8, 8), subsample=(4, 4), padding="same"))
    model.add(ELU())
    model.add(Conv2D(32, (5, 5), subsample=(2, 2), padding="same"))
    model.add(ELU())
    model.add(Conv2D(64, (5, 5), subsample=(2, 2), padding="same"))
    model.add(Flatten())
    model.add(Dropout(.2))
    model.add(ELU())
    model.add(Dense(512))
    model.add(Dropout(.5))
    model.add(ELU())
    model.add(Dense(1))

    model.compile(optimizer="adam", loss=root_mean_squared_error)

    return model


def root_mean_squared_error(y_true, y_pred):
        return K.backend.sqrt(K.backend.mean(K.backend.square(y_pred - y_true), axis=-1)) 

