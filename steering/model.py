from keras.models import Model, Sequential
from keras.layers.core import Dense, Activation, Flatten
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import AveragePooling2D, MaxPooling2D
from keras.layers import Input, Lambda, Dropout, ELU
from keras.layers.normalization import BatchNormalization
from keras.optimizers import Adam
import keras as K
import steering.configs as configs

def nvidia_network():
    
    model = Sequential()

    model.add(Conv2D(24, (5, 5), padding="same", strides=2, input_shape=(configs.image_height, configs.image_width, 3)))
    model.add(ELU())
    model.add(Conv2D(36, (5, 5), padding="same", strides=2))
    model.add(ELU())
    model.add(Conv2D(48, (5, 5), padding="same", strides=2))
    model.add(ELU())
    model.add(Conv2D(64, (3, 3), padding="same", strides=2))
    model.add(ELU())
    model.add(Conv2D(64, (3, 3), padding="same", strides=2))

    model.add(Flatten())
    model.add(ELU())
    model.add(Dense(512))
    model.add(ELU())
    model.add(Dense(256))
    model.add(ELU())
    model.add(Dense(128))
    model.add(ELU())
    model.add(Dense(1))
    adam = Adam(lr=1e-4)
    model.compile(optimizer=adam, loss=root_mean_squared_error)

    print('Model is created and compiled..')
    return model

def small_vgg_network():

    model = Sequential()
    model.add(Conv2D(32, (3, 3), activation='relu', padding='same', input_shape=(configs.image_height, configs.image_width, 3)))
    model.add(MaxPooling2D((2, 2), strides=2))
    model.add(Dropout(0.25))
    model.add(Conv2D(64, (3, 3), activation='relu', padding='same'))
    model.add(MaxPooling2D((2, 2), strides=2))
    model.add(Dropout(0.25))
    model.add(Conv2D(128, (3, 3), activation='relu', padding='same'))
    model.add(MaxPooling2D((2, 2), strides=2))
    model.add(Dropout(0.5))

    model.add(Flatten())
    model.add(Dense(512))
    model.add(Activation('relu'))
    model.add(Dense(256))
    model.add(Activation('relu'))
    model.add(Dense(1))

    adam = Adam(lr=1e-4)
    model.compile(optimizer=adam, loss=root_mean_squared_error)
    print('Model is created and compiled..')
    return model

def commaai_model():
    
    ch, row, col = 3, 160, 320  # camera format
    model = Sequential()
    model.add(Lambda(lambda x: x/127.5 - 1., input_shape=(configs.image_height, configs.image_width, ch), output_shape=(configs.image_height, configs.image_width, ch)))
    model.add(Conv2D(16, (8, 8), strides=4, padding="same"))
    model.add(ELU())
    model.add(Conv2D(32, (5, 5), strides=2, padding="same"))
    model.add(ELU())
    model.add(Conv2D(64, (5, 5), strides=2, padding="same"))
    model.add(Flatten())
    model.add(Dropout(.2))
    model.add(ELU())
    model.add(Dense(512))
    model.add(Dropout(.5))
    model.add(ELU())
    model.add(Dense(1))

    adam = Adam(lr=1e-4)
    model.compile(optimizer=adam, loss=root_mean_squared_error)
    print('Model is created and compiled..')
    return model


def root_mean_squared_error(y_true, y_pred):
        return K.backend.sqrt(K.backend.mean(K.backend.square(y_pred - y_true), axis=-1)) 

