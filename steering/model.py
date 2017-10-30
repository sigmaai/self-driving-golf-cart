from keras.models import Model, Sequential
from keras.layers.core import Dense, Activation, Flatten
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import AveragePooling2D, MaxPooling2D
from keras.layers import Input, Lambda, Dropout
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
    
    inputs = Input(shape=(256, 256, 3))
    x = Conv2D(32, (3, 3), activation='relu', padding='same')(inputs)
    x = MaxPooling2D((2, 2), strides=(2, 2))(x)
    x = Dropout(0.25)(x)

    x = Conv2D(64, (3, 3), activation='relu', padding='same')(x)
    x = MaxPooling2D((2, 2), strides=(2, 2))(x)
    x = Dropout(0.25)(x)

    x = Conv2D(128, (3, 3), activation='relu', padding='same')(x)
    x = MaxPooling2D((2, 2), strides=(2, 2))(x)
    x = Dropout(0.5)(x)

    y = Flatten()(x)
    y = Dense(512, activation='relu')(y)
    y = Dropout(.5)(y)
    y = Dense(256, activation='relu')(y)
    y = Dropout(.5)(y)
    y = Dense(1)(y)

    model = Model(input=inputs, output=y)
    model.compile(optimizer=Adam(lr=1e-4), loss = 'mse')
    
    return model
 
def root_mean_squared_error(y_true, y_pred):
        return K.backend.sqrt(K.backend.mean(K.backend.square(y_pred - y_true), axis=-1)) 

