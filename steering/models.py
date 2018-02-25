from keras.models import Model, Sequential
from keras.layers.core import Dense, Activation, Flatten
from keras.layers import SpatialDropout2D
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import MaxPooling2D
from keras.layers import PReLU, Lambda, Dropout, ELU, Merge
from keras.optimizers import SGD
from keras.regularizers import l2
from keras.optimizers import Adam
import keras as K
import configs as configs

def nvidia_model():

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
    model.compile(optimizer=adam, loss=rmse)

    # print('steering model is created and compiled...')
    return model

def small_vgg_model():

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
    model.compile(optimizer=adam, loss=rmse)
    # print('steering model is created and compiled...')
    return model

def commaai_model():

    model = Sequential()
    model.add(Lambda(lambda x: x/127.5 - 1., input_shape=(configs.image_height, configs.image_width, 3), output_shape=(configs.image_height, configs.image_width, 3)))
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
    model.compile(optimizer=adam, loss=rmse)
    # print('steering model is created and compiled...')
    return model


def create_comma_model_prelu():

    model = Sequential()

    model.add(Conv2D(16, (8, 8), strides=(4, 4), padding="same", input_shape=(configs.image_height, configs.image_width, 3)))
    model.add(PReLU())
    model.add(Conv2D(32, (5, 5), strides=(2, 2), padding="same"))
    model.add(PReLU())
    model.add(Conv2D(64, (5, 5), strides=(2, 2), padding="same"))
    model.add(Flatten())
    model.add(PReLU())
    model.add(Dense(512))
    model.add(PReLU())
    model.add(Dense(1))

    model.compile(optimizer="adam", loss=rmse)

    print('Model is created and compiled..')
    return model


def regression_model(input_shape=(configs.image_height, configs.image_width, 3), use_adadelta=True, learning_rate=0.01, W_l2=0.0001,):
        """
        """
        model = Sequential()
        model.add(Conv2D(16, (5, 5), input_shape=input_shape, kernel_initializer="he_normal", activation='relu', padding='same'))
        model.add(SpatialDropout2D(0.1))
        model.add(MaxPooling2D(pool_size=(2, 2)))
        model.add(Conv2D(20, (5, 5), kernel_initializer="he_normal", activation='relu', padding='same'))
        model.add(SpatialDropout2D(0.1))
        model.add(MaxPooling2D(pool_size=(2, 2)))
        model.add(Conv2D(40, (3, 3), kernel_initializer="he_normal", activation='relu', padding='same'))
        model.add(SpatialDropout2D(0.1))
        model.add(MaxPooling2D(pool_size=(2, 2)))
        model.add(Conv2D(60, (3, 3), kernel_initializer="he_normal", activation='relu', padding='same'))
        model.add(SpatialDropout2D(0.1))
        model.add(MaxPooling2D(pool_size=(2, 2)))
        model.add(Conv2D(80, (2, 2), kernel_initializer="he_normal", activation='relu', padding='same'))
        model.add(SpatialDropout2D(0.1))
        model.add(MaxPooling2D(pool_size=(2, 2)))
        model.add(Conv2D(128, (2, 2), kernel_initializer="he_normal", activation='relu', padding='same'))
        model.add(Flatten())
        model.add(Dropout(0.5))
        model.add(Dense(output_dim=1, kernel_initializer='he_normal', W_regularizer=l2(W_l2)))

        optimizer = SGD(lr=learning_rate, momentum=0.9)

        model.compile(loss=rmse, optimizer=optimizer)

        return model


def create_rambo_model():

    row = 192
    col = 256
    ch = 4

    #First branch
    branch1 = Sequential()
    branch1.add(Conv2D(16, 8, 8, subsample=(4, 4), border_mode="same", input_shape=(row, col, ch)))
    branch1.add(Activation('relu'))
    branch1.add(Conv2D(32, 5, 5, subsample=(2, 2), border_mode="same"))
    branch1.add(Activation('relu'))
    branch1.add(Conv2D(64, 5, 5, subsample=(2, 2), border_mode="same"))
    branch1.add(Flatten())
    branch1.add(Activation('relu'))
    branch1.add(Dense(512))
    branch1.add(Activation('relu'))
    branch1.add(Dense(1, input_dim=512))
    
    #Second branch
    branch2 = Sequential()
    branch2.add(Conv2D(24, 5, 5, subsample=(2, 2), border_mode="same", input_shape=(row, col, ch)))
    branch2.add(Activation('relu'))
    branch2.add(Conv2D(36, 5, 5, subsample=(2, 2), border_mode="same"))
    branch2.add(Activation('relu'))
    branch2.add(Conv2D(48, 5, 5, subsample=(2, 2), border_mode="same"))
    branch2.add(Activation('relu'))
    branch2.add(Conv2D(64, 3, 3, subsample=(2, 2), border_mode="same"))
    branch2.add(Activation('relu'))
    branch2.add(Conv2D(64, 3, 3, subsample=(2, 2), border_mode="same"))
    branch2.add(Flatten())
    branch2.add(Activation('relu'))
    branch2.add(Dense(100))
    branch2.add(Activation('relu'))
    branch2.add(Dense(50))
    branch2.add(Activation('relu'))
    branch2.add(Dense(10))
    branch2.add(Activation('relu'))
    branch2.add(Dense(1, input_dim=10))
    
    #Third branch
    branch3 = Sequential()
    branch3.add(Conv2D(24, 5, 5, subsample=(2, 2), border_mode="same", input_shape=(row, col, ch)))
    branch3.add(Activation('relu'))
    branch3.add(Conv2D(36, 5, 5, subsample=(2, 2), border_mode="same"))
    branch3.add(Activation('relu'))
    branch3.add(Conv2D(48, 5, 5, subsample=(2, 2), border_mode="same"))
    branch3.add(Activation('relu'))
    branch3.add(Conv2D(64, 3, 3, subsample=(2, 2), border_mode="same"))
    branch3.add(Activation('relu'))
    branch3.add(Conv2D(64, 3, 3, subsample=(2, 2), border_mode="same"))
    branch3.add(Activation('relu'))
    branch3.add(Conv2D(64, 3, 3, subsample=(2, 2), border_mode="same"))
    branch3.add(Flatten())
    branch3.add(Activation('relu'))
    branch3.add(Dense(100))
    branch3.add(Activation('relu'))
    branch3.add(Dense(50))
    branch3.add(Activation('relu'))
    branch3.add(Dense(10))
    branch3.add(Activation('relu'))
    branch3.add(Dense(1, input_dim=10))
    
    #Final merge
    model = Sequential()
    model.add(Merge([branch1, branch2, branch3], mode='concat'))
    model.add(Activation('relu'))
    model.add(Dense(1))
    model.compile(optimizer="adam", loss="mse")
    
    print('Model is created and compiled..')
    return model


def rmse(y_true, y_pred):
        return K.backend.sqrt(K.backend.mean(K.backend.square(y_pred - y_true), axis=-1)) 

