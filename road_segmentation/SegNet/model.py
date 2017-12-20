# todo upgrade to keras 2.0
from keras.models import Sequential
from keras.layers import Reshape
from keras.layers import Merge
from keras.layers.core import Layer, Dense, Dropout, Activation, Flatten, Reshape, Permute
from keras.layers.normalization import BatchNormalization
from keras.layers.convolutional import Convolution3D, MaxPooling3D, ZeroPadding3D
from keras.layers.convolutional import Convolution2D, MaxPooling2D, UpSampling2D, ZeroPadding2D
from keras.layers.convolutional import Convolution1D, MaxPooling1D
from keras.layers.advanced_activations import LeakyReLU
from keras.optimizers import Adam, SGD
from keras.layers.embeddings import Embedding
from keras import backend as K


def segnet(nb_classes, optimizer=None, input_height=360, input_width=480):

    kernel = 3
    filter_size = 64
    pad = 1
    pool_size = 2

    model = Sequential()
    model.add(Layer(input_shape=(input_height, input_width, 3)))

    # encoder
    model.add(ZeroPadding2D(padding=(pad, pad)))
    model.add(Convolution2D(filter_size, kernel, kernel, padding='valid'))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(pool_size, pool_size)))

    model.add(ZeroPadding2D(padding=(pad, pad)))
    model.add(Convolution2D(128, kernel, kernel, padding='valid'))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(pool_size, pool_size)))

    model.add(ZeroPadding2D(padding=(pad, pad)))
    model.add(Convolution2D(256, kernel, kernel, padding='valid'))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(pool_size, pool_size)))

    model.add(ZeroPadding2D(padding=(pad, pad)))
    model.add(Convolution2D(512, kernel, kernel, padding='valid'))
    model.add(BatchNormalization())
    model.add(Activation('relu'))

    # decoder
    model.add(ZeroPadding2D(padding=(pad, pad)))
    model.add(Convolution2D(512, kernel, kernel, padding='valid'))
    model.add(BatchNormalization())

    model.add(UpSampling2D(size=(pool_size, pool_size)))
    model.add(ZeroPadding2D(padding=(pad, pad)))
    model.add(Convolution2D(256, kernel, kernel, padding='valid'))
    model.add(BatchNormalization())

    model.add(UpSampling2D(size=(pool_size, pool_size)))
    model.add(ZeroPadding2D(padding=(pad, pad)))
    model.add(Convolution2D(128, kernel, kernel, padding='valid'))
    model.add(BatchNormalization())

    model.add(UpSampling2D(size=(pool_size, pool_size)))
    model.add(ZeroPadding2D(padding=(pad, pad)))
    model.add(Convolution2D(filter_size, kernel, kernel, padding='valid'))
    model.add(BatchNormalization())

    model.add(Convolution2D(nb_classes, 1, 1, padding='valid'))

    model.outputHeight = model.output_shape[-2]
    model.outputWidth = model.output_shape[-1]

    model.add(Reshape((nb_classes, model.output_shape[-2] * model.output_shape[-1]),
                      input_shape=(nb_classes, model.output_shape[-2], model.output_shape[-1])))

    model.add(Permute((2, 1)))
    model.add(Activation('softmax'))

    model.compile(loss="categorical_crossentropy", optimizer=optimizer, metrics=['accuracy'])


    return model

