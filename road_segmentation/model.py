from keras import backend as K
from keras.models import Model
from keras.layers import Input, merge, Convolution2D, MaxPooling2D, UpSampling2D,Lambda, Flatten
from keras.models import Sequential
from keras.layers import Merge, Activation
from keras.layers.core import Layer
from keras.layers.normalization import BatchNormalization
from keras.layers.convolutional import Convolution2D, MaxPooling2D, UpSampling2D, ZeroPadding2D
from keras.optimizers import Adam
from keras import backend as K
from keras.metrics import binary_accuracy


smooth = 1.
img_rows = 480
img_cols = 640


def IOU_calc(y_true, y_pred):
    y_true_f = K.flatten(y_true)
    y_pred_f = K.flatten(y_pred)
    intersection = K.sum(y_true_f * y_pred_f)

    return 2*(intersection + smooth) / (K.sum(y_true_f) + K.sum(y_pred_f) + smooth)


def IOU_calc_loss(y_true, y_pred):
    return -IOU_calc(y_true, y_pred)


def dice_coef(y_true, y_pred):
    y_true_f = K.flatten(y_true)
    y_pred_f = K.flatten(y_pred)
    intersection = K.sum(y_true_f * y_pred_f)
    return (2. * intersection + smooth) / (K.sum(y_true_f) + K.sum(y_pred_f) + smooth)


def dice_coef_loss(y_true, y_pred):
    return -dice_coef(y_true, y_pred)


def segnet(nb_classes, optimizer=Adam(lr=0.001, epsilon=1e-08, decay=0.0), input_height=480, input_width=640):

    kernel = 3
    filter_size = 64
    pad = 1
    pool_size = 2

    model = Sequential()
    model.add(Layer(input_shape=(input_height, input_width, 3)))

    # encoder
    model.add(ZeroPadding2D(padding=(pad, pad)))
    model.add(Convolution2D(filter_size, (kernel, kernel), padding='valid'))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(pool_size, pool_size)))

    model.add(ZeroPadding2D(padding=(pad, pad)))
    model.add(Convolution2D(128, (kernel, kernel), padding='valid'))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(pool_size, pool_size)))

    model.add(ZeroPadding2D(padding=(pad, pad)))
    model.add(Convolution2D(256, (kernel, kernel), padding='valid'))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(pool_size, pool_size)))

    model.add(ZeroPadding2D(padding=(pad, pad)))
    model.add(Convolution2D(512, (kernel, kernel), padding='valid'))
    model.add(BatchNormalization())
    model.add(Activation('relu'))

    # decoder
    model.add(ZeroPadding2D(padding=(pad, pad)))
    model.add(Convolution2D(512, (kernel, kernel), padding='valid'))
    model.add(BatchNormalization())

    model.add(UpSampling2D(size=(pool_size, pool_size)))
    model.add(ZeroPadding2D(padding=(pad, pad)))
    model.add(Convolution2D(256, (kernel, kernel), padding='valid'))
    model.add(BatchNormalization())

    model.add(UpSampling2D(size=(pool_size, pool_size)))
    model.add(ZeroPadding2D(padding=(pad, pad)))
    model.add(Convolution2D(128, (kernel, kernel), padding='valid'))
    model.add(BatchNormalization())

    model.add(UpSampling2D(size=(pool_size, pool_size)))
    model.add(ZeroPadding2D(padding=(pad, pad)))
    model.add(Convolution2D(filter_size, (kernel, kernel), padding='valid'))
    model.add(BatchNormalization())

    model.add(Convolution2D(nb_classes, (1, 1), activation='sigmoid'))
    # model.outputHeight = model.output_shape[-2]
    # model.outputWidth = model.output_shape[-1]
    # model.add(Reshape((model.output_shape[-3] * model.output_shape[-2], nb_classes),
    #                   input_shape=(model.output_shape[-3], model.output_shape[-2], nb_classes)))
    # model.add(Permute((2, 1)))
    # model.add(Activation('softmax'))

    model.compile(loss="categorical_crossentropy", optimizer=optimizer, metrics=['accuracy'])

    return model


def fcn_model():

    inputs = Input((img_rows, img_cols, 3))
    inputs_norm = Lambda(lambda x: x/127.5 - 1.)

    conv1 = Convolution2D(8, (3, 3), activation="relu", padding="same")(inputs)
    conv1 = Convolution2D(8, (3, 3), activation="relu", padding="same")(conv1)
    pool1 = MaxPooling2D(pool_size=(2, 2))(conv1)

    conv2 = Convolution2D(16, (3, 3), activation='relu', padding='same')(pool1)
    conv2 = Convolution2D(16, (3, 3), activation='relu', padding='same')(conv2)
    pool2 = MaxPooling2D(pool_size=(2, 2))(conv2)

    conv3 = Convolution2D(32, (3, 3), activation='relu', padding='same')(pool2)
    conv3 = Convolution2D(32, (3, 3), activation='relu', padding='same')(conv3)
    pool3 = MaxPooling2D(pool_size=(2, 2))(conv3)

    conv4 = Convolution2D(64, (3, 3), activation='relu', padding='same')(pool3)
    conv4 = Convolution2D(64, (3, 3), activation='relu', padding='same')(conv4)
    pool4 = MaxPooling2D(pool_size=(2, 2))(conv4)

    conv5 = Convolution2D(128, (3, 3), activation='relu', padding='same')(pool4)
    conv5 = Convolution2D(128, (3, 3), activation='relu', padding='same')(conv5)

    up6 = merge([UpSampling2D(size=(2, 2))(conv5), conv4], mode='concat', concat_axis=3)
    conv6 = Convolution2D(64, (3, 3), activation='relu', padding='same')(up6)
    conv6 = Convolution2D(64, (3, 3), activation='relu', padding='same')(conv6)

    up7 = merge([UpSampling2D(size=(2, 2))(conv6), conv3], mode='concat', concat_axis=3)
    conv7 = Convolution2D(32, (3, 3), activation='relu', padding='same')(up7)
    conv7 = Convolution2D(32, (3, 3), activation='relu', padding='same')(conv7)

    up8 = merge([UpSampling2D(size=(2, 2))(conv7), conv2], mode='concat', concat_axis=3)
    conv8 = Convolution2D(16, (3, 3), activation='relu', padding='same')(up8)
    conv8 = Convolution2D(16, (3, 3), activation='relu', padding='same')(conv8)

    up9 = merge([UpSampling2D(size=(2, 2))(conv8), conv1], mode='concat', concat_axis=3)
    conv9 = Convolution2D(8, (3, 3), activation='relu', padding='same')(up9)
    conv9 = Convolution2D(8, (3, 3), activation='relu', padding='same')(conv9)

    conv10 = Convolution2D(2, (1, 1), activation='sigmoid')(conv9)

    model = Model(input=inputs, output=conv10)
    model.compile(optimizer=Adam(lr=1e-4), loss="binary_crossentropy", metrics=[binary_accuracy])

    return model
