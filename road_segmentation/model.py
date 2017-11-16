import keras as K
from keras.models import Model
from keras.layers import Input, merge, Convolution2D, MaxPooling2D, UpSampling2D,Lambda

smooth = 1.
img_rows = 640
img_cols = 960


def IOU_calc(y_true, y_pred):
    y_true_f = K.flatten(y_true)
    y_pred_f = K.flatten(y_pred)
    intersection = K.sum(y_true_f * y_pred_f)

    return 2*(intersection + smooth) / (K.sum(y_true_f) + K.sum(y_pred_f) + smooth)


def IOU_calc_loss(y_true, y_pred):
    return -IOU_calc(y_true, y_pred)


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

    conv10 = Convolution2D(1, (1, 1), activation='sigmoid')(conv9)

    model = Model(input=inputs, output=conv10)
    print(model.summary())
    return model
