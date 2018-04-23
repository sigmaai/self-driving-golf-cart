# coding=utf-8
from __future__ import absolute_import, print_function
import keras.backend as K
from keras.engine.topology import Input
from keras.layers.core import Activation, Reshape
from keras.layers import Convolution2D
from keras.models import Model

from . import encoder, decoder


def transfer_weights(model, weights=None):
    """
    Always trains from scratch; never transfers weights
    :param model: 
    :param weights:
    :return: 
    """
    print('ENet has found no compatible pretrained weights! Skipping weight transfer...')
    return model


def IOU_calc(y_true, y_pred):
    y_true_f = K.flatten(y_true)
    y_pred_f = K.flatten(y_pred)
    intersection = K.sum(y_true_f * y_pred_f)

    return 2*(intersection + 1) / (K.sum(y_true_f) + K.sum(y_pred_f) + 1)


def IOU_calc_loss(y_true, y_pred):
    return -IOU_calc(y_true, y_pred)


def build(nc, h, w, loss='categorical_crossentropy', optimizer='adam'):

    inp = Input(shape=(h, w, 3))
    enet = encoder.build(inp)
    enet = decoder.build(enet, nc=nc)

    enet = Activation('sigmoid')(enet)
    # enet = Convolution2D(3, (1, 1), activation='sigmoid')(enet)
    model = Model(inputs=inp, outputs=enet)

    model.compile(optimizer=optimizer, loss=loss, metrics=['accuracy', 'mean_squared_error'])

    return model
