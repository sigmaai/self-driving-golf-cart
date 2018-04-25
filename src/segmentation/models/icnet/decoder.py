# coding=utf-8
from keras import backend as K
from keras.layers import Activation, Lambda
from keras.layers.convolutional import Conv2D
from keras.layers.merge import Add

from ..layers.core import interp, Conv2D_BN as ConvBN


def conv_block_side(x):
    x = ConvBN(
        filters=32,
        kernel_size=3,
        strides=2,
        padding='same',
        activation='relu',
        name='conv1_sub1')(x)

    x = ConvBN(
        filters=32,
        kernel_size=3,
        strides=2,
        padding='same',
        activation='relu',
        name='conv2_sub1')(x)

    x = ConvBN(
        filters=64,
        kernel_size=3,
        strides=2,
        padding='same',
        activation='relu',
        name='conv3_sub1')(x)

    x = ConvBN(
        filters=128,
        kernel_size=1,
        padding='same',
        name='conv3_sub1_proj')(x)

    return x


def build(inp, encoder, nc, valid_shapes):
    side = conv_block_side(inp)

    x = Lambda(
        interp,
        arguments={'shape': valid_shapes[3]},
        name='sub24_sum_interp')(encoder)

    main = ConvBN(
        filters=128,
        kernel_size=3,
        dilation_rate=2,
        padding='same',
        name='conv_sub2')(x)

    x = Add(name='sub12_sum')([main, side])
    x = Activation('relu')(x)

    x = Lambda(
        interp,
        arguments={'shape': valid_shapes[2]},
        name='sub12_sum_interp')(x)

    x = Conv2D(
        filters=nc,
        kernel_size=1,
        name='conv6_cls')(x)

    out = Lambda(
        interp,
        arguments={'shape': valid_shapes[0]},
        name='conv6_interp')(x)

    return out
