# coding=utf-8
from __future__ import absolute_import, print_function

from keras import backend as K
from keras.engine.topology import Input
from keras.layers.core import Activation, Reshape
from keras.models import Model
from keras.utils import plot_model

from . import encoder
from . import decoder


def transfer_weights(model, weights=None):
    """
    Always trains from scratch; never transfers weights
    :param model: 
    :param weights:
    :return: 
    """
    print('ICNet has found no compatible pretrained weights! Skipping weight transfer...')
    return model


def valid_shapes(inp):
    shapes = []
    full_shape = K.int_shape(inp)
    full_h = full_shape[1 if K.image_data_format() == 'channels_last' else 2]
    full_w = full_shape[2 if K.image_data_format() == 'channels_last' else 3]
    shapes.append([full_h, full_w])
    for i in range(1, 10):
        old_h = shapes[-1][0]
        old_w = shapes[-1][1]
        shapes.append([(old_h + 1) // 2, (old_w + 1) // 2])
        if shapes[-1][0] < 2 and shapes[-1][1] < 2:
            break
    return shapes


def build(nc, w, h,
          loss='categorical_crossentropy',
          optimizer='adadelta',
          plot=False,
          **kwargs):
    # data_shape = input_shape[0] * input_shape[1] if input_shape and None not in input_shape else None
    data_shape = w * h if None not in (w, h) else -1  # TODO: -1 or None?
    inp = Input(shape=(h, w, 3))
    shapes = valid_shapes(inp)

    if h < 161 or w < 161:
        errmsg = 'Input image tensor must be at least 161pxs in both width and height'
        raise ValueError(errmsg)

    out = encoder.build(inp, valid_shapes=shapes)
    out = decoder.build(inp=inp, encoder=out, nc=nc, valid_shapes=shapes)

    out = Reshape((data_shape, nc))(out)  # TODO: need to remove data_shape for multi-scale training
    out = Activation('softmax')(out)
    model = Model(inputs=inp, outputs=out)

    model.compile(optimizer=optimizer, loss=loss, metrics=['accuracy', 'mean_squared_error'])
    name = 'icnet'

    if plot:
        plot_model(model, to_file='{}.png'.format(name), show_shapes=True)

    return model, name


if __name__ == "__main__":
    autoencoder, name = build(nc=19, w=1025, h=2049, plot=True)
