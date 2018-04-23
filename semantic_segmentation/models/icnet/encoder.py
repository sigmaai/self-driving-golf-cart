# coding=utf-8
from keras.layers.convolutional import Conv2D
from keras.layers.core import Activation, Lambda
from keras.layers.merge import Add
from keras.layers.normalization import BatchNormalization
from keras.layers.pooling import AveragePooling2D, MaxPooling2D
from ..layers.core import interp, Conv2D_BN as ConvBN
from keras import backend as K


def build(inp, valid_shapes):
    global g_valid_shapes
    g_valid_shapes = valid_shapes

    def conv_block(x,
                   filters,
                   name_infix,
                   kernel_sizes=(1, 1, 1),
                   strides=((1, 1), (1, 1), (1, 1)),
                   padding=('same', 'same', 'same'),
                   dilation_rates=((1, 1), (1, 1), (1, 1)),
                   activations=('relu', 'relu', None)):
        names = ['conv{}_1x1_reduce'.format(name_infix),
                 'conv{}_3x3'.format(name_infix),
                 'conv{}_1x1_increase'.format(name_infix)]
        filters = (filters, filters, filters * 4)
        if kernel_sizes is None:
            kernel_sizes = (1, 1, 1)
        if isinstance(kernel_sizes, int):
            kernel_sizes = [kernel_sizes] * 3
        if strides is None:
            strides = (1, 1, 1)
        if isinstance(strides, int):
            strides = [strides] * 3

        for idx in range(3):
            x = ConvBN(
                filters=filters[idx],
                kernel_size=kernel_sizes[idx],
                strides=strides[idx],
                padding=padding[idx],
                dilation_rate=dilation_rates[idx],
                activation=activations[idx],
                name=names[idx])(x)
        return x

    def block_1(b0):
        b1_1 = ConvBN(
            filters=32,
            kernel_size=3,
            strides=2,
            padding='same',
            activation='relu',
            name='conv1_1_3x3_s2')(b0)
        b1_2 = ConvBN(
            filters=32,
            kernel_size=3,
            padding='same',
            activation='relu',
            name='conv1_2_3x3')(b1_1)

        b1_3 = ConvBN(
            filters=64,
            kernel_size=3,
            padding='same',
            name='conv1_3_3x3',
            activation='relu')(b1_2)

        b1_4 = MaxPooling2D(pool_size=3,
                            strides=2,
                            padding='same')(b1_3)
        return b1_4

    def block_2(b1):
        def block_2_1(x):
            main = conv_block(
                x,
                filters=32,
                name_infix='2_1',
                kernel_sizes=(1, 3, 1))
            skip = ConvBN(
                filters=128,
                kernel_size=1,
                padding='same',
                name='conv2_1_1x1_proj')(x)
            x = Add()([main, skip])
            x = Activation('relu')(x)
            return x

        def block_2_2(x):
            main = conv_block(
                x,
                filters=32,
                name_infix='2_2',
                kernel_sizes=(1, 3, 1))
            skip = x
            x = Add()([main, skip])
            x = Activation('relu')(x)
            return x

        def block_2_3(x):
            main = conv_block(
                x,
                filters=32,
                name_infix='2_3',
                kernel_sizes=(1, 3, 1))
            skip = x
            x = Add()([main, skip])
            x = Activation('relu')(x)
            return x

        b2_1 = block_2_1(b1)
        b2_2 = block_2_2(b2_1)
        b2_3 = block_2_3(b2_2)
        return b2_3

    def block_3(b2):
        def generic_block_3(x, name_infix, strides=None, skip=None):
            if strides is None:
                strides = ((1, 1), (1, 1), (1, 1))
            skip = x if skip is None else skip(x)
            dilation_rates = ((1, 1), (1, 1), (1, 1))
            main = conv_block(
                x,
                filters=64,
                name_infix=name_infix,
                kernel_sizes=(1, 3, 1),
                strides=strides,
                dilation_rates=dilation_rates)
            x = Add()([main, skip])
            x = Activation('relu')(x)
            return x

        def block_3_1(x):
            skip = ConvBN(
                filters=256,
                kernel_size=1,
                strides=2,
                padding='same',
                name='conv3_1_1x1_proj')
            x = generic_block_3(x, name_infix='3_1', strides=[2, 1, 1], skip=skip)
            return x

        def block_3_2(x):
            x = Lambda(
                interp,
                arguments={'shape': g_valid_shapes[5]},
                name='conv3_1_sub4')(x)
            x = generic_block_3(x, name_infix='3_2')
            return x

        b3_1 = block_3_1(b2)
        b3_2 = block_3_2(b3_1)
        b3_3 = generic_block_3(b3_2, name_infix='3_3')
        b3_4 = generic_block_3(b3_3, name_infix='3_4')
        return b3_4, b3_1

    def block_4(b3):
        def generic_block_4(x, name_infix, skip=None):
            skip = x if skip is None else skip(x)
            dilation_rates = ((1, 1), (2, 2), (1, 1))
            main = conv_block(
                x,
                filters=128,
                name_infix=name_infix,
                kernel_sizes=[1, 3, 1],
                dilation_rates=dilation_rates)
            x = Add()([main, skip])
            return x

        def block_4_1(x):
            skip = ConvBN(
                filters=512,
                kernel_size=1,
                padding='same',
                name='conv4_1_1x1_proj')
            x = generic_block_4(x, name_infix='4_1', skip=skip)
            return x

        b4_1 = block_4_1(b3)
        b4_2 = generic_block_4(b4_1, name_infix='4_2')
        b4_3 = generic_block_4(b4_2, name_infix='4_3')
        b4_4 = generic_block_4(b4_3, name_infix='4_4')
        b4_5 = generic_block_4(b4_4, name_infix='4_5')
        b4_6 = generic_block_4(b4_5, name_infix='4_6')
        return b4_6

    def block_5(b4, target_shape=None):
        def generic_block_5(x, name_infix, skip=None):
            skip = x if skip is None else skip(x)
            dilation_rates = ((1, 1), (4, 4), (1, 1))
            main = conv_block(
                x,
                filters=256,
                name_infix=name_infix,
                kernel_sizes=(1, 3, 1),
                dilation_rates=dilation_rates)
            x = Add()([main, skip])
            x = Activation('relu')(x)
            return x

        def block_5_1(x):
            skip = ConvBN(
                filters=1024,
                kernel_size=1,
                padding='same',
                name='conv5_1_1x1_proj')
            x = generic_block_5(x, name_infix='5_1', skip=skip)
            return x

        def pyramid_pooling_5(x):
            shape = g_valid_shapes[5]
            pool6 = AveragePooling2D(
                pool_size=[(d + 1) // 6 for d in shape],
                strides=[d // 6 for d in shape],
                name='conv5_3_pool6')(x)
            pool6 = Lambda(
                interp,
                arguments={'shape': shape},
                name='conv5_3_pool6_interp')(pool6)

            pool3 = AveragePooling2D(
                pool_size=[(d + 1) // 3 for d in shape],
                strides=[d // 3 for d in shape],
                name='conv5_3_pool3')(x)
            pool3 = Lambda(
                interp,
                arguments={'shape': shape},
                name='conv5_3_pool3_interp')(pool3)

            pool2 = AveragePooling2D(
                pool_size=[(d + 1) // 2 for d in shape],
                strides=[d // 2 for d in shape],
                name='conv5_3_pool2')(x)
            pool2 = Lambda(
                interp,
                arguments={'shape': shape},
                name='conv5_3_pool2_interp')(pool2)

            pool1 = AveragePooling2D(
                pool_size=shape,
                strides=shape,
                name='conv5_3_pool1')(x)
            pool1 = Lambda(
                interp,
                arguments={'shape': shape},
                name='conv5_3_pool1_interp')(pool1)
            x = Add()([x, pool1, pool2, pool3, pool6])
            return x

        def block_5_4(x):
            x = ConvBN(
                filters=256,
                padding='same',
                kernel_size=1,
                strides=1,
                activation='relu',
                name='conv5_4_k1')(x)

            x = Lambda(
                interp,
                arguments={'shape': g_valid_shapes[4]},
                name='conv5_4_interp')(x)

            x = ConvBN(
                filters=128,
                kernel_size=3,
                dilation_rate=2,
                padding='same',
                name='conv_sub4')(x)

            return x

        b5_1 = block_5_1(b4)
        b5_2 = generic_block_5(b5_1, name_infix='5_2')
        b5_3a = generic_block_5(b5_2, name_infix='5_3')
        b5_3 = pyramid_pooling_5(b5_3a)
        b5_4 = block_5_4(b5_3)
        return b5_4

    b0 = Lambda(interp,
                arguments={'shape': valid_shapes[1]},
                name='data_sub2')(inp)
    b1 = block_1(b0)
    b2 = block_2(b1)
    b3, b3a = block_3(b2)
    b4 = block_4(b3)

    b5 = block_5(b4, target_shape=valid_shapes[4])

    b3b = ConvBN(
        filters=128,
        kernel_size=1,
        padding='same',
        name='conv3_1_sub2_proj')(b3a)
    x = Add(name='sub24_sum')([b3b, b5])
    out = Activation('relu')(x)

    return out
