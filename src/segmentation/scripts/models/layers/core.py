# coding=utf-8
from keras.layers import Layer
from keras.layers.convolutional import Conv2D
from keras.layers.normalization import BatchNormalization
from keras import backend as K


def interp(x, shape):
    from keras.backend import tf as ktf
    target_h, target_w = shape
    resized = ktf.image.resize_images(x, [target_h, target_w],
                                      align_corners=True)
    return resized


class Interp(Layer):
    def __init__(self, target_h, target_w, **kwargs):
        super(Interp, self).__init__(**kwargs)
        self.target_h = target_h
        self.target_w = target_w

    def call(self, x, **kwargs):
        from keras.backend import tf as ktf
        # target_h, target_w = shape
        resized = ktf.image.resize_images(x, [self.target_h, self.target_w],
                                          align_corners=True)
        return resized

    def compute_output_shape(self, input_shape):
        if K.image_data_format() == 'channels_last':
            n, h, w, c = K.int_shape(input_shape)
            return n, self.target_h, self.target_w, c
        else:
            n, c, h, w = K.int_shape(input_shape)
            return n, c, self.target_h, self.target_w


class Conv2D_BN(Conv2D):
    def __init__(self, filters,
                 kernel_size,
                 strides=(1, 1),
                 padding='valid',
                 data_format=None,
                 dilation_rate=(1, 1),
                 activation=None,
                 use_bias=True,
                 kernel_initializer='glorot_uniform',
                 bias_initializer='zeros',
                 kernel_regularizer=None,
                 bias_regularizer=None,
                 activity_regularizer=None,
                 kernel_constraint=None,
                 bias_constraint=None,
                 momentum=0.95,
                 **kwargs):
        super(Conv2D_BN, self).__init__(
            filters=filters,
            kernel_size=kernel_size,
            strides=strides,
            padding=padding,
            data_format=data_format,
            dilation_rate=dilation_rate,
            activation=activation,
            use_bias=use_bias,
            kernel_initializer=kernel_initializer,
            bias_initializer=bias_initializer,
            kernel_regularizer=kernel_regularizer,
            bias_regularizer=bias_regularizer,
            activity_regularizer=activity_regularizer,
            kernel_constraint=kernel_constraint,
            bias_constraint=bias_constraint,
            **kwargs)
        self.momentum = momentum

    def call(self, inputs, **kwargs):
        outputs = K.conv2d(
            inputs,
            self.kernel,
            strides=self.strides,
            padding=self.padding,
            data_format=self.data_format,
            dilation_rate=self.dilation_rate)
        if self.use_bias:
            outputs = K.bias_add(
                outputs,
                self.bias,
                data_format=self.data_format)
        outputs = BatchNormalization(momentum=self.momentum)(outputs)
        if self.activation is not None:
            return self.activation(outputs)
        return outputs
