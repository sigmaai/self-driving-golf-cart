#
# ICNet Keras Implementation
# Paper: https://arxiv.org/abs/1704.08545
# Originally by @aitorzip
# Adapted by @NeilNie
#
# (c) Yongyang Nie, 2018. All Rights Reserved.
# MIT License
#

from keras.layers import Activation
from keras.layers import Lambda
from keras.layers import Conv2D
from keras.layers import Add
from keras.layers import MaxPooling2D
from keras.layers import AveragePooling2D
from keras.layers import ZeroPadding2D
from keras.layers import Input
from keras.layers import BatchNormalization
from keras.models import Model
import tensorflow as tf


class ICNet:

    def __init__(self, width, height, n_classes, training, mode="early_fusion", weight_path=None, depth=6, ):

        '''

        :param width: width of input image. def. 1024
        :param height: height of input image. def. 512
        :param n_classes: number of classes to be classified. def. 34
        :param mode: choose one of the three: early, mid, late
        :param weight_path: path of the pretrained model
        :param depth: depth of the input
        '''

        self.width = width
        self.height = height
        self.n_classes = n_classes
        self.weight_path = weight_path

        if mode == "early_fusion":
            self.model = self.build_early_fusion(width=self.width, height=self.height, n_classes=self.n_classes, depth=depth)
        elif mode == "mid_fusion":
            self.model = self.build_mid_fusion(width=self.width, height=self.height, n_classes=self.n_classes)
        elif "cross_fusion" in mode:
            self.model = self.build_cross_fusion(width=self.width, height=self.height, n_classes=self.n_classes, training=training)
        else:
            raise ValueError("Can't recognize mode")

        if weight_path:
            self.model.load_weights(weight_path)
            print("Model Created \n Weights Loaded. Path: {}".format(weight_path))
        else:
            print("Model Created \n No weight path provided. ")

    def build_early_fusion(self, width, height, n_classes, weights_path=None, depth=6):

        inp_color = Input(shape=(height, width, depth))
        x_color = Lambda(lambda x: (x - 127.5) / 255.0)(inp_color)

        z = self.build_one_half(x_color)

        y = self.build_one_quarter(z)

        h, w = y.shape[1:3].as_list()
        pool1 = AveragePooling2D(pool_size=(h, w), strides=(h, w), name='conv5_3_pool1')(y)
        pool1 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='conv5_3_pool1_interp')(pool1)
        pool2 = AveragePooling2D(pool_size=(h / 2, w / 2), strides=(h // 2, w // 2), name='conv5_3_pool2')(y)
        pool2 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='conv5_3_pool2_interp')(pool2)
        pool3 = AveragePooling2D(pool_size=(h / 3, w / 3), strides=(h // 3, w // 3), name='conv5_3_pool3')(y)
        pool3 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='conv5_3_pool3_interp')(pool3)
        pool6 = AveragePooling2D(pool_size=(h / 4, w / 4), strides=(h // 4, w // 4), name='conv5_3_pool6')(y)
        pool6 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='conv5_3_pool6_interp')(pool6)

        y = Add(name='conv5_3_sum')([y, pool1, pool2, pool3, pool6])

        y = Conv2D(256, 1, activation='relu', name='conv5_4_k1')(y)
        y = BatchNormalization(name='conv5_4_k1_bn')(y)

        aux_1 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) * 2, int(x.shape[2]) * 2)),
                       name='conv5_4_interp')(y)
        y = ZeroPadding2D(padding=2, name='padding17')(aux_1)
        y = Conv2D(128, 3, dilation_rate=2, name='conv_sub4')(y)
        y = BatchNormalization(name='conv_sub4_bn')(y)
        y_ = Conv2D(128, 1, name='conv3_1_sub2_proj')(z)
        y_ = BatchNormalization(name='conv3_1_sub2_proj_bn')(y_)
        y = Add(name='sub24_sum')([y, y_])
        y = Activation('relu', name='sub24_sum/relu')(y)

        aux_2 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) * 2, int(x.shape[2]) * 2)),
                       name='sub24_sum_interp')(y)
        y = ZeroPadding2D(padding=2, name='padding18')(aux_2)
        y_ = Conv2D(128, 3, dilation_rate=2, name='conv_sub2')(y)
        y_ = BatchNormalization(name='conv_sub2_bn')(y_)

        y = self.build_one(x_color)

        y = Add(name='sub12_sum')([y, y_])
        y = Activation('relu', name='sub12_sum/relu')(y)
        y = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) * 2, int(x.shape[2]) * 2)),
                   name='sub12_sum_interp')(y)

        out = Conv2D(n_classes, 1, activation='softmax', name='conv6_cls')(y)

        model = Model(inputs=inp_color, outputs=out)

        if weights_path is not None:
            model.load_weights(weights_path, by_name=True)
        return model

    def build_mid_fusion(self, width, height, n_classes, weights_path=None):

        inp_color = Input(shape=(height, width, 3))
        inp_depth = Input(shape=(height, width, 3))

        x_depth = Lambda(lambda x: (x - 127.5) / 255.0)(inp_depth)
        x_color = Lambda(lambda x: (x - 127.5) / 255.0)(inp_color)

        z_color = self.build_one_half(x_color)
        z_depth = self.build_one_half_depth(x_depth)

        y_color = self.build_one_quarter(z_color)
        y_depth = self.build_one_quarter_depth(z_depth)

        h, w = y_color.shape[1:3].as_list()
        pool1 = AveragePooling2D(pool_size=(h, w), strides=(h, w), name='conv5_3_pool1')(y_color)
        pool1 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='conv5_3_pool1_interp')(pool1)
        pool2 = AveragePooling2D(pool_size=(h / 2, w / 2), strides=(h // 2, w // 2), name='conv5_3_pool2')(y_color)
        pool2 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='conv5_3_pool2_interp')(pool2)
        pool3 = AveragePooling2D(pool_size=(h / 3, w / 3), strides=(h // 3, w // 3), name='conv5_3_pool3')(y_color)
        pool3 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='conv5_3_pool3_interp')(pool3)
        pool6 = AveragePooling2D(pool_size=(h / 4, w / 4), strides=(h // 4, w // 4), name='conv5_3_pool6')(y_color)
        pool6 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='conv5_3_pool6_interp')(pool6)

        pool1_d = AveragePooling2D(pool_size=(h, w), strides=(h, w), name='d_conv5_3_pool1')(y_depth)
        pool1_d = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='d_conv5_3_pool1_interp')(pool1_d)
        pool2_d = AveragePooling2D(pool_size=(h / 2, w / 2), strides=(h // 2, w // 2), name='d_conv5_3_pool2')(y_depth)
        pool2_d = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='d_conv5_3_pool2_interp')(pool2_d)
        pool3_d = AveragePooling2D(pool_size=(h / 3, w / 3), strides=(h // 3, w // 3), name='d_conv5_3_pool3')(y_depth)
        pool3_d = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='d_conv5_3_pool3_interp')(pool3_d)
        pool6_d = AveragePooling2D(pool_size=(h / 4, w / 4), strides=(h // 4, w // 4), name='d_conv5_3_pool6')(y_depth)
        pool6_d = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='d_conv5_3_pool6_interp')(pool6_d)

        y_color = Add(name='conv5_3_sum')([y_color, pool1, pool2, pool3, pool6])
        y_depth = Add(name='d_conv5_3_sum')([y_depth, pool1_d, pool2_d, pool3_d, pool6_d])
        y = Add(name='conv5_cd_3_sum')([y_color, y_depth])

        y = Conv2D(256, 1, activation='relu', name='conv5_4_k1')(y)
        y = BatchNormalization(name='conv5_4_k1_bn')(y)

        aux_1 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) * 2, int(x.shape[2]) * 2)),
                       name='conv5_4_interp')(y)
        y = ZeroPadding2D(padding=2, name='padding17')(aux_1)
        y = Conv2D(128, 3, dilation_rate=2, name='conv_sub4')(y)
        y = BatchNormalization(name='conv_sub4_bn')(y)
        y_ = Conv2D(128, 1, name='conv3_1_sub2_proj')(z_color)
        y_ = BatchNormalization(name='conv3_1_sub2_proj_bn')(y_)
        y = Add(name='sub24_sum')([y, y_])
        y = Activation('relu', name='sub24_sum/relu')(y)

        aux_2 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) * 2, int(x.shape[2]) * 2)),
                       name='sub24_sum_interp')(y)
        y = ZeroPadding2D(padding=2, name='padding18')(aux_2)
        y_ = Conv2D(128, 3, dilation_rate=2, name='conv_sub2')(y)
        y_ = BatchNormalization(name='conv_sub2_bn')(y_)

        y = self.build_one(x_color)

        y = Add(name='sub12_sum')([y, y_])
        y = Activation('relu', name='sub12_sum/relu')(y)
        y = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) * 2, int(x.shape[2]) * 2)),
                   name='sub12_sum_interp')(y)

        out = Conv2D(n_classes, 1, activation='softmax', name='conv6_cls')(y)

        model = Model(inputs=[inp_color, inp_depth], outputs=out)

        if weights_path is not None:
            model.load_weights(weights_path, by_name=True)
        return model

    # ===================
    # Cross fusion method
    # ===================

    def build_cross_fusion(self, width, height, n_classes, weights_path=None, training=False):

        """
        Build cross fusion model. It's a better model than early and late fusion.
        :param width:
        :param height:
        :param n_classes:
        :param weights_path:
        :return:
        """
        inp_color = Input(shape=(height, width, 3), name="color_input")
        inp_depth = Input(shape=(height, width, 3), name="depth_input")

        x_depth = Lambda(lambda x: (x - 127.5) / 255.0, name="lambda_depth")(inp_depth)
        x_color = Lambda(lambda x: (x - 127.5) / 255.0, name="lambda_color")(inp_color)

        z_color, z_depth = self.build_fusion_one_half(x_color, x_depth)

        y_color, y_depth = self.build_fusion_one_quarter(z_color, z_depth)

        h, w = y_color.shape[1:3].as_list()
        pool1 = AveragePooling2D(pool_size=(h, w), strides=(h, w), name='conv5_3_pool1')(y_color)
        pool1 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='conv5_3_pool1_interp')(pool1)
        pool2 = AveragePooling2D(pool_size=(h / 2, w / 2), strides=(h // 2, w // 2), name='conv5_3_pool2')(y_color)
        pool2 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='conv5_3_pool2_interp')(pool2)
        pool3 = AveragePooling2D(pool_size=(h / 3, w / 3), strides=(h // 3, w // 3), name='conv5_3_pool3')(y_color)
        pool3 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='conv5_3_pool3_interp')(pool3)
        pool6 = AveragePooling2D(pool_size=(h / 4, w / 4), strides=(h // 4, w // 4), name='conv5_3_pool6')(y_color)
        pool6 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='conv5_3_pool6_interp')(pool6)

        pool1_d = AveragePooling2D(pool_size=(h, w), strides=(h, w), name='d_conv5_3_pool1')(y_depth)
        pool1_d = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='d_conv5_3_pool1_interp')(pool1_d)
        pool2_d = AveragePooling2D(pool_size=(h / 2, w / 2), strides=(h // 2, w // 2), name='d_conv5_3_pool2')(y_depth)
        pool2_d = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='d_conv5_3_pool2_interp')(pool2_d)
        pool3_d = AveragePooling2D(pool_size=(h / 3, w / 3), strides=(h // 3, w // 3), name='d_conv5_3_pool3')(y_depth)
        pool3_d = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='d_conv5_3_pool3_interp')(pool3_d)
        pool6_d = AveragePooling2D(pool_size=(h / 4, w / 4), strides=(h // 4, w // 4), name='d_conv5_3_pool6')(y_depth)
        pool6_d = Lambda(lambda x: tf.image.resize_bilinear(x, size=(h, w)), name='d_conv5_3_pool6_interp')(pool6_d)

        y_color = Add(name='conv5_3_sum')([y_color, pool1, pool2, pool3, pool6])
        y_depth = Add(name='d_conv5_3_sum')([y_depth, pool1_d, pool2_d, pool3_d, pool6_d])
        y = Add(name='conv5_cd_3_sum')([y_color, y_depth])
        y = Conv2D(256, 1, activation='relu', name='conv5_4_k1')(y)
        y = BatchNormalization(name='conv5_4_k1_bn')(y)

        aux_1 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) * 2, int(x.shape[2]) * 2)),
                       name='conv5_4_interp')(y)
        y = ZeroPadding2D(padding=2, name='padding17')(aux_1)
        y = Conv2D(128, 3, dilation_rate=2, name='conv_sub4')(y)
        y = BatchNormalization(name='conv_sub4_bn')(y)

        half_merge = Add(name='half_merge')([z_color, z_depth])
        y_ = Conv2D(128, 1, name='conv3_1_sub2_proj')(half_merge)
        y_ = BatchNormalization(name='conv3_1_sub2_proj_bn')(y_)
        y = Add(name='sub24_sum')([y, y_])
        y = Activation('relu', name='sub24_sum/relu')(y)

        aux_2 = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) * 2, int(x.shape[2]) * 2)),
                       name='sub24_sum_interp')(y)
        y = ZeroPadding2D(padding=2, name='padding18')(aux_2)
        y_ = Conv2D(128, 3, dilation_rate=2, name='conv_sub2')(y)
        y_ = BatchNormalization(name='conv_sub2_bn')(y_)

        y = self.build_fusion_one(x_color, x_depth)

        y = Add(name='sub12_sum')([y, y_])
        y = Activation('relu', name='sub12_sum/relu')(y)
        y = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) * 2, int(x.shape[2]) * 2)),
                   name='sub12_sum_interp')(y)

        out = Conv2D(n_classes, 1, activation='softmax', name='conv6_cls')(y)

        if training:
            aux_1 = Conv2D(n_classes, 1, activation='softmax', name='sub4_out')(aux_1)
            aux_2 = Conv2D(n_classes, 1, activation='softmax', name='sub24_out')(aux_2)

            model = Model(inputs=[inp_color, inp_depth], outputs=[out, aux_2, aux_1])
        else:
            model = Model(inputs=[inp_color, inp_depth], outputs=out)

        if weights_path is not None:
            model.load_weights(weights_path, by_name=True)
        return model

    @staticmethod
    def build_one_half(x_color):

        # (1/2)
        y = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) // 2, int(x.shape[2]) // 2)),
                   name='data_sub2')(x_color)
        y = Conv2D(32, 3, strides=2, padding='same', activation='relu', name='conv1_1_3x3_s2')(y)
        y = BatchNormalization(name='conv1_1_3x3_s2_bn')(y)
        y = Conv2D(32, 3, padding='same', activation='relu', name='conv1_2_3x3')(y)
        y = BatchNormalization(name='conv1_2_3x3_s2_bn')(y)
        y = Conv2D(64, 3, padding='same', activation='relu', name='conv1_3_3x3')(y)
        y = BatchNormalization(name='conv1_3_3x3_bn')(y)
        y_ = MaxPooling2D(pool_size=3, strides=2, name='pool1_3x3_s2')(y)

        y = Conv2D(128, 1, name='conv2_1_1x1_proj')(y_)
        y = BatchNormalization(name='conv2_1_1x1_proj_bn')(y)
        y_ = Conv2D(32, 1, activation='relu', name='conv2_1_1x1_reduce')(y_)
        y_ = BatchNormalization(name='conv2_1_1x1_reduce_bn')(y_)
        y_ = ZeroPadding2D(name='padding1')(y_)
        y_ = Conv2D(32, 3, activation='relu', name='conv2_1_3x3')(y_)
        y_ = BatchNormalization(name='conv2_1_3x3_bn')(y_)
        y_ = Conv2D(128, 1, name='conv2_1_1x1_increase')(y_)
        y_ = BatchNormalization(name='conv2_1_1x1_increase_bn')(y_)
        y = Add(name='conv2_1')([y, y_])
        y_ = Activation('relu', name='conv2_1/relu')(y)

        y = Conv2D(32, 1, activation='relu', name='conv2_2_1x1_reduce')(y_)
        y = BatchNormalization(name='conv2_2_1x1_reduce_bn')(y)
        y = ZeroPadding2D(name='padding2')(y)
        y = Conv2D(32, 3, activation='relu', name='conv2_2_3x3')(y)
        y = BatchNormalization(name='conv2_2_3x3_bn')(y)
        y = Conv2D(128, 1, name='conv2_2_1x1_increase')(y)
        y = BatchNormalization(name='conv2_2_1x1_increase_bn')(y)
        y = Add(name='conv2_2')([y, y_])
        y_ = Activation('relu', name='conv2_2/relu')(y)

        y = Conv2D(32, 1, activation='relu', name='conv2_3_1x1_reduce')(y_)
        y = BatchNormalization(name='conv2_3_1x1_reduce_bn')(y)
        y = ZeroPadding2D(name='padding3')(y)
        y = Conv2D(32, 3, activation='relu', name='conv2_3_3x3')(y)
        y = BatchNormalization(name='conv2_3_3x3_bn')(y)
        y = Conv2D(128, 1, name='conv2_3_1x1_increase')(y)
        y = BatchNormalization(name='conv2_3_1x1_increase_bn')(y)
        y = Add(name='conv2_3')([y, y_])
        y_ = Activation('relu', name='conv2_3/relu')(y)

        y = Conv2D(256, 1, strides=2, name='conv3_1_1x1_proj')(y_)
        y = BatchNormalization(name='conv3_1_1x1_proj_bn')(y)
        y_ = Conv2D(64, 1, strides=2, activation='relu', name='conv3_1_1x1_reduce')(y_)
        y_ = BatchNormalization(name='conv3_1_1x1_reduce_bn')(y_)
        y_ = ZeroPadding2D(name='padding4')(y_)
        y_ = Conv2D(64, 3, activation='relu', name='conv3_1_3x3')(y_)
        y_ = BatchNormalization(name='conv3_1_3x3_bn')(y_)
        y_ = Conv2D(256, 1, name='conv3_1_1x1_increase')(y_)
        y_ = BatchNormalization(name='conv3_1_1x1_increase_bn')(y_)
        y = Add(name='conv3_1')([y, y_])
        z = Activation('relu', name='conv3_1/relu')(y)

        return z

    @staticmethod
    def build_one_quarter(z):

        # (1/4)
        y_ = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) // 2, int(x.shape[2]) // 2)),
                    name='conv3_1_sub4')(z)
        y = Conv2D(64, 1, activation='relu', name='conv3_2_1x1_reduce')(y_)
        y = BatchNormalization(name='conv3_2_1x1_reduce_bn')(y)
        y = ZeroPadding2D(name='padding5')(y)
        y = Conv2D(64, 3, activation='relu', name='conv3_2_3x3')(y)
        y = BatchNormalization(name='conv3_2_3x3_bn')(y)
        y = Conv2D(256, 1, name='conv3_2_1x1_increase')(y)
        y = BatchNormalization(name='conv3_2_1x1_increase_bn')(y)
        y = Add(name='conv3_2')([y, y_])
        y_ = Activation('relu', name='conv3_2/relu')(y)

        y = Conv2D(64, 1, activation='relu', name='conv3_3_1x1_reduce')(y_)
        y = BatchNormalization(name='conv3_3_1x1_reduce_bn')(y)
        y = ZeroPadding2D(name='padding6')(y)
        y = Conv2D(64, 3, activation='relu', name='conv3_3_3x3')(y)
        y = BatchNormalization(name='conv3_3_3x3_bn')(y)
        y = Conv2D(256, 1, name='conv3_3_1x1_increase')(y)
        y = BatchNormalization(name='conv3_3_1x1_increase_bn')(y)
        y = Add(name='conv3_3')([y, y_])
        y_ = Activation('relu', name='conv3_3/relu')(y)

        y = Conv2D(64, 1, activation='relu', name='conv3_4_1x1_reduce')(y_)
        y = BatchNormalization(name='conv3_4_1x1_reduce_bn')(y)
        y = ZeroPadding2D(name='padding7')(y)
        y = Conv2D(64, 3, activation='relu', name='conv3_4_3x3')(y)
        y = BatchNormalization(name='conv3_4_3x3_bn')(y)
        y = Conv2D(256, 1, name='conv3_4_1x1_increase')(y)
        y = BatchNormalization(name='conv3_4_1x1_increase_bn')(y)
        y = Add(name='conv3_4')([y, y_])
        y_ = Activation('relu', name='conv3_4/relu')(y)

        y = Conv2D(512, 1, name='conv4_1_1x1_proj')(y_)
        y = BatchNormalization(name='conv4_1_1x1_proj_bn')(y)
        y_ = Conv2D(128, 1, activation='relu', name='conv4_1_1x1_reduce')(y_)
        y_ = BatchNormalization(name='conv4_1_1x1_reduce_bn')(y_)
        y_ = ZeroPadding2D(padding=2, name='padding8')(y_)
        y_ = Conv2D(128, 3, dilation_rate=2, activation='relu', name='conv4_1_3x3')(y_)
        y_ = BatchNormalization(name='conv4_1_3x3_bn')(y_)
        y_ = Conv2D(512, 1, name='conv4_1_1x1_increase')(y_)
        y_ = BatchNormalization(name='conv4_1_1x1_increase_bn')(y_)
        y = Add(name='conv4_1')([y, y_])
        y_ = Activation('relu', name='conv4_1/relu')(y)

        y = Conv2D(128, 1, activation='relu', name='conv4_2_1x1_reduce')(y_)
        y = BatchNormalization(name='conv4_2_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=2, name='padding9')(y)
        y = Conv2D(128, 3, dilation_rate=2, activation='relu', name='conv4_2_3x3')(y)
        y = BatchNormalization(name='conv4_2_3x3_bn')(y)
        y = Conv2D(512, 1, name='conv4_2_1x1_increase')(y)
        y = BatchNormalization(name='conv4_2_1x1_increase_bn')(y)
        y = Add(name='conv4_2')([y, y_])
        y_ = Activation('relu', name='conv4_2/relu')(y)

        y = Conv2D(128, 1, activation='relu', name='conv4_3_1x1_reduce')(y_)
        y = BatchNormalization(name='conv4_3_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=2, name='padding10')(y)
        y = Conv2D(128, 3, dilation_rate=2, activation='relu', name='conv4_3_3x3')(y)
        y = BatchNormalization(name='conv4_3_3x3_bn')(y)
        y = Conv2D(512, 1, name='conv4_3_1x1_increase')(y)
        y = BatchNormalization(name='conv4_3_1x1_increase_bn')(y)
        y = Add(name='conv4_3')([y, y_])
        y_ = Activation('relu', name='conv4_3/relu')(y)

        y = Conv2D(128, 1, activation='relu', name='conv4_4_1x1_reduce')(y_)
        y = BatchNormalization(name='conv4_4_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=2, name='padding11')(y)
        y = Conv2D(128, 3, dilation_rate=2, activation='relu', name='conv4_4_3x3')(y)
        y = BatchNormalization(name='conv4_4_3x3_bn')(y)
        y = Conv2D(512, 1, name='conv4_4_1x1_increase')(y)
        y = BatchNormalization(name='conv4_4_1x1_increase_bn')(y)
        y = Add(name='conv4_4')([y, y_])
        y_ = Activation('relu', name='conv4_4/relu')(y)

        y = Conv2D(128, 1, activation='relu', name='conv4_5_1x1_reduce')(y_)
        y = BatchNormalization(name='conv4_5_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=2, name='padding12')(y)
        y = Conv2D(128, 3, dilation_rate=2, activation='relu', name='conv4_5_3x3')(y)
        y = BatchNormalization(name='conv4_5_3x3_bn')(y)
        y = Conv2D(512, 1, name='conv4_5_1x1_increase')(y)
        y = BatchNormalization(name='conv4_5_1x1_increase_bn')(y)
        y = Add(name='conv4_5')([y, y_])
        y_ = Activation('relu', name='conv4_5/relu')(y)

        y = Conv2D(128, 1, activation='relu', name='conv4_6_1x1_reduce')(y_)
        y = BatchNormalization(name='conv4_6_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=2, name='padding13')(y)
        y = Conv2D(128, 3, dilation_rate=2, activation='relu', name='conv4_6_3x3')(y)
        y = BatchNormalization(name='conv4_6_3x3_bn')(y)
        y = Conv2D(512, 1, name='conv4_6_1x1_increase')(y)
        y = BatchNormalization(name='conv4_6_1x1_increase_bn')(y)
        y = Add(name='conv4_6')([y, y_])
        y = Activation('relu', name='conv4_6/relu')(y)

        y_ = Conv2D(1024, 1, name='conv5_1_1x1_proj')(y)
        y_ = BatchNormalization(name='conv5_1_1x1_proj_bn')(y_)
        y = Conv2D(256, 1, activation='relu', name='conv5_1_1x1_reduce')(y)
        y = BatchNormalization(name='conv5_1_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=4, name='padding14')(y)
        y = Conv2D(256, 3, dilation_rate=4, activation='relu', name='conv5_1_3x3')(y)
        y = BatchNormalization(name='conv5_1_3x3_bn')(y)
        y = Conv2D(1024, 1, name='conv5_1_1x1_increase')(y)
        y = BatchNormalization(name='conv5_1_1x1_increase_bn')(y)
        y = Add(name='conv5_1')([y, y_])
        y_ = Activation('relu', name='conv5_1/relu')(y)

        y = Conv2D(256, 1, activation='relu', name='conv5_2_1x1_reduce')(y_)
        y = BatchNormalization(name='conv5_2_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=4, name='padding15')(y)
        y = Conv2D(256, 3, dilation_rate=4, activation='relu', name='conv5_2_3x3')(y)
        y = BatchNormalization(name='conv5_2_3x3_bn')(y)
        y = Conv2D(1024, 1, name='conv5_2_1x1_increase')(y)
        y = BatchNormalization(name='conv5_2_1x1_increase_bn')(y)
        y = Add(name='conv5_2')([y, y_])
        y_ = Activation('relu', name='conv5_2/relu')(y)

        y = Conv2D(256, 1, activation='relu', name='conv5_3_1x1_reduce')(y_)
        y = BatchNormalization(name='conv5_3_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=4, name='padding16')(y)
        y = Conv2D(256, 3, dilation_rate=4, activation='relu', name='conv5_3_3x3')(y)
        y = BatchNormalization(name='conv5_3_3x3_bn')(y)
        y = Conv2D(1024, 1, name='conv5_3_1x1_increase')(y)
        y = BatchNormalization(name='conv5_3_1x1_increase_bn')(y)
        y = Add(name='conv5_3')([y, y_])
        y = Activation('relu', name='conv5_3/relu')(y)

        return y

    @staticmethod
    def build_one(x_color):

        # (1)
        y = Conv2D(32, 3, strides=2, padding='same', activation='relu', name='conv1_sub1')(x_color)
        y = BatchNormalization(name='conv1_sub1_bn')(y)
        y = Conv2D(32, 3, strides=2, padding='same', activation='relu', name='conv2_sub1')(y)
        y = BatchNormalization(name='conv2_sub1_bn')(y)
        y = Conv2D(64, 3, strides=2, padding='same', activation='relu', name='conv3_sub1')(y)
        y = BatchNormalization(name='conv3_sub1_bn')(y)
        y = Conv2D(128, 1, name='conv3_sub1_proj')(y)
        y = BatchNormalization(name='conv3_sub1_proj_bn')(y)

        return y

    @staticmethod
    def build_one_half_depth(x_color):

        # (1/2)
        y = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) // 2, int(x.shape[2]) // 2)),
                   name='d_data_sub2')(x_color)
        y = Conv2D(32, 3, strides=2, padding='same', activation='relu', name='d_conv1_1_3x3_s2')(y)
        y = BatchNormalization(name='d_conv1_1_3x3_s2_bn')(y)
        y = Conv2D(32, 3, padding='same', activation='relu', name='d_conv1_2_3x3')(y)
        y = BatchNormalization(name='d_conv1_2_3x3_s2_bn')(y)
        y = Conv2D(64, 3, padding='same', activation='relu', name='d_conv1_3_3x3')(y)
        y = BatchNormalization(name='d_conv1_3_3x3_bn')(y)
        y_ = MaxPooling2D(pool_size=3, strides=2, name='d_pool1_3x3_s2')(y)

        y = Conv2D(128, 1, name='d_conv2_1_1x1_proj')(y_)
        y = BatchNormalization(name='d_conv2_1_1x1_proj_bn')(y)
        y_ = Conv2D(32, 1, activation='relu', name='d_conv2_1_1x1_reduce')(y_)
        y_ = BatchNormalization(name='d_conv2_1_1x1_reduce_bn')(y_)
        y_ = ZeroPadding2D(name='d_padding1')(y_)
        y_ = Conv2D(32, 3, activation='relu', name='d_conv2_1_3x3')(y_)
        y_ = BatchNormalization(name='d_conv2_1_3x3_bn')(y_)
        y_ = Conv2D(128, 1, name='d_conv2_1_1x1_increase')(y_)
        y_ = BatchNormalization(name='d_conv2_1_1x1_increase_bn')(y_)
        y = Add(name='d_conv2_1')([y, y_])
        y_ = Activation('relu', name='d_conv2_1/relu')(y)

        y = Conv2D(32, 1, activation='relu', name='d_conv2_2_1x1_reduce')(y_)
        y = BatchNormalization(name='d_conv2_2_1x1_reduce_bn')(y)
        y = ZeroPadding2D(name='d_padding2')(y)
        y = Conv2D(32, 3, activation='relu', name='d_conv2_2_3x3')(y)
        y = BatchNormalization(name='d_conv2_2_3x3_bn')(y)
        y = Conv2D(128, 1, name='d_conv2_2_1x1_increase')(y)
        y = BatchNormalization(name='d_conv2_2_1x1_increase_bn')(y)
        y = Add(name='d_conv2_2')([y, y_])
        y_ = Activation('relu', name='d_conv2_2/relu')(y)

        y = Conv2D(32, 1, activation='relu', name='d_conv2_3_1x1_reduce')(y_)
        y = BatchNormalization(name='d_conv2_3_1x1_reduce_bn')(y)
        y = ZeroPadding2D(name='d_padding3')(y)
        y = Conv2D(32, 3, activation='relu', name='d_conv2_3_3x3')(y)
        y = BatchNormalization(name='d_conv2_3_3x3_bn')(y)
        y = Conv2D(128, 1, name='d_conv2_3_1x1_increase')(y)
        y = BatchNormalization(name='d_conv2_3_1x1_increase_bn')(y)
        y = Add(name='d_conv2_3')([y, y_])
        y_ = Activation('relu', name='d_conv2_3/relu')(y)

        y = Conv2D(256, 1, strides=2, name='d_conv3_1_1x1_proj')(y_)
        y = BatchNormalization(name='d_conv3_1_1x1_proj_bn')(y)
        y_ = Conv2D(64, 1, strides=2, activation='relu', name='d_conv3_1_1x1_reduce')(y_)
        y_ = BatchNormalization(name='d_conv3_1_1x1_reduce_bn')(y_)
        y_ = ZeroPadding2D(name='d_padding4')(y_)
        y_ = Conv2D(64, 3, activation='relu', name='d_conv3_1_3x3')(y_)
        y_ = BatchNormalization(name='d_conv3_1_3x3_bn')(y_)
        y_ = Conv2D(256, 1, name='d_conv3_1_1x1_increase')(y_)
        y_ = BatchNormalization(name='d_conv3_1_1x1_increase_bn')(y_)
        y = Add(name='d_conv3_1')([y, y_])
        z = Activation('relu', name='d_conv3_1/relu')(y)

        return z

    @staticmethod
    def build_one_quarter_depth(z):

        # (1/4)
        y_ = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) // 2, int(x.shape[2]) // 2)),
                    name='d_conv3_1_sub4')(z)
        y = Conv2D(64, 1, activation='relu', name='d_conv3_2_1x1_reduce')(y_)
        y = BatchNormalization(name='d_conv3_2_1x1_reduce_bn')(y)
        y = ZeroPadding2D(name='d_padding5')(y)
        y = Conv2D(64, 3, activation='relu', name='d_conv3_2_3x3')(y)
        y = BatchNormalization(name='d_conv3_2_3x3_bn')(y)
        y = Conv2D(256, 1, name='d_conv3_2_1x1_increase')(y)
        y = BatchNormalization(name='d_conv3_2_1x1_increase_bn')(y)
        y = Add(name='d_conv3_2')([y, y_])
        y_ = Activation('relu', name='d_conv3_2/relu')(y)

        y = Conv2D(64, 1, activation='relu', name='d_conv3_3_1x1_reduce')(y_)
        y = BatchNormalization(name='d_conv3_3_1x1_reduce_bn')(y)
        y = ZeroPadding2D(name='d_padding6')(y)
        y = Conv2D(64, 3, activation='relu', name='d_conv3_3_3x3')(y)
        y = BatchNormalization(name='d_conv3_3_3x3_bn')(y)
        y = Conv2D(256, 1, name='d_conv3_3_1x1_increase')(y)
        y = BatchNormalization(name='d_conv3_3_1x1_increase_bn')(y)
        y = Add(name='d_conv3_3')([y, y_])
        y_ = Activation('relu', name='d_conv3_3/relu')(y)

        y = Conv2D(64, 1, activation='relu', name='d_conv3_4_1x1_reduce')(y_)
        y = BatchNormalization(name='d_conv3_4_1x1_reduce_bn')(y)
        y = ZeroPadding2D(name='d_padding7')(y)
        y = Conv2D(64, 3, activation='relu', name='d_conv3_4_3x3')(y)
        y = BatchNormalization(name='d_conv3_4_3x3_bn')(y)
        y = Conv2D(256, 1, name='d_conv3_4_1x1_increase')(y)
        y = BatchNormalization(name='d_conv3_4_1x1_increase_bn')(y)
        y = Add(name='d_conv3_4')([y, y_])
        y_ = Activation('relu', name='d_conv3_4/relu')(y)

        y = Conv2D(512, 1, name='d_conv4_1_1x1_proj')(y_)
        y = BatchNormalization(name='d_conv4_1_1x1_proj_bn')(y)
        y_ = Conv2D(128, 1, activation='relu', name='d_conv4_1_1x1_reduce')(y_)
        y_ = BatchNormalization(name='d_conv4_1_1x1_reduce_bn')(y_)
        y_ = ZeroPadding2D(padding=2, name='d_padding8')(y_)
        y_ = Conv2D(128, 3, dilation_rate=2, activation='relu', name='d_conv4_1_3x3')(y_)
        y_ = BatchNormalization(name='d_conv4_1_3x3_bn')(y_)
        y_ = Conv2D(512, 1, name='d_conv4_1_1x1_increase')(y_)
        y_ = BatchNormalization(name='d_conv4_1_1x1_increase_bn')(y_)
        y = Add(name='d_conv4_1')([y, y_])
        y_ = Activation('relu', name='d_conv4_1/relu')(y)

        y = Conv2D(128, 1, activation='relu', name='d_conv4_2_1x1_reduce')(y_)
        y = BatchNormalization(name='d_conv4_2_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=2, name='d_padding9')(y)
        y = Conv2D(128, 3, dilation_rate=2, activation='relu', name='d_conv4_2_3x3')(y)
        y = BatchNormalization(name='d_conv4_2_3x3_bn')(y)
        y = Conv2D(512, 1, name='d_conv4_2_1x1_increase')(y)
        y = BatchNormalization(name='d_conv4_2_1x1_increase_bn')(y)
        y = Add(name='d_conv4_2')([y, y_])
        y_ = Activation('relu', name='d_conv4_2/relu')(y)

        y = Conv2D(128, 1, activation='relu', name='d_conv4_3_1x1_reduce')(y_)
        y = BatchNormalization(name='d_conv4_3_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=2, name='d_padding10')(y)
        y = Conv2D(128, 3, dilation_rate=2, activation='relu', name='d_conv4_3_3x3')(y)
        y = BatchNormalization(name='d_conv4_3_3x3_bn')(y)
        y = Conv2D(512, 1, name='d_conv4_3_1x1_increase')(y)
        y = BatchNormalization(name='d_conv4_3_1x1_increase_bn')(y)
        y = Add(name='d_conv4_3')([y, y_])
        y_ = Activation('relu', name='d_conv4_3/relu')(y)

        y = Conv2D(128, 1, activation='relu', name='d_conv4_4_1x1_reduce')(y_)
        y = BatchNormalization(name='d_conv4_4_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=2, name='d_padding11')(y)
        y = Conv2D(128, 3, dilation_rate=2, activation='relu', name='d_conv4_4_3x3')(y)
        y = BatchNormalization(name='d_conv4_4_3x3_bn')(y)
        y = Conv2D(512, 1, name='d_conv4_4_1x1_increase')(y)
        y = BatchNormalization(name='d_conv4_4_1x1_increase_bn')(y)
        y = Add(name='d_conv4_4')([y, y_])
        y_ = Activation('relu', name='d_conv4_4/relu')(y)

        y = Conv2D(128, 1, activation='relu', name='d_conv4_5_1x1_reduce')(y_)
        y = BatchNormalization(name='d_conv4_5_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=2, name='d_padding12')(y)
        y = Conv2D(128, 3, dilation_rate=2, activation='relu', name='d_conv4_5_3x3')(y)
        y = BatchNormalization(name='d_conv4_5_3x3_bn')(y)
        y = Conv2D(512, 1, name='d_conv4_5_1x1_increase')(y)
        y = BatchNormalization(name='d_conv4_5_1x1_increase_bn')(y)
        y = Add(name='d_conv4_5')([y, y_])
        y_ = Activation('relu', name='d_conv4_5/relu')(y)

        y = Conv2D(128, 1, activation='relu', name='d_conv4_6_1x1_reduce')(y_)
        y = BatchNormalization(name='d_conv4_6_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=2, name='d_padding13')(y)
        y = Conv2D(128, 3, dilation_rate=2, activation='relu', name='d_conv4_6_3x3')(y)
        y = BatchNormalization(name='d_conv4_6_3x3_bn')(y)
        y = Conv2D(512, 1, name='d_conv4_6_1x1_increase')(y)
        y = BatchNormalization(name='d_conv4_6_1x1_increase_bn')(y)
        y = Add(name='d_conv4_6')([y, y_])
        y = Activation('relu', name='d_conv4_6/relu')(y)

        y_ = Conv2D(1024, 1, name='d_conv5_1_1x1_proj')(y)
        y_ = BatchNormalization(name='d_conv5_1_1x1_proj_bn')(y_)
        y = Conv2D(256, 1, activation='relu', name='d_conv5_1_1x1_reduce')(y)
        y = BatchNormalization(name='d_conv5_1_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=4, name='d_padding14')(y)
        y = Conv2D(256, 3, dilation_rate=4, activation='relu', name='d_conv5_1_3x3')(y)
        y = BatchNormalization(name='d_conv5_1_3x3_bn')(y)
        y = Conv2D(1024, 1, name='d_conv5_1_1x1_increase')(y)
        y = BatchNormalization(name='d_conv5_1_1x1_increase_bn')(y)
        y = Add(name='d_conv5_1')([y, y_])
        y_ = Activation('relu', name='d_conv5_1/relu')(y)

        y = Conv2D(256, 1, activation='relu', name='d_conv5_2_1x1_reduce')(y_)
        y = BatchNormalization(name='d_conv5_2_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=4, name='d_padding15')(y)
        y = Conv2D(256, 3, dilation_rate=4, activation='relu', name='d_conv5_2_3x3')(y)
        y = BatchNormalization(name='d_conv5_2_3x3_bn')(y)
        y = Conv2D(1024, 1, name='d_conv5_2_1x1_increase')(y)
        y = BatchNormalization(name='d_conv5_2_1x1_increase_bn')(y)
        y = Add(name='d_conv5_2')([y, y_])
        y_ = Activation('relu', name='d_conv5_2/relu')(y)

        y = Conv2D(256, 1, activation='relu', name='d_conv5_3_1x1_reduce')(y_)
        y = BatchNormalization(name='d_conv5_3_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=4, name='d_padding16')(y)
        y = Conv2D(256, 3, dilation_rate=4, activation='relu', name='d_conv5_3_3x3')(y)
        y = BatchNormalization(name='d_conv5_3_3x3_bn')(y)
        y = Conv2D(1024, 1, name='d_conv5_3_1x1_increase')(y)
        y = BatchNormalization(name='d_conv5_3_1x1_increase_bn')(y)
        y = Add(name='d_conv5_3')([y, y_])
        y = Activation('relu', name='d_conv5_3/relu')(y)

        return y

    @staticmethod
    def build_one_depth(x_color):

        # (1)
        y = Conv2D(32, 3, strides=2, padding='same', activation='relu', name='d_conv1_sub1')(x_color)
        y = BatchNormalization(name='d_conv1_sub1_bn')(y)
        y = Conv2D(32, 3, strides=2, padding='same', activation='relu', name='d_conv2_sub1')(y)
        y = BatchNormalization(name='d_conv2_sub1_bn')(y)
        y = Conv2D(64, 3, strides=2, padding='same', activation='relu', name='d_conv3_sub1')(y)
        y = BatchNormalization(name='d_conv3_sub1_bn')(y)
        y = Conv2D(128, 1, name='d_conv3_sub1_proj')(y)
        y = BatchNormalization(name='d_conv3_sub1_proj_bn')(y)

        return y

    # ==================================================================================================================

    @staticmethod
    def build_fusion_one(x_color, x_depth):

        # (1)
        y = Conv2D(32, 3, strides=2, padding='same', activation='relu', name='conv1_sub1')(x_color)
        y = BatchNormalization(name='conv1_sub1_bn')(y)
        y = Conv2D(32, 3, strides=2, padding='same', activation='relu', name='conv2_sub1')(y)
        y = BatchNormalization(name='conv2_sub1_bn')(y)
        y = Conv2D(64, 3, strides=2, padding='same', activation='relu', name='conv3_sub1')(y)
        y = BatchNormalization(name='conv3_sub1_bn')(y)
        y = Conv2D(128, 1, name='conv3_sub1_proj')(y)
        y = BatchNormalization(name='conv3_sub1_proj_bn')(y)

        # (1)
        y_d = Conv2D(32, 3, strides=2, padding='same', activation='relu', name='d_conv1_sub1')(x_depth)
        y_d = BatchNormalization(name='d_conv1_sub1_bn')(y_d)
        y_d = Conv2D(32, 3, strides=2, padding='same', activation='relu', name='d_conv2_sub1')(y_d)
        y_d = BatchNormalization(name='d_conv2_sub1_bn')(y_d)
        y_d = Conv2D(64, 3, strides=2, padding='same', activation='relu', name='d_conv3_sub1')(y_d)
        y_d = BatchNormalization(name='d_conv3_sub1_bn')(y_d)
        y_d = Conv2D(128, 1, name='d_conv3_sub1_proj')(y_d)
        y_d = BatchNormalization(name='d_conv3_sub1_proj_bn')(y_d)

        output = Add(name="one_merge_color")([y, y_d])

        return output

    @staticmethod
    def build_fusion_one_quarter(z, z_d):

        # (1/4)
        # color
        y_ = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) // 2, int(x.shape[2]) // 2)),
                    name='conv3_1_sub4_color')(z)
        y = Conv2D(64, 1, activation='relu', name='conv3_2_1x1_reduce_color')(y_)
        y = BatchNormalization(name='conv3_2_1x1_reduce_bn_color')(y)
        y = ZeroPadding2D(name='padding5_color')(y)
        y = Conv2D(64, 3, activation='relu', name='conv3_2_3x3_color')(y)
        y = BatchNormalization(name='conv3_2_3x3_bn_color')(y)
        y = Conv2D(256, 1, name='conv3_2_1x1_increase_color')(y)
        y = BatchNormalization(name='conv3_2_1x1_increase_bn_color')(y)
        y = Add(name='conv3_2_color')([y, y_])
        y_ = Activation('relu', name='conv3_2/relu_color')(y)

        # depth
        y_d_ = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) // 2, int(x.shape[2]) // 2)),
                    name='conv3_1_sub4_depth')(z_d)
        y_d = Conv2D(64, 1, activation='relu', name='conv3_2_1x1_reduce')(y_d_)
        y_d = BatchNormalization(name='conv3_2_1x1_reduce_bn')(y_d)
        y_d = ZeroPadding2D(name='padding5')(y_d)
        y_d = Conv2D(64, 3, activation='relu', name='conv3_2_3x3')(y_d)
        y_d = BatchNormalization(name='conv3_2_3x3_bn')(y_d)
        y_d = Conv2D(256, 1, name='conv3_2_1x1_increase')(y_d)
        y_d = BatchNormalization(name='conv3_2_1x1_increase_bn')(y_d)
        y_d = Add(name='conv3_2_depth')([y_d, y_d_])
        y_d_ = Activation('relu', name='conv3_2/relu_depth')(y_d)

        # part 1 color
        y = Conv2D(64, 1, activation='relu', name='conv3_3_1x1_reduce')(y_)
        y = BatchNormalization(name='conv3_3_1x1_reduce_bn')(y)
        y = ZeroPadding2D(name='padding6')(y)
        y = Conv2D(64, 3, activation='relu', name='conv3_3_3x3')(y)
        y = BatchNormalization(name='conv3_3_3x3_bn')(y)
        y = Conv2D(256, 1, name='conv3_3_1x1_increase')(y)
        y = BatchNormalization(name='conv3_3_1x1_increase_bn')(y)

        # part 1 depth
        y_d = Conv2D(64, 1, activation='relu', name='dconv3_3_1x1_reduce')(y_d_)
        y_d = BatchNormalization(name='dconv3_3_1x1_reduce_bn')(y_d)
        y_d = ZeroPadding2D(name='dpadding6')(y_d)
        y_d = Conv2D(64, 3, activation='relu', name='dconv3_3_3x3')(y_d)
        y_d = BatchNormalization(name='dconv3_3_3x3_bn')(y_d)
        y_d = Conv2D(256, 1, name='dconv3_3_1x1_increase')(y_d)
        y_d = BatchNormalization(name='dconv3_3_1x1_increase_bn')(y_d)

        # merging
        conv3_3_color = Add(name='conv3_3_color')([y, y_])
        conv3_3_depth = Add(name='conv3_3_depth')([y_d, y_d_])

        y = Add(name="conv3_3_merge_color")([conv3_3_color, conv3_3_depth])
        y_d = Add(name="conv3_3_merge_depth")([conv3_3_depth, conv3_3_color])

        y_ = Activation('relu', name='conv3_3/relu_color')(y)
        y_d_ = Activation('relu', name='conv3_3/relu_depth')(y_d)

        # -----------------------------------------------------------------
        # part 2 color
        y = Conv2D(64, 1, activation='relu', name='conv3_4_1x1_reduce')(y_)
        y = BatchNormalization(name='conv3_4_1x1_reduce_bn')(y)
        y = ZeroPadding2D(name='padding7')(y)
        y = Conv2D(64, 3, activation='relu', name='conv3_4_3x3')(y)
        y = BatchNormalization(name='conv3_4_3x3_bn')(y)
        y = Conv2D(256, 1, name='conv3_4_1x1_increase')(y)
        y = BatchNormalization(name='conv3_4_1x1_increase_bn')(y)
        y = Add(name='conv3_4_color')([y, y_])
        y_ = Activation('relu', name='conv3_4/relu_color')(y)

        y_d = Conv2D(64, 1, activation='relu', name='dconv3_4_1x1_reduce')(y_d_)
        y_d = BatchNormalization(name='dconv3_4_1x1_reduce_bn')(y_d)
        y_d = ZeroPadding2D(name='dpadding7')(y_d)
        y_d = Conv2D(64, 3, activation='relu', name='dconv3_4_3x3')(y_d)
        y_d = BatchNormalization(name='dconv3_4_3x3_bn')(y_d)
        y_d = Conv2D(256, 1, name='dconv3_4_1x1_increase')(y_d)
        y_d = BatchNormalization(name='dconv3_4_1x1_increase_bn')(y_d)
        y_d = Add(name='conv3_4_depth')([y_d, y_d_])
        y_d_ = Activation('relu', name='conv3_4/relu_depth')(y_d)

        # -----------------------------------------------------------------
        # part 3 color
        y = Conv2D(512, 1, name='conv4_1_1x1_proj')(y_)
        y = BatchNormalization(name='conv4_1_1x1_proj_bn')(y)
        y_ = Conv2D(128, 1, activation='relu', name='conv4_1_1x1_reduce')(y_)
        y_ = BatchNormalization(name='conv4_1_1x1_reduce_bn')(y_)
        y_ = ZeroPadding2D(padding=2, name='padding8')(y_)
        y_ = Conv2D(128, 3, dilation_rate=2, activation='relu', name='conv4_1_3x3')(y_)
        y_ = BatchNormalization(name='conv4_1_3x3_bn')(y_)
        y_ = Conv2D(512, 1, name='conv4_1_1x1_increase')(y_)
        y_ = BatchNormalization(name='conv4_1_1x1_increase_bn')(y_)

        # part 3 depth
        y_d = Conv2D(512, 1, name='dconv4_1_1x1_proj')(y_d_)
        y_d = BatchNormalization(name='dconv4_1_1x1_proj_bn')(y_d)
        y_d_ = Conv2D(128, 1, activation='relu', name='dconv4_1_1x1_reduce')(y_d_)
        y_d_ = BatchNormalization(name='dconv4_1_1x1_reduce_bn')(y_d_)
        y_d_ = ZeroPadding2D(padding=2, name='dpadding8')(y_d_)
        y_d_ = Conv2D(128, 3, dilation_rate=2, activation='relu', name='dconv4_1_3x3')(y_d_)
        y_d_ = BatchNormalization(name='dconv4_1_3x3_bn')(y_d_)
        y_d_ = Conv2D(512, 1, name='dconv4_1_1x1_increase')(y_d_)
        y_d_ = BatchNormalization(name='dconv4_1_1x1_increase_bn')(y_d_)

        conv4_1_color = Add(name='conv4_1_color')([y_d, y_d_])
        conv4_1_depth = Add(name='conv4_1_depth')([y, y_])

        y = Add(name="conv4_1_merge_color")([conv4_1_color, conv4_1_depth])
        y_d = Add(name="conv4_1_merge_depth")([conv4_1_depth, conv4_1_color])

        y_ = Activation('relu', name='conv4_1/relu_color')(y)
        y_d_ = Activation('relu', name='conv4_1/relu_depth')(y_d)

        # -----------------------------------------------------------------
        # part 4 color
        y = Conv2D(128, 1, activation='relu', name='conv4_2_1x1_reduce')(y_)
        y = BatchNormalization(name='conv4_2_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=2, name='padding9')(y)
        y = Conv2D(128, 3, dilation_rate=2, activation='relu', name='conv4_2_3x3')(y)
        y = BatchNormalization(name='conv4_2_3x3_bn')(y)
        y = Conv2D(512, 1, name='conv4_2_1x1_increase')(y)
        y = BatchNormalization(name='conv4_2_1x1_increase_bn')(y)
        conv4_2_color = Add(name='conv4_2_color')([y, y_])
        y_ = Activation('relu', name='conv4_2/relu_color')(conv4_2_color)

        # part 4 depth
        y_d = Conv2D(128, 1, activation='relu', name='dconv4_2_1x1_reduce')(y_d_)
        y_d = BatchNormalization(name='dconv4_2_1x1_reduce_bn')(y_d)
        y_d = ZeroPadding2D(padding=2, name='dpadding9')(y_d)
        y_d = Conv2D(128, 3, dilation_rate=2, activation='relu', name='dconv4_2_3x3')(y_d)
        y_d = BatchNormalization(name='dconv4_2_3x3_bn')(y_d)
        y_d = Conv2D(512, 1, name='dconv4_2_1x1_increase')(y_d)
        y_d = BatchNormalization(name='dconv4_2_1x1_increase_bn')(y_d)
        conv4_2_depth = Add(name='dconv4_2_depth')([y_d, y_d_])
        y_d_ = Activation('relu', name='dconv4_2/relu_depth')(conv4_2_depth)

        # -----------------------------------------------------------------
        # part 5 color
        y = Conv2D(128, 1, activation='relu', name='conv4_3_1x1_reduce')(y_)
        y = BatchNormalization(name='conv4_3_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=2, name='padding10')(y)
        y = Conv2D(128, 3, dilation_rate=2, activation='relu', name='conv4_3_3x3')(y)
        y = BatchNormalization(name='conv4_3_3x3_bn')(y)
        y = Conv2D(512, 1, name='conv4_3_1x1_increase')(y)
        y = BatchNormalization(name='conv4_3_1x1_increase_bn')(y)

        y_d = Conv2D(128, 1, activation='relu', name='dconv4_3_1x1_reduce')(y_d_)
        y_d = BatchNormalization(name='dconv4_3_1x1_reduce_bn')(y_d)
        y_d = ZeroPadding2D(padding=2, name='dpadding10')(y_d)
        y_d = Conv2D(128, 3, dilation_rate=2, activation='relu', name='dconv4_3_3x3')(y_d)
        y_d = BatchNormalization(name='dconv4_3_3x3_bn')(y_d)
        y_d = Conv2D(512, 1, name='dconv4_3_1x1_increase')(y_d)
        y_d = BatchNormalization(name='dconv4_3_1x1_increase_bn')(y_d)

        conv4_3_color = Add(name='conv4_3_color')([y, y_])
        conv4_3_depth = Add(name='conv4_3_depth')([y_d, y_d_])

        y = Add(name="conv4_3_merge_color")([conv4_3_color, conv4_3_depth])
        y_d = Add(name="conv4_3_merge_depth")([conv4_3_depth, conv4_3_color])

        y_ = Activation('relu', name='conv4_3/relu_color')(y)
        y_d_ = Activation('relu', name='conv4_3/relu_depth')(y_d)

        # -----------------------------------------------------------------
        # part 6 color
        y = Conv2D(128, 1, activation='relu', name='conv4_4_1x1_reduce')(y_)
        y = BatchNormalization(name='conv4_4_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=2, name='padding11')(y)
        y = Conv2D(128, 3, dilation_rate=2, activation='relu', name='conv4_4_3x3')(y)
        y = BatchNormalization(name='conv4_4_3x3_bn')(y)
        y = Conv2D(512, 1, name='conv4_4_1x1_increase')(y)
        y = BatchNormalization(name='conv4_4_1x1_increase_bn')(y)
        y = Add(name='conv4_4_color')([y, y_])
        y_ = Activation('relu', name='conv4_4/relu_color')(y)

        y_d = Conv2D(128, 1, activation='relu', name='dconv4_4_1x1_reduce')(y_d_)
        y_d = BatchNormalization(name='dconv4_4_1x1_reduce_bn')(y_d)
        y_d = ZeroPadding2D(padding=2, name='dpadding11')(y_d)
        y_d = Conv2D(128, 3, dilation_rate=2, activation='relu', name='dconv4_4_3x3')(y_d)
        y_d = BatchNormalization(name='dconv4_4_3x3_bn')(y_d)
        y_d = Conv2D(512, 1, name='dconv4_4_1x1_increase')(y_d)
        y_d = BatchNormalization(name='dconv4_4_1x1_increase_bn')(y_d)
        y_d = Add(name='conv4_4_depth')([y_d, y_d_])
        y_d_ = Activation('relu', name='conv4_4/relu_depth')(y_d)

        # -----------------------------------------------------------------
        # part 7 color
        y = Conv2D(128, 1, activation='relu', name='conv4_5_1x1_reduce')(y_)
        y = BatchNormalization(name='conv4_5_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=2, name='padding12')(y)
        y = Conv2D(128, 3, dilation_rate=2, activation='relu', name='conv4_5_3x3')(y)
        y = BatchNormalization(name='conv4_5_3x3_bn')(y)
        y = Conv2D(512, 1, name='conv4_5_1x1_increase')(y)
        y = BatchNormalization(name='conv4_5_1x1_increase_bn')(y)

        y_d = Conv2D(128, 1, activation='relu', name='dconv4_5_1x1_reduce')(y_d_)
        y_d = BatchNormalization(name='dconv4_5_1x1_reduce_bn')(y_d)
        y_d = ZeroPadding2D(padding=2, name='dpadding12')(y_d)
        y_d = Conv2D(128, 3, dilation_rate=2, activation='relu', name='dconv4_5_3x3')(y_d)
        y_d = BatchNormalization(name='dconv4_5_3x3_bn')(y_d)
        y_d = Conv2D(512, 1, name='dconv4_5_1x1_increase')(y_d)
        y_d = BatchNormalization(name='dconv4_5_1x1_increase_bn')(y_d)

        conv4_5_color = Add(name='conv4_5_color')([y, y_])
        conv4_5_depth = Add(name='conv4_5_depth')([y_d, y_d_])

        y = Add(name="conv4_5_merge_color")([conv4_5_color, conv4_5_depth])
        y_d = Add(name="conv4_5_merge_depth")([conv4_5_depth, conv4_5_color])

        y_ = Activation('relu', name='conv4_5/relu_color')(y)
        y_d_ = Activation('relu', name='conv4_5/relu_depth')(y_d)

        # ------------------------------------------------------------------
        # part 8 color
        y = Conv2D(128, 1, activation='relu', name='conv4_6_1x1_reduce')(y_)
        y = BatchNormalization(name='conv4_6_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=2, name='padding13')(y)
        y = Conv2D(128, 3, dilation_rate=2, activation='relu', name='conv4_6_3x3')(y)
        y = BatchNormalization(name='conv4_6_3x3_bn')(y)
        y = Conv2D(512, 1, name='conv4_6_1x1_increase')(y)
        y = BatchNormalization(name='conv4_6_1x1_increase_bn')(y)
        y = Add(name='conv4_6_color')([y, y_])
        y = Activation('relu', name='conv4_6/relu_color')(y)

        y_d = Conv2D(128, 1, activation='relu', name='dconv4_6_1x1_reduce')(y_d_)
        y_d = BatchNormalization(name='dconv4_6_1x1_reduce_bn')(y_d)
        y_d = ZeroPadding2D(padding=2, name='dpadding13')(y_d)
        y_d = Conv2D(128, 3, dilation_rate=2, activation='relu', name='dconv4_6_3x3')(y_d)
        y_d = BatchNormalization(name='dconv4_6_3x3_bn')(y_d)
        y_d = Conv2D(512, 1, name='dconv4_6_1x1_increase')(y_d)
        y_d = BatchNormalization(name='dconv4_6_1x1_increase_bn')(y_d)
        y_d = Add(name='dconv4_6_depth')([y_d, y_d_])
        y_d = Activation('relu', name='dconv4_6/relu_depth')(y_d)

        # -------------------------------------------------------------------
        # part 9 color
        y_ = Conv2D(1024, 1, name='conv5_1_1x1_proj')(y)
        y_ = BatchNormalization(name='conv5_1_1x1_proj_bn')(y_)
        y = Conv2D(256, 1, activation='relu', name='conv5_1_1x1_reduce')(y)
        y = BatchNormalization(name='conv5_1_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=4, name='padding14')(y)
        y = Conv2D(256, 3, dilation_rate=4, activation='relu', name='conv5_1_3x3')(y)
        y = BatchNormalization(name='conv5_1_3x3_bn')(y)
        y = Conv2D(1024, 1, name='conv5_1_1x1_increase')(y)
        y = BatchNormalization(name='conv5_1_1x1_increase_bn')(y)

        y_d_ = Conv2D(1024, 1, name='dconv5_1_1x1_proj')(y_d)
        y_d_ = BatchNormalization(name='dconv5_1_1x1_proj_bn')(y_d_)
        y_d = Conv2D(256, 1, activation='relu', name='dconv5_1_1x1_reduce')(y_d)
        y_d = BatchNormalization(name='dconv5_1_1x1_reduce_bn')(y_d)
        y_d = ZeroPadding2D(padding=4, name='dpadding14')(y_d)
        y_d = Conv2D(256, 3, dilation_rate=4, activation='relu', name='dconv5_1_3x3')(y_d)
        y_d = BatchNormalization(name='dconv5_1_3x3_bn')(y_d)
        y_d = Conv2D(1024, 1, name='dconv5_1_1x1_increase')(y_d)
        y_d = BatchNormalization(name='dconv5_1_1x1_increase_bn')(y_d)

        conv5_1_color = Add(name='conv5_1_color')([y, y_])
        conv5_1_depth = Add(name='conv5_1_depth')([y_d, y_d_])

        y = Add(name="conv5_1_merge_color")([conv5_1_color, conv5_1_depth])
        y_d = Add(name="conv5_1_merge_depth")([conv5_1_depth, conv5_1_color])

        y_ = Activation('relu', name='conv5_1/relu_color')(y)
        y_d_ = Activation('relu', name='conv5_1/relu_depth')(y_d)

        # -------------------------------------------------------------------
        # part 10 color
        y = Conv2D(256, 1, activation='relu', name='conv5_2_1x1_reduce')(y_)
        y = BatchNormalization(name='conv5_2_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=4, name='padding15')(y)
        y = Conv2D(256, 3, dilation_rate=4, activation='relu', name='conv5_2_3x3')(y)
        y = BatchNormalization(name='conv5_2_3x3_bn')(y)
        y = Conv2D(1024, 1, name='conv5_2_1x1_increase')(y)
        y = BatchNormalization(name='conv5_2_1x1_increase_bn')(y)
        y = Add(name='conv5_2_color')([y, y_])
        y_ = Activation('relu', name='conv5_2/relu_color')(y)

        y_d = Conv2D(256, 1, activation='relu', name='dconv5_2_1x1_reduce')(y_d_)
        y_d = BatchNormalization(name='dconv5_2_1x1_reduce_bn')(y_d)
        y_d = ZeroPadding2D(padding=4, name='dpadding15')(y_d)
        y_d = Conv2D(256, 3, dilation_rate=4, activation='relu', name='dconv5_2_3x3')(y_d)
        y_d = BatchNormalization(name='dconv5_2_3x3_bn')(y_d)
        y_d = Conv2D(1024, 1, name='dconv5_2_1x1_increase')(y_d)
        y_d = BatchNormalization(name='dconv5_2_1x1_increase_bn')(y_d)
        y_d = Add(name='conv5_2_depth')([y_d, y_d_])
        y_d_ = Activation('relu', name='conv5_2/relu_depth')(y_d)

        # --------------------------------------------------------------------
        # part 11 color
        y = Conv2D(256, 1, activation='relu', name='conv5_3_1x1_reduce')(y_)
        y = BatchNormalization(name='conv5_3_1x1_reduce_bn')(y)
        y = ZeroPadding2D(padding=4, name='padding16')(y)
        y = Conv2D(256, 3, dilation_rate=4, activation='relu', name='conv5_3_3x3')(y)
        y = BatchNormalization(name='conv5_3_3x3_bn')(y)
        y = Conv2D(1024, 1, name='conv5_3_1x1_increase')(y)
        y = BatchNormalization(name='conv5_3_1x1_increase_bn')(y)
        y = Add(name='conv5_3_color')([y, y_])
        y = Activation('relu', name='conv5_3/relu_color')(y)

        y_d = Conv2D(256, 1, activation='relu', name='dconv5_3_1x1_reduce')(y_d_)
        y_d = BatchNormalization(name='dconv5_3_1x1_reduce_bn')(y_d)
        y_d = ZeroPadding2D(padding=4, name='dpadding16')(y_d)
        y_d = Conv2D(256, 3, dilation_rate=4, activation='relu', name='dconv5_3_3x3')(y_d)
        y_d = BatchNormalization(name='dconv5_3_3x3_bn')(y_d)
        y_d = Conv2D(1024, 1, name='dconv5_3_1x1_increase')(y_d)
        y_d = BatchNormalization(name='dconv5_3_1x1_increase_bn')(y_d)
        y_d = Add(name='conv5_3_depth')([y_d, y_d_])
        y_d = Activation('relu', name='conv5_3/relu_depth')(y_d)

        return y, y_d

    @staticmethod
    def build_fusion_one_half(x_color, x_depth):

        # (1/2)
        y = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) // 2, int(x.shape[2]) // 2)),
                   name='data_sub2')(x_color)
        y = Conv2D(32, 3, strides=2, padding='same', activation='relu', name='conv1_1_3x3_s2')(y)
        y = BatchNormalization(name='conv1_1_3x3_s2_bn')(y)
        y = Conv2D(32, 3, padding='same', activation='relu', name='conv1_2_3x3')(y)
        y = BatchNormalization(name='conv1_2_3x3_s2_bn')(y)
        y = Conv2D(64, 3, padding='same', activation='relu', name='conv1_3_3x3')(y)
        y = BatchNormalization(name='conv1_3_3x3_bn')(y)
        y_ = MaxPooling2D(pool_size=3, strides=2, name='pool1_3x3_s2')(y)

        # (1/2)
        y_d = Lambda(lambda x: tf.image.resize_bilinear(x, size=(int(x.shape[1]) // 2, int(x.shape[2]) // 2)),
                     name='d_data_sub2')(x_depth)
        y_d = Conv2D(32, 3, strides=2, padding='same', activation='relu', name='d_conv1_1_3x3_s2')(y_d)
        y_d = BatchNormalization(name='d_conv1_1_3x3_s2_bn')(y_d)
        y_d = Conv2D(32, 3, padding='same', activation='relu', name='d_conv1_2_3x3')(y_d)
        y_d = BatchNormalization(name='d_conv1_2_3x3_s2_bn')(y_d)
        y_d = Conv2D(64, 3, padding='same', activation='relu', name='d_conv1_3_3x3')(y_d)
        y_d = BatchNormalization(name='d_conv1_3_3x3_bn')(y_d)
        y_d_ = MaxPooling2D(pool_size=3, strides=2, name='d_pool1_3x3_s2')(y_d)

        # ------------------------------------------------------------------------
        # part 2 depth
        y_d = Conv2D(128, 1, name='d_conv2_1_1x1_proj')(y_d_)
        y_d = BatchNormalization(name='d_conv2_1_1x1_proj_bn')(y_d)
        y_d_ = Conv2D(32, 1, activation='relu', name='d_conv2_1_1x1_reduce')(y_d_)
        y_d_ = BatchNormalization(name='d_conv2_1_1x1_reduce_bn')(y_d_)
        y_d_ = ZeroPadding2D(name='d_padding1')(y_d_)
        y_d_ = Conv2D(32, 3, activation='relu', name='d_conv2_1_3x3')(y_d_)
        y_d_ = BatchNormalization(name='d_conv2_1_3x3_bn')(y_d_)
        y_d_ = Conv2D(128, 1, name='d_conv2_1_1x1_increase')(y_d_)
        y_d_ = BatchNormalization(name='d_conv2_1_1x1_increase_bn')(y_d_)

        # part 2 color
        y = Conv2D(128, 1, name='conv2_1_1x1_proj')(y_)
        y = BatchNormalization(name='conv2_1_1x1_proj_bn')(y)
        y_ = Conv2D(32, 1, activation='relu', name='conv2_1_1x1_reduce')(y_)
        y_ = BatchNormalization(name='conv2_1_1x1_reduce_bn')(y_)
        y_ = ZeroPadding2D(name='padding1')(y_)
        y_ = Conv2D(32, 3, activation='relu', name='conv2_1_3x3')(y_)
        y_ = BatchNormalization(name='conv2_1_3x3_bn')(y_)
        y_ = Conv2D(128, 1, name='conv2_1_1x1_increase')(y_)
        y_ = BatchNormalization(name='conv2_1_1x1_increase_bn')(y_)

        # part 2 merging
        conv2_1 = Add(name='conv2_1')([y, y_])
        d_conv2_1 = Add(name='d_conv2_1')([y_d, y_d_])

        y = Add(name='conv2_1_merge_color')([conv2_1, d_conv2_1])
        y_d_ = Add(name='conv2_1_merge_depth')([d_conv2_1, conv2_1])

        y_ = Activation('relu', name='conv2_1/relu')(y)
        y_d_ = Activation('relu', name='d_conv2_1/relu')(y_d_)

        # -----------------------------------------------------------------
        # part 3 depth
        y_d = Conv2D(32, 1, activation='relu', name='d_conv2_2_1x1_reduce')(y_d_)
        y_d = BatchNormalization(name='d_conv2_2_1x1_reduce_bn')(y_d)
        y_d = ZeroPadding2D(name='d_padding2')(y_d)
        y_d = Conv2D(32, 3, activation='relu', name='d_conv2_2_3x3')(y_d)
        y_d = BatchNormalization(name='d_conv2_2_3x3_bn')(y_d)
        y_d = Conv2D(128, 1, name='d_conv2_2_1x1_increase')(y_d)
        y_d = BatchNormalization(name='d_conv2_2_1x1_increase_bn')(y_d)

        # part 3 color
        y = Conv2D(32, 1, activation='relu', name='conv2_2_1x1_reduce')(y_)
        y = BatchNormalization(name='conv2_2_1x1_reduce_bn')(y)
        y = ZeroPadding2D(name='padding2')(y)
        y = Conv2D(32, 3, activation='relu', name='conv2_2_3x3')(y)
        y = BatchNormalization(name='conv2_2_3x3_bn')(y)
        y = Conv2D(128, 1, name='conv2_2_1x1_increase')(y)
        y = BatchNormalization(name='conv2_2_1x1_increase_bn')(y)

        # part 3 merging
        d_conv2_2 = Add(name='d_conv2_2')([y_d, y_d_])
        conv2_2 = Add(name='conv2_2')([y, y_])

        y = Add(name='conv2_2_merge_color')([conv2_2, d_conv2_2])
        y_d_ = Add(name='conv2_2_merge_depth')([d_conv2_2, conv2_2])

        y_d_ = Activation('relu', name='d_conv2_2/relu')(y_d_)
        y_ = Activation('relu', name='conv2_2/relu')(y)

        # -----------------------------------------------------------------------
        # part 4 depth
        y_d = Conv2D(32, 1, activation='relu', name='d_conv2_3_1x1_reduce')(y_d_)
        y_d = BatchNormalization(name='d_conv2_3_1x1_reduce_bn')(y_d)
        y_d = ZeroPadding2D(name='d_padding3')(y_d)
        y_d = Conv2D(32, 3, activation='relu', name='d_conv2_3_3x3')(y_d)
        y_d = BatchNormalization(name='d_conv2_3_3x3_bn')(y_d)
        y_d = Conv2D(128, 1, name='d_conv2_3_1x1_increase')(y_d)
        y_d = BatchNormalization(name='d_conv2_3_1x1_increase_bn')(y_d)

        # part 4 color
        y = Conv2D(32, 1, activation='relu', name='conv2_3_1x1_reduce')(y_)
        y = BatchNormalization(name='conv2_3_1x1_reduce_bn')(y)
        y = ZeroPadding2D(name='padding3')(y)
        y = Conv2D(32, 3, activation='relu', name='conv2_3_3x3')(y)
        y = BatchNormalization(name='conv2_3_3x3_bn')(y)
        y = Conv2D(128, 1, name='conv2_3_1x1_increase')(y)
        y = BatchNormalization(name='conv2_3_1x1_increase_bn')(y)

        d_conv2_3 = Add(name='d_conv2_3')([y_d, y_d_])
        conv2_3 = Add(name='conv2_3')([y, y_])

        y = Add(name='conv2_3_merge_color')([conv2_3, d_conv2_3])
        y_d_ = Add(name='conv2_3_merge_depth')([d_conv2_3, conv2_3])

        y_d_ = Activation('relu', name='d_conv2_3/relu')(y_d_)
        y_ = Activation('relu', name='conv2_3/relu')(y)

        # -----------------------------------------------------------------------------
        # part 5
        y = Conv2D(256, 1, strides=2, name='conv3_1_1x1_proj')(y_)
        y = BatchNormalization(name='conv3_1_1x1_proj_bn')(y)
        y_ = Conv2D(64, 1, strides=2, activation='relu', name='conv3_1_1x1_reduce')(y_)
        y_ = BatchNormalization(name='conv3_1_1x1_reduce_bn')(y_)
        y_ = ZeroPadding2D(name='padding4')(y_)
        y_ = Conv2D(64, 3, activation='relu', name='conv3_1_3x3')(y_)
        y_ = BatchNormalization(name='conv3_1_3x3_bn')(y_)
        y_ = Conv2D(256, 1, name='conv3_1_1x1_increase')(y_)
        y_ = BatchNormalization(name='conv3_1_1x1_increase_bn')(y_)
        y = Add(name='conv3_1')([y, y_])
        z = Activation('relu', name='conv3_1/relu')(y)

        y_d = Conv2D(256, 1, strides=2, name='d_conv3_1_1x1_proj')(y_d_)
        y_d = BatchNormalization(name='d_conv3_1_1x1_proj_bn')(y_d)
        y_d_ = Conv2D(64, 1, strides=2, activation='relu', name='d_conv3_1_1x1_reduce')(y_d_)
        y_d_ = BatchNormalization(name='d_conv3_1_1x1_reduce_bn')(y_d_)
        y_d_ = ZeroPadding2D(name='d_padding4')(y_d_)
        y_d_ = Conv2D(64, 3, activation='relu', name='d_conv3_1_3x3')(y_d_)
        y_d_ = BatchNormalization(name='d_conv3_1_3x3_bn')(y_d_)
        y_d_ = Conv2D(256, 1, name='d_conv3_1_1x1_increase')(y_d_)
        y_d_ = BatchNormalization(name='d_conv3_1_1x1_increase_bn')(y_d_)
        y_d = Add(name='d_conv3_1')([y_d, y_d_])
        z_d = Activation('relu', name='d_conv3_1/relu')(y_d)

        return z, z_d
