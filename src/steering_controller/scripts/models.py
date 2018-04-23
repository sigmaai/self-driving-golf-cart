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
import tensorflow as tf
import steering.configs as configs
# import configs as configs


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


def weight_variable(shape):
    initializer = tf.contrib.layers.xavier_initializer_conv2d()
    initial = initializer(shape=shape)
    return tf.Variable(initial)


def bias_variable(shape):
    initial = tf.constant(0.1, shape=shape)
    return tf.Variable(initial)


def conv2d(x, W, stride):
    return tf.nn.conv2d(x, W, strides=[1, stride, stride, 1], padding='VALID')


class AutumnModel(object):

    ''' Implements the ConvNet model from the NVIDIA paper '''
    def __init__(self, dropout_prob=0.2, batch_norm=False, whitening=False, is_training=True):
        x = tf.placeholder(tf.float32, shape=[None, 66, 200, 3], name='x')
        y_ = tf.placeholder(tf.float32, shape=[None, 1])
        keep_prob = tf.placeholder(tf.float32, name='keep_prob')
        x_image = x

        self.W_conv1 = weight_variable([5, 5, 3, 24])
        self.b_conv1 = bias_variable([24])
        self.h_conv1 = tf.nn.relu(conv2d(x_image, self.W_conv1, 2) + self.b_conv1)
        if batch_norm:
            self.h_conv1 = tf.contrib.layers.batch_norm(self.h_conv1, is_training=is_training, trainable=True)

        self.W_conv2 = weight_variable([5, 5, 24, 36])
        self.b_conv2 = bias_variable([36])
        self.h_conv2 = tf.nn.relu(conv2d(self.h_conv1, self.W_conv2, 2) + self.b_conv2)

        self.W_conv3 = weight_variable([5, 5, 36, 48])
        self.b_conv3 = bias_variable([48])
        self.h_conv3 = tf.nn.relu(conv2d(self.h_conv2, self.W_conv3, 2) + self.b_conv3)
        if batch_norm:
            self.h_conv3 = tf.contrib.layers.batch_norm(self.h_conv3, is_training=is_training, trainable=True)

        self.W_conv4 = weight_variable([3, 3, 48, 64])
        self.b_conv4 = bias_variable([64])
        self.h_conv4 = tf.nn.relu(conv2d(self.h_conv3, self.W_conv4, 1) + self.b_conv4)

        self.W_conv5 = weight_variable([3, 3, 64, 64])
        self.b_conv5 = bias_variable([64])
        self.h_conv5 = tf.nn.relu(conv2d(self.h_conv4, self.W_conv5, 1) + self.b_conv5)
        if batch_norm:
            self.h_conv5 = tf.contrib.layers.batch_norm(self.h_conv5, is_training=is_training, trainable=True)

        self.W_fc1 = weight_variable([1152, 1164])
        self.b_fc1 = bias_variable([1164])

        self.h_conv5_flat = tf.reshape(self.h_conv5, [-1, 1152])
        self.h_fc1 = tf.nn.relu(tf.matmul(self.h_conv5_flat, self.W_fc1) + self.b_fc1)
        if batch_norm:
            self.h_fc1 = tf.contrib.layers.batch_norm(self.h_fc1, is_training=is_training, trainable=True)
        self.h_fc1_drop = tf.nn.dropout(self.h_fc1, keep_prob)

        self.W_fc2 = weight_variable([1164, 100])
        self.b_fc2 = bias_variable([100])
        self.h_fc2 = tf.nn.relu(tf.matmul(self.h_fc1_drop, self.W_fc2) + self.b_fc2, name='fc2')
        if batch_norm:
            self.h_fc2 = tf.contrib.layers.batch_norm(self.h_fc2, is_training=is_training, trainable=True)
        self.h_fc2_drop = tf.nn.dropout(self.h_fc2, keep_prob)

        self.W_fc3 = weight_variable([100, 50])
        self.b_fc3 = bias_variable([50])
        self.h_fc3 = tf.nn.relu(tf.matmul(self.h_fc2_drop, self.W_fc3) + self.b_fc3, name='fc3')
        if batch_norm:
            self.h_fc3 = tf.contrib.layers.batch_norm(self.h_fc3, is_training=is_training, trainable=True)
        self.h_fc3_drop = tf.nn.dropout(self.h_fc3, keep_prob)

        self.W_fc4 = weight_variable([50, 10])
        self.b_fc4 = bias_variable([10])
        self.h_fc4 = tf.nn.relu(tf.matmul(self.h_fc3_drop, self.W_fc4) + self.b_fc4, name='fc4')
        if batch_norm:
            self.h_fc4 = tf.contrib.layers.batch_norm(self.h_fc4, is_training=is_training, trainable=True)
        self.h_fc4_drop = tf.nn.dropout(self.h_fc4, keep_prob)

        self.W_fc5 = weight_variable([10, 1])
        self.b_fc5 = bias_variable([1])
        y = tf.mul(tf.atan(tf.matmul(self.h_fc4_drop, self.W_fc5) + self.b_fc5), 2, name='y')

        self.x = x
        self.y_ = y_
        self.y = y
        self.keep_prob = keep_prob
        self.fc2 = self.h_fc2
        self.fc3 = self.h_fc3