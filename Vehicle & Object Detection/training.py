from __future__ import absolute_import
from __future__ import print_function
import cv2
import os, sys
import numpy as np
import glob
import img_util
import training_util
import pandas as pd
from decimal import Decimal

import keras
import keras.losses
from keras.models import Model
from keras.models import load_model
from keras.models import model_from_json
from keras.layers import Input, merge, Convolution2D, MaxPooling2D, UpSampling2D,Lambda
from keras.optimizers import Adam
from keras.callbacks import ModelCheckpoint, LearningRateScheduler
from keras import backend as K
from scipy.ndimage.measurements import label
import time

def prepare_database(path):

    dir_label = ['object-dataset', 'object-detection-crowdai']

    # Make data frame in Pandas
    df_files1 = pd.read_csv(path + dir_label[1]+'/labels.csv', header=0)
    df_vehicles1 = df_files1[(df_files1['Label']=='Car') | (df_files1['Label']=='Truck')].reset_index()
    df_vehicles1 = df_vehicles1.drop('index', 1)
    df_vehicles1['File_Path'] =  dir_label[1] + '/' + df_vehicles1['Frame']
    df_vehicles1 = df_vehicles1.drop('Preview URL', 1)

    ### Get data frame from second source
    ### Renamed columns to correct xmin, xmax, ymin, ymax values.
    ### REnamed frames and labels to match crowd-awi source

    # df_files2 = pd.read_csv(path + 'object-dataset/labels.csv', header=None)
    # df_files2.columns= ['Frame',  'xmin', 'xmax', 'ymin','ymax', 'ind', 'Label']
    # df_vehicles2 = df_files2[(df_files2['Label'] == 'car') | (df_files2['Label'] == 'truck')].reset_index()
    # df_files2 = df_files2[(df_files2['Label'] == 'car') | (df_files2['Label'] == 'truck')].reset_index()
    # df_vehicles2 = df_vehicles2.drop('index', 1)
    # df_vehicles2 = df_vehicles2.drop('ind', 1)

    # df_vehicles2["Frame"] = df_files2["xmin"].values.astype(np.float32)
    # df_vehicles2["xmin"] = df_files2["xmax"].values.astype(np.float32)
    # df_vehicles2["xmax"] = df_files2["ymin"].values.astype(np.float32)
    # df_vehicles2["ymin"] = df_files2["ymax"].values.astype(np.float32)
    # df_vehicles2["ymax"] = df_files2["Frame"].values
    # df_vehicles2["Label"] = df_files2["Label"].values
    # df_vehicles2['File_Path'] = dir_label[0] + '/' + df_files2["Frame"]
    # df_vehicles2.columns =['xmin','ymin','xmax','ymax','Frame','Label','File_Path']

    df_vehicles = pd.concat([df_vehicles1]).reset_index()
    df_vehicles = df_vehicles.drop('index', 1)
    df_vehicles.columns =['xmin','ymin','xmax','ymax','Frame','Label','File_Path']

    return df_vehicles


def get_small_unet():

    img_rows = 640
    img_cols = 960
    inputs = Input((img_rows, img_cols, 3))
    inputs_norm = Lambda(lambda x: x / 127.5 - 1.)
    conv1 = Convolution2D(8, (3, 3), activation='relu', padding='same')(inputs)
    conv1 = Convolution2D(8, (3, 3), activation='relu', padding='same')(conv1)
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

    conv10 = Convolution2D(1, 1, 1, activation='sigmoid')(conv9)

    model = Model(input=inputs, output=conv10)

    return model


### IOU or dice coeff calculation

def IOU_calc(y_true, y_pred):
    y_true_f = K.flatten(y_true)
    y_pred_f = K.flatten(y_pred)
    intersection = K.sum(y_true_f * y_pred_f)

    return 2 * (intersection + smooth) / (K.sum(y_true_f) + K.sum(y_pred_f) + smooth)


def IOU_calc_loss(y_true, y_pred):
    return -IOU_calc(y_true, y_pred)

if __name__ == "__main__":

    os.chdir("/Volumes/Personal_Drive/Datasets/Vehicle_Detection/")
    df_vehicles = prepare_database("https://www.floydhub.com/neilnie/datasets/")

    smooth = 1.
    model = get_small_unet()
    model.compile(optimizer=Adam(lr=1e-4), loss=IOU_calc_loss, metrics=[IOU_calc])
    
    #model = model_from_json(open("/Users/YongyangNie/Desktop/train-1.h5").read())
    #model.load_weights(os.path.join(os.path.dirname("/Users/YongyangNie/Desktop/"), 'train-1.h5'))

    training_gen = training_util.generate_train_batch(df_vehicles, 32)
    history = model.fit_generator(training_gen, steps_per_epoch=500, epochs=1, verbose=1, callbacks=None, validation_data=None)
    print(history)
    model.save('train-1.h5')
