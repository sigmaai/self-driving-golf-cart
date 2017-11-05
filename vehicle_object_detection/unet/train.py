import cv2
import os, sys
import numpy as np
import img_util
import train_util
import model as m
import pandas as pd

from keras.models import load_model
from keras.optimizers import Adam


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


if __name__ == "__main__":

    os.chdir("/Volumes/Personal_Drive/Datasets/Vehicle_Detection/")
    df_vehicles = prepare_database("")

    smooth = 1.
    model = m.fcn_model()
    model.compile(optimizer=Adam(lr=1e-4), loss=m.IOU_calc_loss, metrics=[m.IOU_calc])

    training_gen = train_util.generate_train_batch(df_vehicles, 32)
    model.fit_generator(training_gen, steps_per_epoch=500, epochs=1, verbose=1, callbacks=None, validation_data=None)
    model.save('train-1.h5')
