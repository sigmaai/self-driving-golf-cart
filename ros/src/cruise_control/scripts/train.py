#!/usr/bin/python
#
# training file for i3d model.
# this project supports the udacity self-driving dataset.
# link: https://github.com/udacity/self-driving-car/tree/master/datasets
# also supports the comma ai dataset
#
# Author: Neil (Yongyang) Nie
# Copyright: (c) 2018
# Licence: MIT
# Contact: contact@neilnie.com
#

from i3d import Inception3D
import configs
import pandas as pd
import communication

if __name__ == '__main__':

    load_model_name = './i3d_speed_comma_flow_' + str(configs.LENGTH) + '_7.h5'
    save_model_name = './i3d_speed_comma_flow_' + str(configs.LENGTH) + '_8.h5'

    labels = pd.read_csv('/home/neil/dataset/speedchallenge/data/data.csv').values
    val_labels = pd.read_csv('/home/neil/dataset/speedchallenge/data/validation.csv').values

    i3d_flow = Inception3D(input_shape=(configs.LENGTH, configs.IMG_HEIGHT, configs.IMG_WIDTH, 2),
                   weights_path=load_model_name)
    i3d_flow.summary()

    i3d_flow.train(type='flow', labels=labels,
                   val_labels=val_labels,
                   epochs=5, epoch_steps=800,
                   validation=True, val_steps=500,
                   save_path=save_model_name,
                   log_path='logs/flow_32')

    communication.notify_training_completion(save_model_name)


    # single frame
    # model = ConvModel(input_shape=(configs.IMG_HEIGHT, configs.IMG_WIDTH, 3), weights_path='./conv_speed_comma_frgb_1.h5')
    # model.train(type='flow', epochs=2, epoch_steps=1000, validation=True, val_steps=200, save_path='./conv_speed_comma_frgb_2.h5')
