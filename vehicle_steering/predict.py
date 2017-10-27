import numpy as np
from keras.models import load_model
import pandas as pd
import glob
import argparse
from keras.models import Model, Sequential
from keras.layers.core import Dense, Activation, Flatten
from keras.layers.convolutional import Conv2D
from keras.optimizers import Adam
from keras.models import load_model
import keras as K
import cv2

def create_nvidia_model1():
    model = Sequential()

    model.add(Conv2D(24, (5, 5), padding="same", strides=2, input_shape=(480, 640, 3)))
    model.add(Activation('relu'))
    model.add(Conv2D(36, (5, 5), padding="same", strides=2))
    model.add(Activation('relu'))
    model.add(Conv2D(48, (5, 5), padding="same", strides=2))
    model.add(Activation('relu'))
    model.add(Conv2D(64, (3, 3), padding="same", strides=2))
    model.add(Activation('relu'))
    model.add(Conv2D(64, (3, 3), padding="same", strides=2))
    model.add(Flatten())
    model.add(Activation('relu'))
    model.add(Dense(512))
    model.add(Activation('relu'))
    model.add(Dense(256))
    model.add(Activation('relu'))
    model.add(Dense(128))
    model.add(Activation('relu'))
    model.add(Dense(1))
    adam = Adam(lr=1e-4)
    model.compile(optimizer=adam, loss="mse")

    print('Model is created and compiled..')
    return model

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Remote Driving')
    parser.add_argument('--model', type=str,
        help='Path to model h5 file. Model should be on the same path.')
    parser.add_argument('--image_folder', type=str, default='',
        help='Path to image folder. This is where the images from the run will be saved.')
    parser.add_argument('--output_path', type=str, default='',
        help='Path to image folder. This is where the images from the run will be saved.')
    args = parser.parse_args()

    data_path = args.image_folder
    model_path = args.model
    output_path = args.output_path

    print("Loading model...")
    model = create_nvidia_model1()
    model.load_weights(model_path)

    print("loading dataset")
    df_imgs = pd.read_csv(data_path + "/interpolated.csv")


    predictions = np.empty(len(df_imgs))
    for i in range(len(df_imgs)):

        if i%500 == 0:
           print('.', end=" ")
        path = data_path + "/center/" + str(df_imgs["frame_id"][i]) + ".jpg"
        img = cv2.imread(path)
        b, g, r = cv2.split(img)  # get b,g,r
        img = cv2.merge([r, g, b])  # switch it to rgb
        image = np.array([img])  # the model expects 4D array
        predicted_steers = model.predict(image)[0][0]
        predictions[i] = predicted_steers


    print("Writing predictions...")
    pd.DataFrame({"frame_id": df_imgs["frame_id"], "steering_angle": predictions}).to_csv(output_path, index=False, header=True)

    print("Done!")