import numpy as np
from rambo import Rambo
from keras.models import load_model
import pandas as pd
import glob
import argparse
import cv2
import utils
import models
from scipy import misc
from autumn import AutumnModel


def test_komanda():

    parser = argparse.ArgumentParser(description='Remote Driving')
    parser.add_argument('--model', type=str,
                        help='Path to model h5 file. Model should be on the same path.')
    parser.add_argument('--image_folder', type=str, default='',
                        help='Path to image folder. This is where the images from the run will be saved.')
    parser.add_argument('--output_path', type=str, default='',
                        help='Path to image folder. This is where the images from the run will be saved.')
    args = parser.parse_args()

    data_path = args.image_folder
    output_path = args.output_path

    cnn_graph = "./weights/autumn/autumn-cnn-model-tf.meta"
    lstm_json = "./weights/autumn/autumn-lstm-model-keras.json"
    cnn_weights = "./weights/autumn/autumn-cnn-weights.ckpt"
    lstm_weights = "./weights/autumn/autumn-lstm-weights.hdf5"

    steering_predictor = AutumnModel(cnn_graph, lstm_json, cnn_weights, lstm_weights)

    print(steering_predictor.model.summary())

    print("loading dataset")
    df_imgs = pd.read_csv(data_path + "/interpolated.csv")

    print("predicting")
    predictions = np.empty(len(df_imgs))
    for i in range(len(df_imgs)):
        if i % 500 == 0:
            print('.', end=" ")
        path = data_path + "/center/" + str(df_imgs["frame_id"][i]) + ".jpg"
        img = cv2.imread(path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        angle = steering_predictor.predict(img)
        predictions[i] = angle

    print("Writing predictions...")
    pd.DataFrame({"frame_id": df_imgs["frame_id"], "steering_angle": predictions}).to_csv(output_path, index=False,
                                                                                          header=True)
    print("Done!")


def test_autumn():

    parser = argparse.ArgumentParser(description='Remote Driving')
    parser.add_argument('--model', type=str,
                        help='Path to model h5 file. Model should be on the same path.')
    parser.add_argument('--image_folder', type=str, default='',
                        help='Path to image folder. This is where the images from the run will be saved.')
    parser.add_argument('--output_path', type=str, default='',
                        help='Path to image folder. This is where the images from the run will be saved.')
    args = parser.parse_args()

    data_path = args.image_folder
    output_path = args.output_path

    cnn_graph = "./weights/autumn/autumn-cnn-model-tf.meta"
    lstm_json = "./weights/autumn/autumn-lstm-model-keras.json"
    cnn_weights = "./weights/autumn/autumn-cnn-weights.ckpt"
    lstm_weights = "./weights/autumn/autumn-lstm-weights.hdf5"

    steering_predictor = AutumnModel(cnn_graph, lstm_json, cnn_weights, lstm_weights)

    print(steering_predictor.model.summary())

    print("loading dataset")
    df_imgs = pd.read_csv(data_path + "/interpolated.csv")

    print("predicting")
    predictions = np.empty(len(df_imgs))
    for i in range(len(df_imgs)):
        if i % 500 == 0:
            print('.', end=" ")
        path = data_path + "/center/" + str(df_imgs["frame_id"][i]) + ".jpg"
        img = cv2.imread(path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        angle = steering_predictor.predict(img)
        predictions[i] = angle

    print("Writing predictions...")
    pd.DataFrame({"frame_id": df_imgs["frame_id"], "steering_angle": predictions}).to_csv(output_path, index=False,
                                                                                          header=True)
    print("Done!")


def test_rambo():

    parser = argparse.ArgumentParser(description='Remote Driving')
    parser.add_argument('--model', type=str,
                        help='Path to model h5 file. Model should be on the same path.')
    parser.add_argument('--image_folder', type=str, default='',
                        help='Path to image folder. This is where the images from the run will be saved.')
    parser.add_argument('--output_path', type=str, default='',
                        help='Path to image folder. This is where the images from the run will be saved.')
    args = parser.parse_args()

    data_path = args.image_folder
    output_path = args.output_path

    print("Loading model...")
    steering_predictor = Rambo("./final_model.hdf5", "./X_train_mean.npy")
    print(steering_predictor.model.summary())
    print("loading dataset")
    df_imgs = pd.read_csv(data_path + "/interpolated.csv")

    print("predicting")
    predictions = np.empty(len(df_imgs))
    for i in range(len(df_imgs)):
        if i % 500 == 0:
            print('.', end=" ")
        path = data_path + "/center/" + str(df_imgs["frame_id"][i]) + ".jpg"
        angle = steering_predictor.predict_path(path)
        predictions[i] = -1 * angle

    print("Writing predictions...")
    pd.DataFrame({"frame_id": df_imgs["frame_id"], "steering_angle": predictions}).to_csv(output_path, index=False,
                                                                                          header=True)
    print("Done!")


def test_homemade():

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
    model = models.commaai_model()
    model.load_weights(model_path)

    print("loading dataset")
    df_imgs = pd.read_csv(data_path + "/interpolated.csv")

    print("predicting")
    predictions = np.empty(len(df_imgs))
    for i in range(len(df_imgs)):

        if i % 500 == 0:
            print('.', end=" ")
        path = data_path + "/center/" + str(df_imgs["frame_id"][i]) + ".jpg"
        img = utils.load_image(path)
        image = np.array([img])
        predicted_steers = model.predict(image)[0][0]
        predictions[i] = predicted_steers

    print("Writing predictions...")
    pd.DataFrame({"frame_id": df_imgs["frame_id"], "steering_angle": predictions}).to_csv(output_path, index=False,
                                                                                          header=True)
    print("Done!")


if __name__ == "__main__":
    test_autumn()
