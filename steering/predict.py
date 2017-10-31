import numpy as np
from keras.models import load_model
import pandas as pd
import glob
import argparse
import cv2
import utils
import model


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
    model = model.nvidia_network()
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
        img = img[160:480, 0: 640]
        img = cv2.resize(img, (128, 128))
        image = np.array([img])  # the model expects 4D array
        predicted_steers = model.predict(image)[0][0]
        predictions[i] = predicted_steers


    print("Writing predictions...")
    pd.DataFrame({"frame_id": df_imgs["frame_id"], "steering_angle": predictions}).to_csv(output_path, index=False, header=True)

    print("Done!")