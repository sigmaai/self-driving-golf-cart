import pandas as pd
import cv2
import numpy as np
import utils
import model

dir = "/home/ubuntu/dataset/udacity-driving-testing-ds/"
dir2 = "/home/ubuntu/dataset/udacity-driving/"
val_dir = "/home/ubuntu/dataset/small-testing-ds/"

f1 = "/Volumes/Personal_Drive/Datasets/Udacity_Self-Driving-Car/dataset/"
f2 = "/Volumes/Personal_Drive/Datasets/Udacity_Self-Driving-Car/udacity-driving-testing-ds/"

labels = utils.preprocess_dataset(dir, dir2)
print(labels.shape)
val_labels = pd.read_csv(val_dir + "interpolated.csv")

cnn = model.commaai_model()
cnn.load_weights("./trained-vgg-v4.h5")
cnn.summary()

training_gen = utils.batch_generator(dir, labels, 8, True)
validation_gen = utils.validation_generator(val_dir, val_labels, 2)

cnn.fit_generator(training_gen,
                  steps_per_epoch=4000, epochs=10, verbose=1,)

cnn.save('trained-cai-v5.h5')