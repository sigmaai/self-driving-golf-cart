import pandas as pd
import cv2
import numpy as np
import utils
import model

dir = "/home/ubuntu/dataset/udacity-driving-testing-ds/"
val_dir = "/home/ubuntu/dataset/small-testing-ds/"

labels = pd.read_csv(dir + "interpolated.csv")
val_labels = pd.read_csv(val_dir + "interpolated.csv")
center_labels = utils.clean_steering_label(labels.values)
print(center_labels.shape)

cnn = model.commaai_model()
# cnn.load_weights("./trained-vgg-v1.h5")
cnn.summary()

training_gen = utils.batch_generator(dir, center_labels, 8, True)
validation_gen = utils.validation_generator(val_dir, val_labels, 2)

cnn.fit_generator(training_gen,
                  steps_per_epoch=3000, epochs=10, verbose=1,
                  validation_data=validation_gen,
                  validation_steps=1000)

cnn.save('trained-cai-v1.h5')