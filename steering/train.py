import pandas as pd
import cv2
import numpy as np
import utils
import model
import configs


labels = utils.preprocess_dataset(configs.dir, configs.dir2)
print(labels.shape)
val_labels = pd.read_csv(configs.val_dir + "interpolated.csv")

cnn = model.commaai_model()
if configs.load_weights:
    cnn.load_weights(configs.weights_path)
cnn.summary()

training_gen = utils.batch_generator(labels, 32, True)
validation_gen = utils.validation_generator(configs.val_dir, val_labels, 2)

cnn.fit_generator(training_gen, steps_per_epoch=1000, epochs=15, verbose=1)

cnn.save('./trained-cai-v7.h5')
