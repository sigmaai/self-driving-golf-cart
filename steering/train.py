import pandas as pd
import cv2
import numpy as np
import steering.utils as utils
import model
import steering.configs as configs

validation = False

labels = utils.preprocess_dataset(configs.dir, configs.dir2, configs.dir3)
print("data length {}".format(len(labels)))

# create the network or load weights
cnn = model.commaai_model()
if configs.load_weights:
    cnn.load_weights(configs.model_path)
cnn.summary()

training_gen = utils.batch_generator(labels, 16, True)
if validation:
    val_labels = pd.read_csv(configs.val_dir + "interpolated.csv")
    validation_gen = utils.validation_generator(configs.val_dir, val_labels, 2)

cnn.fit_generator(training_gen, steps_per_epoch=1000, epochs=3, verbose=1)

cnn.save('./trained-cai-v2.2.h5')
