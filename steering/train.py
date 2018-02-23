import pandas as pd
import cv2
import numpy as np
import utils as utils
import models
import configs as configs
import matplotlib.pyplot as plt

validation = True
visualize = False

labels = utils.preprocess_dataset(configs.dir, configs.dir2, configs.dir3)
print("data length {}".format(len(labels)))

# create the network or load weights
cnn = models.commaai_model()
if configs.load_weights:
    cnn.load_weights(configs.train_model_path)
cnn.summary()

training_gen = utils.batch_generator(labels, 8, True)

if visualize == True:
    images, steerings = next(training_gen)
    for i in range(16):
        img = images[i]
        plt.imshow(np.array(img, dtype=np.uint8))
        plt.show()

if validation:
    val_labels = pd.read_csv(configs.val_dir + "interpolated.csv")
    validation_gen = utils.validation_generator(configs.val_dir, val_labels, 2)

cnn.fit_generator(training_gen, steps_per_epoch=1000, epochs=3, verbose=1, validation_data=validation_gen, validation_steps=1000)

cnn.save('./trained-cai-v7.h5')
