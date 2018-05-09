#
# training script
# for training deep learning models
# By Neil Nie
# (c) Yongyang Nie, 2018. All Rights Reserved
# Contact: contact@neilnie.com
#
#

import pandas as pd
import numpy as np
import utils as utils
import models.models as models
import configs as configs
import matplotlib.pyplot as plt

validation = True
visualize = False


def train(data_type="UDAT", model_type="Comma", load_weights=False, steps=1000, epochs=5, validation=False):

    # --------- check for dataset type ------------
    if data_type == "UDAT":
        labels = utils.preprocess_udacity_dataset(configs.dir, configs.dir2) # , configs.dir3
    elif data_type == "SELF":
        labels = utils.process_self_dataset(configs.dataset_dirs)
    else:
        raise Exception('Please enter a valid dataset type')

    training_gen = utils.self_batch_generator(labels, 8, True)

    # --------- check for model type  -------------
    if model_type == "comma":
        model = models.commaai_model()
    elif model_type == "nvidia":
        model = models.nvidia_model()
    elif model_type == "rambo":
        model = models.create_rambo_model()
    elif model_type == "convLSTM":
        model = models.autumn()
    else:
        raise Exception("Unknown model type: " + model_type + ". Please enter a valid model type")

    print("data length {}".format(len(labels)))

    # -------- create the network or load weights -----
    if load_weights:
        model.load_weights(configs.train_model_path)
    model.summary()

    if visualize == True:
        images, steerings = next(training_gen)
        for i in range(len(images)):
            img = images[i]
            plt.imshow(np.array(img, dtype=np.uint8))
            plt.show()

    if validation:
        val_labels = pd.read_csv(configs.val_dir + "labels.csv")
        validation_gen = utils.validation_generator(dir=configs.val_dir, data=val_labels, batch_size=8)
        model.fit_generator(training_gen, steps_per_epoch=steps, epochs=epochs, verbose=1, validation_data=validation_gen, validation_steps=2000)
    else:
        model.fit_generator(training_gen, steps_per_epoch=steps, epochs=epochs, verbose=1, validation_data=validation_gen, validation_steps=2000)


    # standard model naming scheme
    # "[purpose]-[model type]-[dataset used]-[version].h5"
    # example:
    #         "str-cai-self-v1.h5"

    model.save('str-cai-self-v1.h5')


if __name__ == "__main__":

    train(data_type="SELF", model_type="comma", epochs=3, load_weights=True, validation=True)
