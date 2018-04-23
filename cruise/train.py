#
# cruise control training
# (c) Neil Nie 2017-2018
#

import pandas as pd
import matplotlib.pyplot as plt
import utils
import cruise.configs as configs
import numpy as np
import cruise.model as m


def train(labels=None, visualize_gen=False, ds_type="UD", model_type="CM", weight_path="", temporal=False):

    # parameters:
    # labels:  input labels
    # visualize_gen: whether or not to visualize & debug the generator
    # ds_type: two options, UD: udacity, NN, Nie dataset
    # model_type:

    # ------- processing dataset ------
    if ds_type == "UD":
        print("using udacity dataset")
        labels = utils.preprocess_dataset(configs.ud_data_path + "dataset-1/",
                                          configs.ud_data_path + "dataset-2/",
                                          configs.ud_data_path + "dataset-3/")
    if ds_type == "NN":
        print("using the Nie dataset")
        labels = pd.read_csv(configs.data_path + "labels.csv").values
    else:
        print('unrecognized dataset')
    print(labels.shape)
    print(labels[0])

    # ---------------------------------
    generator = utils.batch_generator(labels, 2, ds_type)

    if visualize_gen:
        images, throttles = next(generator)
        for i in range(11):
            img = images[i]
            plt.imshow(np.array(img, dtype=np.uint8))
            plt.show()

    if temporal:
        input_shape = (configs.img_h, configs.img_w, 9)
    else:
        input_shape = (configs.img_h, configs.img_w, 3)

    if model_type == "CM":      # comma AI:
        model = m.commaai_model(input_shape=input_shape)
    elif model_type == "NV":    # nvidia net
        model = m.nvidia_network(input_shape=input_shape)
    elif model_type == "VG":    # VGG style
        model = m.small_vgg_network(input_shape=input_shape)

    print(model.summary())

    if weight_path != "":
        model.load_weights(weight_path)

    model.fit_generator(generator, steps_per_epoch=5000, epochs=3, verbose=1)
    model.save("./trained-nv-4.h5")


if __name__ == "__main__":

    train(labels=None, visualize_gen=False, ds_type="UD", model_type="NV", weight_path="./trained-nv-3.h5", temporal=False)

