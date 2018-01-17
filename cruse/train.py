from DenseNet import densenet
from keras.optimizers import Adam
import keras as K
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import utils
import configs
import numpy as np
import model as m


def train(labels=None, visualize_gen=False, ds_type="UD", model_type="CM", weight_path="", temporal=False):

    # ------- processing dataset ------
    # ---------------------------------
    if ds_type == "UD":
        print("using udacity dataset")
        labels = utils.preprocess_dataset(configs.ud_data_path + "dataset-1/",
                                                   configs.ud_data_path + "dataset-2/")
    if ds_type == "SF":
        print("using the Nie dataset")
        labels = pd.read_csv(configs.data_path + "labels.csv").values
    else:
        print('unrecognized dataset')
    print(labels.shape)

    # ---------------------------------
    generator = utils.batch_generator(labels, 1, ds_type)

    if visualize_gen:
        images, throttles = next(generator)
        for i in range(3):
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
    elif model_type == "DN":    # dense net
        model = densenet.DenseNet(classes=1, input_shape=input_shape, depth=28, growth_rate=12,
                                  bottleneck=True, reduction=0.5)

    print(model.summary())

    if weight_path != "":
        model.load_weights(weight_path)

    model.fit_generator(generator, steps_per_epoch=2000, epochs=10, verbose=1)
    model.save("./trained-nv-v6.h5")


if __name__ == "__main__":
    train(labels=None, visualize_gen=False, ds_type="UD", model_type="NV", weight_path="./trained-nv-v4.h5", temporal=False)

