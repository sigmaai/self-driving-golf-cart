import pandas as pd
import cv2
import numpy as np
import steering.utils as utils
import steering.models as models
import steering.configs as configs
import matplotlib.pyplot as plt

validation = True
visualize = False


def train(data_type="UDA", model_type="Comma", load_weights=False, steps=1000, epochs=5):

    # --------- check for dataset type ------------
    if data_type == "UDA":
        labels = utils.preprocess_dataset(configs.dir, configs.dir2) # , configs.dir3
        training_gen = utils.batch_generator(labels, 8, True)
    elif data_type == "SEF":
        labels = utils.gather_self_dataset(configs.own_dataset_dirs)
        training_gen = utils.self_batch_generator(labels, 8, True)
    else:
        raise Exception('Please enter a valid dataset type')

    # --------- check for model type  -------------
    if model_type == "Comma":
        model = models.commaai_model()
    elif model_type == "Nvidia":
        model = models.nvidia_model()
    elif model_type == "Rambo":
        model = models.create_rambo_model()
    elif model_type == "ConvLSTM":
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
        val_labels = pd.read_csv(configs.val_dir + "interpolated.csv")
        validation_gen = utils.validation_generator(configs.val_dir, val_labels, 2)
    else:
        validation_gen = None

    model.fit_generator(training_gen, steps_per_epoch=steps, epochs=epochs, verbose=1, validation_data=validation_gen, validation_steps=2000)

    # standard model naming scheme
    # "[purpose]-[model type]-[dataset used]-[version].h5"
    # example:
    #         "str-cai-self-v1.h5"

    model.save('str-cai-self-v2.h5')


if __name__ == "__main__":

    train(data_type="SEF", model_type="Comma", epochs=3, load_weights=True)
