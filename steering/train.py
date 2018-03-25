import pandas as pd
import cv2
import numpy as np
import steering.utils as utils
import steering.models as models
import steering.configs as configs
import matplotlib.pyplot as plt

validation = True
visualize = False

def train(data_type="UDA", model_type="COM", load_weights=False, steps=100, epochs=5):

    if data_type == "UDA":
        labels = utils.preprocess_dataset(configs.dir, configs.dir2) # , configs.dir3
    elif data_type == "SEF":
        labels = utils.gather_self_dataset(configs.own_dataset_dirs)
    else:
        raise Exception('Please enter a valid dataset type')

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

    # create the network or load weights

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

    cnn.fit_generator(training_gen, steps_per_epoch=1000, epochs=3, verbose=1, validation_data=validation_gen,
                      validation_steps=2000)

    cnn.save('./trained-cai-v7.h5')

if __name__ == "__main__":
    train(data_type="SEF", model_type="COM", epochs=3, load_weights=True)