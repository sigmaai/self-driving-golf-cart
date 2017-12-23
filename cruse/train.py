from DenseNet import densenet
from keras.optimizers import Adam
import keras as K

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import utils
import configs
import numpy as np


def root_mean_squared_error(y_true, y_pred):
    return K.backend.sqrt(K.backend.mean(K.backend.square(y_pred - y_true), axis=-1))

throttle_labels = pd.read_csv(configs.data_path + "/labels.csv")
print(throttle_labels.shape)
throttle_labels.head()

generator = utils.batch_generator(throttle_labels.values, 1)

# images, throttles = next(generator)
#
# for i in range(3):
#     img= images[i]
#     imgplot = plt.imshow(np.array(img, dtype=np.uint8))
#     plt.show()

input = (512, 512, 3)

model = densenet.DenseNet(classes=1, input_shape=input, depth=19, growth_rate=12, bottleneck=True, reduction=0.5)
adam = Adam(lr=1e-4)
model.compile(optimizer=adam, loss=root_mean_squared_error)
print(model.summary())

model.fit_generator(generator, steps_per_epoch=1000, epochs=10, verbose=1)
