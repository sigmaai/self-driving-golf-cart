

from models.enet_naive_upsampling import model
import models.icnet.model as icnet
import configs
import utils
import numpy as np
import matplotlib.pyplot as plt
import pandas

label_path = configs.data_path + "labels.csv"
labels = pandas.read_csv(label_path).values

train_generator = utils.train_generator(labels, 10)

images, targets = next(train_generator)

for i in range(9):

    im = np.array(images[i], dtype=np.uint8)
    im_mask = np.array(targets[i], dtype=np.uint8)
    plt.subplot(1, 3, 1)
    plt.imshow(im)
    plt.axis('off')
    plt.subplot(1, 3, 2)
    plt.imshow(im_mask)
    print("image")
    print(im_mask)
    plt.axis('off')
    plt.show()