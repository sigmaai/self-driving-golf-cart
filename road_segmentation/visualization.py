import model as m
import configs
import utils
import numpy as np
import matplotlib.pyplot as plt
import os
from keras.optimizers import Adam

inputs, masks = utils.prepare_dataset(configs.data_path)
print(len(inputs))
smooth = 1.
model = m.fcn_model()
model.load_weights("./segmentation-train-1.h5")
model.compile(optimizer=Adam(lr=1e-4), loss=m.IOU_calc_loss, metrics=[m.IOU_calc])

train_generator = utils.gen_batch_function(os.path.join(configs.data_path, 'data_road/training'), (configs.img_height, configs.img_width), 8)
images, targets = next(train_generator)

for i in range(len(images)):

    im = np.array(images[i], dtype=np.uint8)
    im_mask = model.predict(np.array([im]))[0]
    print(im_mask.shape)
    plt.subplot(1, 3, 1)
    plt.imshow(im)
    plt.axis('off')
    plt.subplot(1, 3, 2)
    plt.imshow(im_mask[:, :, 2])
    plt.axis('off')
    plt.subplot(1, 3, 3)
    plt.imshow(im)
    plt.axis('off')
    plt.show()
