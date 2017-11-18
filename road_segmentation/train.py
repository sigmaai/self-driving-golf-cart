import model as m
import configs
import utils
import numpy as np
import matplotlib.pyplot as plt
import os
from keras.optimizers import Adam

smooth = 1.
model = m.fcn_model()
model.compile(optimizer=Adam(lr=1e-4), loss=m.IOU_calc_loss, metrics=[m.IOU_calc])

train_generator = utils.gen_batch_function(os.path.join(configs.data_path, 'data_road/training'), (configs.img_height, configs.img_width), 8)
model.fit_generator(train_generator,
                    steps_per_epoch=1000,
                    epochs=1, verbose=1,
                    callbacks=None, validation_data=None)
model.save('./segmentation-train-1.h5')

### Plotting generator output

images, targets = next(train_generator)

for i in range(len(images)):
    print("image size size", images[i].shape)
    print("mask shape", targets.shape)

    im = np.array(images[i], dtype=np.uint8)
    im_mask = np.array(targets[i], dtype=np.uint8)
    plt.subplot(1, 3, 1)
    plt.imshow(im)
    plt.axis('off')
    plt.subplot(1, 3, 2)
    plt.imshow(im_mask[:, :, 0])
    plt.axis('off')
    plt.subplot(1, 3, 3)
    plt.imshow(im)
    plt.axis('off')
    plt.show()