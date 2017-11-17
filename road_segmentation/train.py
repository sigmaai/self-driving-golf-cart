import model as m
import configs
import utils
import numpy as np
import matplotlib.pyplot as plt
import cv2
from keras.optimizers import Adam

inputs, masks = utils.prepare_dataset(configs.data_path)
# smooth = 1.
# model = m.fcn_model()
# model.compile(optimizer=Adam(lr=1e-4), loss=m.IOU_calc_loss, metrics=[m.IOU_calc])

# training_gen = utils.train_generator(inputs, masks, 4)
# model.fit_generator(training_gen, steps_per_epoch=1000, epochs=3, verbose=1, callbacks=None, validation_data=None)
# model.save('./segmentation-train-1.h5')


training_gen = utils.train_generator(inputs, masks, 5)
batch_img, batch_mask = next(training_gen)

### Plotting generator output
for i in range(3):

    im = np.array(batch_img[i],dtype=np.uint8)
    print(im.shape)
    im_mask = np.array(batch_mask[i],dtype=np.uint8)
    print(im_mask.shape)
    plt.subplot(1,3,1)
    plt.imshow(im)
    plt.axis('off')
    plt.subplot(1,3,2)
    plt.imshow(im_mask)
    plt.axis('off')
    plt.subplot(1,3,3)
    plt.imshow(im)
    plt.axis('off')
    plt.show()