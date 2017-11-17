import model as m
import configs
import utils
import numpy as np
import matplotlib.pyplot as plt
import scipy.misc
from keras.optimizers import Adam

inputs, masks = utils.prepare_dataset(configs.data_path)
print(len(inputs))
smooth = 1.
model = m.fcn_model()
model.load_weights("./segmentation-train-1.h5")
model.compile(optimizer=Adam(lr=1e-4), loss=m.IOU_calc_loss, metrics=[m.IOU_calc])


training_gen = utils.train_generator(inputs, masks, 5)
batch_img, batch_mask = next(training_gen)

### Plotting generator output
for i in range(1):

    im = np.array([batch_img[i]],dtype=np.uint8)
    im_mask = model.predict(im)
    plt.subplot(1,3,1)
    plt.imshow(im[0])
    plt.subplot(1,3,2)
    print(im_mask.shape)
    plt.imshow(im_mask[0])
    scipy.misc.imsave('./image.png', im[0])
    scipy.misc.imsave('./mask.png', im_mask[0])
