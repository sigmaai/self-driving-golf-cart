from models.enet_naive_upsampling import model
import configs
import utils
import numpy as np
import matplotlib.pyplot as plt
import os

test_results = False

epochs = 2

enet = model.build(len(utils.labels), configs.img_height, configs.img_width)
# model.load_weights("./segmentation-fcn-3.h5")
print(model.summary())
train_generator = utils.gen_batch_function(os.path.join(configs.data_path, 'data_road/training'),
                                           (configs.img_height, configs.img_width), 2)

images, targets = next(train_generator)

# for i in range(2):
#     im = np.array(images[i], dtype=np.uint8)
#     im_mask = np.array(targets[i], dtype=np.uint8)
#     print(im_mask.shape)
#     plt.subplot(1, 3, 1)
#     plt.imshow(im)
#     plt.axis('off')
#     plt.subplot(1, 3, 2)
#     plt.imshow(im_mask[:, :, 0])
#     plt.axis('off')
#     plt.show()

model.fit_generator(train_generator, steps_per_epoch=steps_per_epoch, epochs=epochs, verbose=1)

model.save('./segmentation-fcn-4.h5')

if test_results:
    model.load_weights("./segmentation-fcn-3.h5")

    # Plotting generator output
    images, targets = next(train_generator)

    for i in range(4):
        im = np.array(images[i], dtype=np.uint8)
        im_mask = np.array(targets[i], dtype=np.uint8)
        img = np.array([im], dtype=np.uint8)
        im_prediction = model.predict(img)[0]
        plt.subplot(1, 3, 1)
        plt.imshow(im)
        plt.axis('off')
        plt.subplot(1, 3, 2)
        plt.imshow(im_mask[:, :, 0])
        plt.axis('off')
        plt.subplot(1, 3, 3)
        plt.imshow(im_prediction[:, :, 0])
        plt.axis('off')
        plt.show()