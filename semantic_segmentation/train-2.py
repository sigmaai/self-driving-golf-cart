import model
import configs
import utils
import numpy as np
import matplotlib.pyplot as plt
import os

test_results = False
steps_per_epoch = 1000
epochs = 1
nb_classes = 2

fcn = model.fcn_model()
print(fcn.summary())
# SegNet.load_weights("./segmentation-train-3.h5")
train_generator = utils.gen_batch_function(os.path.join("/Volumes/Personal_Drive/Datasets/", 'data_road/training'),
                                           (configs.img_height, configs.img_width), 2)

images, targets = next(train_generator)

for i in range(2):

    im = np.array(images[i], dtype=np.uint8)
    print(im)
    im_mask = np.array(targets[i], dtype=np.uint8)
    print(im_mask.shape)
    plt.subplot(1, 3, 1)
    plt.imshow(im)
    plt.axis('off')
    plt.subplot(1, 3, 2)
    plt.imshow(im_mask[:, :, 0])
    plt.axis('off')
    plt.show()

fcn.fit_generator(train_generator, steps_per_epoch=steps_per_epoch, epochs=epochs, verbose=1)

fcn.save('./fcn-train-4.h5')

if test_results:
    SegNet.load_weights("./segmentation-train-4.h5")

    # Plotting generator output
    images, targets = next(train_generator)

    for i in range(4):
        im = np.array(images[i], dtype=np.uint8)
        im_mask = np.array(targets[i], dtype=np.uint8)
        img = np.array([im], dtype=np.uint8)
        im_prediction = SegNet.predict(img)[0]
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