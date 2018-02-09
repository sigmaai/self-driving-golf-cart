import models.enet_naive_upsampling.model as enet
import models.icnet.model as icnet
import configs
import utils
import numpy as np
import matplotlib.pyplot as plt
import pandas

test_results = False
visualize_gen = False

if __name__ == "__main__":

    epochs = 2

    m = enet.build(len(utils.labels), configs.img_height, configs.img_width)
    m.load_weights("./enet-c-v1-2.h5")
    print(m.summary())

    label_path = configs.data_path + "extra_labels.csv"
    labels = pandas.read_csv(label_path).values

    train_generator = utils.train_generator(labels, 2)

    images, targets = next(train_generator)

    if visualize_gen:

        for i in range(2):
            im = np.array(images[i], dtype=np.uint8)
            im_mask = np.array(targets[i], dtype=np.uint8)
            plt.subplot(1, 3, 1)
            plt.imshow(im)
            plt.axis('off')
            plt.subplot(1, 3, 2)
            plt.imshow(utils.convert_class_to_rgb(im_mask))
            print(im_mask.shape)
            plt.axis('off')
            plt.show()

    m.fit_generator(train_generator, steps_per_epoch=1000, epochs=epochs, verbose=1)

    m.save('./enet-c-v1-3.h5')

    if test_results:
        m.load_weights("./segmentation-fcn-3.h5")

        # Plotting generator output
        images, targets = next(train_generator)

        for i in range(4):
            im = np.array(images[i], dtype=np.uint8)
            im_mask = np.array(targets[i], dtype=np.uint8)
            img = np.array([im], dtype=np.uint8)
            im_prediction = m.predict(img)[0]
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