import cv2
import models.enet_naive_upsampling.model as enet
import models.icnet.model as icnet
import numpy as np
import utils
import configs
import matplotlib.pyplot as plt
import time


path = [# "./testing_imgs/test_0.jpg",
        "./testing_imgs/test_1.jpg",
        "./testing_imgs/test_2.jpg",
        "./testing_imgs/test_3.jpg",
        "./testing_imgs/test_4.jpg",
        # "./testing_imgs/test_6.jpg",
        "./testing_imgs/test_7.jpg",
        "./testing_imgs/test_8.jpg",
        "./testing_imgs/test_9.jpg"
        ]

m = enet.build(len(utils.labels), configs.img_height, configs.img_width)
# m = icnet.build(3, 512, 512)
m.load_weights("./new-enet-coarse-3.h5")
m.summary()

for i in range(len(path)):


    image = utils.load_image(path[i])
    image = np.array(image, dtype=np.uint8)
    start = time.time()
    im_mask = m.predict(np.array([image]))[0]
    end = time.time()
    print(end - start)
    im_mask = utils.convert_class_to_rgb(im_mask)
    img_pred = cv2.addWeighted(im_mask, 0.8, image, 0.8, 0)
    img_pred = cv2.cvtColor(img_pred, cv2.COLOR_RGB2BGR)
    img_pred = cv2.resize(img_pred, (1280, 800))
    cv2.imwrite("./result3_{}.png".format(i), img_pred)
    # plt.imshow(img_pred)
    # plt.show()

