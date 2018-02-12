import cv2
import models.enet_naive_upsampling.model as enet
import models.icnet.model as icnet
import numpy as np
import utils
import configs
import matplotlib.pyplot as plt
import time


# path = [# "./testing_imgs/test_0.jpg",
#         "./testing_imgs/test_1.jpg",
#         "./testing_imgs/test_2.jpg",
#         "./testing_imgs/test_3.jpg",
#         "./testing_imgs/test_4.jpg",
#         # "./testing_imgs/test_6.jpg",
#         "./testing_imgs/test_7.jpg",
#         "./testing_imgs/test_8.jpg",
#         "./testing_imgs/test_9.jpg"
#         ]
path = ["./testing_imgs/start-1.jpg",
        "./testing_imgs/stop-2.jpeg",
        "./testing_imgs/stop3.jpeg"]

m = enet.build(len(utils.labels), configs.img_height, configs.img_width)
# m = icnet.build(3, 512, 512)
m.load_weights("./enet-c-v1-2.h5")
m.summary()

for i in range(len(path)):

    image = utils.load_image(path[i])
    image = np.array(image, dtype=np.uint8)
    start = time.time()
    im_mask = m.predict(np.array([image]))[0]

    im_mask = utils.convert_class_to_rgb(im_mask)
    img_pred = cv2.addWeighted(im_mask, 0.8, image, 0.8, 0)
    img_pred = cv2.cvtColor(img_pred, cv2.COLOR_RGB2BGR)
    img_pred = cv2.resize(img_pred, (configs.img_width, configs.img_height))

    end = time.time()
    print(end - start)
    cv2.imwrite("./result3_{}.png".format(i), img_pred)

