import cv2
import models.enet_naive_upsampling.model as enet
import models.icnet.model as icnet
import numpy as np
import utils
import configs
import matplotlib.pyplot as plt
import time

#
# path = ["./testing_imgs/test_1.jpg",
#         "./testing_imgs/test_2.jpg",
#         "./testing_imgs/test_3.jpg",
#         "./testing_imgs/test_4.jpg",
#         "./testing_imgs/test_5.jpg",
#         "./testing_imgs/test_6.jpg",
#         "./testing_imgs/test_7.jpg"]
# path = ["./testing_imgs/sd-1.jpg",
#         "./testing_imgs/sd-2.jpg",
#         "./testing_imgs/sd-3.jpg"]
# path = ["./testing_imgs/sn-1.jpg",
#         "./testing_imgs/sn-2.jpg",
#         "./testing_imgs/sn-3.jpg"]
# path = ["./testing_imgs/da-1.jpg",
#         "./testing_imgs/da-2.jpg",
#         "./testing_imgs/da-3.jpg",
#         "./testing_imgs/da-4.jpg"]
# path = ["./testing_imgs/start-1.jpg",
#         "./testing_imgs/stop-2.jpeg",
#         "./testing_imgs/stop3.jpeg"]

m = enet.build(len(utils.labels), configs.img_height, configs.img_width)
m.load_weights("./enet-c-v1-3.h5")
m.summary()

for i in range(len(path)):

    org = cv2.imread(path[i])
    org = cv2.cvtColor(org, cv2.COLOR_RGB2BGR)
    print(org.shape)
    image = utils.load_image(path[i])
    image = np.array(image, dtype=np.uint8)
    start = time.time()
    im_mask = m.predict(np.array([image]))[0]

    im_mask = utils.convert_class_to_rgb(im_mask)
    im_mask = cv2.resize(im_mask, (org.shape[1], org.shape[0]))
    img_pred = cv2.addWeighted(im_mask, 0.8, org, 0.8, 0)
    img_pred = cv2.cvtColor(img_pred, cv2.COLOR_RGB2BGR)
    # img_pred = cv2.resize(img_pred, (configs.img_width, configs.img_height))

    end = time.time()
    print(end - start)
    cv2.imwrite("./result3_{}.png".format(i), img_pred)

