import cv2
import models.enet_naive_upsampling.model as enet
import models.icnet.model as icnet
import numpy as np
import utils
import configs
import matplotlib.pyplot as plt

path = ["./testing_imgs/test_0.jpg",
        "./testing_imgs/test_1.jpg",
        "./testing_imgs/test_2.jpg"
        # "./testing_imgs/test_3.jpeg",
        # "./testing_imgs/test_4.jpeg",
        # "./testing_imgs/test_5.jpeg",
        # "./testing_imgs/test_6.jpeg"
        ]

m = enet.build(len(utils.labels), configs.img_height, configs.img_width)
# m = icnet.build(3, 512, 512)
m.load_weights("./enet-v1-2.h5")
m.summary()

for i in range(len(path)):

    image = cv2.imread(path[i])
    image = cv2.resize(image, (512, 512))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = np.array(image, dtype=np.uint8)

    im_mask = m.predict(np.array([image]))[0]
    print(im_mask)
    # img_pred = cv2.addWeighted(im_mask, 1.0, image, 1.0, 0)
    plt.imshow(im_mask)
    plt.show()
