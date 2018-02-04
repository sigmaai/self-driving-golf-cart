import cv2
import configs
import numpy as np
import model as m
import matplotlib.pyplot as plt

path = ["./testing_imgs/test_0.jpeg",
        "./testing_imgs/test_1.jpeg",
        "./testing_imgs/test_2.jpeg",
        "./testing_imgs/test_3.jpeg",
        "./testing_imgs/test_4.jpeg",
        "./testing_imgs/test_5.jpeg",
        "./testing_imgs/test_6.jpeg"]

model = m.fcn_model()
model.load_weights("./segmentation-fcn-3.h5")
model.summary()

for i in range(len(path)):

    image = cv2.imread(path[i])
    image = cv2.resize(image, (640, 480))
    image = np.array(image, dtype=np.uint8)

    im_mask = model.predict(np.array([image]))[0][:, :, 1]
    im_mask = np.array(255 * im_mask, dtype=np.uint8)
    im_mask = cv2.cvtColor(im_mask, cv2.COLOR_GRAY2RGB)
    ret, mask = cv2.threshold(im_mask, 0, 255, cv2.THRESH_BINARY)
    mask[:, :, 1:3] = 0 * mask[:, :, 1:3]

    img_pred = cv2.addWeighted(mask, 1.0, image, 1.0, 0)

    plt.imshow(im_mask)
    plt.show()
