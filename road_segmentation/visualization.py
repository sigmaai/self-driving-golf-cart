# -----------------------------------
# visualization.py, visualize road segmentation
# on Udacity testing data
# (c) Neil Nie, 2017
# All Rights Reserved.
# -----------------------------------

import model as m
import configs
import numpy as np
from keras.optimizers import Adam
import cv2
import pandas as pd
import numpy as np
from keras.metrics import binary_accuracy
from PIL import Image
import pygame


# init pygame
pygame.init()
size = (640, 480)
pygame.display.set_caption("road segmentation")
screen = pygame.display.set_mode(size, pygame.DOUBLEBUF)
screen.set_alpha(None)

camera_surface = pygame.surface.Surface((640, 480), 0, 24).convert()
clock = pygame.time.Clock()

# init model
model = m.fcn_model()
model.load_weights("./segmentation-train-1.h5")
model.compile(optimizer=Adam(lr=1e-4), loss="binary_crossentropy", metrics=[binary_accuracy])

# load testing data
steering_labels = configs.test_dataset + 'interpolated.csv'
df_truth = pd.read_csv(steering_labels, usecols=['frame_id', 'steering_angle'], index_col=None)

print("loaded dataset", len(df_truth))
for i in range(len(df_truth)):

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            break

    path = configs.test_dataset + "center/" + str(df_truth["frame_id"][i]) + ".jpg"

    image = Image.open(path)
    image = np.array(image, dtype=np.uint8)

    im_mask = model.predict(np.array([image]))[0][:, :, 0]
    im_mask = np.array(255 * im_mask, dtype=np.uint8)
    im_mask = cv2.cvtColor(im_mask, cv2.COLOR_GRAY2RGB)
    im_mask[:, :, 1:3] = 0 * im_mask[:, :, 1:2]

    # im_mask = np.expand_dims(im_mask, axis=2)
    # im_mask = np.tile(im_mask, 3)
    # print(im_mask.shape)
    img_pred = cv2.addWeighted(im_mask, 0.9, image, 0.2, 0)

    # im_mask = cv2.flip(im_mask, 0)
    # final_im = cv2.bitwise_and(image, image, mask=im_mask)

    # show it in pygame
    # -----------------
    pygame.surfarray.blit_array(camera_surface, img_pred.swapaxes(0, 1))
    screen.blit(camera_surface, (0, 0))
    pygame.display.flip()



