import model as m
import configs
import utils
import numpy as np
import os
from keras.optimizers import Adam
import cv2
import pandas as pd
import numpy as np
from keras.metrics import binary_accuracy
from PIL import Image, ImageDraw, ImageFont
import pygame


def bgr_rgb(img):
    b, g, r = cv2.split(img)  # get b,g,r
    img = cv2.merge([r, g, b])  # switch it to rgb
    return img


def resize_image(img, w, h):
    return cv2.resize(bgr_rgb(img), w, h)


if __name__ == '__main__':

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

        if i%100:
            print("one sec")
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break

        path = configs.test_dataset + "center/" + str(df_truth["frame_id"][i]) + ".jpg"

        image = Image.open(path)
        image = np.array(image, dtype='float32')
        print(image.shape)
        im_mask = model.predict(np.array([image]))[0]
        print(im_mask.shape)
        # show it in pygame
        # -----------------
        pygame.surfarray.blit_array(camera_surface, np.array(image).swapaxes(0, 1))
        screen.blit(camera_surface, (0, 0))
        pygame.display.flip()
