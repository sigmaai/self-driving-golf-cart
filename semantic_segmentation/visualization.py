# -----------------------------------
# visualization.py, visualize road segmentation
# on Udacity testing data
# (c) Neil Nie, 2017
# All Rights Reserved.
# -----------------------------------

import models.enet_naive_upsampling.model as enet
import utils
import configs
import os
import matplotlib.pyplot as plt
from keras.optimizers import Adam
import cv2
import pandas as pd
import numpy as np
from keras.metrics import binary_accuracy
from PIL import Image
import pygame


# init pygame
pygame.init()
size = (configs.img_height, configs.img_width)
pygame.display.set_caption("road segmentation")
screen = pygame.display.set_mode(size, pygame.DOUBLEBUF)
screen.set_alpha(None)

camera_surface = pygame.surface.Surface((configs.img_height, configs.img_width), 0, 24).convert()
clock = pygame.time.Clock()


def test_video_stream():

    # init model
    m = enet.build(len(utils.labels), configs.img_height, configs.img_width)
    # m = icnet.build(3, 512, 512)
    m.load_weights("./new-enet-coarse-3.h5")
    m.summary()

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
        image = cv2.resize(image, (configs.img_height, configs.img_width))
        output = m.predict(np.array([image]))[0]
        im_mask = utils.convert_class_to_rgb(output)
        img_pred = cv2.addWeighted(im_mask, 0.8, image, 0.8, 0)

        # show it in pygame
        # -----------------
        pygame.surfarray.blit_array(camera_surface, img_pred.swapaxes(0, 1))
        screen.blit(camera_surface, (0, 0))
        pygame.display.flip()


def test_images():

    # init model
    model = m.segnet(nb_classes=2, input_height=configs.img_height, input_width=configs.img_width)
    model.load_weights("./segmentation-train-3.h5")
    model.compile(optimizer=Adam(lr=1e-4), loss="binary_crossentropy", metrics=[binary_accuracy])

    # generator
    train_generator = utils.gen_batch_function(os.path.join(configs.data_path, 'data_road/training'),
                                               (configs.img_height, configs.img_width), 8)

    images, targets = next(train_generator)

    for i in range(8):
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
        plt.imshow(im_prediction[:, :, 1])
        plt.axis('off')
        plt.show()

if __name__ == '__main__':

    # test_images()

    test_video_stream()



