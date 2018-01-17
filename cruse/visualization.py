'''
Results video generator Udacity Challenge 2
Original By: Comma.ai and Chris Gundling
Revised and used by Neil Nie
'''

import numpy as np
import cv2
import pygame
import pandas as pd
import model as m
from DenseNet import densenet
from keras.models import load_model
import configs
from keras.optimizers import Adam
import keras as K
import utils


size = (640, 480)
starting_index = 4000
ds_type = "UD"
steering_labels = None

pygame.init()
pygame.display.set_caption("throttle control")
screen = pygame.display.set_mode(size, pygame.DOUBLEBUF)
screen.set_alpha(None)

camera_surface = pygame.surface.Surface(size, 0, 24).convert()
clock = pygame.time.Clock()


def root_mean_squared_error(y_true, y_pred):
    return K.backend.sqrt(K.backend.mean(K.backend.square(y_pred - y_true), axis=-1))


if __name__ == "__main__":

    input = (512, 512, 3)
    model = m.nvidia_network(input_shape=input)
    # model = densenet.DenseNet(classes=1, input_shape=input, depth=28, growth_rate=12, bottleneck=True, reduction=0.5)
    adam = Adam(lr=1e-4)
    model.compile(optimizer=adam, loss=root_mean_squared_error)
    model.load_weights(configs.model_path)
    model.summary()

    # steerings and images
    if ds_type == "UD":
        steering_labels = utils.preprocess_visualization(configs.ud_data_path)
    if ds_type == "SF":
        steering_labels = None

    myFont = pygame.font.SysFont("monospace", 26)
    randNumLabel = myFont.render('predicted:', 1, configs.blue)
    randNumLabel2 = myFont.render('human:', 1, configs.red)

    # Run through all images
    for i in range(starting_index, len(steering_labels)):

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break

        if i % 100 == 0:
            print('%.2f seconds elapsed' % (i / 20))

        path = str(steering_labels[i][5])
        img = utils.bgr_rgb(cv2.imread(path))
        image = np.array([cv2.resize(img, (configs.img_h, configs.img_w))])  # the model expects 4D array

        prd_speed = model.predict(image)[0][0]
        act_speed = steering_labels[i][8]

        # draw on
        pygame.surfarray.blit_array(camera_surface, img.swapaxes(0, 1))
        screen.blit(camera_surface, (0, 0))

        if prd_speed < 5:
            diceDisplay = myFont.render(str(round(prd_speed, 3)) + " very slow", 1, configs.blue)
        elif prd_speed >= 5 and prd_speed < 10:
            diceDisplay = myFont.render(str(round(prd_speed, 3)) + " slow", 1, configs.blue)
        elif prd_speed >= 10 and prd_speed < 20:
            diceDisplay = myFont.render(str(round(prd_speed, 3)) + " medium", 1, configs.blue)
        elif prd_speed >= 20 and prd_speed < 40:
            diceDisplay = myFont.render(str(round(prd_speed, 3)) + " fast", 1, configs.blue)
        elif prd_speed >= 40:
            diceDisplay = myFont.render(str(round(prd_speed, 3)) + " very fast", 1, configs.blue)

        if act_speed < 5:
            diceDisplay2 = myFont.render(str(round(act_speed, 3)) + " very slow", 1, configs.red)
        elif act_speed >= 5 and act_speed < 10:
            diceDisplay2 = myFont.render(str(round(act_speed, 3)) + " slow", 1, configs.red)
        elif act_speed >= 10 and act_speed < 20:
            diceDisplay2 = myFont.render(str(round(act_speed, 3)) + " medium", 1, configs.red)
        elif act_speed >= 20 and act_speed < 40:
            diceDisplay2 = myFont.render(str(round(act_speed, 3)) + " fast", 1, configs.red)
        elif act_speed >= 40:
            diceDisplay2 = myFont.render(str(round(act_speed, 3)) + " very fast", 1, configs.red)

        screen.blit(randNumLabel, (50, 420))
        screen.blit(randNumLabel2, (400, 420))
        screen.blit(diceDisplay, (50, 450))
        screen.blit(diceDisplay2, (400, 450))
        clock.tick(60)
        pygame.display.flip()