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
import time
from src.steering_control.scripts.weights.training import configs
import keras as K
import utils


size = (512, 512)
starting_index = 100
ds_type = "NN"
steering_labels = None

pygame.init()
pygame.display.set_caption("cruise control")
screen = pygame.display.set_mode(size, pygame.DOUBLEBUF)
screen.set_alpha(None)

camera_surface = pygame.surface.Surface(size, 0, 24).convert()
clock = pygame.time.Clock()


def root_mean_squared_error(y_true, y_pred):
    return K.backend.sqrt(K.backend.mean(K.backend.square(y_pred - y_true), axis=-1))


if __name__ == "__main__":

    model = m.nvidia_network(input_shape=(512, 512, 3))
    model.load_weights(configs.visualization_path)
    model.summary()

    # steerings and images
    if ds_type == "UD":
        steering_labels = utils.preprocess_visualization(configs.ud_data_path)
    if ds_type == "NN":
        labels = pd.read_csv(configs.data_path + "/labels.csv").values

    myFont = pygame.font.SysFont("monospace", 30)
    randNumLabel = myFont.render('predicted:', 1, configs.blue)
    randNumLabel2 = myFont.render('human:', 1, configs.red)

    # Run through all images
    for i in range(starting_index, len(labels)):

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break

        if i % 100 == 0:
            print('%.2f seconds elapsed' % (i / 20))

        if ds_type == "UD":
            path = str(steering_labels[i][5])
            img = utils.bgr_rgb(cv2.imread(path))
            image = np.array([cv2.resize(img, (configs.img_h, configs.img_w))])  # the model expects 4D array
            act_speed = steering_labels[i][8]
        elif ds_type == "NN":
            path = str(labels[i][2])
            img = utils.bgr_rgb(cv2.imread(configs.data_path + path + ".jpeg"))
            image = np.array([cv2.resize(img, (configs.img_h, configs.img_w))])
            act_speed = labels[i][0]

        prd_speed = model.predict(image)[0][0]
        # draw on
        pygame.surfarray.blit_array(camera_surface, image[0].swapaxes(0, 1))
        screen.blit(camera_surface, (0, 0))
        time.sleep(0.2)
        if prd_speed < 1:
            diceDisplay = myFont.render(str(round(prd_speed, 3)) + " stop", 1, configs.blue)
        elif prd_speed < 5:
            diceDisplay = myFont.render(str(round(prd_speed, 3)) + " very slow", 1, configs.blue)
        elif prd_speed >= 5 and prd_speed < 10:
            diceDisplay = myFont.render(str(round(prd_speed, 3)) + " slow", 1, configs.blue)
        elif prd_speed >= 10 and prd_speed < 20:
            diceDisplay = myFont.render(str(round(prd_speed, 3)) + " medium", 1, configs.blue)
        elif prd_speed >= 20 and prd_speed < 40:
            diceDisplay = myFont.render(str(round(prd_speed, 3)) + " fast", 1, configs.blue)
        elif prd_speed >= 40:
            diceDisplay = myFont.render(str(round(prd_speed, 3)) + " very fast", 1, configs.blue)

        if act_speed < 1:
            diceDisplay2 = myFont.render(str(round(act_speed, 3)) + " stop", 1, configs.red)
        elif act_speed < 5:
            diceDisplay2 = myFont.render(str(round(act_speed, 3)) + " very slow", 1, configs.red)
        elif act_speed >= 5 and act_speed < 10:
            diceDisplay2 = myFont.render(str(round(act_speed, 3)) + " slow", 1, configs.red)
        elif act_speed >= 10 and act_speed < 20:
            diceDisplay2 = myFont.render(str(round(act_speed, 3)) + " medium", 1, configs.red)
        elif act_speed >= 20 and act_speed < 40:
            diceDisplay2 = myFont.render(str(round(act_speed, 3)) + " fast", 1, configs.red)
        elif act_speed >= 40:
            diceDisplay2 = myFont.render(str(round(act_speed, 3)) + " very fast", 1, configs.red)

        screen.blit(randNumLabel, (50, 400))
        screen.blit(randNumLabel2, (400, 400))
        screen.blit(diceDisplay, (50, 430))
        screen.blit(diceDisplay2, (400, 430))

        clock.tick(60)
        pygame.display.flip()