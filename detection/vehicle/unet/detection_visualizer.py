'''
Vehicle detection and lane marking
Original By: Comma.ai and Chris Gundling
Revised and used by Neil Nie
'''

from __future__ import print_function
import argparse
import sys
import numpy as np
import cv2
import pygame
import json
import pandas as pd
from os import path
from keras.models import load_model
import matplotlib
import matplotlib.backends.backend_agg as agg
import pylab
import model as m
import util
import img_util

pygame.init()
size = (960, 640)
pygame.display.set_caption("detection & lane marking viewer")
screen = pygame.display.set_mode(size, pygame.DOUBLEBUF)
screen.set_alpha(None)

camera_surface = pygame.surface.Surface((960, 640), 0, 24).convert()
clock = pygame.time.Clock()


# ***** main loop *****
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Path viewer')
    parser.add_argument('--model', type=str, help='path to the trained model')
    parser.add_argument('--dataset', type=str, help='dataset folder with csv and image folders')
    parser.add_argument('--camera', type=str, default='center', help='camera to use, default is center')
    args = parser.parse_args()

    dataset_path = args.dataset
    camera = args.camera
    fcn = m.fcn_model()
    fcn.load_weights(args.model)

    # steerings and images
    steering_labels = path.join(dataset_path, 'interpolated.csv')
    df_truth = pd.read_csv(steering_labels, usecols=['frame_id', 'steering_angle'], index_col=None)
    speed_ms = 5

    red = (255, 0, 0)
    blue = (0, 0, 255)
    myFont = pygame.font.SysFont("monospace", 18)
    randNumLabel = myFont.render('Human Steer Angle:', 1, blue)
    randNumLabel2 = myFont.render('Model Steer Angle:', 1, red)

    print("begin program")
    print(len(df_truth))
    # Run through all images
    for i in range(len(df_truth)):

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break

        if i % 100 == 0:
            print('%.2f seconds elapsed' % (i / 20))

        path = dataset_path + "center/" + str(df_truth["frame_id"][i]) + ".jpg"
        img = img_util.get_image_path(path)
        img_input = np.array(img)
        bbox_img, seg_img = util.run_predictions(img_input, fcn)

        # draw on
        pygame.surfarray.blit_array(camera_surface, bbox_img.swapaxes(0, 1))
        screen.blit(camera_surface, (0, 0))

        diceDisplay = myFont.render("segmentation", 1, blue)
        diceDisplay2 = myFont.render("bonding box", 1, red)
        screen.blit(diceDisplay, (50, 450))
        screen.blit(diceDisplay2, (400, 450))
        clock.tick(60)
        pygame.display.flip()
