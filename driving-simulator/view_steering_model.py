#!/usr/bin/env python
import cv2
import numpy as np
import h5py
import pygame
import pandas as pd
from keras.models import load_model
import time

# pygame.init()
# size = (640, 480)
# pygame.display.set_caption("comma.ai data viewer")
# screen = pygame.display.set_mode(size, pygame.DOUBLEBUF)
#
# camera_surface = pygame.surface.Surface((640, 480),0,24).convert()

# ***** get perspective transform for images *****
from skimage import transform as tf

pygame.init()
size = (640, 480)
pygame.display.set_caption("self driving data viewer")
screen = pygame.display.set_mode(size)
camera_surface = pygame.surface.Surface((640, 480), 0, 24)

clock = pygame.time.Clock()

rsrc = \
    [[43.45456230828867, 118.00743250075844],
     [104.5055617352614, 69.46865203761757],
     [114.86050156739812, 60.83953551083698],
     [129.74572757609468, 50.48459567870026],
     [132.98164627363735, 46.38576532847949],
     [301.0336906326895, 98.16046448916306],
     [238.25686790036065, 62.56535881619311],
     [227.2547443287154, 56.30924933427718],
     [209.13359962247614, 46.817221154818526],
     [203.9561297064078, 43.5813024572758]]
rdst = \
    [[10.822125594094452, 1.42189132706374],
     [21.177065426231174, 1.5297552836484982],
     [25.275895776451954, 1.42189132706374],
     [36.062291434927694, 1.6376192402332563],
     [40.376849698318004, 1.42189132706374],
     [11.900765159942026, -2.1376192402332563],
     [22.25570499207874, -2.1376192402332563],
     [26.785991168638553, -2.029755283648498],
     [37.033067044190524, -2.029755283648498],
     [41.67121717733509, -2.029755283648498]]

tform3_img = tf.ProjectiveTransform()
tform3_img.estimate(np.array(rdst), np.array(rsrc))


def perspective_tform(x, y):

    p1, p2 = tform3_img((x, y))[0]
    return p2, p1

# ***** functions to draw lines *****


def draw_pt(img, x, y, color, sz=2):

    row, col = perspective_tform(x, y)

    if row >= 0 and row < img.shape[0] and \
                    col >= 0 and col < img.shape[1]:
        img[int(row-sz):int(row+sz), int(col-sz):int(col+sz)] = color

def draw_path(img, path_x, path_y, color):
    for x, y in zip(path_x, path_y):
        draw_pt(img, x, y, color)

# ***** functions to draw predicted path *****


def calc_curvature(v_ego, angle_steers, angle_offset=0):
    deg_to_rad = np.pi/180.
    slip_fator = 0.0014 # slip factor obtained from real data
    steer_ratio = 15.3  # from http://www.edmunds.com/acura/ilx/2016/road-test-specs/
    wheel_base = 2.67   # from http://www.edmunds.com/acura/ilx/2016/sedan/features-specs/

    angle_steers_rad = (angle_steers - angle_offset) * deg_to_rad
    curvature = angle_steers_rad/(steer_ratio * wheel_base * (1. + slip_fator * v_ego**2))
    return curvature


def calc_lookahead_offset(v_ego, angle_steers, d_lookahead, angle_offset=0):

    # this function returns the lateral offset given the steering angle, speed and the lookahead distance
    curvature = calc_curvature(v_ego, angle_steers, angle_offset)

    # clip is to avoid arcsin NaNs due to too sharp turns
    y_actual = d_lookahead * np.tan(np.arcsin(np.clip(d_lookahead * curvature, -0.999, 0.999))/2.)
    return y_actual, curvature


def draw_path_on(img, speed_ms, angle_steers, color=(0,0,255)):

    path_x = np.arange(0., 50.1, 0.5)
    path_y, _ = calc_lookahead_offset(speed_ms, angle_steers, path_x)
    draw_path(img, path_x, path_y, color)

# ***** main loop *****


if __name__ == "__main__":

    data_path = "/Volumes/Personal_Drive/Datasets/Udacity_Self-Driving-Car/udacity-driving-testing-ds/"
    model_path = "/Users/yongyangnie/Documents/Developer/ALVNS/driving-simulator/trained3-v1.h5"

    # load model
    model = load_model(model_path)

    model.compile("sgd", "mse")
    model.summary()

    # default dataset is the validation data on the highway
    log = pd.read_csv(data_path + "interpolated.csv")
    print(len(log))
    skip = 0

    for i in range(skip, len(log)):

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break

        if i % 3 == 0:
            if i % 100 == 0:
                print("%.2f seconds elapsed" % (i / 100.0))

            file_name = log.iloc[i]["filename"]
            img = cv2.imread(data_path + file_name)

            b, g, r = cv2.split(img)  # get b,g,r
            img = cv2.merge([r, g, b])  # switch it to rgb
            image = np.array([img])  # the model expects 4D array

            predicted_steers = model.predict(image)
            angle_steers = log['angle'][i]
            speed = log["speed"][i]

            draw_path_on(img, speed, angle_steers*50)
            draw_path_on(img, speed, predicted_steers[0][0]*50, (0, 255, 0))

            myfont = pygame.font.SysFont("monospace", 18)

            # render text
            label1 = myfont.render("Blue: Label", 1, (0, 0, 255))
            label2 = myfont.render("Green: Prediction", 1, (0, 255, 0))

            # Display
            pygame.surfarray.blit_array(camera_surface, img.swapaxes(0, 1))
            screen.blit(camera_surface, (0, 0))
            screen.blit(label1, (150, 400))
            screen.blit(label2, (350, 400))
            pygame.display.flip()
            clock.tick(60)