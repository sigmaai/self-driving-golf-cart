'''
Dataset visualization tool
Original By: Comma.ai and Chris Gundling
Revised and used by Neil Nie
'''

from __future__ import print_function
import argparse
import numpy as np
import cv2
import pygame
import pandas as pd
from os import path
import matplotlib
import matplotlib.backends.backend_agg as agg
import pylab
from komada_eval import KomandaModel
from models import utils
from models import configs

pygame.init()
size = (320*2, 160*4)
pygame.display.set_caption("self driving data viewer")
screen = pygame.display.set_mode(size, pygame.DOUBLEBUF)
screen.set_alpha(None)

camera_surface = pygame.surface.Surface((640,480),0,24).convert()
clock = pygame.time.Clock()

PI_RAD = (180 / np.pi)
red = (255, 0, 0)
blue = (0, 0, 255)

# ***** get perspective transform for images *****
from skimage import transform as tf

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
    row = row * 2
    col = col * 2
    if row >= 0 and row < img.shape[0] * 2 / 2 and col >= 0 and col < img.shape[1] * 2 / 2:
        img[int(row - sz):int(row + sz), int(col - sz):int(col + sz)] = color


def draw_path(img, path_x, path_y, color):
    for x, y in zip(path_x, path_y):
        draw_pt(img, x, y, color)


# ***** functions to draw predicted path *****

def calc_curvature(v_ego, angle_steers, angle_offset=0):
    deg_to_rad = np.pi / 180.
    slip_fator = 0.0014  # slip factor obtained from real data
    steer_ratio = 15.3  # from http://www.edmunds.com/acura/ilx/2016/road-test-specs/
    wheel_base = 2.67  # from http://www.edmunds.com/acura/ilx/2016/sedan/features-specs/

    angle_steers_rad = (angle_steers - angle_offset)  # * deg_to_rad
    curvature = angle_steers_rad / (steer_ratio * wheel_base * (1. + slip_fator * v_ego ** 2))
    return curvature


def calc_lookahead_offset(v_ego, angle_steers, d_lookahead, angle_offset=0):
    # *** this function returns the lateral offset given the steering angle, speed and the lookahead distance
    curvature = calc_curvature(v_ego, angle_steers, angle_offset)

    # clip is to avoid arcsin NaNs due to too sharp turns
    y_actual = d_lookahead * np.tan(np.arcsin(np.clip(d_lookahead * curvature, -0.999, 0.999)) / 2.)
    return y_actual, curvature


def draw_path_on(img, speed_ms, angle_steers, color=(0, 0, 255)):
    path_x = np.arange(0, 50.1, 0.5)  # 50.1
    path_y, _ = calc_lookahead_offset(speed_ms, angle_steers, path_x)
    draw_path(img, path_x, path_y, color)


def preprocess_img(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return img



# ***** main loop *****
if __name__ == "__main__":

    checkpoint = "/home/neil/Workspace/self-driving-golf-cart/src/steering_control/scripts/jupyter/v3"
    metagraph = "/home/neil/Workspace/self-driving-golf-cart/src/steering_control/scripts/jupyter/v3/checkpoint-sdc-ch2.meta"

    model = KomandaModel(checkpoint_dir=checkpoint, metagraph_file=metagraph)

    # steerings and images
    steering_labels = path.join(configs.VAL_DIR, 'labels.csv')

    # read the steering labels and image path
    df_truth = pd.read_csv(steering_labels, usecols=['frame_id', 'steering_angle'], index_col=None)

    # Create second screen with matplotlibs
    fig = pylab.figure(figsize=[6.4, 1.6], dpi=100)
    ax = fig.gca()
    ax.tick_params(axis='x', labelsize=8)
    ax.tick_params(axis='y', labelsize=8)
    # ax.legend(loc='upper left',fontsize=8)
    line1, = ax.plot([], [], 'b.-', label='Human')
    line2, = ax.plot([], [], 'r.-', label='Model')
    A = []
    B = []
    ax.legend(loc='upper left', fontsize=8)

    myFont = pygame.font.SysFont("monospace", 18)
    randNumLabel = myFont.render('Human Steer Angle:', 1, blue)
    randNumLabel2 = myFont.render('Model Steer Angle:', 1, red)
    speed_ms = 5  # log['speed'][i]

    # Run through all images
    for i in range(len(df_truth)):

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break

        if i%100 == 0:
            print('%.2f seconds elapsed' % (i/20))

        p = configs.VAL_DIR + "center/" + str(df_truth['frame_id'].loc[i]) + ".jpg"

        if path.isfile(p):

            actual_steers = df_truth['steering_angle'].loc[i] # * 0.1 - 8 * 0.0174533  # 1 degree right correction

            img = cv2.cvtColor(cv2.imread(p), cv2.COLOR_BGR2RGB)
            prediction = model.predict(img)

            draw_path_on(img, speed_ms, actual_steers * 0.05)            # human is blue
            draw_path_on(img, speed_ms, prediction * 0.05, (255, 0, 0))  # prediction is red

            A.append(actual_steers)
            B.append(prediction)
            line1.set_ydata(A)
            line1.set_xdata(range(len(A)))
            line2.set_ydata(B)
            line2.set_xdata(range(len(B)))
            ax.relim()
            ax.autoscale_view()

            canvas = agg.FigureCanvasAgg(fig)
            canvas.draw()
            renderer = canvas.get_renderer()
            raw_data = renderer.tostring_rgb()
            size = canvas.get_width_height()
            surf = pygame.image.fromstring(raw_data, size, "RGB")
            screen.blit(surf, (0, 480))

            # draw on
            pygame.surfarray.blit_array(camera_surface, img.swapaxes(0, 1))
            screen.blit(camera_surface, (0, 0))

            diceDisplay = myFont.render(str(round(actual_steers * PI_RAD, 4)), 1, blue)
            diceDisplay2 = myFont.render(str(round(prediction * PI_RAD, 4)), 1, red)
            screen.blit(randNumLabel, (50, 420))
            screen.blit(randNumLabel2, (400, 420))
            screen.blit(diceDisplay, (50, 450))
            screen.blit(diceDisplay2, (400, 450))
            clock.tick(60)
            pygame.display.flip()

