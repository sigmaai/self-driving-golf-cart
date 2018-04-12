'''
Results video generator Udacity Challenge 2
Original By: Comma.ai and Chris Gundling
Revised and used by Neil Nie
'''

from __future__ import print_function
import numpy as np
import cv2
from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw
import matplotlib
from skimage import transform as tf

from keras.models import *
from keras.callbacks import *
import keras.backend as K
from steering.visualization.model import *
from steering.visualization.data import *
import cv2
import argparse


matplotlib.use("Agg")

# ***** get perspective transform for images *****


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
    if row >= 0 and row < img.shape[0] * 2 / 2 and \
                    col >= 0 and col < img.shape[1] * 2 / 2:
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

# main methods --

# TODO: Test this method!
def visualize_line(img, speed_ms, angle_steers, color=(0, 0, 255)):

    path_x = np.arange(0, 50.1, 0.5)
    path_y, _ = calc_lookahead_offset(speed_ms, angle_steers, path_x)
    draw_path(img, path_x, path_y, color)

    return img


def visualize_steering_wheel(image, angle):

    background = Image.fromarray(np.uint8(image))
    sw = Image.open("./steering/resources/sw.png")
    sw = sw.rotate(angle * 180 / np.pi)
    sw = sw.resize((80, 80), Image.ANTIALIAS)
    background.paste(sw, (10, 10), sw)

    draw = ImageDraw.Draw(background)
    font = ImageFont.truetype("./steering/resources/FiraMono-Medium.otf", 16)
    draw.text((80, 200), str(round(angle, 3)), (255, 255, 255), font=font)
    steering_img = cv2.resize(np.array(background), (640, 480))
    return steering_img


# TODO: Actually test this method
def visualize_class_activation_map(model, image):

    original_img = image
    width, height, _ = original_img.shape

    # Reshape to the network input shape (3, w, h).
    img = np.array([np.transpose(np.float32(original_img), (2, 0, 1))])

    # Get the 512 input weights to the softmax.
    class_weights = model.layers[-1].get_weights()[0]
    final_conv_layer = get_output_layer(model, "conv5_3")
    get_output = K.function([model.layers[0].input], [final_conv_layer.output, model.layers[-1].output])
    [conv_outputs, predictions] = get_output([img])
    conv_outputs = conv_outputs[0, :, :, :]

    # Create the class activation map.
    cam = np.zeros(dtype = np.float32, shape = conv_outputs.shape[1:3])
    for i, w in enumerate(class_weights[:, 1]):
        cam += w * conv_outputs[i, :, :]
    print("predictions", predictions)
    cam /= np.max(cam)
    cam = cv2.resize(cam, (height, width))
    heatmap = cv2.applyColorMap(np.uint8(255 * cam), cv2.COLORMAP_JET)
    heatmap[np.where(cam < 0.2)] = 0
    img = heatmap * 0.5 + original_img

    return img
