#! /usr/bin/env python
"""Run a YOLO_v2 style detection model on test images."""
import colorsys
import cv2
import os
import random
import pandas as pd

import numpy as np
from keras import backend as K
from keras.models import load_model
from PIL import Image, ImageDraw, ImageFont

import configs.configs as configs
from yad2k.models.keras_yolo import yolo_eval, yolo_head
import pygame

pygame.init()
size = (640, 480)
pygame.display.set_caption("detection visualizer")
screen = pygame.display.set_mode(size, pygame.DOUBLEBUF)
screen.set_alpha(None)

camera_surface = pygame.surface.Surface((640,480),0,24).convert()
clock = pygame.time.Clock()


def bgr_rgb(img):
    b, g, r = cv2.split(img)  # get b,g,r
    img = cv2.merge([r, g, b])  # switch it to rgb
    return img


def resize_image(img, w, h):
    return cv2.resize(bgr_rgb(img), w, h)


if __name__ == '__main__':

    # -------------------
    # setup dataset stuff
    # -------------------

    steering_labels = configs.test_dataset + 'interpolated.csv'
    df_truth = pd.read_csv(steering_labels, usecols=['frame_id', 'steering_angle'], index_col=None)

    # -----------------
    # set up yolo stuff
    # -----------------

    sess = K.get_session()  # TODO: Remove dependence on Tensorflow session.

    with open(configs.classes_path) as f:
        class_names = f.readlines()
    class_names = [c.strip() for c in class_names]

    with open(configs.anchors_path) as f:
        anchors = f.readline()
        anchors = [float(x) for x in anchors.split(',')]
        anchors = np.array(anchors).reshape(-1, 2)

    yolo_model = load_model(configs.model_path)

    # Verify model, anchors, and classes are compatible
    num_classes = len(class_names)
    num_anchors = len(anchors)
    model_output_channels = yolo_model.layers[-1].output_shape[-1]

    # Check if model is fully convolutional, assuming channel last order.
    model_image_size = yolo_model.layers[0].input_shape[1:3]
    is_fixed_size = model_image_size != (None, None)

    # Generate colors for drawing bounding boxes.
    hsv_tuples = [(x / len(class_names), 1., 1.)
                  for x in range(len(class_names))]
    colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
    colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), colors))
    random.seed(10101)  # Fixed seed for consistent colors across runs.
    random.shuffle(colors)  # Shuffle colors to decorrelate adjacent classes.
    random.seed(None)  # Reset seed to default.

    # Generate output tensor targets for filtered bounding boxes.
    # TODO: Wrap these backend operations with Keras layers.
    yolo_outputs = yolo_head(yolo_model.output, anchors, len(class_names))
    input_image_shape = K.placeholder(shape=(2, ))
    boxes, scores, classes = yolo_eval(yolo_outputs, input_image_shape, score_threshold=configs.score_threshold, iou_threshold=configs.iou_threshold)

    # Go through images and make predictions

    for i in range(len(df_truth)):

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break

        path = configs.test_dataset + "center/" + str(df_truth["frame_id"][i]) + ".jpg"

        image = Image.open(path)
        resized_image = image.resize((608, 608), Image.BICUBIC)
        image_data = np.array(resized_image, dtype='float32')

        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

        # making predictions
        out_boxes, out_scores, out_classes = sess.run(
            [boxes, scores, classes],
            feed_dict={
                yolo_model.input: image_data,
                input_image_shape: [image.size[1], image.size[0]],
                K.learning_phase(): 0
            })

        font = ImageFont.truetype(font='font/FiraMono-Medium.otf', size=np.floor(3e-2 * image.size[1] + 0.5).astype('int32'))
        thickness = (image.size[0] + image.size[1]) // 300

        # draw the bounding boxes
        for i, c in reversed(list(enumerate(out_classes))):
            predicted_class = class_names[c]
            box = out_boxes[i]
            score = out_scores[i]

            label = '{} {:.2f}'.format(predicted_class, score)
            draw = ImageDraw.Draw(image)
            label_size = draw.textsize(label, font)

            top, left, bottom, right = box
            top = max(0, np.floor(top + 0.5).astype('int32'))
            left = max(0, np.floor(left + 0.5).astype('int32'))
            bottom = min(image.size[1], np.floor(bottom + 0.5).astype('int32'))
            right = min(image.size[0], np.floor(right + 0.5).astype('int32'))

            if top - label_size[1] >= 0:
                text_origin = np.array([left, top - label_size[1]])
            else:
                text_origin = np.array([left, top + 1])

            # My kingdom for a good redistributable image drawing library.
            for i in range(thickness):
                draw.rectangle([left + i, top + i, right - i, bottom - i], outline=colors[c])
            draw.rectangle([tuple(text_origin), tuple(text_origin + label_size)], fill=colors[c])
            draw.text(text_origin, label, fill=(0, 0, 0), font=font)
            del draw

        # show it in pygame
        # -----------------
        pygame.surfarray.blit_array(camera_surface, np.array(image).swapaxes(0, 1))
        screen.blit(camera_surface, (0, 0))
        pygame.display.flip()

    sess.close()
