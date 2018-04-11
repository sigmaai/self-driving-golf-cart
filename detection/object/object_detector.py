#! /usr/bin/env python3

# MIT License

# Copyright (c) 2017 Yongyang Nie

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Run a YOLO_v2 style detection model test images."""


import colorsys
import cv2
import os
import random

import numpy as np
from keras import backend as K
from keras.models import load_model
from PIL import Image, ImageDraw, ImageFont
import detection.object.model_data.configs as configs
from detection.object.yad2k.models.keras_yolo import yolo_eval, yolo_head


class ObjectDetector:

    def __init__(self):

        model_path = os.path.expanduser(configs.model_path)
        assert model_path.endswith('.h5'), 'Keras model must be a .h5 file.'

        self.sess = K.get_session()

        with open(configs.classes_path) as f:
            class_names = f.readlines()
        self.class_names = [c.strip() for c in class_names]

        with open(configs.anchors_path) as f:
            anchors = f.readline()
            anchors = [float(x) for x in anchors.split(',')]
            anchors = np.array(anchors).reshape(-1, 2)

        self.yolo_model = load_model(model_path)
        print(self.yolo_model.summary())
        self.model_image_size = self.yolo_model.layers[0].input_shape[1:3]
        print('{} model, anchors, and classes loaded.'.format(model_path))

        # Generate colors for drawing bounding boxes.
        hsv_tuples = [(x / len(class_names), 1., 1.)
                      for x in range(len(class_names))]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(
            map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)),
                self.colors))
        random.seed(10101)  # Fixed seed for consistent colors across runs.
        random.shuffle(self.colors)  # Shuffle colors to decorrelate adjacent classes.
        random.seed(None)  # Reset seed to default.

        # Generate output tensor targets for filtered bounding boxes.
        yolo_outputs = yolo_head(self.yolo_model.output, anchors, len(class_names))
        self.input_image_shape = K.placeholder(shape=(2,))
        self.boxes, self.scores, self.classes = yolo_eval(
            yolo_outputs,
            self.input_image_shape,
            score_threshold=configs.score_threshold,
            iou_threshold=configs.iou_threshold)

        # Graphics of stuff
        self.font = ImageFont.truetype(font='./detection/object/font/FiraMono-Medium.otf',
                                       size=np.floor(3e-2 * configs.height + 0.5).astype('int32'))
        self.thickness = (configs.width + configs.height) // 300

    def detect_object(self, image, visualize=False):

        image = Image.fromarray(cv2.resize(image, (configs.width, configs.height)))

        resized_image = image.resize(
            tuple(reversed(self.model_image_size)), Image.BICUBIC)
        image_data = np.array(resized_image, dtype='float32')

        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.
        print(image_data.shape)

        out_boxes, out_scores, out_classes = self.sess.run(
            [self.boxes, self.scores, self.classes],
            feed_dict={
                self.yolo_model.input: image_data,
                self.input_image_shape: [image.size[1], image.size[0]],
                K.learning_phase(): 0
            })

        array = np.uint8((image))
        image = Image.fromarray(array)
        image = self.draw_bboxes(image, b_boxes=out_boxes, scores=out_scores, classes=out_classes)

        if visualize:
            return out_boxes, out_scores, out_classes
        else:
            return np.array(image)

    def draw_bboxes(self, image, b_boxes, scores, classes):

        # draw the bounding boxes
        for i, c in reversed(list(enumerate(classes))):

            predicted_class = self.class_names[c]
            box = b_boxes[i]
            score = scores[i]

            label = '{} {:.2f}'.format(predicted_class, score)
            draw = ImageDraw.Draw(image)
            label_size = draw.textsize(label, self.font)

            top, left, bottom, right = box
            top = max(0, np.floor(top + 0.5).astype('int32'))
            left = max(0, np.floor(left + 0.5).astype('int32'))
            bottom = min(image.size[1], np.floor(bottom + 0.5).astype('int32'))
            right = min(image.size[0], np.floor(right + 0.5).astype('int32'))

            if top - label_size[1] >= 0:
                text_origin = np.array([left, top - label_size[1]])
            else:
                text_origin = np.array([left, top + 1])

            # a good redistributable image drawing library.
            for i in range(self.thickness):
                draw.rectangle([left + i, top + i, right - i, bottom - i], outline=self.colors[c])
            draw.rectangle([tuple(text_origin), tuple(text_origin + label_size)], fill=self.colors[c])
            draw.text(text_origin, label, fill=(0, 0, 0), font=self.font)
            del draw

        return image