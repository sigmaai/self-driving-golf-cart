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

"""
Run a YOLO_v3 style detection model test images.
The ros node calls (uses) this class to run object detection
for questions, contact: contact@neilnie.com

"""
import colorsys
import cv2
import os
import model_data.configs as configs
import numpy as np
from keras import backend as K
from keras.models import load_model
from keras.layers import Input
from PIL import Image, ImageDraw, ImageFont
from yolo3.model import yolo_eval, yolo_body, tiny_yolo_body


class ObjectDetector:

    def __init__(self, model_path, classes_path, anchors_path, score_threshold, iou_threshold, size):

        """
        :param model_path:
        :param classes_path:
        :param anchors_path:
        :param score_threshold:
        :param iou_threshold:
        :param size

        """
        self.model_path = model_path  # rospy.get_param("/object_detection/model_path")
        self.classes_path = classes_path  # rospy.get_param("/object_detection/classes_path")
        self.anchors_path = anchors_path  # rospy.get_param("/object_detection/anchors_path")
        self.iou = iou_threshold  # rospy.get_param("/object_detection/")
        self.score = score_threshold  # rospy.get_param("/object_detection/")
        self.model_image_size = size  # rospy.get_param("/object_detection/")

        self.font = ImageFont.truetype(font=configs.font_path, size=np.floor(3e-2 * size[1] + 0.5).astype('int32'))
        self.thickness = (size[0] + size[1]) // 300

        self.class_names = self._get_class()
        self.anchors = self._get_anchors()
        self.sess = K.get_session()

        # Load model, or construct model and load weights.
        num_anchors = len(self.anchors)
        num_classes = len(self.class_names)
        is_tiny_version = num_anchors==6 # default setting
        try:
            self.yolo_model = load_model(self.model_path, compile=False)
        except:
            self.yolo_model = tiny_yolo_body(Input(shape=(None,None,3)), num_anchors//2, num_classes) \
                if is_tiny_version else yolo_body(Input(shape=(None,None,3)), num_anchors//3, num_classes)
            self.yolo_model.load_weights(self.model_path) # make sure model, anchors and classes match
        else:
            assert self.yolo_model.layers[-1].output_shape[-1] == \
                num_anchors/len(self.yolo_model.output) * (num_classes + 5), \
                'Mismatch between model and given anchor and class sizes'

        print('{} model, anchors, and classes loaded.'.format(self.model_path))

        # Generate colors for drawing bounding boxes.
        hsv_tuples = [(x / len(self.class_names), 1., 1.)
                      for x in range(len(self.class_names))]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), self.colors))
        np.random.seed(10101)  # Fixed seed for consistent colors across runs.
        np.random.shuffle(self.colors)  # Shuffle colors to decorrelate adjacent classes.
        np.random.seed(None)  # Reset seed to default.

        # Generate output tensor targets for filtered bounding boxes.
        self.input_image_shape = K.placeholder(shape=(2, ))
        self.boxes, self.scores, self.classes = yolo_eval(self.yolo_model.output, self.anchors,
                                           len(self.class_names), self.input_image_shape,
                                           score_threshold=self.score, iou_threshold=self.iou)

    def _get_class(self):
        classes_path = os.path.expanduser(self.classes_path)
        with open(classes_path) as f:
            class_names = f.readlines()
        class_names = [c.strip() for c in class_names]
        return class_names

    def _get_anchors(self):
        anchors_path = os.path.expanduser(self.anchors_path)
        with open(anchors_path) as f:
            anchors = f.readline()
        anchors = [float(x) for x in anchors.split(',')]
        return np.array(anchors).reshape(-1, 2)

    def detect_object(self, image, visualize=True):

        """
        :param image:
        :param visualize:
        :return:
        """

        image = Image.fromarray(cv2.resize(image, (self.model_image_size[0], self.model_image_size[1])))

        resized_image = image.resize(tuple(reversed(self.model_image_size)), Image.BICUBIC)
        image_data = np.array(resized_image, dtype='float32')

        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

        out_boxes, out_scores, out_classes = self.sess.run(
            [self.boxes, self.scores, self.classes],
            feed_dict={
                self.yolo_model.input: image_data,
                self.input_image_shape: [image.size[1], image.size[0]],
                K.learning_phase(): 0
            })

        if visualize:
            image = self.draw_bboxes(image, b_boxes=out_boxes, scores=out_scores, classes=out_classes)
            image = cv2.resize(np.array(image), (640, 480))
            return image, out_boxes, out_scores, out_classes
        else:
            return None, out_boxes, out_scores, out_classes

    def draw_bboxes(self, image, b_boxes, scores, classes):

        array = np.uint8((image))
        image = Image.fromarray(array)

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