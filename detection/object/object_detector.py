#
# Vehicle detection class
# Self-driving golf cart
# Developed by Neil Nie | (c) 2018
#

import cv2
import random
import colorsys
import numpy as np
from keras.models import load_model
from PIL import ImageDraw, ImageFont, Image
from keras import backend as K
import detection.object.model_data.configs as configs
from detection.object.yad2k.models.keras_yolo import yolo_eval, yolo_head


class ObjectDetector:

    def __init__(self):

        self.sess = K.get_session()

        with open(configs.classes_path) as f:
            class_names = f.readlines()
        self.class_names = [c.strip() for c in class_names]

        with open(configs.anchors_path) as f:
            anchors = f.readline()
            anchors = [float(x) for x in anchors.split(',')]
            anchors = np.array(anchors).reshape(-1, 2)

        self.yolo_model = load_model(configs.model_path)
        self.yolo_model.summary()

        model_image_size = self.yolo_model.layers[0].input_shape
        is_fixed_size = model_image_size != (None, None)
        print(model_image_size)

        # Generate colors for drawing bounding boxes.
        hsv_tuples = [(x / len(class_names), 1., 1.) for x in range(len(class_names))]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), self.colors))
        random.seed(10101)  # Fixed seed for consistent colors across runs.
        random.shuffle(self.colors)  # Shuffle colors to decorrelate adjacent classes.
        random.seed(None)  # Reset seed to default.

        self.font = ImageFont.truetype(font='detection/object/font/FiraMono-Medium.otf',
                                  size=np.floor(3e-2 * configs.height + 0.5).astype('int32'))
        self.thickness = (configs.width + configs.height) // 300

        # Generate output tensor targets for filtered bounding boxes.
        yolo_outputs = yolo_head(self.yolo_model.output, anchors, len(class_names))
        self.input_image_shape = K.placeholder(shape=(2,))
        self.boxes, self.scores, self.classes = yolo_eval(yolo_outputs, self.input_image_shape, score_threshold=configs.score_threshold,
                                           iou_threshold=configs.iou_threshold)

    def detect_objects(self, image, details=False):

        # don't need to resize image
        # when calling the method

        resized_image = cv2.resize(image, (416, 416))
        image_data = np.array(resized_image, dtype='float32')

        image_data /= 255.
        # image_data = np.expand_dims(, 0)  # Add batch dimension.
        print(image_data.shape)
        # making predictions
        out_boxes, out_scores, out_classes = self.sess.run(
            [self.boxes, self.scores, self.classes],
            feed_dict={
                self.yolo_model.input: np.array([image_data]),
                self.input_image_shape: [image.shape[1], image.shape[0]],
                K.learning_phase(): 0
            })

        array = np.uint8((image))
        image = Image.fromarray(array)
        image = self.draw_bboxes(image, b_boxes=out_boxes, scores=out_scores, classes=out_classes)

        if details:
            return np.array(image), out_boxes, out_scores, out_classes
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
