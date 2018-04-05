#
# MIT License

# Copyright (c) 2017 Yongyang Nie, Michael Meng

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

import cv2
import numpy as np
import colorsys
from keras import backend as K
from PIL import ImageDraw, ImageFont, Image
import model_data.configs as configs
from keras.models import load_model
from yad2k.models.keras_yolo import yolo_eval, yolo_head
import random


def read_cam():

    sess = K.get_session()  

    with open(configs.classes_path) as f:
        class_names = f.readlines()
    class_names = [c.strip() for c in class_names]

    with open(configs.anchors_path) as f:
        anchors = f.readline()
        anchors = [float(x) for x in anchors.split(',')]
        anchors = np.array(anchors).reshape(-1, 2)

    yolo_model = load_model(configs.model_path)
    yolo_model.summary()

    # Check if model is fully convolutional, assuming channel last order.
    model_image_size = yolo_model.layers[0].input_shape[1:3]
    is_fixed_size = model_image_size != (None, None)

    # Generate colors for drawing bounding boxes.
    hsv_tuples = [(x / len(class_names), 1., 1.)
                  for x in range(len(class_names))]
    colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
    colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), colors))
    random.seed(10101)      # Fixed seed for consistent colors across runs.
    random.shuffle(colors)  # Shuffle colors to decorrelate adjacent classes.
    random.seed(None)       # Reset seed to default.

    # Generate output tensor targets for filtered bounding boxes.
    yolo_outputs = yolo_head(yolo_model.output, anchors, len(class_names))
    input_image_shape = K.placeholder(shape=(2, ))
    boxes, scores, classes = yolo_eval(yolo_outputs, input_image_shape, score_threshold=configs.score_threshold, iou_threshold=configs.iou_threshold)

    # OpenCV main loop
    cap = cv2.VideoCapture("nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)416, height=(int)416,format=(string)I420, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")

    if cap.isOpened():

        windowName = "car detection"
        cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(windowName, 416, 416)
        cv2.moveWindow(windowName, 0, 0)
        cv2.setWindowTitle(windowName, "car detection")

        while True:

            # -----------------------------------------------------
            # Check to see if the user closed the window
            if cv2.getWindowProperty(windowName, 0) < 0:
                # This will fail if the user closed the window; get printed to the console
                break
            ret_val, image = cap.read()

            # -----------------------------------------------------
            # run the network and detection

            resized_image = cv2.resize(image, (416, 416))
            image_data = np.array(resized_image, dtype='float32')

            image_data /= 255.
            image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

            # making predictions
            out_boxes, out_scores, out_classes = sess.run(
                [boxes, scores, classes],
                feed_dict={
                    yolo_model.input: image_data,
                    input_image_shape: [image.shape[1], image.shape[0]],
                    K.learning_phase(): 0
                })

            font = ImageFont.truetype(font='font/FiraMono-Medium.otf',
                                      size=np.floor(3e-2 * image.shape[1] + 0.5).astype('int32'))
            thickness = (image.shape[0] + image.shape[1]) // 300

            array = np.uint8((image))
            image = Image.fromarray(array)

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

            displayBuf = np.array(image)

            # show the stuff
            # -----------------------------------------------------

            cv2.imshow(windowName, displayBuf)
            key = cv2.waitKey(10)
            if key == 27:  # ESC key
                cv2.destroyAllWindows()
                break

    else:
        print("camera open failed")


if __name__ == '__main__':
    read_cam()
