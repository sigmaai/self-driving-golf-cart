#import rospy
#from steering_node import SteeringNode

from collections import deque
import argparse
import scipy.misc
import cv2
from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw
import tensorflow as tf
from keras import backend as K
from keras.models import *
from keras.layers import *
from keras.layers.recurrent import LSTM
import tensorflow as tf
from keras import backend as K
from keras.models import *
from keras.layers import *
from keras.layers.recurrent import LSTM


class AutumnModel(object):

    def __init__(self, cnn_graph, lstm_json, cnn_weights, lstm_weights):

        sess = tf.InteractiveSession()
        saver = tf.train.import_meta_graph(cnn_graph)
        saver.restore(sess, cnn_weights)
        self.cnn = tf.get_default_graph()

        self.fc3 = self.cnn.get_tensor_by_name("fc3/mul:0")
        self.y = self.cnn.get_tensor_by_name("y:0")
        self.x = self.cnn.get_tensor_by_name("x:0")
        self.keep_prob = self.cnn.get_tensor_by_name("keep_prob:0")

        with open(lstm_json, 'r') as f:
            json_string = f.read()
        self.model = model_from_json(json_string)
        self.model.load_weights(lstm_weights)

        self.prev_image = None
        self.last = []
        self.steps = []


    def post_process_image(self, image, angle):

        background = Image.fromarray(np.uint8(image))
        sw = Image.open("./steering/resources/sw.png")
        sw = sw.rotate(angle * 180 / np.pi)
        sw = sw.resize((80, 80), Image.ANTIALIAS)
        background.paste(sw, (10, 10), sw)

        draw = ImageDraw.Draw(background)
        font = ImageFont.truetype("./steering/resources/FiraMono-Medium.otf", 16)
        draw.text((80, 200), str(round(angle, 3)), (255, 255, 255), font=font)

        return np.array(background)

    def process(self, img):

        prev_image = self.prev_image if self.prev_image is not None else img
        self.prev_image = img
        prev = cv2.cvtColor(prev_image, cv2.COLOR_RGB2GRAY)
        next = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        flow = cv2.calcOpticalFlowFarneback(prev, next, None, 0.5, 3, 15, 3, 5, 1.2, 0)

        self.last.append(flow)

        if len(self.last) > 4:
            self.last.pop(0)

        weights = [1, 1, 2, 2]
        last = list(self.last)
        for x in range(len(last)):
            last[x] = last[x] * weights[x]

        avg_flow = sum(last) / sum(weights)
        mag, ang = cv2.cartToPolar(avg_flow[..., 0], avg_flow[..., 1])

        hsv = np.zeros_like(prev_image)
        hsv[..., 1] = 255
        hsv[..., 0] = ang * 180 / np.pi / 2
        hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
        rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
        return rgb

    def predict(self, img):

        img = self.process(img)
        image = scipy.misc.imresize(img[-400:], [66, 200]) / 255.0
        cnn_output = self.fc3.eval(feed_dict={self.x: [image], self.keep_prob: 1.0})
        self.steps.append(cnn_output)
        if len(self.steps) > 100:
            self.steps.pop(0)
        output = self.y.eval(feed_dict={self.x: [image], self.keep_prob: 1.0})
        return output[0][0]


if __name__ == '__main__':

    cnn_graph = "./aweights/autumn-cnn-model-tf.meta"
    lstm_json = "./aweights/autumn-lstm-model-keras.json"
    cnn_weights = "./aweights/autumn-cnn-weights.ckpt"
    lstm_weights = "./aweights/autumn-lstm-weights.hdf5"

    def make_predictor():
        model = AutumnModel(cnn_graph, lstm_json, cnn_weights, lstm_weights)
        return lambda img: model.predict(img)

    def process(predictor, img):
        return predictor(img)

    model = make_predictor()

