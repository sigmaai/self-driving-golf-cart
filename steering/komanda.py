#!/usr/bin/env python

"""
Udacity self-driving car challenge 2
Team komanda steering model
Author: Ilya Edrenkin, ilya.edrenkin@gmail.com
"""

import argparse
import tempfile
from collections import deque
from scipy import misc

import cv2
import numpy as np
import tensorflow as tf

from math import pi
import time


class KomandaModel(object):

    def __init__(self, checkpoint_dir, metagraph_file):

        self.graph = tf.Graph()
        self.LEFT_CONTEXT = 5 # TODO remove hardcode; store it in the graph
        with self.graph.as_default():
            saver = tf.train.import_meta_graph(metagraph_file)
            ckpt = tf.train.latest_checkpoint(checkpoint_dir)

        self.session = tf.Session(graph=self.graph)
        saver.restore(self.session, ckpt)
        self.input_images = deque()     # will be of size self.LEFT_CONTEXT + 1
        self.internal_state = []        # will hold controller_{final -> initial}_state_{0,1,2}

        # TODO controller state names should be stored in the graph
        self.input_tensors = map(self.graph.get_tensor_by_name, ["input_images:0", "controller_initial_state_0:0", "controller_initial_state_1:0", "controller_initial_state_2:0"])
        self.output_tensors = map(self.graph.get_tensor_by_name, ["output_steering:0", "controller_final_state_0:0", "controller_final_state_1:0", "controller_final_state_2:0"])

    def predict(self, img):

        if len(self.input_images) == 0:
            self.input_images += [img] * (self.LEFT_CONTEXT + 1)
        else:
            self.input_images.popleft()
            self.input_images.append(img)

        input_images_tensor = np.stack(self.input_images)

        if not self.internal_state:
            input = list(self.input_tensors)[0]
            feed_dict = {input: input_images_tensor}

        else:
            feed_dict = dict(list(zip(list(self.input_tensors), ([input_images_tensor] + self.internal_state))))

        steering, c0, c1, c2 = self.session.run(list(self.output_tensors), feed_dict=feed_dict)
        self.internal_state = [c0, c1, c2]

        self.input_tensors = map(self.graph.get_tensor_by_name,
                                 ["input_images:0", "controller_initial_state_0:0", "controller_initial_state_1:0",
                                  "controller_initial_state_2:0"])
        self.output_tensors = map(self.graph.get_tensor_by_name,
                                  ["output_steering:0", "controller_final_state_0:0", "controller_final_state_1:0",
                                   "controller_final_state_2:0"])
        return steering[0][0]


if __name__ == '__main__':

    checkpoint_dir = "./weights/komanda"
    metagraph_file = "./weights/komanda/komanda.test-subgraph.meta"

    steering_predictor = KomandaModel(checkpoint_dir=checkpoint_dir, metagraph_file=metagraph_file)

    img = misc.imread("/Users/yongyangnie/Desktop/img0.png")
    img1 = misc.imread("/Users/yongyangnie/Desktop/img1.png")
    img0 = misc.imresize(img, (480, 640))
    img1 = misc.imresize(img1, (480, 640))
    img2 = misc.imresize(img, (480, 640))
    print(steering_predictor.predict(img0))
    print(steering_predictor.predict(img1))
    print(steering_predictor.predict(img2))

