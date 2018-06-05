#!/usr/bin/env python

"""
Udacity self-driving car challenge 2
Team komanda steering model
Author: Ilya Edrenkin, ilya.edrenkin@gmail.com
"""

import argparse
import tempfile
from collections import deque
import numpy as np
from models.ConvLSTM import ConvLSTM
import models.configs as configs
from models.utils import BatchGenerator
import tensorflow as tf


class KomandaModel(object):

    def __init__(self, checkpoint_dir, metagraph_file):

        self.graph = tf.Graph()
        self.LEFT_CONTEXT = 5 # TODO remove hardcode; store it in the graph
        with self.graph.as_default():
            saver = tf.train.import_meta_graph(metagraph_file)
            ckpt = tf.train.latest_checkpoint(checkpoint_dir)
        self.session = tf.Session(graph=self.graph)
        saver.restore(self.session, ckpt)
        self.input_images = deque() # will be of size self.LEFT_CONTEXT + 1
        self.internal_state = [] # will hold controller_{final -> initial}_state_{0,1,2}

        self.inputs = tf.placeholder(shape=(configs.BATCH_SIZE, configs.LEFT_CONTEXT + configs.SEQ_LEN),
                                     dtype=tf.string)  # pathes to png files from the central camera
        self.targets = tf.placeholder(shape=(configs.BATCH_SIZE, configs.SEQ_LEN, configs.OUTPUT_DIM),
                                      dtype=tf.float32)

        graph = ConvLSTM()
        self.steering_predictions = graph.steering_predictions
        self.controller_final_state_autoregressive = graph.controller_final_state_autoregressive

        tf.initialize_all_variables()
        # # TODO controller state names should be stored in the graph
        # self.input_tensors = map(self.graph.get_tensor_by_name, ["input_images:0", "controller_initial_state_0:0", "controller_initial_state_1:0", "controller_initial_state_2:0"])
        # self.output_tensors = map(self.graph.get_tensor_by_name, ["output_steering:0", "controller_final_state_0:0", "controller_final_state_1:0", "controller_final_state_2:0"])

    def predict(self, img):

        # if it's the first frame, create the sequence by filling the queue
        if len(self.input_images) == 0:
            self.input_images += [img] * (self.LEFT_CONTEXT + 1)
        else:
            # remove the leftest and add one frame to the right
            self.input_images.popleft()
            self.input_images.append(img)

        input_images_tensor = np.stack(self.input_images)

        if not self.internal_state:
            feed_dict = {self.inputs[0]: input_images_tensor}
        else:
            feed_dict = dict(zip(self.inputs, [input_images_tensor] + self.internal_state))

        steering, c0, c1, c2 = self.session.run(self.targets, feed_dict=feed_dict)
        self.internal_state = [c0, c1, c2]
        print(self.internal_state)
        return 0.0


# some python notes to myself
# ---------------------------
# what's the zip method?
# >>> a = ["a" , "b", "c"]
# >>> b = [1, 2, 3]
# >>> d = zip(a, b)
# >>> d
# [('a', 1), ('b', 2), ('c', 3)]
#
