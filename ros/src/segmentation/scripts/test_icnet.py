#!/usr/bin/python

import argparse
import time
import json

import numpy as np
import matplotlib.pyplot as plt
from keras import backend as K
from keras.models import load_model
import tensorflow as tf
from keras import optimizers
import utils
import cv2

from models.icnet import ICNet
from utils import apply_color_map


#### Test ####

# Workaround to forbid tensorflow from crashing the gpu
config = tf.ConfigProto()
config.gpu_options.allow_growth = True
sess = tf.Session(config=config)
K.set_session(sess)

# Model
optim = optimizers.SGD(lr=0.01, momentum=0.9)
net = ICNet(width=512, height=512, n_classes=13, weight_path='./icnet3-v13.h5', training=False)
print(net.model.summary())

# Testing
x = cv2.resize(cv2.imread("./testing_imgs/test_1.jpg", 1), (512, 512))
x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)
x = np.array([x])
y = net.model.predict(x)[0]
start_time = time.time()
for i in range(50):
    y = net.model.predict(x)[0]
duration = time.time() - start_time
print('Generated segmentations in %s seconds -- %s FPS' % (duration / 50, 1.0/(duration/50)))
y = cv2.resize(y, (512, 512))
image = utils.convert_class_to_rgb(y, threshold=0.25)
plt.figure(1)
plt.subplot(1, 2, 1)
plt.imshow(x[0])
plt.subplot(1, 2, 2)
plt.imshow(image)
plt.show()

