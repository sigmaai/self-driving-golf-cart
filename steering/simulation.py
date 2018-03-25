import argparse         # parsing command line arguments
import base64           # decoding camera images
from datetime import datetime
import os
import cv2
import numpy as np
import socketio         #real-time server
import eventlet.wsgi
from PIL import Image   #image manipulation
from flask import Flask #web framework
from io import BytesIO  #input output
from scipy import misc

import utils as utils
import models as models
from autumn import AutumnModel
from rambo import Rambo
from komanda import KomandaModel
from auto_pilot import AutoPilot

#initialize our server
sio = socketio.Server()
#our flask (web) app
app = Flask(__name__)
prev_image_array = None

#set min/max speed for our autonomous car
MAX_SPEED = 10
MIN_SPEED = 10

#and a speed limit
speed_limit = MAX_SPEED

#registering event handler for the server
@sio.on('telemetry')
def telemetry(sid, data):

    if data:

        # The current steering angle of the car
        steering_angle = float(data["steering_angle"])
        # The current throttle of the car, how hard to push peddle
        throttle = float(data["throttle"])
        # The current speed of the car
        speed = float(data["speed"])
        # The current image from the center camera of the car
        image = Image.open(BytesIO(base64.b64decode(data["image"])))
        
        try:

            if mode == "OWN":
                image = np.asarray(image)  # from PIL image to numpy array
                image = utils.preprocess(image)  # apply the preprocessing
                image = np.array([image])  # the model expects 4D array
                steering_angle = -1.0 * float(cnn.predict(image, batch_size=1))
            elif mode == "AUT":
                image = np.asarray(image)
                steering_angle = -1 * steering_predictor.predict(image)
            elif mode == "KOM":
                image = np.asarray(image)
                image = misc.imresize(image, (480, 640))
                steering_angle = steering_predictor.predict(image)
            elif mode == "RAM":
                steering_angle = -1 * steering_predictor.predict_image(image)
            elif mode == "AUP":
                image = np.asarray(image)
                steering_angle = steering_predictor.predict(image)

            print("steering: " + str(steering_angle))

            # lower the throttle as the speed increases
            # if the speed is above the current speed limit, we are on a downhill.
            # make sure we slow down first and then go back to the original max speed.
            global speed_limit
            if speed > speed_limit:
                speed_limit = MIN_SPEED  # slow sdown
            else:
                speed_limit = MAX_SPEED

            throttle = 1.0 - (speed/speed_limit)**2

            send_control(steering_angle, throttle)
        except Exception as e:
            print(e)

    else:
        sio.emit('manual', data={}, skip_sid=True)


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    send_control(0, 0)


def send_control(steering_angle, throttle):
    sio.emit(
        "steer",
        data={
            'steering_angle': steering_angle.__str__(),
            'throttle': throttle.__str__()},
        skip_sid=True)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Remote Driving')
    parser.add_argument('--model', type=str, help='Path to model h5 file. Model should be on the same path.')
    parser.add_argument('--type', type=str, help='Type of model')
    args = parser.parse_args()

    mode = args.type

    if mode == "OWN":
        # load model
        cnn = models.commaai_model()
        cnn.summary()
        cnn.load_weights(args.model)
    elif mode == "AUT":
        cnn_graph = "./weights/autumn/autumn-cnn-model-tf.meta"
        lstm_json = "./weights/autumn/autumn-lstm-model-keras.json"
        cnn_weights = "./weights/autumn/autumn-cnn-weights.ckpt"
        lstm_weights = "./weights/autumn/autumn-lstm-weights.hdf5"
        steering_predictor = AutumnModel(cnn_graph, lstm_json, cnn_weights, lstm_weights)
        print(steering_predictor.model.summary())
    elif mode == "KOM":
        checkpoint_dir = "./weights/komanda"
        metagraph_file = "./weights/komanda/komanda.test-subgraph.meta"
        steering_predictor = KomandaModel(checkpoint_dir=checkpoint_dir, metagraph_file=metagraph_file)
    elif mode == "RAM":
        steering_predictor = Rambo("./final_model.hdf5", "./X_train_mean.npy")
        print(steering_predictor.model.summary())
    elif mode == "AUP":
        steering_predictor = AutoPilot()

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
