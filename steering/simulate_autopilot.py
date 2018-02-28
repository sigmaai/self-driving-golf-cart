import argparse         # parsing command line arguments
import base64           # decoding camera images
from datetime import datetime
import os               #high level file operations #reading and writing files
import shutil
import numpy as np
import socketio         #real-time server
import eventlet.wsgi
from PIL import Image   #image manipulation
from flask import Flask #web framework
from io import BytesIO  #input output
from steering.AutoPilot import AutoPilot
import time


#initialize our server
sio = socketio.Server()
buffer = None
#our flask (web) app
app = Flask(__name__)
prev_image_array = None

#set min/max speed for our autonomous car
MAX_SPEED = 20
MIN_SPEED = 10

#and a speed limit
speed_limit = MAX_SPEED

#registering event handler for the server
@sio.on('telemetry')
def telemetry(sid, data):
    if data:

        # The current speed of the car
        speed = float(data["speed"])
        # The current image from the center camera of the car
        image = Image.open(BytesIO(base64.b64decode(data["image"])))

        try:
            image = np.asarray(image)
            steering_angle = steering_predictor.predict(image)
            print("steering: " + str(steering_angle))
            # print(steering_angle)
            # lower the throttle as the speed increases
            # if the speed is above the current speed limit, we are on a downhill.
            # make sure we slow down first and then go back to the original max speed.
            global speed_limit
            if speed > speed_limit:
                speed_limit = MIN_SPEED  # slow sdown
            else:
                speed_limit = MAX_SPEED
            throttle = 1.0 - steering_angle**2 - (speed/speed_limit)**2

            # print('{} {} {}'.format(steering_angle, throttle, speed))
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

    cnn_graph = "./weights/autumn/autumn-cnn-model-tf.meta"
    lstm_json = "./weights/autumn/autumn-lstm-model-keras.json"
    cnn_weights = "./weights/autumn/autumn-cnn-weights.ckpt"
    lstm_weights = "./weights/autumn/autumn-lstm-weights.hdf5"

    steering_predictor = AutoPilot()

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
