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
import scipy.misc
import utils
import cv2
from rambo import Rambo


#initialize our server
sio = socketio.Server()
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
        # The current steering angle of the car
        steering_angle = float(data["steering_angle"])
        # The current throttle of the car, how hard to push peddle
        throttle = float(data["throttle"])
        # The current speed of the car
        speed = float(data["speed"])
        # The current image from the center camera of the car
        image = Image.open(BytesIO(base64.b64decode(data["image"])))
        
        try:
            image = np.asarray(image)       # from PIL image to numpy array

            steering_angle = steering_predictor.predict(cv2.cvtColor(cv2.resize(image, (256, 192)), cv2.COLOR_BGR2GRAY))
            # lower the throttle as the speed increases
            # if the speed is above the current speed limit, we are on a downhill.
            # make sure we slow down first and then go back to the original max speed.
            global speed_limit
            if speed > speed_limit:
                speed_limit = MIN_SPEED  # slow sdown
            else:
                speed_limit = MAX_SPEED
            throttle = 1.0 - steering_angle**2 - (speed/speed_limit)**2

            print('{} {} {}'.format(steering_angle, throttle, speed))
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

    steering_predictor = Rambo("./final_model.hdf5", "./X_train_mean.npy")

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
