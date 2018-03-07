
import base64               # decoding camera images
import numpy as np
import socketio             #real-time server
import eventlet.wsgi
from flask import Flask     #web framework
from io import BytesIO      #input output
from semantic_segmentation.segmentor import Segmentor
from semantic_segmentation.segmentation_analyzer import SegAnalyzer
from PIL import Image
from scipy import misc

#initialize our server
sio = socketio.Server()
#our flask (web) app
app = Flask(__name__)
prev_image_array = None

#set min/max speed for our autonomous car
MAX_SPEED = 3
MIN_SPEED = 3

#and a speed limit
speed_limit = MAX_SPEED

#registering event handler for the server
@sio.on('telemetry')
def telemetry(sid, data):

    if data:
        # # The current steering angle of the car
        # steering_angle = float(data["steering_angle"])
        # # The current throttle of the car, how hard to push peddle
        # throttle = float(data["throttle"])
        # The current speed of the car
        speed = float(data["speed"])
        # The current image from the center camera of the car
        image = misc.imread(BytesIO(base64.b64decode(data["image"])))

        try:
            image = np.asarray(image)
            right_img = image[0: int(image.shape[0]), int(image.shape[1] / 2):int(image.shape[1])]
            left_img  = image[0: int(image.shape[0]), 0: int(image.shape[1] / 2)]

            left_result = segmentor.semantic_segmentation(left_img)
            right_result = segmentor.semantic_segmentation(right_img)
            steering_angle = seg_analyzer.analyze_side_cam(left=left_result[0], right=right_result[0])

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

    segmentor = Segmentor("ENET")  # init segmentor
    seg_analyzer = SegAnalyzer(threshold=0.05)

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
