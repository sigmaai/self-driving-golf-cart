'''
Dataset visualization tool
Original By: Comma.ai and Chris Gundling
Revised and used by Neil Nie
'''

import matplotlib.backends.backend_agg as agg
import numpy as np
import cv2
import pygame
import os
import pylab
from i3d import Inception3D
import helper
import configs
from termcolor import colored

pygame.init()
size = (640, 640)
pygame.display.set_caption("speed prediction viewer")
screen = pygame.display.set_mode(size, pygame.DOUBLEBUF)
screen.set_alpha(None)

camera_surface = pygame.surface.Surface((640, 480), 0, 24).convert()
clock = pygame.time.Clock()

PI_RAD = (180 / np.pi)
white = (255, 255, 255)

# Create second screen with matplotlibs
fig = pylab.figure(figsize=[6.4, 1.6], dpi=100)
ax = fig.gca()
ax.tick_params(axis='x', labelsize=8)
ax.tick_params(axis='y', labelsize=8)
line1, = ax.plot([], [], 'b.-', label='Human')
A = []
ax.legend(loc='upper left', fontsize=8)

myFont = pygame.font.SysFont("monospace", 18)
randNumLabel = myFont.render('Human Driving Speed:', 1, white)


def test_loop(model_path, model_type):

    '''
    for visualizing the model with the comma AI
    test dataset. The ds doesn't contain training labels.

    :param model_path: the path of the trained Keras model
    :param model_type: the type of model, rgb, flow or rgb-flow
    :return: None
    '''

    print(colored('Preparing', 'blue'))

    model = Inception3D(weights_path=model_path, input_shape=(configs.LENGTH, configs.IMG_HEIGHT, configs.IMG_WIDTH, configs.CHANNELS))

    # read the steering labels and image path
    files = os.listdir(configs.TEST_DIR)

    inputs = []
    starting_index = 10000

    start = input("prompt")
    if start is not 'start':
        exit(0)

    if model_type == 'rgb':

        for i in range(starting_index, starting_index + configs.LENGTH):
            img = helper.load_image(configs.TEST_DIR + "frame" + str(i) + ".jpg")
            inputs.append(img)

        print(colored('Started', 'blue'))

        # Run through all images
        for i in range(starting_index + configs.LENGTH + 1, len(files) - 1):

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    break

            img = helper.load_image(configs.TEST_DIR + "frame" + str(i) + ".jpg", resize=False)
            in_frame = cv2.resize(img, (configs.IMG_WIDTH, configs.IMG_HEIGHT))
            inputs.pop(0)
            inputs.append(in_frame)
            input_array = np.array([input])
            prediction = model.model.predict(input_array)[0][0]

            pygame_loop(prediction=prediction, img=img)

    elif model_type == 'flow':

        previous = helper.load_image(configs.TEST_DIR + "frame" + str(starting_index) + ".jpg")

        for i in range(starting_index, starting_index + configs.LENGTH):

            img = helper.load_image(configs.TEST_DIR + "frame" + str(i) + ".jpg")
            in_frame = cv2.resize(img, (configs.IMG_WIDTH, configs.IMG_HEIGHT))
            flow = helper.optical_flow(previous=previous, current=in_frame)
            inputs.append(flow)

        previous = helper.load_image(configs.TEST_DIR + "frame" + str(starting_index + configs.LENGTH) + ".jpg")

        for i in range(starting_index + configs.LENGTH + 1, len(files) - 1):

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    break

            img = helper.load_image(configs.TEST_DIR + "frame" + str(i) + ".jpg", resize=False)
            in_frame = cv2.resize(img, (configs.IMG_WIDTH, configs.IMG_HEIGHT))
            flow = helper.optical_flow(previous, in_frame)
            inputs.pop(0)
            inputs.append(flow)
            input_array = np.array([np.asarray(inputs)])
            prediction = model.model.predict(input_array)[0][0]

            pygame_loop(prediction=prediction, img=img)

    else:
        raise Exception('Sorry, the model type is not recognized')


def pygame_loop(prediction, img):

    if prediction <= 10:
        speed_label = myFont.render('Slow', 1, white)
    elif prediction > 10 and prediction <= 25:
        speed_label = myFont.render('Medium', 1, white)
    elif prediction > 25 and prediction <= 40:
        speed_label = myFont.render('Fast', 1, white)
    else:
        speed_label = myFont.render('Very Fast', 1, white)

    A.append(prediction)
    line1.set_ydata(A)
    line1.set_xdata(range(len(A)))
    ax.relim()
    ax.autoscale_view()

    canvas = agg.FigureCanvasAgg(fig)
    canvas.draw()
    renderer = canvas.get_renderer()
    raw_data = renderer.tostring_rgb()
    size = canvas.get_width_height()
    surf = pygame.image.fromstring(raw_data, size, "RGB")
    screen.blit(surf, (0, 480))

    # draw on
    pygame.surfarray.blit_array(camera_surface, img.swapaxes(0, 1))
    screen.blit(camera_surface, (0, 0))

    diceDisplay = myFont.render(str(prediction), 1, white)
    screen.blit(randNumLabel, (50, 420))
    screen.blit(speed_label, (300, 420))
    screen.blit(diceDisplay, (50, 450))
    clock.tick(60)
    pygame.display.flip()


if __name__ == "__main__":

    test_loop(model_path='i3d_speed_comma_rgb_64_3.h5', model_type='rgb')