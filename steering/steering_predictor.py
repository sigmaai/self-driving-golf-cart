
import numpy as np
import cv2
import pygame
import steering.configs as configs
import matplotlib.backends.backend_agg as agg
import pylab
import steering.model as model
import tensorflow as tf


class SteeringPredictor:

    rsrc = \
        [[43.45456230828867, 118.00743250075844],
         [104.5055617352614, 69.46865203761757],
         [114.86050156739812, 60.83953551083698],
         [129.74572757609468, 50.48459567870026],
         [132.98164627363735, 46.38576532847949],
         [301.0336906326895, 98.16046448916306],
         [238.25686790036065, 62.56535881619311],
         [227.2547443287154, 56.30924933427718],
         [209.13359962247614, 46.817221154818526],
         [203.9561297064078, 43.5813024572758]]
    rdst = \
        [[10.822125594094452, 1.42189132706374],
         [21.177065426231174, 1.5297552836484982],
         [25.275895776451954, 1.42189132706374],
         [36.062291434927694, 1.6376192402332563],
         [40.376849698318004, 1.42189132706374],
         [11.900765159942026, -2.1376192402332563],
         [22.25570499207874, -2.1376192402332563],
         [26.785991168638553, -2.029755283648498],
         [37.033067044190524, -2.029755283648498],
         [41.67121717733509, -2.029755283648498]]

    tform3_img = tf.ProjectiveTransform()
    tform3_img.estimate(np.array(rdst), np.array(rsrc))

    @staticmethod
    def calc_curvature(self, v_ego, angle_steers, angle_offset=0):
        deg_to_rad = np.pi / 180.
        slip_fator = 0.0014  # slip factor obtained from real data
        steer_ratio = 15.3  # from http://www.edmunds.com/acura/ilx/2016/road-test-specs/
        wheel_base = 2.67  # from http://www.edmunds.com/acura/ilx/2016/sedan/features-specs/

        angle_steers_rad = (angle_steers - angle_offset)  # * deg_to_rad
        curvature = angle_steers_rad / (steer_ratio * wheel_base * (1. + slip_fator * v_ego ** 2))
        return curvature

    def perspective_tform(self, x, y):
        p1, p2 = self.tform3_img((x, y))[0]
        return p2, p1

    # ***** functions to draw lines *****
    def draw_pt(self, img, x, y, color, sz=2):
        row, col = self.perspective_tform(x, y)
        row = row * 2
        col = col * 2
        if row >= 0 and row < img.shape[0] * 2 / 2 and \
                col >= 0 and col < img.shape[1] * 2 / 2:
            img[int(row - sz):int(row + sz), int(col - sz):int(col + sz)] = color

    def draw_path(self, img, path_x, path_y, color):
        for x, y in zip(path_x, path_y):
            self.draw_pt(img, x, y, color)

    # ***** functions to draw predicted path *****

    def calc_lookahead_offset(self, v_ego, angle_steers, d_lookahead, angle_offset=0):
        # *** this function returns the lateral offset given the steering angle, speed and the lookahead distance
        curvature = self.calc_curvature(v_ego, angle_steers, angle_offset)

        # clip is to avoid arcsin NaNs due to too sharp turns
        y_actual = d_lookahead * np.tan(np.arcsin(np.clip(d_lookahead * curvature, -0.999, 0.999)) / 2.)
        return y_actual, curvature

    def draw_path_on(self, img, speed_ms, angle_steers, color=(0, 0, 255)):
        path_x = np.arange(0, 50.1, 0.5)
        path_y, _ = self.calc_lookahead_offset(speed_ms, angle_steers, path_x)
        self.draw_path(img, path_x, path_y, color)

    def __init__(self):

        self.cnn = model.small_vgg_network()
        self.cnn.load_weights(configs.model_path)
        print("steering model loaded")

        # # Create second screen with matplotlib
        # fig = pylab.figure(figsize=[6.4, 1.6], dpi=100)
        # self.ax = fig.gca()
        # self.ax.tick_params(axis='x', labelsize=8)
        # self.ax.tick_params(axis='y', labelsize=8)
        # # ax.legend(loc='upper left',fontsize=8)
        # self.line, = self.ax.plot([], [], 'r.-', label='Model')
        # self.A = []
        # self.ax.legend(loc='upper left', fontsize=8)

        blue = (0, 0, 255)
        myFont = pygame.font.SysFont("monospace", 18)
        self.randNumLabel = myFont.render('Model Steer Angle:', 1, blue)

    def predict_steering(self, image):

        predicted_steers = self.cnn.predict(image)[0][0]

        self.draw_path_on(image, 5, predicted_steers / 5.0, (255, 0, 0))

        # self.A.append(predicted_steers)
        # self.line.set_ydata(self.A)
        # self.line.set_xdata(range(len(self.A)))
        # self.ax.relim()
        # self.ax.autoscale_view()

        return predicted_steers, image

