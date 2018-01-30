'''
Results video generator Udacity Challenge 2
Original By: Comma.ai and Chris Gundling
Revised and used by Neil Nie
'''

from __future__ import print_function
import argparse
import numpy as np
import cv2
import pandas as pd
from os import path
import model

matplotlib.use("Agg")



# ***** get perspective transform for images *****
from skimage import transform as tf

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


def perspective_tform(x, y):
    p1, p2 = tform3_img((x, y))[0]
    return p2, p1


# ***** functions to draw lines *****
def draw_pt(img, x, y, color, sz=2):
    row, col = perspective_tform(x, y)
    row = row * 2
    col = col * 2
    if row >= 0 and row < img.shape[0] * 2 / 2 and \
                    col >= 0 and col < img.shape[1] * 2 / 2:
        img[int(row - sz):int(row + sz), int(col - sz):int(col + sz)] = color


def draw_path(img, path_x, path_y, color):
    for x, y in zip(path_x, path_y):
        draw_pt(img, x, y, color)


# ***** functions to draw predicted path *****

def calc_curvature(v_ego, angle_steers, angle_offset=0):
    deg_to_rad = np.pi / 180.
    slip_fator = 0.0014  # slip factor obtained from real data
    steer_ratio = 15.3  # from http://www.edmunds.com/acura/ilx/2016/road-test-specs/
    wheel_base = 2.67  # from http://www.edmunds.com/acura/ilx/2016/sedan/features-specs/

    angle_steers_rad = (angle_steers - angle_offset)  # * deg_to_rad
    curvature = angle_steers_rad / (steer_ratio * wheel_base * (1. + slip_fator * v_ego ** 2))
    return curvature


def calc_lookahead_offset(v_ego, angle_steers, d_lookahead, angle_offset=0):
    # *** this function returns the lateral offset given the steering angle, speed and the lookahead distance
    curvature = calc_curvature(v_ego, angle_steers, angle_offset)

    # clip is to avoid arcsin NaNs due to too sharp turns
    y_actual = d_lookahead * np.tan(np.arcsin(np.clip(d_lookahead * curvature, -0.999, 0.999)) / 2.)
    return y_actual, curvature


def draw_path_on(img, speed_ms, angle_steers, color=(0, 0, 255)):
    path_x = np.arange(0, 50.1, 0.5) #50.1
    path_y, _ = calc_lookahead_offset(speed_ms, angle_steers, path_x)
    draw_path(img, path_x, path_y, color)


def preprocess_img(img):
    
    b, g, r = cv2.split(img)  # get b,g,r
    img = cv2.merge([r, g, b])  # switch it to rgb
    img = img[160: 480, 0:640]
    img = cv2.resize(img, (320, 160))
    return img

# ***** main loop *****
if __name__ == "__main__":

    windowName = "car detection"
    cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(windowName, 640, 480)
    cv2.moveWindow(windowName, 0, 0)
    cv2.setWindowTitle(windowName, "car detection")

    parser = argparse.ArgumentParser(description='Path viewer')
    parser.add_argument('--model', type=str, help='path to the trained model')
    parser.add_argument('--dataset', type=str, help='dataset folder with csv and image folders')
    parser.add_argument('--camera', type=str, default='center', help='camera to use, default is center')
    args = parser.parse_args()

    dataset_path = args.dataset
    camera = args.camera
    cnn = model.small_vgg_network()
    cnn.load_weights(args.model)
    cnn.summary()

    # steerings and images
    steering_labels = path.join(dataset_path, 'interpolated.csv')

    # read the steering labels and image path
    df_truth = pd.read_csv(steering_labels, usecols=['frame_id', 'steering_angle'], index_col=None)

    speed_ms = 5  # log['speed'][i]

    # Run through all images
    for i in range(len(df_truth)):

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break

        if i%100 == 0:
            print('%.2f seconds elapsed' % (i/20))

        path = dataset_path + "center/" + str(df_truth["frame_id"][i]) + ".jpg"
        img = cv2.imread(path)
        image = np.array([preprocess_img(img)])  # the model expects 4D array

        predicted_steers = cnn.predict(image)[0][0]
        actual_steers = df_truth['steering_angle'].loc[i]

        draw_path_on(img, speed_ms, actual_steers / 5.0)
        draw_path_on(img, speed_ms, predicted_steers / 5.0, (255, 0, 0))

        ax.autoscale_view()


        displayBuf = img

            # show the stuff
            # ----------------------------------------------------
        cv2.imshow(windowName, displayBuf)


