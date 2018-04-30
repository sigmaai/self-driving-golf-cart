#!/usr/bin/python
#
# Results video generator Udacity Challenge 2
# Original By: Comma.ai and Chris Gundling
# Revised and used by Neil Nie
#

import numpy as np
from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw
import matplotlib
from skimage import transform as tf
import matplotlib.pyplot as plt
from vis.visualization import visualize_cam
import cv2

# ros
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError

class Visualization():

    # ***** get perspective transform for images *****
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

    def perspective_tform(self, x, y):
        p1, p2 = self.tform3_img((x, y))[0]
        return p2, p1

    # ***** functions to draw lines *****
    def draw_pt(self, img, x, y, color, sz=2):
        row, col = self.perspective_tform(x, y)
        row = row * 2
        col = col * 2
        if row >= 0 and row < img.shape[0] * 2 / 2 and col >= 0 and col < img.shape[1] * 2 / 2:
            img[int(row - sz):int(row + sz), int(col - sz):int(col + sz)] = color

    def draw_path(self, img, path_x, path_y, color):
        for x, y in zip(path_x, path_y):
            self.draw_pt(img, x, y, color)

    # ***** functions to draw predicted path *****

    def calc_curvature(self, v_ego, angle_steers, angle_offset=0):
        deg_to_rad = np.pi / 180.
        slip_fator = 0.0014  # slip factor obtained from real data
        steer_ratio = 15.3  # from http://www.edmunds.com/acura/ilx/2016/road-test-specs/
        wheel_base = 2.67  # from http://www.edmunds.com/acura/ilx/2016/sedan/features-specs/

        angle_steers_rad = (angle_steers - angle_offset)  # * deg_to_rad
        curvature = angle_steers_rad / (steer_ratio * wheel_base * (1. + slip_fator * v_ego ** 2))
        return curvature

    def calc_lookahead_offset(self, v_ego, angle_steers, d_lookahead, angle_offset=0):
        # *** this function returns the lateral offset given the steering angle, speed and the lookahead distance
        curvature = self.calc_curvature(v_ego, angle_steers, angle_offset)

        # clip is to avoid arcsin NaNs due to too sharp turns
        y_actual = d_lookahead * np.tan(np.arcsin(np.clip(d_lookahead * curvature, -0.999, 0.999)) / 2.)
        return y_actual, curvature

    # main methods --

    # TODO: Test this method!
    def visualize_line(self, img, speed_ms, angle_steers, color=(0, 0, 255)):

        path_x = np.arange(0, 50.1, 0.5)
        path_y, _ = self.calc_lookahead_offset(speed_ms, angle_steers, path_x)
        self.draw_path(img, path_x, path_y, color)

        return img

    def visualize_steering_wheel(self, image, angle):

        background = Image.fromarray(np.uint8(image))
        sw = Image.open("./steering/resources/sw.png")
        sw = sw.rotate(angle * 180 / np.pi)
        sw = sw.resize((80, 80), Image.ANTIALIAS)
        background.paste(sw, (10, 10), sw)

        draw = ImageDraw.Draw(background)
        font = ImageFont.truetype("./steering/resources/FiraMono-Medium.otf", 16)
        draw.text((80, 200), str(round(angle, 3)), (255, 255, 255), font=font)
        steering_img = cv2.resize(np.array(background), (640, 480))
        return steering_img

    # TODO: Actually test this method
    def visualize_class_activation_map(self, model, image):

        image = cv2.resize(image, (320, 160))
        heatmap = visualize_cam(model, layer_idx=-1, filter_indices=0, seed_input=image, grad_modifier=None)
        heatmap = cv2.cvtColor(heatmap, cv2.COLORRGB2BGR)
        heatmap = cv2.addWeighted(image, 1.0, heatmap, 0.5, 0)
        heatmap = cv2.resize(heatmap, (640, 480))
        return heatmap

    def image_update_callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.current_frame = cv_image

    def steering_cmd_callback(self, data):

        self.steering_angle = data.data

    def __init__(self):

        rospy.init_node('steering_visualization')
        rospy.Subscriber('/cv_camera_node/image_raw', Image, callback=self.image_update_callback, queue_size=5)
        rospy.Subscriber('/vehicle/dbw/steering_cmds/', Float32, callback=self.steering_cmd_callback)

        visual_pub = rospy.Publisher('/visual/steering/angle_img', Image, queue_size=5)
        rate = rospy.Rate(15)

        self.current_frame = None
        self.bridge = CvBridge()
        self.steering_angle = 0.0

        while not rospy.is_shutdown():

            if self.current_frame is not None:
                image = self.visualize_line(img=self.current_frame, angle_steers=self.steering_angle * 0.1, speed_ms=5)
                img_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                visual_pub.publish(img_msg)
                rate.sleep()
            else:
                rate.sleep()


if __name__ == "__main__":

    try:
        Visualization()
    except rospy.ROSInterruptException:
        pass