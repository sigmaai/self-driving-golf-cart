#!/usr/bin/python
#
# The AutoPilot module <visualization> node
# Part of the self-driving golf cart project
#
# Original By: Comma.ai and Chris Gundling
# Revised and used by Neil Nie
#

# General Dependencies
import numpy as np
from skimage import transform as tf
import cv2
from PIL import Image as PILImage
from PIL import ImageFont
from PIL import ImageDraw

# ROS Dependencies
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError


class Visualization():

    # ***** get perspective transform for images *****
    rsrc = \
        [[43.45456230828867, 118.00743250075844],
         [104.5055617352614, 83.46865203761757],
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
    def visualize_line(self, img, speed_ms, angle_steers, color=(0, 0, 255)):

        path_x = np.arange(0, 50.1, 0.5)
        path_y, _ = self.calc_lookahead_offset(speed_ms, angle_steers, path_x)
        self.draw_path(img, path_x, path_y, color)

        return img, path_x, path_y

    def visualize_steering_wheel(self, image, angle):

        background = PILImage.fromarray(np.uint8(image))
        sw = PILImage.open("./steering/resources/sw.png")
        sw = sw.rotate(angle * 180 / np.pi)
        sw = sw.resize((80, 80), PILImage.ANTIALIAS)
        background.paste(sw, (10, 10), sw)

        draw = ImageDraw.Draw(background)
        font = ImageFont.truetype("./steering/resources/FiraMono-Medium.otf", 16)
        draw.text((80, 200), str(round(angle, 3)), (255, 255, 255), font=font)
        steering_img = cv2.resize(np.array(background), (640, 480))
        return steering_img

    def apply_accel_visualization(self, image, accel):

        background = PILImage.fromarray(np.uint8(image))
        if accel < -1:
            sw = PILImage.open("/home/neil/Workspace/self-driving-golf-cart/ros/src/autopilot/scripts/resources/stop.png")
            sw = sw.resize((80, 80), PILImage.ANTIALIAS)
            background.paste(sw, (10, 38), sw)

        draw = ImageDraw.Draw(background)
        font = ImageFont.truetype("/home/neil/Workspace/self-driving-golf-cart/ros/src/autopilot/scripts/resources/FiraMono-Medium.otf", 20)
        draw.text((40, 420), str(round(accel, 3)), (255, 255, 255), font=font)
        draw.text((10, 10), "Cruise Control Running... v1.0.0", (255, 255, 255), font=font)
        text = ""
        if accel < 0:
            text = 'Slow'
        elif accel > 0:
            text = 'Speed Up'
        draw.text((40, 380), text, (255, 255, 255), font=font)
        steering_img = cv2.resize(np.array(background), (640, 480))
        return steering_img

    # ------------------------------------------------------------------------------------------------------------------
    # cv_camera callback
    def camera_update_callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            raise e

        self.current_frame = cv_image

    def steering_cmd_callback(self, data):

        self.steering_angle = data.data

    def cruise_callback(self, data):

        self.cruise_cmds = data.data

    # ------------------------------------------------------------------------------------------------------------------

    def __init__(self):

        rospy.init_node('autopilot_visualization_node')

        self.current_frame = None
        self.bridge = CvBridge()
        self.steering_angle = 0.0
        self.cruise_cmds = 0.0

        # -----------------------------------------
        rospy.Subscriber("/zed/rgb/image_raw_color", Image,
                         callback=self.camera_update_callback, queue_size=8)
        # -----------------------------------------

        rospy.Subscriber('/vehicle/dbw/steering_cmds', Float32, callback=self.steering_cmd_callback)
        rospy.Subscriber('/vehicle/dbw/cruise_cmds', Float32, callback=self.cruise_callback)

        steering_viz_pub = rospy.Publisher('/visual/autopilot/angle_img', Image, queue_size=5)
        accel_viz_pub = rospy.Publisher('/visual/autopilot/cruise_img', Image, queue_size=5)
        steering_ios_path = rospy.Publisher('/visual/ios/steering/path', Float32MultiArray, queue_size=5)

        rate = rospy.Rate(24)

        while not rospy.is_shutdown():

            if self.current_frame is not None:

                # Apply Steering Visualization #  -0.025
                image, path_x, path_y = self.visualize_line(img=self.current_frame.copy(),
                                                            angle_steers=self.steering_angle * -0.035,
                                                            speed_ms=5)

                # Generate steering path message
                path_msg = Float32MultiArray()
                path_msg.data = np.hstack([path_x, path_y])
                steering_ios_path.publish(path_msg)

                # Generate steering image message
                img_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                steering_viz_pub.publish(img_msg)

                # Apply Accel Visualization
                image = self.apply_accel_visualization(image=self.current_frame, accel=self.cruise_cmds)
                img_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                accel_viz_pub.publish(img_msg)

            rate.sleep()


if __name__ == "__main__":

    try:
        Visualization()
    except rospy.ROSInterruptException:
        pass