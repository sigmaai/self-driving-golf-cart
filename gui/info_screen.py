#
# The information display for the vehicle
# Cruise control (segmentation) has its own info screen
# Steering has a info screen
# Detection info screen is coming...
#
# By Neil Nie
# (c) Yongyang Nie, 2018. All Rights Reserved. MIT License
# Contact: yongyang.nie@gmail.com
#


from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw
import numpy as np


class InfoScreen:

    def __init__(self):

        height = 70
        width = 640

        self.__background = Image.open("./gui/resources/bkg.jpg")
        self.__background = self.__background.resize((width, height), Image.ANTIALIAS)
        self.__font = ImageFont.truetype("./gui/resources/san-reg.otf", 16)

    def draw_cruise_info_screen(self, speed, obj_size, stopping_disabled, obj_type="vehicle"):

        img = self.__background.copy()
        draw = ImageDraw.Draw(img)

        if speed == 0:
            display_text = "[Status]: STOP!"
        else:
            display_text = "[Status]: Go! All Clear"
        size_text = "[Object Size]: " + str(round((obj_size * 57.2958), 3))
        draw.text((50, 13), display_text, (0, 0, 0), font=self.__font)
        draw.text((260, 13), "[Object type]: " + obj_type, (0, 0, 0), font=self.__font)
        draw.text((50, 37), size_text, (0, 0, 0), font=self.__font)
        draw.text((260, 37), "[Stopping]: " + "Disabled" if stopping_disabled else "[Stopping]: Enabled", (0, 0, 0), font=self.__font)
        return np.array(img)

    def draw_steering_info_screen(self, angle, fps):

        img = self.__background.copy()
        draw = ImageDraw.Draw(img)
        draw.text((50, 13), "[Steering Angle]: " + str(round(angle, 4)), (0, 0, 0), font=self.__font)
        draw.text((50, 37), "[Frame Rate]: " + str(round(fps, 3)) + "fps", (0, 0, 0), font=self.__font)
        return np.array(img)



