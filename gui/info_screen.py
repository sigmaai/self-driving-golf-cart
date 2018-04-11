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


def draw_cruise_info_screen(speed, obj_size, stopping_enabled, obj_type="vehicle", height=120, width=640):

    image = Image.open("./gui/resources/bkg.jpg")
    image = image.resize((width, height), Image.ANTIALIAS)

    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype("./gui/resources/san-reg.otf", 18)

    if speed == 0:
        display_text = "[Status]: STOP!"
    else:
        display_text = "[Status]: Go! All Clear"
    size_text = "[Object Size]: " + str(round(obj_size, 3))
    draw.text((50, 25), display_text, (0, 0, 0), font=font)
    draw.text((260, 25), "[Object type]: " + obj_type, (0, 0, 0), font=font)
    draw.text((50, 50), size_text, (0, 0, 0), font=font)
    draw.text((260, 50), "[Option]: Press the 'a' key to disable stopping", (0, 0, 0), font=font)
    draw.text((50, 75), "[Stopping]: " + "Enabled" if stopping_enabled else "Disabled" , (0, 0, 0), font=font)
    return np.array(image)


def draw_steering_info_screen(angle, fps, height=120, width=640):

    image = Image.open("./gui/resources/bkg.jpg")
    image = image.resize((width, height), Image.ANTIALIAS)

    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype("./gui/resources/san-reg.otf", 18)

    draw.text((50, 25), "[Steering Angle]: " + str(angle), (0, 0, 0), font=font)
    draw.text((260, 50), "[Frame Rate]: " + str(fps), (0, 0, 0), font=font)
    draw.text((50, 75), "[Options]: Turn on heatmap with the 'H' key", (0, 0, 0), font=font)
    return np.array(image)
