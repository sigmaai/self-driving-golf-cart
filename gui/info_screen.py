from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw
import numpy as np

def draw_cruise_info_screen(speed, obj_size, obj_type="vehicle", height=100, width=640):

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
    return np.array(image)
