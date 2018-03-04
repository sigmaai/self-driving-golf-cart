import cv2
import os
from time import gmtime, strftime
import configs.configs as configs

class Recorder:

    def __init__(self, path):
        self.labels = None
        self.count = 0
        if not os.path.exists(path):
            os.makedirs(path)
        self.path = path
        self.write_text()

    def write_text(self):
        strftime("%Y-%m-%d %H:%M:%S", gmtime())
        file = open(os.path.join(self.path, strftime("%Y-%m-%d %H:%M", gmtime()) + "_test"), "w")
        file.write(strftime("%Y-%m-%d %H/%M \n", gmtime()))
        file.write("--------------------------- \n".format(configs.st_fac))
        file.write("steering scaling factor: {} \n".format(configs.st_fac))
        file.write("img size: {} \n".format(configs.default_img_size))
        file.write("detection: {} \n".format(configs.detection))
        file.write("nagivation: {} \n".format(configs.navigation))
        file.write("default_st_port: {} \n".format(configs.default_st_port))
        file.close()

    def save_image(self, image, angle):
        cv2.imwrite(os.path.join(self.path, '{}.png'.format(self.count)), image)
        count = count + 1
