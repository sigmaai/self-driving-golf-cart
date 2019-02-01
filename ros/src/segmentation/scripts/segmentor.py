#
# Semantic Segmentator
# Class for driving image segmentation
# Integrated in drive.py. For more
# information see README.md.
# ------------------------------------
# Neil Nie & Michael Meng
# (c) Yongyang Nie 2018


from models.icnet import ICNet
import utils as utils
import configs as configs
import cv2
import numpy as np
from PIL import Image


class Segmentor():

    def __init__(self, weight_path):

        self.model = ICNet(width=1024, height=512, n_classes=configs.nb_classes, weight_path=weight_path, training=False)
        print(self.model.model.summary())
        self.backgrounds = self.load_color_backgrounds()

    @staticmethod
    def load_color_backgrounds():

        backgrounds = []

        for i in range(len(utils.labels)):
            color = utils.labels[i][7]

            bg = np.zeros((480, 640, 3), dtype=np.uint8)
            bg[:, :, 0].fill(color[0])
            bg[:, :, 1].fill(color[1])
            bg[:, :, 2].fill(color[2])
            backgrounds.append(bg)

        return backgrounds

    def semantic_segmentation(self, image, visualize=False):

        # parameters
        # image: input image
        # visualize: whether to visualize the segmentation results
        # return
        # output: output of ConvNet
        # img_pred: visualization

        image = cv2.resize(image, (configs.img_width, configs.img_height))
        output = self.model.model.predict(np.array([image]))[0]

        if visualize:
            im_mask = self.convert_class_to_rgb(image_labels=output)
            viz = cv2.addWeighted(im_mask, 0.8, cv2.resize(image, (640, 480)), 0.8, 0)
            viz = cv2.resize(viz, (640, 480))
            return output, viz
        else:
            return output, None

    def convert_class_to_rgb(self, image_labels, threshold=0.75):

        # convert any pixel > threshold to 1
        # convert any pixel < threshold to 0
        # then use bitwise_and

        output = np.zeros((480, 640, 3), dtype=np.uint8)

        for i in range(configs.nb_classes):
            split = image_labels[:, :, i]
            split[split > threshold] = 1
            split[split < threshold] = 0
            split[:] *= 255
            split = cv2.resize(split.astype(np.uint8), (640, 480))

            res = cv2.bitwise_and(self.backgrounds[i], self.backgrounds[i], mask=split)

            output = cv2.addWeighted(output, 1.0, res, 1.0, 0)

        return output
