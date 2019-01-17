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
import cv2
import numpy as np


class Segmentor():

    def __init__(self, weight_path, img_width, img_height, nb_classes, disp_width, disp_height):

        self.model = ICNet(width=img_width, height=img_height, n_classes=nb_classes,
                           weight_path=weight_path, training=False)

        self.img_width = img_width
        self.img_height = img_height
        self.disp_width = disp_width
        self.disp_height = disp_height
        self.nb_classes = nb_classes

        print(self.model.model.summary())
        self.backgrounds = self.load_color_backgrounds()

    def load_color_backgrounds(self):

        backgrounds = []

        for i in range(len(utils.labels)):
            color = utils.labels[i][7]

            bg = np.zeros((self.disp_height, self.disp_width, 3), dtype=np.uint8)
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

        in_img = cv2.resize(image, (self.img_width, self.img_height))
        disp_img = cv2.resize(in_img, (self.disp_width, self.disp_height))
        output = self.model.model.predict(np.array([in_img]))[0]
        output = cv2.resize(output, (self.disp_width, self.disp_height))

        if visualize:
            im_mask = self.convert_class_to_rgb(image_labels=output)
            viz = cv2.addWeighted(im_mask, 0.8, disp_img, 0.8, 0)
            return output, viz
        else:
            return output, None

    def convert_class_to_rgb(self, image_labels, threshold=0.50):

        # convert any pixel > threshold to 1
        # convert any pixel < threshold to 0
        # then use bitwise_and

        output = np.zeros((self.disp_height, self.disp_width, 3), dtype=np.uint8)

        for i in range(self.nb_classes):

            split = image_labels[:, :, i]
            split[split > threshold] = 1
            split[split < threshold] = 0
            split[:] *= 255

            res = cv2.bitwise_and(self.backgrounds[i], self.backgrounds[i], mask=split)

            output = cv2.addWeighted(output, 1.0, res, 1.0, 0)

        return output





