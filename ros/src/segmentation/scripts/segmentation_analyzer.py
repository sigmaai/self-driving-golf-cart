import cv2
import math
# import semantic_segmentation.configs as configs
from scipy import misc
from semantic_segmentation.segmentor import Segmentor
import matplotlib.pyplot as plt

class SegAnalyzer:

    def __init__(self, threshold):
        self.threshold = threshold

    def analyze_image(self, im_mask):

        # parameter:
        # image
        # return:
        # breaking value, from 1-3

        total_obs = 0
        im_mask = im_mask[int(im_mask.shape[0] / 5 * 4): im_mask.shape[0],
                  int(im_mask.shape[1] / 4): int(im_mask.shape[1] / 4 * 3), :]

        for i in range(9, 13):
            sub_mask = im_mask[:, :, i]

            sub_mask[sub_mask > self.threshold] = 1
            sub_mask[sub_mask < self.threshold] = 0
            obs_mask = sub_mask[sub_mask == 1]
            total_obs += len(obs_mask)

        perc_obs = total_obs / (im_mask.shape[0] * im_mask.shape[1])
        if perc_obs >= 0.20:
            return 0, perc_obs
        else:
            return 1, perc_obs

    def analyze_side_cam(self, left, right):

        # parameter: image mask (output of segmentation network)
        # return: steering value
        left_mask = left[:, :, 2]
        right_mask = right[:, :, 2]

        left_mask[left_mask > self.threshold] = 1
        sub_left = left_mask[left_mask == 1]
        left_len = len(sub_left)

        right_mask[right_mask > self.threshold] = 1
        sub_right = right_mask[right_mask == 1]
        right_len = len(sub_right)

        if left_len > right_len:
            angle = right_len / left_len * math.pi / 6 * -1
            return angle
        elif left_len < right_len:
            angle = left_len / right_len * math.pi / 6
            return angle
        else:
            return 0.0


if __name__ == "__main__":

    segmentor = Segmentor("ENET")  # init segmentor
    seg_analyzer = SegAnalyzer(threshold=0.05)

    image = misc.imread("/Users/yongyangnie/Desktop/1479425551051871594.jpg")
    right_img = image[0: int(image.shape[0]), int(image.shape[1] / 2):int(image.shape[1])]
    left_img = image[0: int(image.shape[0]), 0: int(image.shape[1] / 2)]

    left_result = segmentor.semantic_segmentation(left_img, visualize=True)
    plt.imshow(left_result[1])
    plt.show()
    print(left_result[0].shape)
    right_result = segmentor.semantic_segmentation(right_img)
    plt.imshow(right_result[1])
    plt.show()
    steering_angle = seg_analyzer.analyze_side_cam(left=left_result[0], right=right_result[0])

    print(steering_angle)