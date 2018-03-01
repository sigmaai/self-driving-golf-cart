import cv2
import math
# import semantic_segmentation.configs as configs


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
        print(perc_obs)
        if perc_obs >= 0.20:
            return 0
        else:
            return 1

    def analyze_side_cam(self, left, right):

        # parameter: image mask (output of segmentation network)
        # return: steering value

        left_mask = left[:, :, 2]
        right_mask = right[:, :, 2]

        left_mask[left_mask > self.threshold] = 1
        left_mask[left_mask < self.threshold] = 0
        left_len = len(left_mask)

        right_mask[right_mask > self.threshold] = 1
        right_mask[right_mask < self.threshold] = 0
        right_len = len(right_mask)

        if left_len > right_len:
            angle = right_len / left_len * math.pi / 4
            return angle
        elif left_len < right_len:
            angle = left_len / right_len * math.pi / 4 * -1
            return angle
        else:
            return 0.0
