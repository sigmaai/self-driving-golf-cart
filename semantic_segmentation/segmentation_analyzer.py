import cv2
# import semantic_segmentation.configs as configs
# import

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
