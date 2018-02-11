import cv2
import semantic_segmentation.configs as configs

class SegAnalyzer:

    def __init__(self, threshold):
        self.threshold = threshold


    def analyze_image(self, img):

        # parameter:
        # image
        # return:
        # breaking value, from 1-3
        crop1 = img[int(img.shape[0] / 5 * 3):img.shape[0], int(img.shape[1] / 5):int(img.shape[1]), :]

        return 3
