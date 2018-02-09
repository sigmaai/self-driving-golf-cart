import cv2
import semantic_segmentation.configs as configs

class SegAnalyzer:

    def __init__(self, threshold):
        self.threshold = threshold


    def analyze_image(self, image):

        # parameter:
        # image
        # return:
        # breaking value, from 1-3

        return 3