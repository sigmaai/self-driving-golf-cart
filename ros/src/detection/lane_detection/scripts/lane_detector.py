#
# Lane Detection Class
# Part of the <lane_detection> node
# The <detection> module
#
# The self-driving-golf cart
# (c) Neil Nie

import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt
import utils


class LaneDetector():

    def __init__(self):
        self.objpoints, self.imgpoints = utils.calibrate_camera()
        print("finished calibration")
        self.test_detector()

    def test_detector(self):

        f, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 9))
        f.tight_layout()
        fig, axies = plt.subplots(2, 3, figsize=(13, 6))
        f.tight_layout()
        fig.suptitle('Plot', fontsize=20)
        test = utils.get_test_images()
        for i in range(6):
            row = int(i / 3)
            col = i % 3
            axies[row][col].set_title('test' + str(i + 1), fontdict={'fontsize': 15})
            img = self.lane_detection(test[i])
            axies[row][col].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
        plt.show()

    def main_first(self, img):
        height = img.shape[0]
        warped, _ = utils.perspective(utils.undistort(img, self.objpoints, self.imgpoints))
        warped_edge = utils.edge(warped)
        left_base, right_base = utils.base(warped_edge)
        left_fit, _ = utils.fit_first(warped_edge, left_base)
        right_fit, _ = utils.fit_first(warped_edge, right_base)

        return left_fit, right_fit

    def main_rest(self, img, prev_left_fit, prev_right_fit):
        height = img.shape[0]
        width = img.shape[1]
        warped, M = utils.perspective(utils.undistort(img, self.objpoints, self.imgpoints))
        warped_edge = utils.edge(warped)
        left_base, right_base = utils.base(warped_edge)
        left_fit, left_fit_m = utils.fit_rest(warped_edge, prev_left_fit)
        right_fit, right_fit_m = utils.fit_rest(warped_edge, prev_right_fit)
        curv = utils.curvature(left_fit_m, right_fit_m, height)
        shif = utils.shift(left_fit_m, right_fit_m, height, width)

        return left_fit, right_fit, curv, shif, M

    def draw(self, img, left_fit, right_fit, curv, shif, M):
        # Create an image to draw the lines on
        canvas = np.zeros_like(img)
        # get fitted points
        ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
        # Recast the x and y points into usable format for cv2.fillPoly()

        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(canvas, np.int_([pts]), (0, 255, 0))

        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        warped = cv2.warpPerspective(canvas, np.linalg.inv(M), (img.shape[1], img.shape[0]))
        # Combine the result with the original image
        result = cv2.addWeighted(img, 1, warped, 0.3, 0)

        cv2.putText(result, 'Curvature:{}m'.format(int(curv)), (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, [255, 255, 255],
                    2)
        cv2.putText(result, 'Shift:{:.2f}m'.format(shif), (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, [255, 255, 255], 2)

        return result

    # main method for lane detection, return an image
    def lane_detection(self, img):
        init_left_fit, init_right_fit = self.main_first(img)
        left_fit, right_fit, curv, shif, M = self.main_rest(img, init_left_fit, init_right_fit)
        result = self.draw(img, left_fit, right_fit, curv, shif, M)

        return result


